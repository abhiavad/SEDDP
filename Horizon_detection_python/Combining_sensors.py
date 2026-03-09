import numpy as np
import math
from image_generation_V3 import simulate_earth
from Switching_logic import HorizonSensorManager
from horizon_detection_V6 import vector, integrate_angles, convert_coordinates_canonical


# --- 1. GLOBAL CONFIGURATION (C++: constexpr / PROGMEM) ---
CONFIG_THRESHOLD = 35 
SENSOR_RES = (24, 32)
ORIGIN_COORD = (11, 15)

# The physical mounting offsets of the 4 sensors around the roll axis
# C++: const float SENSOR_ROLL_OFFSETS[4] = {0.0f, 90.0f, 180.0f, 270.0f};
SENSOR_ROLL_OFFSETS = [0.0, 90.0, 180.0, 270.0]

# --- 2. PRE-ALLOCATE MEMORY (C++: Initialize arrays in Flash/RAM once) ---
X_angles_deg = np.loadtxt("FOV_Files/horizontal_angles.txt", delimiter="\t") 
Y_angles_deg = np.loadtxt("FOV_Files/vertical_angles.txt", delimiter="\t") 
X_angles = np.radians(X_angles_deg).astype(np.float32)
Y_angles = np.radians(Y_angles_deg).astype(np.float32)
origin_rad = np.array([X_angles[ORIGIN_COORD], Y_angles[ORIGIN_COORD]], dtype=np.float32)

# Load the 6-column banded table
conv_table = np.load("conversion_table.npy").astype(np.float32)

# --- 3. MAIN LOOP (C++: while(1) or RTOS Task) ---
if __name__=="__main__":
    # Initialize the State Machine
    sensor_manager = HorizonSensorManager()
    
    # C++: Allocate fixed buffers for the current frame
    # float current_pitches[4]; float current_rolls[4]; bool current_valids[4];
    current_pitches = [0.0, 0.0, 0.0, 0.0]
    current_rolls = [0.0, 0.0, 0.0, 0.0]
    current_valids = [False, False, False, False]
    
    print(f"{'TRUE ATTITUDE':<15} | {'SENSOR MANAGER OUTPUT':<40}")
    print("-" * 65)

    # Simulating the satellite rolling 360 degrees over time
    # This represents the continuous loop running on your STM32
    for true_roll in range(0, 360, 5): 
        true_pitch = 25.0 
        
        # 1. Poll all 4 sensors
        for i in range(4):
            # Simulate what THIS specific sensor sees based on its 90-degree mounting
            # In real C++, this would be a call to I2C_Read_MLX90640(sensor_i)
            sensor_view_roll = (true_roll - SENSOR_ROLL_OFFSETS[i]) % 360
            raw_data = simulate_earth(true_pitch, sensor_view_roll, 0, plot=False)
            
            # 2. Process the frame
            v_x, v_y = vector(raw_data, CONFIG_THRESHOLD, X_angles, Y_angles, origin_rad)
            
            if not np.isnan(v_x):
                area = integrate_angles(raw_data, CONFIG_THRESHOLD, X_angles, Y_angles)

                 # Use the Canonical function with the specific sensor's mounting offset
                 # Note: Body Roll transformation is now INSIDE this function
                detected_p, body_roll = convert_coordinates_canonical(
                      v_x, v_y, area, conv_table, SENSOR_ROLL_OFFSETS[i]
                )

                if not np.isnan(detected_p):
                    current_pitches[i] = detected_p
                    current_rolls[i] = body_roll
                    current_valids[i] = True
                    continue # Skip the failure state below
            
            # If we reach here, the sensor didn't see the Earth
            current_pitches[i] = 0.0
            current_rolls[i] = 0.0
            current_valids[i] = False

        # 5. Feed the completely populated buffers to the Manager
        final_p, final_r, active_id, confidence = sensor_manager.update(
            current_pitches[0], current_rolls[0], current_valids[0],
            current_pitches[1], current_rolls[1], current_valids[1],
            current_pitches[2], current_rolls[2], current_valids[2],
            current_pitches[3], current_rolls[3], current_valids[3]
        )
        
        # Output the results
        conf_str = f"Conf: {confidence}/4" if confidence > 0 else "COASTING"
        print(f"P: {true_pitch:04.1f} R: {true_roll:05.1f} | Final -> P: {final_p:04.1f}, R: {final_r:05.1f} | ID: {active_id} [{conf_str}]")