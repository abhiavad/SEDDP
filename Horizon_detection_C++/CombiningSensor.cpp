
#include "CombiningSensor.hpp"
#include <math.h>

// Assuming you have an external driver function for the MLX90640. 
// e.g., extern bool ReadThermalCamera(int sensor_id, float* buffer);

HorizonSubsystem::HorizonSubsystem() {}

HorizonOutput HorizonSubsystem::process_sensors() {
    float current_pitches[NUM_SENSORS] = {0};
    float current_rolls[NUM_SENSORS] = {0};
    bool current_valids[NUM_SENSORS] = {false};
    HorizonOutput final_output = {0};

    // Buffer for a single camera frame (768 pixels)
    float frame_buffer[HorizonDetector::PIXELS_PER_SENSOR] = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        
        // =================================================================
        // PLACEHOLDER: READ ACTUAL HARDWARE HERE
        // Replace the line below with your I2C reading function.
        // Make sure it populates 'frame_buffer' with 768 float temperatures.
        // =================================================================
        // bool read_success = ReadThermalCamera(i, frame_buffer);
        bool read_success = false; // Set to true when hardware is linked
        
        if (read_success) {
            float local_p, local_r;
            
            if (detector.process_frame(frame_buffer, local_p, local_r)) {
                current_pitches[i] = local_p;
                
                // Add the physical offset to convert local roll to body roll
                float global_r = fmodf(local_r + SENSOR_ROLL_OFFSETS[i], 360.0f);
                if (global_r < 0) global_r += 360.0f;
                
                current_rolls[i] = global_r;
                current_valids[i] = true;
            }
        }
    }

    // Pass the 4 sensor streams into the Manager
    manager.update(current_pitches, current_rolls, current_valids, final_output);
    
    return final_output;
}