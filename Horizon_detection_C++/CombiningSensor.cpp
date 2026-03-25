#include "CombiningSensor.hpp"
#include <math.h>

HorizonSubsystem::HorizonSubsystem() {}

// ==============================================================================
// HARDWARE INITIALIZATION
// ==============================================================================
bool HorizonSubsystem::init_sensors(SystemData &adcsState) {
    static uint16_t eeMLX90640[832]; 
    bool atLeastOneWorking = false;

    for (int i = 0; i < 4; i++) {
        // Determine which bus to use based on index
        // EHS_0, EHS_1 -> I2C1 | EHS_2, EHS_3 -> I2C2
        I2C_HandleTypeDef* currentBus = (i < 2) ? &hi2c1 : &hi2c2;
        uint8_t addr = SENSOR_ADDRESSES[i % 2]; // 0x33 or 0x34

        // IMPORTANT: You may need to tell your I2C driver which bus is active 
        // before calling these library functions if they don't take a bus handle.
        set_active_i2c_bus(currentBus); 

        // 1. Configure the sensor
        if (MLX90640_SetRefreshRate(addr, 0x04) != 0 || 
            MLX90640_SetResolution(addr, 0x03) != 0) {
            adcsState.componentConnections[EHS_0 + i] = false;
            continue; // Skip to next sensor
        }

        // 2. Dump the EEPROM
        MLX90640_I2CFreqSet(400); // Set I2C frequency to 400 kHz for safe EEPROM read&write
        if (MLX90640_DumpEE(addr, eeMLX90640) != 0) {
            adcsState.componentConnections[EHS_0 + i] = false;
            continue; 
        }

        // 3. Extract parameters
        if (MLX90640_ExtractParameters(eeMLX90640, &mlx_params[i]) != 0) {
            adcsState.componentConnections[EHS_0 + i] = false;
            continue;
        }

        // If we reached here, this specific sensor is healthy
        adcsState.componentConnections[EHS_0 + i] = true;
        atLeastOneWorking = true;
    }

    return atLeastOneWorking; 
}

// ==============================================================================
// HARDWARE READING
// ==============================================================================
bool HorizonSubsystem::read_thermal_camera(int sensor_idx, float* buffer) {
    // Array to hold the raw frame data. 
    // Declared static to save stack space (1668 bytes).
    static uint16_t mlx90640Frame[834]; 
    
    uint8_t addr = SENSOR_ADDRESSES[sensor_idx];

    MLX90640_I2CFreqSet(1000); // Update frequency to 1 MHz

    // Read the raw ADC values from the sensor RAM
    int status = MLX90640_GetFrameData(addr, mlx90640Frame);
    if (status < 0) return false;

    // Calculate final temperatures (in degrees Celsius)
    float emissivity = 0.95f;
    float tr = 23.15f; // Reflected ambient temperature

    // This Melexis function populates your 768-element 'buffer' directly!
    MLX90640_CalculateTo(mlx90640Frame, &mlx_params[sensor_idx], emissivity, tr, buffer);
    
    return true;
}

// ==============================================================================
// ADCS MAIN PROCESSING LOOP
// ==============================================================================
HorizonOutput HorizonSubsystem::process_sensors() {
    float current_pitches[NUM_SENSORS] = {0};
    float current_rolls[NUM_SENSORS] = {0};
    float current_areas[NUM_SENSORS] = {0}; 
    bool current_valids[NUM_SENSORS] = {false};
    HorizonOutput final_output = {0};

    // Buffer for a single camera frame (768 pixels)
    float frame_buffer[HorizonDetector::PIXELS_PER_SENSOR] = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        
        // --- THE ACTUAL HARDWARE CALL ---
        bool read_success = read_thermal_camera(i, frame_buffer);
        
        if (read_success) {
            float local_p, local_r, local_area;
            
            // Pass the populated frame_buffer straight to your detector
            if (detector.process_frame(frame_buffer, local_p, local_r, local_area)) {
                current_pitches[i] = local_p;
                current_areas[i] = local_area;

                // Add the physical offset to convert local roll to body roll
                float global_r = fmodf(local_r + SENSOR_ROLL_OFFSETS[i], 360.0f);
                if (global_r < 0) global_r += 360.0f;
                
                current_rolls[i] = global_r;
                current_valids[i] = true;
            }
        }
    }

    // Pass the 4 sensor streams into the Manager
    manager.update(current_pitches, current_rolls, current_valids, current_areas, final_output);
    
    return final_output;
}