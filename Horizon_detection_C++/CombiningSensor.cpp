#include "CombiningSensor.hpp"
#include "../Drivers/MLX90640/Inc/MLX90642.h"
#include <math.h>
#include "main.h"

extern I2C_HandleTypeDef hi2c1; // First channel for 0x66 and 0x67
extern I2C_HandleTypeDef hi2c2; // Second channel for 0x68 and 0x69

HorizonSubsystem::HorizonSubsystem() {}

// ==============================================================================
// I2C HARDWARE HELPERS FOR MLX90642
// ==============================================================================

// ADD THIS HELPER TO ROUTE THE BUS CORRECTLY
static I2C_HandleTypeDef* get_i2c_bus(int sensor_idx) {
    // Indices 0,1 -> I2C1 | Indices 2,3 -> I2C2
    return (sensor_idx < 2) ? &hi2c1 : &hi2c2;
}

bool HorizonSubsystem::I2C_ReadReg(int sensor_idx, uint16_t reg_addr, uint16_t* data) {
    uint8_t rx_buf[2];
    uint8_t i2c_addr = SENSOR_ADDRESSES[sensor_idx];
    I2C_HandleTypeDef* hi2c = get_i2c_bus(sensor_idx);
    
    if (HAL_I2C_Mem_Read(hi2c, (i2c_addr << 1), reg_addr, I2C_MEMADD_SIZE_16BIT, rx_buf, 2, 20) == HAL_OK) {
        *data = (rx_buf[0] << 8) | rx_buf[1];
        return true;
    }
    return false;
}

bool HorizonSubsystem::I2C_WriteReg(int sensor_idx, uint16_t reg_addr, uint16_t data) {
    uint8_t i2c_addr = SENSOR_ADDRESSES[sensor_idx];
    I2C_HandleTypeDef* hi2c = get_i2c_bus(sensor_idx);
    uint8_t cmd[2];

    cmd[0] = data >> 8;
    cmd[1] = data & 0xFF;

    if (HAL_I2C_Mem_Write(hi2c, (i2c_addr << 1), reg_addr, I2C_MEMADD_SIZE_16BIT, cmd, 2, 20) == HAL_OK) {
        HAL_Delay(5); // Required 5ms EEPROM burn time
        return true;
    }
    return false;
}

bool HorizonSubsystem::I2C_ReadBlock(int sensor_idx, uint16_t reg_addr, uint16_t* data, uint16_t length) {
    static uint8_t rx_buf[1536]; //static to avoid memory issues in RTOS
    uint8_t i2c_addr = SENSOR_ADDRESSES[sensor_idx];
    I2C_HandleTypeDef* hi2c = get_i2c_bus(sensor_idx);
    
    if (HAL_I2C_Mem_Read(hi2c, (i2c_addr << 1), reg_addr, I2C_MEMADD_SIZE_16BIT, rx_buf, length * 2, 40) == HAL_OK) {
        for (int i = 0; i < length; i++) {
            data[i] = (rx_buf[i * 2] << 8) | rx_buf[i * 2 + 1];
        }
        return true;
    }
    return false;
}

// ==============================================================================
// HARDWARE INITIALIZATION
// ==============================================================================
bool HorizonSubsystem::init_sensors() {
    bool atLeastOneWorking = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        uint16_t fw_version = 0;

        // Check if the sensor is alive on the bus
        if (I2C_ReadReg(i, 0xFFF8, &fw_version)) {
            atLeastOneWorking = true;

            // --- NATIVE REFRESH RATE CONFIGURATION ---
            uint16_t rate_reg = 0;
            // Read the current configuration
            if (I2C_ReadReg(i, MLX90642_REFRESH_RATE_ADDRESS, &rate_reg)) {

                // Clear bits 0,1,2 and OR with the 4Hz mask
                rate_reg = (rate_reg & ~MLX90642_REFRESH_RATE_MASK) | MLX90642_REF_RATE_4HZ;

                // Write back to the sensor EEPROM
                I2C_WriteReg(i, MLX90642_REFRESH_RATE_ADDRESS, rate_reg);
            }
        }
    }

    return atLeastOneWorking; 
}
// ==============================================================================
// HARDWARE READING
// ==============================================================================
bool HorizonSubsystem::read_thermal_camera(int sensor_idx, float* buffer) {
    uint16_t status_reg = 0;
    
    // CRITICAL FIX: Pass 'sensor_idx' into these functions, NOT 'addr'
    // 1. Read DSP status flags
    if (!I2C_ReadReg(sensor_idx, MLX90642_FLAGS_ADDRESS, &status_reg)) return false;
    
    // 2. Check the READY flag
    if ((status_reg & MLX90642_FLAGS_READY_MASK) == 0) return false;
    
    // 3. Clear the READY flag by performing a dummy read
    uint16_t dummy;
    I2C_ReadReg(sensor_idx, MLX90642_TO_DATA_ADDRESS, &dummy);
    
    // 4. Block read the entire 768-pixel temperature frame
    static uint16_t mlx90642Frame[768];
    if (!I2C_ReadBlock(sensor_idx, MLX90642_TO_DATA_ADDRESS, mlx90642Frame, 768)) return false;
    
    // 5. Convert to Celsius
    for (int i = 0; i < 768; i++) {
        int16_t signed_val = (int16_t)mlx90642Frame[i];
        buffer[i] = (float)signed_val / 50.0f;
    }
    
    return true;
}

// ==============================================================================
// ADCS MAIN PROCESSING LOOP
// ==============================================================================
HorizonOutput HorizonSubsystem::process_sensors() {
    float current_pitches[4] = {0};
    float current_rolls[4] = {0};
    float current_areas[4] = {0};
    bool current_valids[4] = {false};
    HorizonOutput final_output = {0};

    for (int i = 0; i < NUM_SENSORS; i++) {
        
        bool read_success = read_thermal_camera(i, debug_thermal[i]);
        
        if (read_success) {
            float local_p, local_r, local_area;
            
            if (detector.process_frame(debug_thermal[i], local_p, local_r, local_area, debug_mask[i])) {
                current_pitches[i] = local_p;
                current_areas[i] = local_area;

                float global_r = fmodf(local_r + SENSOR_ROLL_OFFSETS[i], 360.0f);
                if (global_r < 0) global_r += 360.0f;
                
                current_rolls[i] = global_r;
                current_valids[i] = true;
            }
        }
    }

    manager.update(current_pitches, current_rolls, current_areas, current_valids, final_output);
    
    // Remove comment from this for loop when DEBUGGING, this will retrieve individual sensor's pitch and roll estimations
    for (int i = 0; i < NUM_SENSORS; i++) {
    	final_output.ind_pitch[i] = current_pitches[i];
    	final_output.ind_roll[i] = current_rolls[i];
    }

    return final_output;
}
