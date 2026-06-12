#pragma once
#include "HorizonTypes.hpp"
#include "HorizonDetector.hpp"
#include "SwitchingLogic.hpp"

// Replaced the MLX90640 API with standard STM32 HAL
#include "stm32l4xx_hal.h" 

class HorizonSubsystem {
public:
    HorizonSubsystem();
    
    // Call this ONCE during satellite boot
    bool init_sensors();

    // The single function your ADCS task calls in a loop
    HorizonOutput process_sensors();

    float debug_thermal[4][768];
    uint8_t debug_mask[4][768];

private:
    static constexpr int NUM_SENSORS = 4; // Adjust based on how many you have active; nominally 4
    const float SENSOR_ROLL_OFFSETS[4] = {0.0f, 90.0f, 180.0f, 270.0f};

    // I2C addresses
    const uint8_t SENSOR_ADDRESSES[4] = {0x66, 0x67, 0x68, 0x69};

    HorizonDetector detector;
    HorizonSensorManager manager;

    // Hardware read wrapper
    bool read_thermal_camera(int sensor_idx, float* buffer);

    // Custom lightweight I2C helpers for the MLX90642
    bool I2C_ReadReg(int sensor_idx, uint16_t reg_addr, uint16_t* data);
    bool I2C_WriteReg(int sensor_idx, uint16_t reg_addr, uint16_t data);
    bool I2C_ReadBlock(int sensor_idx, uint16_t reg_addr, uint16_t* data, uint16_t length);
};
