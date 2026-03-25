
#pragma once
#include "HorizonTypes.hpp"
#include "HorizonDetector.hpp"
#include "SwitchingLogic.hpp"

// Include the official Melexis driver header
#include "MLX90640_API.h" 

class HorizonSubsystem {
public:
    HorizonSubsystem();
    
    // Call this ONCE during satellite boot
    bool init_sensors();

    // The single function your ADCS task calls in a loop
    HorizonOutput process_sensors();

private:
    static constexpr int NUM_SENSORS = 4;
    const float SENSOR_ROLL_OFFSETS[NUM_SENSORS] = {0.0f, 90.0f, 180.0f, 270.0f};

    // The I2C addresses of your 4 sensors 
    // (If using a multiplexer, these might all be 0x33, and you switch channels before reading)
    const uint8_t SENSOR_ADDRESSES[NUM_SENSORS] = {0x33, 0x34, 0x35, 0x36}; 

    // Calibration parameters for each sensor extracted from their EEPROMs
    paramsMLX90640 mlx_params[NUM_SENSORS];

    HorizonDetector detector;
    HorizonSensorManager manager;

    // Hardware read wrapper
    bool read_thermal_camera(int sensor_idx, float* buffer);
};