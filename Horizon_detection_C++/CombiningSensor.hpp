
#pragma once
#include "HorizonTypes.hpp"
#include "HorizonDetector.hpp"
#include "SwitchingLogic.hpp"

class HorizonSubsystem {
public:
    HorizonSubsystem();
    
    // The single function your ADCS task calls
    HorizonOutput process_sensors();

private:
    static constexpr int NUM_SENSORS = 4;
    const float SENSOR_ROLL_OFFSETS[NUM_SENSORS] = {0.0f, 90.0f, 180.0f, 270.0f};

    HorizonDetector detector;
    HorizonSensorManager manager;
};