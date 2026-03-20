
#pragma once
#include <math.h>

// include earth horizon sensors system
#include "CombiningSensor.hpp"


// include magnetometer system
//#include "Magnetometer.hpp"


// include magnetorquer system
// #include "Magnetorquer.hpp"

// Define the Operating Modes
enum ADCSMode {
    MODE_SAFE = 0,
    MODE_HIBERNATION = 1,
    MODE_NOMINAL = 2
};


// Define an index for component health monitoring
enum ComponentIdx {
    EHS_0 = 0, EHS_1, EHS_2, EHS_3, //4 IR sensors
    MAG_0,                          //1 magnetometer
    MTQ_X, MTQ_Y, MTQ_Z,            //3 magnetorquers
    NUM_COMPONENTS
};

struct SystemData {
    float batteryVoltage;
    float batteryCurrent;
    bool componentConnections[NUM_COMPONENTS];
    float updateRate; // in Hz (80MHz is max)
    uint32_t errorCount; // increments on any error, used to check if components fail

    uint32_t pitch;
    uint32_t roll;
    uint32_t yaw;

    ADCSmode currentMode;
    bool lastResetWatchdog;
    // Add more fields as needed, lot defined in the subsystem files already
};
