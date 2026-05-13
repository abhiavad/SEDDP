#include "HorizonBridge.h"
#include "CombiningSensor.hpp"

// Statically instantiate your C++ class here
static HorizonSubsystem system_manager;

extern "C" bool Bridge_Init_Sensors(void) {
    return system_manager.init_sensors();
}

extern "C" C_HorizonOutput Bridge_Process_Sensors(void) {
    HorizonOutput cpp_out = system_manager.process_sensors();

    C_HorizonOutput c_out;
    c_out.pitch = cpp_out.pitch;
    c_out.roll = cpp_out.roll;
    c_out.active_sensor_id = cpp_out.active_sensor_id;
    c_out.confidence = cpp_out.confidence;
    c_out.is_valid = cpp_out.is_valid;

    return c_out;
}

extern "C" void Bridge_Get_Debug_Images(int sensor_idx, float** thermal_ptr, uint8_t** mask_ptr) {
    // Point the C pointers directly to the C++ arrays
    *thermal_ptr = system_manager.debug_thermal[sensor_idx];
    *mask_ptr = system_manager.debug_mask[sensor_idx];
}
