
#pragma once
#include <stdint.h>

// Adding __attribute__((packed)) ensures the compiler doesn't add padding bytes
struct __attribute__((packed)) LookupEntry {
    int32_t table_roll;   // 4 bytes
    int32_t table_pitch;  // 4 bytes
    int16_t v_x;          // 2 bytes
    int16_t v_y;          // 2 bytes
    int16_t area;         // 2 bytes
    int16_t atan2_val;    // 2 bytes
};


// The final output of the manager
struct HorizonOutput {
    float pitch;
    float roll;
    int active_sensor_id;
    int confidence;
    bool is_valid;
};