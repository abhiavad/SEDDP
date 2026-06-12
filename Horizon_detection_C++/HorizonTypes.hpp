
#pragma once
#include <stdint.h>

// Adding __attribute__((packed)) ensures the compiler doesn't add padding bytes
struct LookupEntry {
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
    float ind_pitch[4];    // remove comment when DEBUGGING; this is a DEBUG variable
    float ind_roll[4];     // remove comment when DEBUGGING; this is a DEBUG variable
    int confirm_counts[4]; // remove comment when DEBUGGING; this is a DEBUG variable
    int grace_counts[4];   // remove comment when DEBUGGING; this is a DEBUG variable
};
