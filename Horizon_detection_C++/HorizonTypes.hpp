
#pragma once
#include <stdint.h>

// Matches the 6-column Python .bin file structure
struct LookupEntry {
    int32_t table_roll;  // Col 0: True Roll (Scaled by 4096)
    int32_t table_pitch; // Col 1: True Pitch (Scaled by 4096)
    int32_t v_x;         // Col 2: (Scaled by 4096)
    int32_t v_y;         // Col 3: (Scaled by 4096)
    int32_t area;        // Col 4: (Scaled by 4096)
    int32_t atan2_val;   // Col 5: Roll estimation for search (Scaled by 4096)
};

// The final output of the manager
struct HorizonOutput {
    float pitch;
    float roll;
    int active_sensor_id;
    int confidence;
    bool is_valid;
};