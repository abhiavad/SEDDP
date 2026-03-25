#pragma once
#include "HorizonTypes.hpp"

class HorizonDetector {
public:
    static constexpr int SENSOR_WIDTH = 32;
    static constexpr int SENSOR_HEIGHT = 24;
    static constexpr int PIXELS_PER_SENSOR = SENSOR_WIDTH * SENSOR_HEIGHT; // 768
    static constexpr int THRESHOLD_TEMP = 35;
    static constexpr int32_t SCALER = 4096;

    // Returns true if a valid horizon was found in the 100-degree local window
    bool process_frame(const float* frame_data, float& local_pitch, float& local_roll, float& out_area);

private:
    bool perform_banded_search(int32_t v_x_fixed, int32_t v_y_fixed, int32_t area_fixed, 
                               int32_t& out_pitch_fixed, int32_t& out_roll_fixed);

    // New helper to isolate the main Earth body
    int extract_largest_blob(const float* frame_data, bool* valid_mask);
};