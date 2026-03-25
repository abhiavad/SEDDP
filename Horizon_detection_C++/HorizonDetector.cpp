#include "HorizonDetector.hpp"
#include <math.h>

extern const int32_t X_ANGLES_FIXED[768];
extern const int32_t Y_ANGLES_FIXED[768];
extern const int32_t PIXEL_AREA_FIXED[768];
extern const LookupEntry CONVERSION_TABLE[];
extern const int TABLE_SIZE;

// --- Isolate the largest coherent thermal blob ---
int HorizonDetector::extract_largest_blob(const float* frame_data, bool* valid_mask) {
    bool visited[PIXELS_PER_SENSOR] = {false};
    int queue[PIXELS_PER_SENSOR];
    int current_blob[PIXELS_PER_SENSOR];
    
    int max_blob_size = 0;
    int best_blob_indices[PIXELS_PER_SENSOR];

    for (int i = 0; i < PIXELS_PER_SENSOR; i++) {
        valid_mask[i] = false; // Initialize output mask
        
        // If we find an unvisited hot pixel, start a flood fill
        if (!visited[i] && frame_data[i] > THRESHOLD_TEMP) {
            int head = 0, tail = 0;
            int blob_size = 0;

            queue[tail++] = i;
            visited[i] = true;

            while (head < tail) {
                int p = queue[head++];
                current_blob[blob_size++] = p;

                int x = p % SENSOR_WIDTH;
                int y = p / SENSOR_WIDTH;

                // Check 4-way neighbors (Left, Right, Up, Down)
                int neighbors[4] = {-1, -1, -1, -1};
                if (x > 0) neighbors[0] = p - 1;                     // Left
                if (x < SENSOR_WIDTH - 1) neighbors[1] = p + 1;      // Right
                if (y > 0) neighbors[2] = p - SENSOR_WIDTH;          // Up
                if (y < SENSOR_HEIGHT - 1) neighbors[3] = p + SENSOR_WIDTH; // Down

                for (int n = 0; n < 4; n++) {
                    int idx = neighbors[n];
                    if (idx != -1 && !visited[idx] && frame_data[idx] > THRESHOLD_TEMP) {
                        visited[idx] = true;
                        queue[tail++] = idx;
                    }
                }
            }

            // Track if this is the largest object we've seen in the frame
            if (blob_size > max_blob_size) {
                max_blob_size = blob_size;
                for (int j = 0; j < blob_size; j++) {
                    best_blob_indices[j] = current_blob[j];
                }
            }
        }
    }

    // Populate the output mask using only the largest blob
    for (int j = 0; j < max_blob_size; j++) {
        valid_mask[best_blob_indices[j]] = true;
    }

    return max_blob_size;
}


bool HorizonDetector::process_frame(const float* frame_data, float& local_pitch, float& local_roll, float& out_area) {
    int32_t sum_x_fixed = 0, sum_y_fixed = 0, total_area_fixed = 0;
    
    // 1. Get a mask of ONLY the main Earth blob (ignores scattered noise/stars)
    bool is_earth_pixel[PIXELS_PER_SENSOR];
    int valid_pixels = extract_largest_blob(frame_data, is_earth_pixel);

    // 2. Gatekeeping: Ensure the blob is the right size
    if (valid_pixels < 20 || valid_pixels > 748) return false;

    // Output the area of the detected blob for weighting sensors
    out_area = (float)valid_pixels;

    // 3. Accumulate vectors using ONLY the masked pixels
    for (int i = 0; i < PIXELS_PER_SENSOR; i++) {
        if (is_earth_pixel[i]) {
            sum_x_fixed += X_ANGLES_FIXED[i];
            sum_y_fixed += Y_ANGLES_FIXED[i];
            total_area_fixed += PIXEL_AREA_FIXED[i];
        }
    }

    // Normalize magnitude
    float sum_x_f = (float)sum_x_fixed / SCALER;
    float sum_y_f = (float)sum_y_fixed / SCALER;
    float mag = sqrtf(sum_x_f * sum_x_f + sum_y_f * sum_y_f);
    if (mag == 0.0f) return false;

    int32_t v_x_fixed = (int32_t)((sum_x_f / mag) * SCALER);
    int32_t v_y_fixed = (int32_t)((sum_y_f / mag) * SCALER);

    int32_t pitch_fixed, roll_fixed;
    if (perform_banded_search(v_x_fixed, v_y_fixed, total_area_fixed, pitch_fixed, roll_fixed)) {
        local_pitch = (float)pitch_fixed / SCALER;
        local_roll = (float)roll_fixed / SCALER;
        return true;
    }
    return false;
}

bool HorizonDetector::perform_banded_search(int32_t v_x_fixed, int32_t v_y_fixed, int32_t area_fixed, 
                                            int32_t& out_pitch_fixed, int32_t& out_roll_fixed) {
    float approx_roll = atan2f((float)v_y_fixed, (float)v_x_fixed);
    int32_t approx_roll_fixed = (int32_t)(approx_roll * SCALER);
    
    int32_t tolerance = (int32_t)(0.15f * SCALER);
    int32_t pi_fixed = (int32_t)(M_PI * SCALER);
    
    int32_t lower_bound = approx_roll_fixed - tolerance;
    int32_t upper_bound = approx_roll_fixed + tolerance;
    int32_t lower_wrap = 999999, upper_wrap = -999999;

    if (upper_bound > pi_fixed) {
        lower_wrap = -pi_fixed;
        upper_wrap = upper_bound - 2 * pi_fixed;
    } else if (lower_bound < -pi_fixed) {
        lower_wrap = lower_bound + 2 * pi_fixed;
        upper_wrap = pi_fixed;
    }

    int64_t min_dist_sq = -1;
    int best_idx = -1;

    for (int i = 0; i < TABLE_SIZE; i++) {
        int32_t t_roll = CONVERSION_TABLE[i].atan2_val;
        if ((t_roll >= lower_bound && t_roll <= upper_bound) || 
            (t_roll >= lower_wrap && t_roll <= upper_wrap)) {
            
            int64_t dx = (int64_t)CONVERSION_TABLE[i].v_x - v_x_fixed;
            int64_t dy = (int64_t)CONVERSION_TABLE[i].v_y - v_y_fixed;
            int64_t da = (int64_t)CONVERSION_TABLE[i].area - area_fixed;
            int64_t dist_sq = (dx*dx) + (dy*dy) + (da*da);
            
            if (min_dist_sq == -1 || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_idx = i;
            }
        }
    }

    // Reject poor matches and Pinhole Phantoms (Pitch > 90 deg)
    if (best_idx == -1 || min_dist_sq > 167772160LL) return false;
    if (CONVERSION_TABLE[best_idx].table_pitch > 368640) return false; // 90 * 4096

    out_roll_fixed = CONVERSION_TABLE[best_idx].table_roll;
    out_pitch_fixed = CONVERSION_TABLE[best_idx].table_pitch;
    return true;
}