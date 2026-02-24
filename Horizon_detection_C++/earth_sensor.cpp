#include "Earth_sensor.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

// SHIFT and SCALER remain for binary interpretation
const int SHIFT = 12;
const float SCALER = static_cast<float>(1 << SHIFT);

std::vector<RealEntry> loadFixedTable(std::string filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open " << filename << std::endl;
        return {};
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    size_t num_rows = size / sizeof(FixedEntry);
    std::vector<RealEntry> converted_table;
    converted_table.reserve(num_rows);

    FixedEntry buffer;
    while (file.read(reinterpret_cast<char*>(&buffer), sizeof(FixedEntry))) {
        RealEntry entry;
        // Divide by float SCALER to convert fixed-point back to float
        entry.pitch = static_cast<float>(buffer.pitch) / SCALER;
        entry.roll  = static_cast<float>(buffer.roll)  / SCALER;
        entry.vx    = static_cast<float>(buffer.vx)    / SCALER;
        entry.vy    = static_cast<float>(buffer.vy)    / SCALER;
        entry.area  = static_cast<float>(buffer.area)  / SCALER;
        converted_table.push_back(entry);
    }
    return converted_table;
}

void findClosestMatch(const std::vector<RealEntry>& table, float target_vx, float target_vy, float target_area) {
    float min_dist_sq = 1e10f; // Sufficiently large float starting point
    RealEntry best_match = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    for (const auto& row : table) {
        float dvx = row.vx - target_vx;
        float dvy = row.vy - target_vy;
        float darea = row.area - target_area;
        
        // Fast hardware-accelerated float math
        float dist_sq = (dvx * dvx) + (dvy * dvy) + (darea * darea);

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            best_match = row;
        }
    }
    std::cout << "Best Match Found -> Pitch: " << best_match.pitch << ", Roll: " << best_match.roll << std::endl;
    std::cout << "Match Confidence (Distance Squared): " << min_dist_sq << std::endl;
}

std::pair<float, float> calculate_vector(const float data[24][32], const float X_angles[24][32], const float Y_angles[24][32], float origin_x, float origin_y) {
    int count = 0;
    float sum_x = 0.0f, sum_y = 0.0f;

    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 32; ++j) {
            if (data[i][j] > 35.0f) { 
                count++;
                sum_x += (X_angles[i][j] - origin_x);
                sum_y += (Y_angles[i][j] - origin_y);
            }
        }
    }

    // Standard Satellite FOV constraints
    if (count < 20 || count > 748) return {NAN, NAN};

    float magnitude = std::sqrt(sum_x * sum_x + sum_y * sum_y);
    return {sum_x / magnitude, sum_y / magnitude};
}

float integrate_angles(const float data[24][32], const float X_angles[24][32], const float Y_angles[24][32]) {
    float total_area = 0.0f;

    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 32; ++j) {
            if (data[i][j] > 35.0f) { 
                float dx, dy;

                // X Gradient (Horizontal)
                if (j == 0) dx = X_angles[i][j + 1] - X_angles[i][j];
                else if (j == 31) dx = X_angles[i][j] - X_angles[i][j - 1];
                else dx = (X_angles[i][j + 1] - X_angles[i][j - 1]) / 2.0f;

                // Y Gradient (Vertical)
                if (i == 0) dy = Y_angles[i + 1][j] - Y_angles[i][j];
                else if (i == 23) dy = Y_angles[i][j] - Y_angles[i - 1][j];
                else dy = (Y_angles[i + 1][j] - Y_angles[i - 1][j]) / 2.0f;

                total_area += std::abs(dx * dy);
            }
        }
    }
    return total_area;
}