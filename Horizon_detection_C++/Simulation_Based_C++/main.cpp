#include "Earth_sensor.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

// Helper to load the 24x32 text files into float arrays
bool loadFOVFile(const std::string& filename, float target[24][32]) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    for (int i = 0; i < 24; ++i) {
        for (int j = 0; j < 32; ++j) {
            file >> target[i][j];
        }
    }
    return true;
}

// Loads raw sensor data. 
// Note: If Python saved as float64, we read as double and cast.
// If Python saved as float32, we read directly into the float array.
bool loadSensorBin(const std::string& filename, float target[24][32]) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) return false;
    
    // Assuming Python still exports as float64 (8 bytes) for now:
    double temp_buffer[24][32];
    file.read(reinterpret_cast<char*>(temp_buffer), 24 * 32 * sizeof(double));
    
    if (file.gcount() != (24 * 32 * sizeof(double))) return false;

    // Cast double to float for STM32-style processing
    for(int i=0; i<24; ++i) {
        for(int j=0; j<32; ++j) {
            target[i][j] = static_cast<float>(temp_buffer[i][j]);
        }
    }
    return true;
}

int main() {
    // 1. Data structures (now using float for FPU efficiency)
    float X_angles[24][32];
    float Y_angles[24][32];
    float sensor_data[24][32]; 

    // 2. Load FOV files
    if (!loadFOVFile("FOV_Files/horizontal_angles.txt", X_angles) ||
        !loadFOVFile("FOV_Files/vertical_angles.txt", Y_angles)) {
        std::cerr << "Error loading FOV text files!" << std::endl;
        return 1;
    }
    
    // 2b. Convert to Radians (Essential for Area matching)
    for(int i=0; i<24; ++i) {
        for(int j=0; j<32; ++j) {
            X_angles[i][j] *= (M_PI / 180.0f);
            Y_angles[i][j] *= (M_PI / 180.0f);
        }
    }

    // 3. Load conversion table
    std::vector<RealEntry> table = loadFixedTable("conversion_table_pow2.bin");
    if (table.empty()) return 1;

    // 4. Setup Origin (Row 11, Col 15)
    float origin_x = X_angles[11][15];
    float origin_y = Y_angles[11][15];

    // 5. Load real test data
    if (!loadSensorBin("test_frame.bin", sensor_data)) {
        std::cerr << "Error: Could not load test_frame.bin!" << std::endl;
        return 1;
    }
    std::cout << "Loaded test frame into float buffer." << std::endl;

    // 6. CALCULATE (Using float logic)
    std::pair<float, float> vec = calculate_vector(sensor_data, X_angles, Y_angles, origin_x, origin_y);
    float area = integrate_angles(sensor_data, X_angles, Y_angles);

    // 7. SEARCH
    if (std::isnan(vec.first)) {
        std::cout << "Earth not detected correctly (Too small or too large)." << std::endl;
    } else {
        std::cout << "Calculated Vector: (" << vec.first << ", " << vec.second << ")" << std::endl;
        std::cout << "Calculated Area: " << area << std::endl;
        findClosestMatch(table, vec.first, vec.second, area);
    }

    return 0;
}
