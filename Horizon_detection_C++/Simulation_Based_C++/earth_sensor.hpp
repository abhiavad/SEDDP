#ifndef EARTH_SENSOR_HPP
#define EARTH_SENSOR_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <utility> // for std::pair

// Pack for exact binary alignment
struct __attribute__((packed)) FixedEntry {
    int32_t pitch, roll, vx, vy, area;
};

struct RealEntry {
    float pitch, roll, vx, vy, area;
};

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

// Logic Prototypes - Use float for STM32 hardware acceleration
std::vector<RealEntry> loadFixedTable(std::string filename);
void findClosestMatch(const std::vector<RealEntry>& table, float target_vx, float target_vy, float target_area);

std::pair<float, float> calculate_vector(const float data[24][32], const float X_angles[24][32], const float Y_angles[24][32], float origin_x, float origin_y);
float integrate_angles(const float data[24][32], const float X_angles[24][32], const float Y_angles[24][32]);


#endif
