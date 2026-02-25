#ifndef EARTH_SENSOR_HPP
#define EARTH_SENSOR_HPP

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pi Guard: Ensures Pi is defined for our calculations */
#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

/* Struct matches your Python binary output exactly */
typedef struct __attribute__((packed)) {
    int32_t pitch;
    int32_t roll;
    int32_t vx;
    int32_t vy;
    int32_t area;
} FixedEntry;

typedef struct {
    float pitch;
    float roll;
    bool valid;
} AttitudeResult;

/* Function to be called from main.c */
AttitudeResult calculate_attitude(const float* mlx_data);

#ifdef __cplusplus
}
#endif

#endif