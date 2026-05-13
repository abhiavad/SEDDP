#ifndef HORIZON_BRIDGE_H
#define HORIZON_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// A C-friendly version of your HorizonOutput struct
typedef struct {
    float pitch;
    float roll;
    int active_sensor_id;
    int confidence;
    bool is_valid;
} C_HorizonOutput;

// Bridge functions
bool Bridge_Init_Sensors(void);
C_HorizonOutput Bridge_Process_Sensors(void);
void Bridge_Get_Debug_Images(int sensor_idx, float** thermal_ptr, uint8_t** mask_ptr);

#ifdef __cplusplus
}
#endif

#endif // HORIZON_BRIDGE_H
