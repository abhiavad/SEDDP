#ifndef HORIZON_DETECTOR_H
#define HORIZON_DETECTOR_H

#include <stdbool.h> /* Required for 'bool' in C */

#ifdef __cplusplus
extern "C" {
#endif

/* * This is the only function main.c is allowed to see.
 * Notice we use pointers (*) instead of C++ references (&)
 */
bool HorizonDetector_Process(const float* frame_data, float* pitch, float* roll, float* area, uint8_t* mask);

#ifdef __cplusplus
}
#endif

#endif /* HORIZON_DETECTOR_H */
