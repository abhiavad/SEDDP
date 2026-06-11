#ifndef CHANGING_ADDRESS_H
#define CHANGING_ADDRESS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function declaration
// Returns 1 on success, 0 on failure.
uint8_t Program_MLX90642_Address(uint8_t current_address, uint8_t new_address);

#ifdef __cplusplus
}
#endif

#endif 