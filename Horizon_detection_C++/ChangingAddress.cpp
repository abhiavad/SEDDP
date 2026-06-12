#include "ChangingAddress.h"
#include "../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal.h"
#include <stdio.h> 

// Ensure you include your updated driver here
#include "../Drivers/MLX90640/Inc/MLX90642_I2C_Driver.h" 

// Returns 1 on success, 0 on failure.
uint8_t Program_MLX90642_Address(uint8_t current_address, uint8_t new_address) {
    uint16_t eeprom_val;
    uint16_t verify_val;

    // 1. Read the current configuration at 0x11FE (I2C configuration address) [cite: 1844, 1909]
    MLX90642_I2CRead(current_address, 0x11FE, 1, &eeprom_val);

    // Print the starting state
    printf("Current address is 0x%02X\n", current_address);

    // 2. Isolate the upper reserved bits and insert the new 7-bit address [cite: 1909, 2000]
    uint16_t new_eeprom_val = (eeprom_val & 0xFF80) | (new_address & 0x007F);

    // 3. Write the new address configuration to 0x11FE
    // NOTE: The MLX90642's internal MCU automatically handles the EEPROM write sequence.
    // IMPORTANT WARNING: For FW 1.16.5 and above, changing settings requires the 
    // dedicated Configuration Command (opcode 0x3A2E). 
    // Your underlying MLX90642_I2CWrite() MUST be structured to send this specific opcode 
    // rather than a standard memory write.
    MLX90642_I2CWrite(current_address, 0x11FE, new_eeprom_val);
    
    // 4. Wait for the burn to complete
    // The write sequence can take up to 500ms depending on the refresh rate[cite: 1907].
    HAL_Delay(500); 

    // 5. SAFETY CHECK: Verify the data was written correctly
    // The new slave address is effective immediately after a successful write.
    // Therefore, you must read from the NEW address to verify it!
    int status = MLX90642_I2CRead(new_address, 0x11FE, 1, &verify_val);

    // Assuming your driver returns 0 upon a successful read operation
    if (status != 0 || verify_val != new_eeprom_val) {
        printf("Write failed or sensor not responding at 0x%02X.\n", new_address);
        return 0;
    }

    // Print the final successful state
    printf("Successfully changed to Address 0x%02X\n", new_address);

    // Success! No power cycle is required for the MLX90642.
    return 1;
}