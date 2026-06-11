#include "MLX90640_I2C_Driver.h"
#include "stm32f4xx_hal.h"

// Returns 1 on success, 0 on failure. 
// DO NOT UNPLUG POWER IF THIS RETURNS 0!
uint8_t Program_MLX90640_Address(uint8_t current_address, uint8_t new_address) {
    uint16_t eeprom_val;
    uint16_t verify_val;
    
    // 1. Read the current configuration at 0x240F
    MLX90640_I2CRead(current_address, 0x240F, 1, &eeprom_val);
    
    // 2. Isolate the upper bits and insert the new 7-bit address
    uint16_t new_eeprom_val = (eeprom_val & 0xFF80) | new_address;
    
    // 3. THE CRITICAL STEP: Erase the cell first to reset the ECC
    MLX90640_I2CWrite(current_address, 0x240F, 0x0000);
    HAL_Delay(10); // Wait 10ms for the EEPROM to physically burn the zeros
    
    // 4. Write the new address configuration
    MLX90640_I2CWrite(current_address, 0x240F, new_eeprom_val);
    HAL_Delay(10); // Wait 10ms for the burn to complete
    
    // 5. SAFETY CHECK: Verify the data was written correctly
    MLX90640_I2CRead(current_address, 0x240F, 1, &verify_val);
    
    if (verify_val != new_eeprom_val) {
        // THE WRITE FAILED. DO NOT POWER CYCLE!
        // If verify_val is 0x0000 and you power cycle, the sensor bricks.
        // In a real application, you might want a retry loop here.
        return 0; 
    }
    
    // Success! Now unplug the power to the sensor, and plug it back in.
    return 1; 
}
