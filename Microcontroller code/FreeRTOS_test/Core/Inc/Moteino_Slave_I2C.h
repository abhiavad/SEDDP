/*
 * Moteino_Slave_I2C.h
 *
 *  Created on: 7 May 2026
 *      Author: Maur
 */

#ifndef INC_MOTEINO_SLAVE_I2C_H_
#define INC_MOTEINO_SLAVE_I2C_H_

#include "stm32l4xx_hal.h"

#define RxSIZE 6 // for testing

// memory mapping (remember to space them by their size!!!!)
#define I2C_BX_POS 0
#define I2C_BY_POS 4
#define I2C_BZ_POS 8
#define I2C_ROLL_POS 12
#define I2C_PITCH_POS 16
#define I2C_TIMESTAMP_POS 20
// other things to send to moteino go here....
#define I2C_REGISTER_SIZE 24 // should be enough to fit everything above

void GetMoteinoRXData(uint8_t toBeChanged[]);
void GetMoteinoTXData(uint8_t toBeChanged[]);
void SetMoteinoTXData(uint8_t changer[]);
void PrepareFloatForI2CSending(float* var, uint8_t* position, uint8_t i2c[]);

// Using this to make transmitting floats over I2C possible
// Source - https://stackoverflow.com/a/57003083
// Posted by Stephan Lechner, modified by community. See post 'Timeline' for change history
// Retrieved 2026-05-08, License - CC BY-SA 4.0
typedef union {
    float floatValue;
    uint8_t bin[sizeof(float)];
} floatOrBytes_t;

#endif /* INC_MOTEINO_SLAVE_I2C_H_ */
