/*
 * Moteino_Slave_I2C.c
 *
 *  Created on: 7 May 2026
 *      Author: Maurizio Pavel (inspired by Arun Rawat's work in https://controllerstech.com/stm32-as-i2c-slave-part-6/)
 */

#include "Moteino_Slave_I2C.h"

uint8_t RxData[RxSIZE];
uint8_t txCount = 0;
uint8_t rxCount = 0;
uint8_t startPosition = 0;

// this array right here is the memory addresses that we will allow the I2C network to access.
// memory mapping of this array will be done later, but it will contain such things as the operational mode, sensor readings, and anything else that might be transmitted
uint8_t I2C_REGISTERS[I2C_REGISTER_SIZE];

// this function was used in the beginning to ease communication between main.c and this file, might be revived later
void GetMoteinoRXData(uint8_t toBeChanged[]) {
	for(uint8_t i=0;i<RxSIZE;i++)
		toBeChanged[i] = RxData[i];
}

void GetMoteinoTXData(uint8_t toBeChanged[]) {
	for(uint8_t i=0;i<I2C_REGISTER_SIZE;i++)
		toBeChanged[i] = I2C_REGISTERS[i];
}

void SetMoteinoTXData(uint8_t changer[]) {
	for(uint8_t i=0;i<I2C_REGISTER_SIZE;i++)
		I2C_REGISTERS[i] = changer[i];
}

void PrepareFloatForI2CSending(float* var, uint8_t* position, uint8_t i2c[]) {
	// a float is 4 bytes long, so it needs 4 locations within i2c[]
	floatOrBytes_t var_changed;
	var_changed.floatValue = *var;
	i2c[*position    ] = var_changed.bin[0];
	i2c[*position + 1] = var_changed.bin[1];
	i2c[*position + 2] = var_changed.bin[2];
	i2c[*position + 3] = var_changed.bin[3];
}

// this basically makes sure that even after we're done with one i2c communication, the STM32 goes back to listening
extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

// this function is called once the STM32's slave address is called in the i2c channel
extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
	{
		RxData[0] = 0; // reset Rx buffer
		rxCount = 0; // reset Rx counter (for buffer reasons)
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxCount, 1, I2C_FIRST_AND_LAST_FRAME); // receive, not 100% sure how this works
	}
	else  // master requesting data
	{
		txCount = 0; // reset Tx counter for buffer reasons
		startPosition = 0; // previously, the master sent where it wants data from. RxData[0] is the location where it wants data from
		RxData[0] = 0; // reset this because it's already copied
		HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txCount, 1, I2C_FIRST_FRAME); // again, not 100% sure how this works exactly, but it does
	}
}

// this function is called after the slave Tx is complete
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	txCount++;
	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, I2C_REGISTERS+startPosition+txCount, 1, I2C_NEXT_FRAME);
}

// this function is called after the slave Rx is complete. it basically just keeps the Rx going.
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	rxCount++;
	if (rxCount < RxSIZE)
	{
		if (rxCount == RxSIZE-1)
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxCount, 1, I2C_LAST_FRAME);
		}
		else
		{
			HAL_I2C_Slave_Seq_Receive_IT(hi2c, RxData+rxCount, 1, I2C_NEXT_FRAME);
		}
	}

	if (rxCount == RxSIZE);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	uint32_t errorcode = HAL_I2C_GetError(hi2c);
	if (errorcode == 4)  // AF error
	{
		// communication is complete (probably)
	}
	HAL_I2C_EnableListen_IT(hi2c);
}
