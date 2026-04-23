/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include <stdio.h>
#include "../Inc/i2c.h"
#include "../Inc/MLX90640_I2C_Driver.h"

#define READ_BUFFER_SIZE 32


void MLX90640_I2CInit()
{
	MX_I2C1_Init();
}

// Original version from the previous SEP
int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t *p = (uint8_t *)data;
    int ack = 0;
    int cnt = 0;
    uint16_t bytesRemaining = nMemAddressRead * 2;  // EEPROM stores 16-bit words
    uint16_t offset = 0;

    while (bytesRemaining > 0)
    {
        uint16_t chunkSize = (bytesRemaining > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : bytesRemaining;

        // Read in chunks
        ack = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr<<1), startAddress + (offset / 2), I2C_MEMADD_SIZE_16BIT, &p[offset], chunkSize, 500);
        if (ack != HAL_OK)
        {
            printf("I2C Read Failed at offset %d, status: %d\n", offset, ack);
            return -1;
        }

        bytesRemaining -= chunkSize;
        offset += chunkSize;
    }

    // Swap bytes correctly (ensuring 16-bit words are correctly interpreted)
    for (cnt = 0; cnt < nMemAddressRead; cnt++)
    {
        data[cnt] = (data[cnt] << 8) | (data[cnt] >> 8);
    }

    return 0;
}



int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{

	uint8_t sa;
	int ack = 0;
	uint8_t cmd[2];
	static uint16_t dataCheck;

	sa = (slaveAddr << 1);

	cmd[0] = data >> 8;
	cmd[1] = data & 0xFF;


	ack = HAL_I2C_Mem_Write(&hi2c1, sa, writeAddress, I2C_MEMADD_SIZE_16BIT, cmd, sizeof(cmd), 500);

	if (ack != HAL_OK)
	{
			return -1;
	}

	MLX90640_I2CRead(slaveAddr,writeAddress,1, &dataCheck);

	if ( dataCheck != data)
	{
			return -2;
	}

	return 0;
}

