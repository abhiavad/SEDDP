/*
 * magneto.h
 *
 * VERY IMPORTANT: This file was basically translated from a previous SEP group. They wrote their driver in C++,
 * but i just can't get this project to work in C++. So i resorted to translating their stuff into C.
 * It's been a major pain and I'm not even sure that it works fine. If you find a way to just run everything in C++,
 * that will probably be better than using this.
 *
 *  Created on: 29 Apr 2026
 *      Author: Maur
 */

#ifndef INC_RM3100_H_
#define INC_RM3100_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "math.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_uart.h"
#include <stdarg.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup Variables
// Magic numbers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAGNETOMETER_MAX_BUFFER_SIZE 4
static const float timeStepSimilarThreshold = 0.01;

#define DEBUG_LOG

#ifdef DEBUG_LOG
#define LOG(...) printf("[DEBUG]: "); printf(__VA_ARGS__); printf("\n")
#define NLOG(...) printf("\n[DEBUG]: "); printf(__VA_ARGS__); printf("\n")
#define ALOG(...) printf("         "); printf(__VA_ARGS__); printf("\n")
#define LLOG(...) printf(__VA_ARGS__)
#else
#define LOG(...)
#define NLOG(...)
#define ALOG(...)
#define LLOG(...)
#endif

#define RM3100Address 0x21
#define RM3100_REVID_REG 0x36
//#define RM3100_REVID_REG 0xB6

#define RM3100_POLL_REG 0x00
#define RM3100_CMM_REG 0x01
#define RM3100_STATUS_REG 0x34
#define RM3100_CCX1_REG 0x04
#define RM3100_CCX0_REG 0x05
#define RM3100_CCY1_REG 0x06
#define RM3100_CCY0_REG 0x07
#define RM3100_CCZ1_REG 0x08
#define RM3100_CCZ0_REG 0x09
#define RM3100_MX_REG 0x24

#define absVal(x) (x<0) ? -x : x

extern I2C_HandleTypeDef hi2c1;

#define I2C_HANDLE &hi2c1

// functions labeled as "derivatives" from last year's group, but I'm not sure if we're going to use them or not.

float y_dot_2(const float y0, const float y1, const float dt) {
    return (y0 - y1) / dt;
}

float y_dot_3(const float y0, const float y1, const float y2, const float dt) {
    return (3*y0 - 4*y1 + y2) / (2*dt);
}

float y_dot_4(const float y0, const float y1, const float y2, const float y3, const float dt) {
    return (11*y0 - 18*y1 + 9*y2 - 2*y3) / (6*dt);
}


// no documentation was provided on any of the below functions.
// Function to read the register "reg". You also need to the size and you also need to give a buffer
//#define READ_BUFFER_SIZE 32
//int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead, uint8_t *data)
//{
//    uint8_t *p = (uint8_t *)data;
//    int ack = 0;
//    int cnt = 0;
//    uint16_t bytesRemaining = nMemAddressRead * 2;  // EEPROM stores 16-bit words
//    uint16_t offset = 0;
//
//    while (bytesRemaining > 0)
//    {
//        uint16_t chunkSize = (bytesRemaining > READ_BUFFER_SIZE) ? READ_BUFFER_SIZE : bytesRemaining;
//
//        // Read in chunks
//        ack = HAL_I2C_Mem_Read(&hi2c1, (slaveAddr<<1), startAddress + (offset / 2), I2C_MEMADD_SIZE_16BIT, &p[offset], chunkSize, 500);
//        if (ack != HAL_OK)
//        {
//            printf("I2C Read Failed at offset %d, status: %d\n", offset, ack);
//            return -1;
//        }
//
//        bytesRemaining -= chunkSize;
//        offset += chunkSize;
//    }
//
//    // Swap bytes correctly (ensuring 16-bit words are correctly interpreted)
//    for (cnt = 0; cnt < nMemAddressRead; cnt++)
//    {
//        data[cnt] = (data[cnt] << 8) | (data[cnt] >> 8);
//    }
//
//    return 0;
//}

void readRegs(uint16_t deviceAddress, uint16_t reg, uint16_t size, uint8_t *buffer) {
    HAL_StatusTypeDef hal_check;
    hal_check = HAL_I2C_Mem_Read(I2C_HANDLE, (deviceAddress << 1), reg, 1, buffer, size, HAL_MAX_DELAY);
    if (hal_check) {
        printf("Error while reading register %x: HAL_CHECK: %x\n", reg, hal_check);
    }
}

uint8_t readReg(uint16_t deviceAddress, uint16_t reg) {
    uint8_t result_buffer[1] = {0};
    readRegs(deviceAddress, reg, 1, result_buffer);
    return result_buffer[0];
}

void writeReg(uint16_t deviceAddress, uint16_t reg, uint8_t value) {
    HAL_StatusTypeDef hal_check;
    uint8_t value_buffer[1] = {value};

    hal_check = HAL_I2C_Mem_Write(I2C_HANDLE, (deviceAddress << 1), reg, 1, value_buffer, 1, HAL_MAX_DELAY);
    if (hal_check) {
        printf("Error while writing register %x: HAL_CHECK: %x - (returning result 0)\n", reg, hal_check);
    }
}
void processRawMag(const uint8_t *rawBuffer, float *processedBuffer, const float modifiers[3]) {
    for (size_t i = 0; i < 3; ++i) {
        // 1. Combine the 3 bytes into a signed 32-bit integer
        // Shift bits: [MSB] [MID] [LSB]
        int32_t val = (int32_t)((rawBuffer[3 * i] << 16) |
                                (rawBuffer[3 * i + 1] << 8) |
                                 rawBuffer[3 * i + 2]);

        // 2. Handle the 24-bit sign extension
        // If the 24th bit (0x800000) is set, it's a negative number
        if (val & 0x800000) {
            val |= 0xFF000000; // Fill the top 8 bits with 1s to make it a negative 32-bit int
        }

        // 3. Convert to float and apply gain
        // Note: Make sure modifiers[i] is NOT zero!
        if (modifiers[i] != 0) {
            processedBuffer[i] = (float)val / modifiers[i];
        } else {
            processedBuffer[i] = (float)val; // Fallback
        }
    }
}
//void processRawMag(const uint8_t *rawBuffer, float *processedBuffer, const float modifiers[3]) {
//
//    for (size_t i = 0; i < 3; ++i) {
//        int32_t newResult = (rawBuffer[3 * i] << 16) + ((rawBuffer[3 * i + 1]) << 8) + rawBuffer[3 * i + 2];
//
//        if (newResult & 0x800000) {
//            newResult = -((newResult ^ 0xFFFFFF) + 1);
//        }
//
//        processedBuffer[i] = (newResult / modifiers[i]);
//    }
//}


// Data Buffer class translation. Make sure to set bufferData to {0} when calling it

struct dataBuffer {
    float bufferData[MAGNETOMETER_MAX_BUFFER_SIZE]; // used to be = {0} but C doesn't like that
    uint8_t bufferFront_;
    uint8_t bufferRear_;
};

// turns the struct into a datatype (god, C is a pain)
typedef struct dataBuffer DataBuffer;

uint8_t getBufferSize(DataBuffer* buffer) {
    return (uint8_t) (buffer->bufferRear_ - buffer->bufferFront_) % MAGNETOMETER_MAX_BUFFER_SIZE;
}

bool bufferIsFull(DataBuffer* buffer) {
	return (getBufferSize(buffer) == MAGNETOMETER_MAX_BUFFER_SIZE -1);
}

bool bufferIsEmpty(DataBuffer* buffer) {
	return (getBufferSize(buffer) == 0);
}

float dequeueOldest(DataBuffer* buffer) {
    if (bufferIsEmpty(buffer)) {
        return 0;
    }

    buffer->bufferFront_ = (buffer->bufferFront_ + 1) % MAGNETOMETER_MAX_BUFFER_SIZE;

    float item = buffer->bufferData[buffer->bufferFront_];
    return item;
}


void Buffer_newData(DataBuffer* buffer, float item) {
    if (bufferIsFull(buffer)) {
        (void) dequeueOldest(buffer);
    }
    buffer->bufferRear_ = (buffer->bufferRear_ + 1) % MAGNETOMETER_MAX_BUFFER_SIZE;
    buffer->bufferData[buffer->bufferRear_] = item;
    // i tested this function and it works fine
}

// i suppose this function just looks at what the last item in the buffer is. Just be sure to set index=0 if you don't care
// because C doesn't accept default arguments
float peekBuffer(DataBuffer* buffer, int8_t index) {
//	printf("\npeekBuffer is called\n");
    if (bufferIsEmpty(buffer)) {
        // Queue is empty, handle error
    	printf("\n----Warning: peekBuffer() called but buffer is empty\n");
    }
    index += (index < 0) ? buffer->bufferRear_ + 1 : buffer->bufferFront_;
//    printf("   The thing to be peeked is at index %d, which is %f", index, buffer->bufferData[index % MAGNETOMETER_MAX_BUFFER_SIZE]);
    return buffer->bufferData[index % MAGNETOMETER_MAX_BUFFER_SIZE];
}

void clearBuffer(DataBuffer buffer) {
    buffer.bufferFront_ = 0;
    buffer.bufferRear_ = 0;
}

// use this instead of directly creating a new data buffer. it contains the original constructor instructions
DataBuffer NewDataBuffer() {
	DataBuffer buffer;
	buffer.bufferFront_ = 0;
	buffer.bufferRear_ = 0;
	return buffer;
}


void printBuffer(DataBuffer *buffer, char *name) {
    LLOG("         Buffer %s (%d) : [", name, getBufferSize(buffer));
    for (uint8_t i = 0; i < getBufferSize(buffer); i++) {
        LLOG("%d: %f, ", i, peekBuffer(buffer, i));
    }
    LLOG("]\n");
}

// Translation of Magnetometer3Axis

struct magnetometer3Axis {
	DataBuffer timeBuffer_;
	DataBuffer bXBuffer_;
	DataBuffer bYBuffer_;
	DataBuffer bZBuffer_;

	GPIO_TypeDef *drdyPort_;
	uint16_t drdyPin_;
	uint16_t deviceAddress_;

	bool dataReady_; // default value is false. Again, C doesn't like it

	uint8_t cycleCountX_; // default value is 0
	uint8_t cycleCountY_; // default value is 0
	uint8_t cycleCountZ_; // default value is 0

	float gains_[3]; // default value is {1.0, 1.0, 1.0}
};
typedef struct magnetometer3Axis Magnetometer3Axis;

// use this function right after the creation of a new m3a object
void NewMagnetometer3Axis(Magnetometer3Axis* mm, GPIO_TypeDef* drdyPort, uint16_t drdyPin, uint16_t deviceAddress) {
	mm->timeBuffer_ = NewDataBuffer();
	mm->bXBuffer_ = NewDataBuffer();
	mm->bYBuffer_ = NewDataBuffer();
	mm->bZBuffer_ = NewDataBuffer();
	mm->drdyPort_ = drdyPort;
	mm->drdyPin_ = drdyPin;
	mm->deviceAddress_ = deviceAddress;
	mm->dataReady_ = false;
	mm->cycleCountX_ = 0;
	mm->cycleCountY_ = 0;
	mm->cycleCountZ_ = 0;
	mm->gains_[0] = 1.0;
	mm->gains_[1] = 1.0;
	mm->gains_[2] = 1.0;
}

bool RM3100_CheckDevice(Magnetometer3Axis* mm) {
    if (HAL_I2C_IsDeviceReady(I2C_HANDLE, (mm->deviceAddress_) << 1, 1, HAL_MAX_DELAY)
        || HAL_I2C_IsDeviceReady(I2C_HANDLE, ((mm->deviceAddress_) << 1) + 1, 1, HAL_MAX_DELAY)) {
        return false;
    }
    return true;
}

void RM3100_readRegs(Magnetometer3Axis* mm, uint16_t reg, uint16_t size, uint8_t *buffer) {
    readRegs(mm->deviceAddress_, reg, size, buffer);
}

uint8_t RM3100_readReg(Magnetometer3Axis* mm, uint16_t reg) {
    return readReg(mm->deviceAddress_, reg);
}

void RM3100_writeReg(Magnetometer3Axis* mm, uint16_t reg, uint8_t value) {
    writeReg(mm->deviceAddress_, reg, value);
}


void RM3100_getMagField(Magnetometer3Axis* mm, float* bX, float* bY, float* bZ) {
    *bX = peekBuffer(&(mm->bXBuffer_), -1);
    *bY = peekBuffer(&(mm->bYBuffer_), -1);
    *bZ = peekBuffer(&(mm->bZBuffer_), -1);
}

//  default threshold is 0.01, no idea how they got to that number.
bool RM3100_lastTimeStepsSimilar_(Magnetometer3Axis* mm, float threshold) {
    float dt1 = peekBuffer(&(mm->timeBuffer_), -1) - peekBuffer(&(mm->timeBuffer_), -2);
    float dt2 = peekBuffer(&(mm->timeBuffer_), -2) - peekBuffer(&(mm->timeBuffer_), -3);

    return (absVal(dt1 - dt2) / ((dt1 > dt2) ? dt1 : dt2) < threshold);
}

void RM3100_getMagFieldDerivative(Magnetometer3Axis* mm, float* bDotX, float* bDotY, float* bDotZ) {
	// Set some default values
    *bDotX = 0;
    *bDotY = 0;
    *bDotZ = 0;

	if (!bufferIsFull(&(mm->timeBuffer_))) {
		LOG("Buffer not full. Current size is %d", getBufferSize(&(mm->timeBuffer_)));
		printBuffer(&(mm->timeBuffer_), "Time");
		printBuffer(&(mm->bXBuffer_), "X");
		return;
	}

	// Calculate the last time step
	float timeStep = peekBuffer(&(mm->timeBuffer_), -1) - peekBuffer(&(mm->timeBuffer_), -2);

	if (timeStep <= 0.0) {
		LOG("Time step is <= 0. Time step is %f", timeStep);
		return;
	}

	// If the last 2 time steps are similar, use 3-point derivative, otherwise use 2-point derivative
	if (RM3100_lastTimeStepsSimilar_(mm, 0.01)) {
		*bDotX = y_dot_3(peekBuffer(&(mm->bXBuffer_), -1), peekBuffer(&(mm->bXBuffer_), -2), peekBuffer(&(mm->bXBuffer_), -3), timeStep);
		*bDotY = y_dot_3(peekBuffer(&(mm->bYBuffer_), -1), peekBuffer(&(mm->bYBuffer_), -2), peekBuffer(&(mm->bYBuffer_), -3), timeStep);
		*bDotZ = y_dot_3(peekBuffer(&(mm->bZBuffer_), -1), peekBuffer(&(mm->bZBuffer_), -2), peekBuffer(&(mm->bZBuffer_), -3), timeStep);
	}
	else {
		*bDotX = y_dot_2(peekBuffer(&(mm->bXBuffer_), -1), peekBuffer(&(mm->bXBuffer_), -2), timeStep);
		*bDotY = y_dot_2(peekBuffer(&(mm->bYBuffer_), -1), peekBuffer(&(mm->bYBuffer_), -2), timeStep);
		*bDotZ = y_dot_2(peekBuffer(&(mm->bZBuffer_), -1), peekBuffer(&(mm->bZBuffer_), -2), timeStep);
	}
}

void RM3100_setMagField(Magnetometer3Axis* mm, float time, float bX, float bY, float bZ) {
    Buffer_newData(&(mm->timeBuffer_), time);
    Buffer_newData(&(mm->bXBuffer_), bX);
    Buffer_newData(&(mm->bYBuffer_), bY);
    Buffer_newData(&(mm->bZBuffer_), bZ);
}

void RM3100_startContinuousMeasurement(Magnetometer3Axis* mm) {
    // Start continuous measurement
    RM3100_writeReg(mm, RM3100_CMM_REG, 0b01111001); // address is from the datasheet

    // Configure the DRDY pin as an input with an interrupt
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // GPIO_InitStruct.Pin = drdyPin_;
    // GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    // GPIO_InitStruct.Pull = GPIO_NOPULL;
    // HAL_GPIO_Init(drdyPort_, &GPIO_InitStruct);

    // Enable the interrupt
    // HAL_NVIC_SetPriority(EXTI9_10_IRQn, 2, 0);
    // HAL_NVIC_EnableIRQ(EXTI9_10_IRQn);
}

void RM3100_handleInterrupt(Magnetometer3Axis* mm) {
    if (HAL_GPIO_ReadPin(mm->drdyPort_, mm->drdyPin_) == GPIO_PIN_SET) {
        // Clear the interrupt flag
        // __HAL_GPIO_EXTI_CLEAR_IT(drdyPin_);

        // Set a flag to indicate that data is ready
        mm->dataReady_ = true;
    }
}

void update(Magnetometer3Axis* mm, float time) {
    if (mm->dataReady_) {
        // Reset the flag
        mm->dataReady_ = false;

        // Read and process the data
        uint8_t magRaw[9];
        float magProcessed[3];
        RM3100_readRegs(mm, RM3100_MX_REG, 9, magRaw);
        processRawMag(magRaw, magProcessed, mm->gains_);
        RM3100_setMagField(mm, time, magProcessed[0], magProcessed[1], magProcessed[2]);
    }
}

void RM3100_pollAndReadMagField(Magnetometer3Axis* mm, float time) {
    uint8_t magRaw[9];
    float magProcessed[3];

    // Poll sensor
    RM3100_writeReg(mm, RM3100_POLL_REG, 0b01110000); // address from the datasheet

    // Wait for DRDY to pull high
    while (HAL_GPIO_ReadPin(mm->drdyPort_, mm->drdyPin_) == GPIO_PIN_RESET) {
        // Optionally, add a delay here to prevent the loop from spinning too fast
         osDelay(1);
         // printf("Waiting for DRDY pin....\n");
    }
    // Read sensor
    RM3100_readRegs(mm, RM3100_MX_REG, 9, magRaw);
    // Convert to magnetic field and store
    processRawMag(magRaw, magProcessed, mm->gains_);
    RM3100_setMagField(mm, time, magProcessed[0], magProcessed[1], magProcessed[2]);
}

void RM3100_updateGains(Magnetometer3Axis* mm) {
	uint8_t buffer[6];
	readRegs(mm->deviceAddress_, RM3100_CCX1_REG, 6, buffer);
	// Implemented burst reading instead of making 6 I2C calls
//    mm->cycleCountX_ = (RM3100_readReg(mm, RM3100_CCX1_REG) << 8) | RM3100_readReg(mm, RM3100_CCX0_REG);
//    mm->cycleCountY_ = (RM3100_readReg(mm, RM3100_CCY1_REG) << 8) | RM3100_readReg(mm, RM3100_CCY0_REG);
//    mm->cycleCountZ_ = (RM3100_readReg(mm, RM3100_CCZ1_REG) << 8) | RM3100_readReg(mm, RM3100_CCZ0_REG);

	mm->cycleCountX_ = buffer[0] << 8 | buffer[1];
	mm->cycleCountY_ = buffer[2] << 8 | buffer[3];
	mm->cycleCountZ_ = buffer[4] << 8 | buffer[5];

    mm->gains_[0] = (0.3671 * (float) mm->cycleCountX_) + 1.5;
    mm->gains_[1] = (0.3671 * (float) mm->cycleCountY_) + 1.5;
    mm->gains_[2] = (0.3671 * (float) mm->cycleCountZ_) + 1.5;

}

void RM3100_clearBuffers(Magnetometer3Axis* mm) {
    clearBuffer(mm->timeBuffer_);
    clearBuffer(mm->bXBuffer_);
    clearBuffer(mm->bYBuffer_);
    clearBuffer(mm->bZBuffer_);
}

void printMagnetometer(Magnetometer3Axis* mm, char *name) {
    float bXDot, bYDot, bZDot;
    RM3100_getMagFieldDerivative(mm, &bXDot, &bYDot, &bZDot);
    NLOG("Magnetometer %s:", name);
    printBuffer(&(mm->timeBuffer_), "Time");
    printBuffer(&(mm->bXBuffer_), "BX");
    printBuffer(&(mm->bYBuffer_), "BY");
    printBuffer(&(mm->bZBuffer_), "BZ");
    ALOG("Bdot: [%f, %f, %f]", bXDot, bYDot, bZDot);
}



#endif /* INC_RM3100_H_ */
