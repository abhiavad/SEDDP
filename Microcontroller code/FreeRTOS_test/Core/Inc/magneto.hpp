#ifndef __MAGNETO_H
#define __MAGNETO_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "math.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_uart.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Setup Variables
// Magic numbers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static const uint8_t MAGNETOMETER_MAX_BUFFER_SIZE = 4;
static const double timeStepSimilarThreshold = 0.01;

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

#define RM3100Address 0x20
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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Math
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename T>
T absVal(T x) {
    return (x < 0) ? -x : x;
}

template<typename T>
T maxVal(T x) {
    return x;
}

template<typename T, typename... Args>
T maxVal(T x, Args... args) {
    T maxArg = maxVal(args...);
    return (x > maxArg) ? x : maxArg;
}

template<typename T>
T minVal(T x) {
    return x;
}

template<typename T, typename... Args>
T minVal(T x, Args... args) {
    T minArg = minVal(args...);
    return (x < minArg) ? x : minArg;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Derivatives
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


double y_dot_2(const double y0, const double y1, const double dt) {
    return (y0 - y1) / dt;
}

double y_dot_3(const double y0, const double y1, const double y2, const double dt) {
    return (3*y0 - 4*y1 + y2) / (2*dt);
}

double y_dot_4(const double y0, const double y1, const double y2, const double y3, const double dt) {
    return (11*y0 - 18*y1 + 9*y2 - 2*y3) / (6*dt);
}

template<uint16_t SIZE>
double l2Norm(const double array[SIZE]) {
    double sum = 0;
    for (uint16_t i = 0; i < SIZE; ++i) {
        sum += array[i] * array[i];
    }
    return sqrt(sum);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void readRegs(I2C_HandleTypeDef &hi2c, uint16_t deviceAddress, uint16_t reg, uint16_t size, uint8_t *buffer) {
    HAL_StatusTypeDef hal_check;
    hal_check = HAL_I2C_Mem_Read(&hi2c, (deviceAddress << 1), reg, 1, buffer, size, HAL_MAX_DELAY);
    if (hal_check) {
        printf("Error while reading register %x: HAL_CHECK: %x\n", reg, hal_check);
    }
}

uint8_t readReg(I2C_HandleTypeDef &hi2c, uint16_t deviceAddress, uint16_t reg) {
    uint8_t result_buffer[1] = {0};
    readRegs(hi2c, deviceAddress, reg, 1, result_buffer);
    return result_buffer[0];
}

void writeReg(I2C_HandleTypeDef &hi2c, uint16_t deviceAddress, uint16_t reg, uint8_t value) {
    HAL_StatusTypeDef hal_check;
    uint8_t value_buffer[1] = {value};

    hal_check = HAL_I2C_Mem_Write(&hi2c, (deviceAddress << 1), reg, 1, value_buffer, 1, HAL_MAX_DELAY);
    if (hal_check) {
        printf("Error while reading register %x: HAL_CHECK: %x - (returning result 0)\n", reg, hal_check);
    }
}

void processRawMag(const uint8_t *rawBuffer, double *processedBuffer, const double modifiers[3]) {

    for (size_t i = 0; i < 3; ++i) {
        int32_t newResult = (rawBuffer[3 * i] << 16) + ((rawBuffer[3 * i + 1]) << 8) + rawBuffer[3 * i + 2];

        if (newResult & 0x800000) {
            newResult = -((newResult ^ 0xFFFFFF) + 1);
        }

        processedBuffer[i] = (newResult / modifiers[i]);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// State Machines
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class MagnetoState {
    SLEEP,
    CALIBRATE,
    MEASURE,
    BDOT,
    TEST
};

// UART Reception State Enum
enum class UartRxState { WAIT_FOR_START, WAIT_FOR_SIZE, WAIT_FOR_MESSAGE };



// TODO: Implement allowable transitions
class MagnetoStateMachine {
public:
    MagnetoStateMachine(MagnetoState currentState) : currentState_(currentState) {
    }

    ~MagnetoStateMachine() = default;

    void setState(MagnetoState state) {
        currentState_ = state;
    }

    MagnetoState getState() {
        return currentState_;
    }

private:
    MagnetoState currentState_;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Commands
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class Command : uint8_t {
    NONE,
    STATE_SLEEP,
    STATE_CALIBRATE,
    STATE_MEASURE,
    STATE_BDOT,
    STATE_TEST
};

// Command byte to Command enum, with default to NONE
static inline Command byteToCommand(uint8_t commandByte) {
    switch (commandByte) {
        case (0x00):
            return Command::NONE;
        case (0x01):
            return Command::STATE_SLEEP;
        case (0x02):
            return Command::STATE_CALIBRATE;
        case (0x03):
            return Command::STATE_MEASURE;
        case (0x04):
            return Command::STATE_BDOT;
        case (0x05):
            return Command::STATE_TEST;
        default:
            return Command::NONE;
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// UART Interface
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



template<uint16_t BUFFER_SIZE = 200>
class UartInterface {
public:
    explicit UartInterface(UART_HandleTypeDef &huart) : huart_(huart), logger_(nullptr), uartRxState(UartRxState::WAIT_FOR_START), expected_message_size_(0), header_received_(false) {}

    UartInterface(UART_HandleTypeDef &huart, UartInterface<BUFFER_SIZE> &logger) : huart_(huart), logger_(&logger), uartRxState(UartRxState::WAIT_FOR_START), expected_message_size_(0), header_received_(false) {}

    ~UartInterface() = default;

    void print(const std::string& messageText) {
        uint8_t size = messageText.size();
        HAL_UART_Transmit(&huart_, reinterpret_cast<const uint8_t*>(messageText.c_str()), size, HAL_MAX_DELAY);
    }

    void printInterrupt(const std::string& messageText) {
        uint8_t size = messageText.size();
        HAL_UART_Transmit_IT(&huart_, reinterpret_cast<const uint8_t*>(messageText.c_str()), size);
    }

    void printMessage(const std::string& messageText) {
    	printf(messageText.c_str());
        XbeeMessage<BUFFER_SIZE> message;
        message.generateMessage("%s", messageText.c_str());
        print(std::string(reinterpret_cast<char*>(message.messageBuffer_), message.completeMessageSize_));
    }

    HAL_StatusTypeDef readBuffer(uint8_t *buffer, uint8_t size) {
        // return HAL_UART_Receive(&huart_, buffer, size, 50);
        return HAL_UART_Receive_IT(&huart_, buffer, size);
    }

    void processReceivedData(UART_HandleTypeDef *huart, MagnetoStateMachine &stateMachine) {
        if (huart == &huart_) {
            // Check for Overrun Errors
            if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
                __HAL_UART_CLEAR_OREFLAG(huart);
            }

            switch (uartRxState) {
                case UartRxState::WAIT_FOR_START:
                    if (header_buffer_[0] == 0x7E) {
                        uartRxState = UartRxState::WAIT_FOR_SIZE;
                        HAL_UART_Receive_DMA(huart, header_buffer_ + 1, 2); // Receive the size bytes
                    } else {
                        HAL_UART_Receive_DMA(huart, header_buffer_, 1); // Wait for the start byte again
                    }
                    break;

                case UartRxState::WAIT_FOR_SIZE:
                    expected_message_size_ = (header_buffer_[1] << 8) | header_buffer_[2];
                    if (expected_message_size_ <= 125) {
                        uartRxState = UartRxState::WAIT_FOR_MESSAGE;
                        HAL_UART_Receive_DMA(huart, receive_buffer_, expected_message_size_ + 1); // Receive the actual message + checksum
                    } else {
                        uartRxState = UartRxState::WAIT_FOR_START;
                        HAL_UART_Receive_DMA(huart, header_buffer_, 1); // Size too large, wait for the start byte again
                    }
                    break;

                case UartRxState::WAIT_FOR_MESSAGE:
                    if (receive_message_.readMessage(receive_buffer_, expected_message_size_ + 1)) {
                        Command command = byteToCommand(receive_message_.dataBuffer_[0]);

                        switch (command) {
                            case Command::NONE:
                                break;
                            case Command::STATE_SLEEP:
                                stateMachine.setState(MagnetoState::SLEEP);
                                printMessage("# State: SLEEP\n");
                                if (logger_) logger_->print("# State: SLEEP\n");
                                break;
                            case Command::STATE_CALIBRATE:
                                stateMachine.setState(MagnetoState::CALIBRATE);
                                printMessage("# State: CALIBRATE\n");
                                if (logger_) logger_->print("# State: CALIBRATE\n");
                                break;
                            case Command::STATE_MEASURE:
                                stateMachine.setState(MagnetoState::MEASURE);
                                printMessage("# State: MEASURE\n");
                                if (logger_) logger_->print("# State: MEASURE\n");
                                break;
                            case Command::STATE_BDOT:
                                stateMachine.setState(MagnetoState::BDOT);
                                printMessage("# State: BDOT\n");
                                if (logger_) logger_->print("# State: BDOT\n");
                                break;
                            case Command::STATE_TEST:
                                stateMachine.setState(MagnetoState::TEST);
                                printMessage("# State: TEST\n");
                                if (logger_) logger_->print("# State: TEST\n");
                                break;
                            default:
                                break;
                        }
                    }
                    uartRxState = UartRxState::WAIT_FOR_START;
                    HAL_UART_Receive_DMA(huart, header_buffer_, 1); // Wait for the start byte again
                    break;
            }
        }
    }

    void startReception() {
        uartRxState = UartRxState::WAIT_FOR_START;
        HAL_UART_Receive_DMA(&huart_, header_buffer_, 1);
    }

    // Command getLastCommand() {
    //     return lastCommand_;
    // }

    // uint8_t *getLastCommandDetails() {
    //     return last_command_details_;
    // }

private:
    UART_HandleTypeDef &huart_;
    UartInterface<BUFFER_SIZE> *logger_;
    UartRxState uartRxState;
    uint8_t receive_buffer_[125];
    // Command last_command_;
    // uint8_t last_command_details_[8];
    char receive_message_[125];
    uint8_t header_buffer_[3]; // Buffer to store the initial 3-byte header
    uint16_t expected_message_size_;
    bool header_received_;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DataBuffer
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class DataBuffer {
public:
    DataBuffer() : bufferFront_(0), bufferRear_(0) {
    }

    ~DataBuffer() = default;

    bool bufferIsFull() const {
        return (getBufferSize() == MAGNETOMETER_MAX_BUFFER_SIZE - 1);
    }

    bool bufferIsEmpty() const {
        return (getBufferSize() == 0);
    }

    double dequeueOldest() {
        if (bufferIsEmpty()) {
            return 0;
        }

        bufferFront_ = (bufferFront_ + 1) % MAGNETOMETER_MAX_BUFFER_SIZE;

        double item = data[bufferFront_];
        return item;
    }

    void newData(double item) {
        if (bufferIsFull()) {
            (void) dequeueOldest();
        }

        bufferRear_ = (bufferRear_ + 1) % MAGNETOMETER_MAX_BUFFER_SIZE;
        data[bufferRear_] = item;
    }

    double peekBuffer(uint8_t index = 0) {
        if (bufferIsEmpty()) {
            // Queue is empty, handle error
        }
        index += (index < 0) ? bufferRear_ + 1 : bufferFront_;
        return data[index % MAGNETOMETER_MAX_BUFFER_SIZE];
    }

    uint8_t getBufferSize() const {
        return (uint8_t) (bufferRear_ - bufferFront_) % MAGNETOMETER_MAX_BUFFER_SIZE;
    }

    void clearBuffer() {
        bufferFront_ = 0;
        bufferRear_ = 0;
    }

private:
    double data[MAGNETOMETER_MAX_BUFFER_SIZE] = {0};
    uint8_t bufferFront_;
    uint8_t bufferRear_;

};

void printBuffer(DataBuffer &buffer, const char *name);

class Magnetometer3Axis {
public:
    explicit Magnetometer3Axis(I2C_HandleTypeDef &hi2c, GPIO_TypeDef *drdyPort, uint16_t drdyPin, uint16_t deviceAddress = RM3100Address)
        : hi2c_(hi2c), drdyPort_(drdyPort), drdyPin_(drdyPin), deviceAddress_(deviceAddress) {
    };

    ~Magnetometer3Axis() = default;

    bool checkDevice() {
        if (HAL_I2C_IsDeviceReady(&hi2c_, deviceAddress_ << 1, 1, HAL_MAX_DELAY)
            || HAL_I2C_IsDeviceReady(&hi2c_, (deviceAddress_ << 1) + 1, 1, HAL_MAX_DELAY)) {
            return false;
        }
        return true;
    }

    void readRegs(uint16_t reg, uint16_t size, uint8_t *buffer) {
        ::readRegs(hi2c_, deviceAddress_, reg, size, buffer);
    }

    uint8_t readReg(uint16_t reg) {
        return ::readReg(hi2c_, deviceAddress_, reg);
    }

    void writeReg(uint16_t reg, uint8_t value) {
        ::writeReg(hi2c_, deviceAddress_, reg, value);
    }

    void getMagField(double &bX, double &bY, double &bZ) {
        bX = bXBuffer_.peekBuffer(-1);
        bY = bYBuffer_.peekBuffer(-1);
        bZ = bZBuffer_.peekBuffer(-1);
    }

    void getMagFieldDerivative(double &bDotX, double &bDotY, double &bDotZ) {
        // Set some default values
        bDotX = 0;
        bDotY = 0;
        bDotZ = 0;

        if (!timeBuffer_.bufferIsFull()) {
            LOG("Buffer not full. Current size is %d", timeBuffer_.getBufferSize());
            printBuffer(timeBuffer_, "Time");
            printBuffer(bXBuffer_, "X");
            return;
        }

        // Calculate the last time step
        double timeStep = timeBuffer_.peekBuffer(-1) - timeBuffer_.peekBuffer(-2);

        if (timeStep <= 0.0) {
            LOG("Time step is <= 0. Time step is %f", timeStep);
            return;
        }

        // If the last 2 time steps are similar, use 3-point derivative, otherwise use 2-point derivative
        if (lastTimeStepsSimilar_()) {
            bDotX = y_dot_3(bXBuffer_.peekBuffer(-1), bXBuffer_.peekBuffer(-2), bXBuffer_.peekBuffer(-3), timeStep);
            bDotY = y_dot_3(bYBuffer_.peekBuffer(-1), bYBuffer_.peekBuffer(-2), bYBuffer_.peekBuffer(-3), timeStep);
            bDotZ = y_dot_3(bZBuffer_.peekBuffer(-1), bZBuffer_.peekBuffer(-2), bZBuffer_.peekBuffer(-3), timeStep);
        }
        else {
            bDotX = y_dot_2(bXBuffer_.peekBuffer(-1), bXBuffer_.peekBuffer(-2), timeStep);
            bDotY = y_dot_2(bYBuffer_.peekBuffer(-1), bYBuffer_.peekBuffer(-2), timeStep);
            bDotZ = y_dot_2(bZBuffer_.peekBuffer(-1), bZBuffer_.peekBuffer(-2), timeStep);
        }
    }

    void setMagField(double time, double bX, double bY, double bZ) {
        timeBuffer_.newData(time);
        bXBuffer_.newData(bX);
        bYBuffer_.newData(bY);
        bZBuffer_.newData(bZ);
    }

    void startContinuousMeasurement() {
        // Start continuous measurement
        writeReg(RM3100_CMM_REG, 0b01111001);

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

    void handleInterrupt() {
        if (HAL_GPIO_ReadPin(drdyPort_, drdyPin_) == GPIO_PIN_SET) {
            // Clear the interrupt flag
            // __HAL_GPIO_EXTI_CLEAR_IT(drdyPin_);

            // Set a flag to indicate that data is ready
            dataReady_ = true;
        }
    }

    void update(double time) {
        if (dataReady_) {
            // Reset the flag
            dataReady_ = false;

            // Read and process the data
            uint8_t magRaw[9];
            double magProcessed[3];
            readRegs(RM3100_MX_REG, 9, magRaw);
            processRawMag(magRaw, magProcessed, gains_);
            setMagField(time, magProcessed[0], magProcessed[1], magProcessed[2]);
        }
    }

    void pollAndReadMagField(double time) {
        uint8_t magRaw[9];
        double magProcessed[3];

        // Poll sensor
        writeReg(RM3100_POLL_REG, 0b01110000);

        // Wait for DRDY to pull high
        while (HAL_GPIO_ReadPin(drdyPort_, drdyPin_) == GPIO_PIN_RESET) {
            // Optionally, add a delay here to prevent the loop from spinning too fast
            // HAL_Delay(1);
        }

        // Read sensor
        readRegs(RM3100_MX_REG, 9, magRaw);

        // Convert to magnetic field and store
        processRawMag(magRaw, magProcessed, gains_);
        setMagField(time, magProcessed[0], magProcessed[1], magProcessed[2]);
    }

    void updateGains() {
        cycleCountX_ = (readReg(RM3100_CCX1_REG) << 8) | readReg(RM3100_CCX0_REG);
        cycleCountY_ = (readReg(RM3100_CCY1_REG) << 8) | readReg(RM3100_CCY0_REG);
        cycleCountZ_ = (readReg(RM3100_CCZ1_REG) << 8) | readReg(RM3100_CCZ0_REG);

        gains_[0] = (0.3671 * (float) cycleCountX_) + 1.5;
        gains_[1] = (0.3671 * (float) cycleCountY_) + 1.5;
        gains_[2] = (0.3671 * (float) cycleCountZ_) + 1.5;

    }

    void clearBuffers() {
        timeBuffer_.clearBuffer();
        bXBuffer_.clearBuffer();
        bYBuffer_.clearBuffer();
        bZBuffer_.clearBuffer();
    }

    DataBuffer &getTimeBuffer() { return timeBuffer_; }

    DataBuffer &getBXBuffer() { return bXBuffer_; }

    DataBuffer &getBYBuffer() { return bYBuffer_; }

    DataBuffer &getBZBuffer() { return bZBuffer_; }

    double getGainX() { return gains_[0]; }

    double getGainY() { return gains_[1]; }

    double getGainZ() { return gains_[2]; }

private:
    bool lastTimeStepsSimilar_(double threshold = 0.01) {
        double dt1 = timeBuffer_.peekBuffer(-1) - timeBuffer_.peekBuffer(-2);
        double dt2 = timeBuffer_.peekBuffer(-2) - timeBuffer_.peekBuffer(-3);

        return (absVal(dt1 - dt2) / maxVal(dt1, dt2) < threshold);
    }

private:
    DataBuffer timeBuffer_;
    DataBuffer bXBuffer_;
    DataBuffer bYBuffer_;
    DataBuffer bZBuffer_;

    I2C_HandleTypeDef &hi2c_;
    GPIO_TypeDef *drdyPort_;
    uint16_t drdyPin_;
    uint16_t deviceAddress_;

    bool dataReady_ = false;

    uint8_t cycleCountX_ = 0;
    uint8_t cycleCountY_ = 0;
    uint8_t cycleCountZ_ = 0;

    double gains_[3] = {1.0, 1.0, 1.0};
};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnetorquer
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum class MagnetorquerDriveState {
    COAST = 0,
    POSITIVE = 1,
    NEGATIVE = 2,
    BRAKE = 3,
};

class Magnetorquer {
public:
    Magnetorquer(uint32_t numWindings, double effectiveArea, double voltage, double resistance,
                 GPIO_TypeDef *nSleepPort, uint16_t nSleepPin,
                 GPIO_TypeDef *phasePort, uint16_t phasePin,
                 GPIO_TypeDef *enablePort, uint16_t enablePin)
            : numWindings_(numWindings), effectiveArea_(effectiveArea), voltage_(voltage), resistance_(resistance),
              nSleepPort_(nSleepPort), nSleepPin_(nSleepPin),
              phasePort_(phasePort), phasePin_(phasePin),
              enablePort_(enablePort), enablePin_(enablePin),

              driveState_(MagnetorquerDriveState::COAST) {
        updateActualCurrent_();
        setReqDipoleMoment(0.0);
    }

    ~Magnetorquer() = default;

    void setReqDipoleMoment(double dipoleMoment) {
        requiredDipoleMoment_ = dipoleMoment;
        requiredCurrent_ = getRequiredCurrent_(dipoleMoment);
        // reqDutyCycle_ = requiredCurrent_ / actualCurrent_; // Note: this can be > 1 and it should be (scaled before use).
        // issue with line above: can result in negative duty cycle
        reqDutyCycle_ = abs(requiredCurrent_ / actualCurrent_); // Note: this can be > 1 and it should be (scaled before use).+
    }

    double getDutyCycle() const {
        return reqDutyCycle_;
    }

    double getDipoleMoment() const {
        return requiredDipoleMoment_;
    }

    MagnetorquerDriveState getDriveState() {
        return driveState_;
    }

    void setDriveState(MagnetorquerDriveState driveState) {
        driveState_ = driveState;
        updateHardware();
    }

    void switchOnForward() {
        setDriveState(MagnetorquerDriveState::POSITIVE);
    }

    void switchOnReverse() {
        setDriveState(MagnetorquerDriveState::NEGATIVE);
    }

    void switchOffCoast() {
        setDriveState(MagnetorquerDriveState::COAST);
    }

    void switchOffBrake() {
        setDriveState(MagnetorquerDriveState::BRAKE);
    }

    void relax() {
        requiredDipoleMoment_ = 0.0;
        driveState_ = MagnetorquerDriveState::COAST;
        updateHardware();
    }

    void updateHardware() {
        if (driveState_ == MagnetorquerDriveState::POSITIVE) {
            HAL_GPIO_WritePin(nSleepPort_, nSleepPin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(phasePort_, phasePin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_SET);
        }
        else if (driveState_ == MagnetorquerDriveState::NEGATIVE) {
            HAL_GPIO_WritePin(nSleepPort_, nSleepPin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(phasePort_, phasePin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_SET);
        }
        else if (driveState_ == MagnetorquerDriveState::COAST) {
            HAL_GPIO_WritePin(nSleepPort_, nSleepPin_, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_RESET);
        }
        else if (driveState_ == MagnetorquerDriveState::BRAKE) {
            HAL_GPIO_WritePin(nSleepPort_, nSleepPin_, GPIO_PIN_SET);
            HAL_GPIO_WritePin(enablePort_, enablePin_, GPIO_PIN_RESET);
        }
    }

private:

    void updateActualCurrent_() {
        actualCurrent_ = voltage_ / resistance_;
    }

    double getRequiredCurrent_(double dipoleMoment) const {
        return dipoleMoment / (numWindings_ * effectiveArea_);
    }

private:
    uint32_t numWindings_;
    double effectiveArea_;
    double voltage_;
    double resistance_;
    double actualCurrent_ = 0.0;

    double requiredDipoleMoment_ = 0.0;
    double requiredCurrent_ = 0.0;
    double reqDutyCycle_ = 0.0;

    GPIO_TypeDef *nSleepPort_;
    uint16_t nSleepPin_;
    GPIO_TypeDef *phasePort_;
    uint16_t phasePin_;
    GPIO_TypeDef *enablePort_;
    uint16_t enablePin_;

    MagnetorquerDriveState driveState_ = MagnetorquerDriveState::COAST;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Controllers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MagnetorquerDriveState decideDriveState(double dipoleMoment, double dutyCycle, double dutyCycleFractionCompleted) {
    if (dutyCycleFractionCompleted >= dutyCycle) {
        return MagnetorquerDriveState::COAST;
    }

    if (dipoleMoment > 0) {
        return MagnetorquerDriveState::POSITIVE;
    }
    else {
        return MagnetorquerDriveState::NEGATIVE;
    }
}

class MagControl {

public:
    MagControl(Magnetorquer &magnetorquerX,
               Magnetorquer &magnetorquerY,
               Magnetorquer &magnetorquerZ,
               Magnetometer3Axis &magnetometerXYZ,
               uint32_t countsPerCycle = 10000)
            : magnetorquerX_(magnetorquerX),
              magnetorquerY_(magnetorquerY),
              magnetorquerZ_(magnetorquerZ),
              magnetometerXYZ_(magnetometerXYZ),
              countsPerCycle_(countsPerCycle) {
        relaxMagnetorquers();
    }

    ~MagControl() = default;

    void relaxMagnetorquers() {
        magnetorquerX_.relax();
        magnetorquerY_.relax();
        magnetorquerZ_.relax();
    }

    void setTorquersDipoleMoment(double mX, double mY, double mZ) {
        magnetorquerX_.setReqDipoleMoment(mX);
        magnetorquerY_.setReqDipoleMoment(mY);
        magnetorquerZ_.setReqDipoleMoment(mZ);
        updateLocalTorquerSettings_();
    }

    virtual void calcDipoles() {};

    //! Method to make sure that the vector direction is correct (no duty cycles > 1
    void scaleDutyCycles() {
        double maxDutyCycle = maxVal(dutyCycleX_, dutyCycleY_, dutyCycleZ_);

        if (maxDutyCycle > 1.0) {
            dutyCycleX_ /= maxDutyCycle;
            dutyCycleY_ /= maxDutyCycle;
            dutyCycleZ_ /= maxDutyCycle;
        }
    }

    void stepDutyCycle(uint32_t loopCount) {
        currentLoopCount_ = loopCount;
        uint32_t dutyCycleCount = currentLoopCount_ % countsPerCycle_;
        // if (dutyCycleCount == 0) {
        //     calcDipoles();
        // }

        double dutyCycleFraction = static_cast<double>(dutyCycleCount) / static_cast<double> (countsPerCycle_);
        magnetorquerX_.setDriveState(decideDriveState(dipoleMomentX_, dutyCycleX_, dutyCycleFraction));
        magnetorquerY_.setDriveState(decideDriveState(dipoleMomentY_, dutyCycleY_, dutyCycleFraction));
        magnetorquerZ_.setDriveState(decideDriveState(dipoleMomentZ_, dutyCycleZ_, dutyCycleFraction));
    }

    uint32_t getCountsPerDutyCycle() const {
        return countsPerCycle_;
    }

    uint32_t getCurrentLoopCount() const {
        return currentLoopCount_;
    }

    // Return the an array of the duty cycles
    void getDutyCycles(double *dutyCycles) {
        dutyCycles[0] = dutyCycleX_;
        dutyCycles[1] = dutyCycleY_;
        dutyCycles[2] = dutyCycleZ_;
    }


private:
    void updateLocalTorquerSettings_() {
        dipoleMomentX_ = magnetorquerX_.getDipoleMoment();
        dipoleMomentY_ = magnetorquerY_.getDipoleMoment();
        dipoleMomentZ_ = magnetorquerZ_.getDipoleMoment();

        dutyCycleX_ = magnetorquerX_.getDutyCycle();
        dutyCycleY_ = magnetorquerY_.getDutyCycle();
        dutyCycleZ_ = magnetorquerZ_.getDutyCycle();
        scaleDutyCycles();
    }

protected:
    Magnetorquer &magnetorquerX_;
    Magnetorquer &magnetorquerY_;
    Magnetorquer &magnetorquerZ_;
    double dipoleMomentX_ = 0.0;
    double dipoleMomentY_ = 0.0;
    double dipoleMomentZ_ = 0.0;
    Magnetometer3Axis &magnetometerXYZ_;
    uint32_t currentLoopCount_ = 0;
    uint32_t countsPerCycle_;

private:
    double dutyCycleX_ = 0.0;
    double dutyCycleY_ = 0.0;
    double dutyCycleZ_ = 0.0;

};

class BDotControl : public MagControl {
public:
    BDotControl(Magnetorquer &magnetorquerX,
                Magnetorquer &magnetorquerY,
                Magnetorquer &magnetorquerZ,
                Magnetometer3Axis &magnetometerXYZ,
                uint32_t countsPerCycle = 10000)
            : MagControl(magnetorquerX, magnetorquerY, magnetorquerZ, magnetometerXYZ, countsPerCycle) {
    }

    ~BDotControl() = default;

    void calcDipoles() override {
        magnetometerXYZ_.getMagFieldDerivative(bDotX_, bDotY_, bDotZ_);

        dipoleMomentX_ = -mGain_ * bDotX_ - mXBias_;
        dipoleMomentY_ = -mGain_ * bDotY_ - mYBias_;
        dipoleMomentZ_ = -mGain_ * bDotZ_ - mZBias_;

        setTorquersDipoleMoment(dipoleMomentX_, dipoleMomentY_, dipoleMomentZ_);
    }

    void setDipoleGain(double gain) {
        mGain_ = gain;
    }

    void setDipoleBias(double biasX, double biasY, double biasZ) {
        mXBias_ = biasX;
        mYBias_ = biasY;
        mZBias_ = biasZ;
    }

public:
    double bDotX_ = 0.0;
    double bDotY_ = 0.0;
    double bDotZ_ = 0.0;

    double mGain_ = 0.05;
    double mXBias_ = 0.0;
    double mYBias_ = 0.0;
    double mZBias_ = 0.0;
};

class SineControl : public MagControl {
public:
    SineControl(Magnetorquer &magnetorquerX,
                Magnetorquer &magnetorquerY,
                Magnetorquer &magnetorquerZ,
                Magnetometer3Axis &magnetometerXYZ)
            : MagControl(magnetorquerX, magnetorquerY, magnetorquerZ, magnetometerXYZ) {
    }

    void calcDipoles() override {
        double currentPeriodFraction =
                static_cast<double>(currentLoopCount_ % stepsPerPeriod_) / static_cast<double> (stepsPerPeriod_);

        dipoleMomentX_ = sin(2 * M_PI * currentPeriodFraction);
        dipoleMomentY_ = -sin(2 * M_PI * currentPeriodFraction) / 2;
        dipoleMomentZ_ = -sin(2 * M_PI * currentPeriodFraction) / 4;

        setTorquersDipoleMoment(dipoleMomentX_, dipoleMomentY_, dipoleMomentZ_);
    }

private:
    uint32_t stepsPerPeriod_ = 1000;

};


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Utility Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void printline() {
    printf("--------------------------------------------------\n");
}

void printBuffer(DataBuffer &buffer, const char *name) {
    LLOG("         Buffer %s (%d) : [", name, buffer.getBufferSize());
    for (uint8_t i = 0; i < buffer.getBufferSize(); i++) {
        LLOG("%d: %f, ", i, buffer.peekBuffer(i));
    }
    LLOG("]\n");
}

void printMagnetometer(Magnetometer3Axis &mag, const char *name) {
    double bXDot, bYDot, bZDot;
    mag.getMagFieldDerivative(bXDot, bYDot, bZDot);
    NLOG("Magnetometer %s:", name);
    printBuffer(mag.getTimeBuffer(), "Time");
    printBuffer(mag.getBXBuffer(), "BX");
    printBuffer(mag.getBYBuffer(), "BY");
    printBuffer(mag.getBZBuffer(), "BZ");
    ALOG("Bdot: [%f, %f, %f]", bXDot, bYDot, bZDot);
}

void printMagnetorquer(Magnetorquer &mag, const char *name) {
    NLOG("Magnetorquer %s:", name);
    ALOG("Dipole Moment: %E", mag.getDipoleMoment());
    ALOG("Duty Cycle: %E", mag.getDutyCycle());
    ALOG("Drive State: %d", (int) mag.getDriveState());
    ALOG("Actual Current: %E", mag.getDutyCycle());
    ALOG("Required Current: %E", mag.getDutyCycle());
}




#endif // __MAGNETO_H
