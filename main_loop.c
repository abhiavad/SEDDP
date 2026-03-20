

// this will be the main loop of the program, it will call the other functions and classes to run the program

#pragma once

#include "main_loop.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <cstdint>


// Assuming STM32 HAL is included somewhere in the project, but not including it here to avoid compilation issues
// #include "stm32l4xx_hal.h"

bool OBC_data_request() {
    // Placeholder for actual OBC data request check, e.g., checking an interrupt pin
    return false; // Return true if OBC has requested new data
}

IWDG_HandlyTypeDef hiwdg; // 
void MX_IWDG_Init(void){
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // 32KHz / 64 = 500Hz
    hiwdg.Init.Reload = 1000;         // 2 second timeout
    hiwdg.Init.Window = 4095;         
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        // Initialization Error
        // Include a system wide reset check if this is correct
        NVIC_SystemReset();
    }
}

int main() {

    HAL_Init();

    MX_IWDG_Init(); // Initialize the watchdog timer

    SystemData adcsState = {0};
    
    adcsState.currentMode = MODE_SAFE; // Set initial mode
    
    
    //1. Initialize all subsystems
    if (!horizonSubsystem.init_sensors(adcsState)) {
        std::cerr << "Failed to initialize horizon sensors!" << std::endl;
        return 1;
    }

    if (!magnetometerSubsystem.init(adcsState)) {
        std::cerr << "Failed to initialize magnetometer!" << std::endl;
        return 1;
    }

    if (!magnetorquerSubsystem.init(adcsState)) {
        std::cerr << "Failed to initialize magnetorquers!" << std::endl;
        return 1;
    }

    // --- Timing Configuration (in milliseconds) ---
    const uint32_t EHS_READ_INTERVAL = 125;
    const uint32_t MM_REQ_INTERVAL = 30;
    const uint32_t MTQ_CYCLE_TIME = 1000;
    const uint32_t MTQ_ON_DURATION = 100; // Duration to keep magnetorquers on (in ms)


    // --- State Variables ---
    uint32_t last_ehs_time = 0;
    uint32_t last_mm_req_time = 0;
    uint32_t last_mtq_start_time = 0;

    bool ehs_flag = false;
    bool mtq_active = false;

    // Check if we resumed from a Watchdog Reset
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
    adcsState.lastResetWatchdog = true;
    adcsState.errorCount++; 
    // Clear the reset flags so we don't misread it next time
    __HAL_RCC_CLEAR_RESET_FLAGS();
    } else {
    adcsState.lastResetWatchdog = false;
    }


    while (true) {
        // Start timer
        uint32_t currentTime = HAL_GetTick();
        
        // 1. Check if OBC requests new data via interrupt
        if (OBC_data_request()) {
            // process_OBC_command(); placeholder

            continue;
        }
        
        if (adcsState.currentMode == MODE_HIBERNATION) {
            HAL_Delay(1000); // Sleep for 1 second to save power, adjust as needed
            // Choose delay equal to the update frequency of the OBC
            // Basically syncs the frequency to that of the OBC
            continue;
        }

        if (adcsState.currentMode == MODE_SAFE) {

            //Do we also want the b-dot algorithm here?
            //We can run magnetometer and magnetorquers

        }

        // 2. Reading MM and checking MM DRDY pin
        
        if (currentTime - last_mm_req_time >= MM_REQ_INTERVAL) {
            // magnetometerSubsystem.send_request(); // Placeholder
            last_mm_req_time = currentTime;
        }

        if (magnetometerSubsystem.data_ready()) {
            magnetometerSubsystem.read_data(); // Placeholder
        }

        // 3. Reading EHS with flag as to not lag the loop 

        if (currentTime - last_ehs_time >= EHS_READ_INTERVAL) {
            ehs_flag = true;
            last_ehs_time = currentTime;
        }

        if (ehs_flag) {
            horizonSubsystem.process_sensors(); 
            ehs_flag = false;
        }

        Attitude result = //pulls information from the MM+EHS script that determines final attitude
        adcsState.pitch = result.pitch;
        adcsState.roll = result.roll;
        
        // 3. Fire magnetorquers as needed
        // Check if sufficient time has passed wrt magnetometer

        if ((currentTime - last_mtq_start_time >= MTQ_CYCLE_TIME) && !mtq_active) {
            magnetorquerSubsystem.activate(); // Placeholder
            mtq_active = true;
            last_mtq_start_time = currentTime;
        }

        if (mtq_active && (currentTime - last_mtq_start_time >= MTQ_ON_DURATION)) {
            magnetorquerSubsystem.deactivate(); // Placeholder
            mtq_active = false;
        }

        HAL_IWDG_Refresh(&hiwdg); // Refresh the watchdog timer to prevent reset

    }
    return 0;
}

