#pragma once

#include "main_loop.h"
#include <math.h>
#include <iostream>
#include <vector>
#include <cstdint>

// Assuming STM32 HAL is included
// #include "stm32l4xx_hal.h"

// --- Global Watchdog Handle ---
IWDG_HandleTypeDef hiwdg;

// --- FDIR Constants ---
const uint8_t MAX_CONSECUTIVE_ERRORS = 3; 

// --- Watchdog Initialization ---
void MX_IWDG_Init(void) {
    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // 32kHz / 64 = 500Hz as watchdog runs on seperate LSI clock
    hiwdg.Init.Reload = 1000;                 // 1000 / 500Hz = 2.0 seconds timeout
    hiwdg.Init.Window = 4095;                 // Disable window feature for now

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        // Initialization Error - Ideally log this or force a safe state
    }
}

// --- OBC Data Request Placeholder ---
bool OBC_data_request() {
    return false; // Return true if OBC has requested new data
}

int main() {
    // 1. MCU System Initialization
    HAL_Init();
    // SystemClock_Config(); // Standard STM32 clock configuration (80 MHz)
    
    // 2. Initialize Watchdog IMMEDIATELY
    MX_IWDG_Init();

    SystemData adcsState = {0};
    adcsState.currentMode = MODE_SAFE; // Set initial mode
    
    // 3. Initialize FDIR Component Tracking BEFORE subsystem inits
    for (int i = 0; i < NUM_COMPONENTS; i++) {
        adcsState.componentActive[i] = true;
        adcsState.consecutiveErrorCount[i] = 0;
    }

    // 4. Check if we just recovered from a Watchdog Reset
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
        // Log the error in your state
        adcsState.watchdogCounter++; 
        // Clear the flag so we don't misread it on the next normal reboot
        __HAL_RCC_CLEAR_RESET_FLAGS();
    } else {
        adcsState.lastResetWatchdog = false; // Normal boot, reset the counter
    }

    // 5. Initialize all subsystems
    // (These functions can now set componentActive[i] = false if a self-test fails)
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
    const uint32_t CONTROL_CYCLE_TIME = 1000; // Torque calculation update rate (1 Hz)
    const uint32_t MTQ_SETTLING_TIME = 10;    // Magnetic field dissipation time

    // --- State Variables ---
    uint32_t last_ehs_time = 0;
    uint32_t last_mm_req_time = 0;
    uint32_t last_control_cycle_time = 0;
    uint32_t last_mtq_start_time = 0;
    uint32_t mtq_off_time = 0; 
    uint32_t current_mtq_duration = 0; // Dynamically calculated MTQ firing time

    bool mtq_active = false;
    bool mm_request_allowed = true; // Master lock for the magnetometer

    while (true) {
        uint32_t currentTime = HAL_GetTick();
        
        // ---------------------------------------------------------
        // A. Handle OBC Requests
        // ---------------------------------------------------------
        if (OBC_data_request()) {
            // process_OBC_command(); placeholder
        }
        
        // ---------------------------------------------------------
        // B. Handle Hibernation Mode
        // ---------------------------------------------------------
        if (adcsState.currentMode == MODE_HIBERNATION) {
            // Sleep for 1000ms (1 second) at a time to save power, but still loop 
            // fast enough to kick the 2-second watchdog.
            HAL_Delay(1000); 
            HAL_IWDG_Refresh(&hiwdg);
            continue; 
        }

        // ---------------------------------------------------------
        // C. Control Cycle & Variable Magnetorquer Actuation
        // ---------------------------------------------------------
        if (adcsState.currentMode == MODE_NOMINAL || adcsState.currentMode == MODE_SAFE) {
            
            // 1. Start a new Control Cycle (e.g., every 1000 ms)
            if (currentTime - last_control_cycle_time >= CONTROL_CYCLE_TIME) {
                last_control_cycle_time = currentTime;
                
                // PLACEHOLDER: Call your external script/class to get required duration
                // current_mtq_duration = calculate_mtq_duration(adcsState);
                current_mtq_duration = 250; // Example: 250ms requested
                
                if (current_mtq_duration > 0) {
                    magnetorquerSubsystem.activate(); 
                    mtq_active = true;
                    mm_request_allowed = false; // Blind the magnetometer immediately
                    last_mtq_start_time = currentTime;
                }
            }

            // 2. Turn MTQ OFF when dynamic duration expires
            if (mtq_active && (currentTime - last_mtq_start_time >= current_mtq_duration)) {
                magnetorquerSubsystem.deactivate(); 
                mtq_active = false;
                mtq_off_time = currentTime; // Record exact turn-off time
            }

            // 3. Release the Magnetometer Lock after Settling Time
            if (!mtq_active && !mm_request_allowed && (currentTime - mtq_off_time >= MTQ_SETTLING_TIME)) {
                mm_request_allowed = true;      
                last_mm_req_time = currentTime; // Trigger an MM read immediately
            }
        }

        // ---------------------------------------------------------
        // D. Read Magnetometer (Protected by the allowed flag)
        // ---------------------------------------------------------
        if (mm_request_allowed) {
            if (currentTime - last_mm_req_time >= MM_REQ_INTERVAL) {
                // magnetometerSubsystem.send_request(); 
                last_mm_req_time = currentTime;
            }

            if (magnetometerSubsystem.data_ready()) {
                // magnetometerSubsystem.read_data(); 
            }
        }

        // ---------------------------------------------------------
        // E. Read Earth Horizon Sensors (Strictly every 125 ms)
        // ---------------------------------------------------------
        if (currentTime - last_ehs_time >= EHS_READ_INTERVAL) {
            last_ehs_time = currentTime;
            
            // Loop through each of the 4 EHS sensors individually
            for (int i = EHS_0; i <= EHS_3; i++) {
                
                if (adcsState.componentActive[i]) {
                    
                    // Attempt non-blocking read
                    bool success = horizonSubsystem.read_single_sensor(i); 
                    
                    if (success) {
                        adcsState.consecutiveErrorCount[i] = 0; 
                    } else {
                        adcsState.consecutiveErrorCount[i]++;
                        adcsState.errorCount++; // Increment global lifetime errors
                        
                        // Isolate component if it fails repeatedly
                        if (adcsState.consecutiveErrorCount[i] >= MAX_CONSECUTIVE_ERRORS) {
                            adcsState.componentActive[i] = false; 
                        }
                    }
                }
            }

            // Pass the active array so the combiner algorithm knows which ones to ignore
            // Attitude result = horizonSubsystem.calculate_attitude(adcsState.componentActive); 
            // adcsState.pitch = result.pitch;
            // adcsState.roll = result.roll;
        }

        // ---------------------------------------------------------
        // F. Kick the Watchdog (Must happen every loop!)
        // ---------------------------------------------------------
        HAL_IWDG_Refresh(&hiwdg); 
    }
    
    return 0;
}