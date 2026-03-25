
#include "SwitchingLogic.hpp"
#include <math.h>

HorizonSensorManager::HorizonSensorManager() {
    active_id = 0; last_pitch = 0.0f; last_roll = 0.0f;
    is_acquired = false; lock_counter = 0; lost_counter = 0;
    current_gate = GATE_WIDE;
    for (int i = 0; i < 4; i++) { confirm_counts[i] = 0; grace_counts[i] = 0; }
}

void HorizonSensorManager::update(const float pitches[4], const float rolls[4], const float areas[4], const bool valids[4], HorizonOutput& output) {
    int candidates[4] = {-1, -1, -1, -1};
    int candidate_count = 0;
    int total_confirms = 0;
    
    for (int i = 0; i < 4; i++) total_confirms += confirm_counts[i];

    if (total_confirms == 0) {
        is_acquired = false; lock_counter = 0; lost_counter = 0;
        current_gate = GATE_WIDE;
    }

    bool was_acquired = is_acquired;

    for (int i = 0; i < 4; i++) {
        bool is_valid = valids[i];

        if (was_acquired && is_valid) {
            if (fabsf(pitches[i] - last_pitch) > current_gate) is_valid = false;
        }

        if (is_valid) {
            confirm_counts[i]++;
            grace_counts[i] = 0;
            if (confirm_counts[i] >= CONFIRM_THRESH) {
                candidates[candidate_count++] = i;
                is_acquired = true;
            }
        } else {
            grace_counts[i]++;
            if (grace_counts[i] > GRACE_LIMIT) confirm_counts[i] = 0;
        }
    }

    if (candidate_count > 0) {
        lost_counter = 0;
        if (current_gate == GATE_WIDE) {
            lock_counter++;
            if (lock_counter >= STABILITY_LOCK_TIME) current_gate = GATE_NARROW;
        }
    } else {
        lock_counter = 0;
        bool any_detecting = false;
        for (int i = 0; i < 4; i++) if (valids[i]) any_detecting = true;

        if (any_detecting && current_gate == GATE_NARROW) {
            lost_counter++;
            if (lost_counter >= HYSTERESIS_LOST_TIME) current_gate = GATE_WIDE;
        }
    }

    select_output(candidates, candidate_count, pitches, rolls, areas, output);
}

void HorizonSensorManager::select_output(const int candidates[4], int candidate_count, const float pitches[4], const float rolls[4], const float areas[4], HorizonOutput& output) {
    if (candidate_count == 0) {
        output.pitch = last_pitch; output.roll = last_roll;
        output.active_sensor_id = -1; output.confidence = 0;
        output.is_valid = false;
        return;
    }

    float sum_pitch = 0.0f, sum_sin_roll = 0.0f, sum_cos_roll = 0.0f;
    float total_weight = 0.0f;
    int best_sensor_id = candidates[0];
    float max_weight = -1.0f;
    
    // Ideal area for MLX90640 (32x24) where horizon is perfectly centered
    const float IDEAL_AREA = 384.0f; 

    for (int i = 0; i < candidate_count; i++) {
        int id = candidates[i];
        
        // Calculate how far the area is from 384
        float area_error = fabsf(areas[id] - IDEAL_AREA);
        
        // Invert the error so an area of 384 yields the maximum weight (384)
        float weight = IDEAL_AREA - area_error;
        
        // Safeguard to prevent zero or negative division if area somehow exceeds 768 bounds
        if (weight <= 0.0f) weight = 0.001f; 
        
        // Track which sensor is closest to the ideal area (highest weight)
        if (weight > max_weight) {
            max_weight = weight;
            best_sensor_id = id;
        }

        // Weighted pitch
        sum_pitch += pitches[id] * weight;
        
        // Weighted circular roll 
        float r_rad = rolls[id] * (M_PI / 180.0f);
        sum_sin_roll += sinf(r_rad) * weight;
        sum_cos_roll += cosf(r_rad) * weight;
        
        total_weight += weight;
    }

    if (total_weight > 0.0f) {
        output.pitch = sum_pitch / total_weight; 
        
        // Recover the weighted circular average for roll
        output.roll = atan2f(sum_sin_roll, sum_cos_roll) * (180.0f / M_PI);
        if (output.roll < 0) output.roll += 360.0f;
    } else {
        output.pitch = last_pitch;
        output.roll = last_roll;
    }

    output.active_sensor_id = best_sensor_id; 
    output.confidence = candidate_count; 
    output.is_valid = true;

    last_pitch = output.pitch;
    last_roll = output.roll;
}