
#include "SwitchingLogic.hpp"
#include <math.h>

HorizonSensorManager::HorizonSensorManager() {
    active_id = 0; last_pitch = 0.0f; last_roll = 0.0f;
    is_acquired = false; lock_counter = 0; lost_counter = 0;
    current_gate = GATE_WIDE;
    for (int i = 0; i < 4; i++) { confirm_counts[i] = 0; grace_counts[i] = 0; }
}

void HorizonSensorManager::update(const float pitches[4], const float rolls[4], const bool valids[4], HorizonOutput& output) {
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

    select_output(candidates, candidate_count, pitches, rolls, output);
}

void HorizonSensorManager::select_output(const int candidates[4], int candidate_count, const float pitches[4], const float rolls[4], HorizonOutput& output) {
    if (candidate_count == 0) {
        output.pitch = last_pitch; output.roll = last_roll;
        output.active_sensor_id = -1; output.confidence = 0;
        output.is_valid = false;
        return;
    }

    float sum_pitch = 0.0f, sum_sin_roll = 0.0f, sum_cos_roll = 0.0f;
    for (int i = 0; i < candidate_count; i++) {
        int id = candidates[i];
        sum_pitch += pitches[id];
        float r_rad = rolls[id] * (M_PI / 180.0f);
        sum_sin_roll += sinf(r_rad);
        sum_cos_roll += cosf(r_rad);
    }

    last_pitch = sum_pitch / candidate_count;
    last_roll = fmodf(atan2f(sum_sin_roll, sum_cos_roll) * (180.0f / M_PI), 360.0f);
    if (last_roll < 0) last_roll += 360.0f;

    output.pitch = last_pitch;
    output.roll = last_roll;
    output.active_sensor_id = candidates[0];
    output.confidence = candidate_count;
    output.is_valid = true;
}