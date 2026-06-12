
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
                    // --- Pitch Gate (with circular wrap-around fix) ---
                    float pitch_diff = fabsf(pitches[i] - last_pitch);
                    if (pitch_diff > 180.0f) {
                        pitch_diff = 360.0f - pitch_diff; // Take the shorter path around the circle
                    }

                    // --- Roll Gate (with circular wrap-around fix) ---
                    float roll_diff = fabsf(rolls[i] - last_roll);
                    if (roll_diff > 180.0f) {
                        roll_diff = 360.0f - roll_diff; // Take the shorter path around the circle
                    }

                    // If EITHER the pitch or the roll jumps further than the gate, reject it
                    if (pitch_diff > current_gate || roll_diff > current_gate) {
                        is_valid = false;
                    }
                }

        if (is_valid) {
            confirm_counts[i]++;
            grace_counts[i] = 0;
            if (confirm_counts[i] >= CONFIRM_THRESH) {
                candidates[candidate_count++] = i;
                is_acquired = true;
            }
        } else {
        	// Increment, but cap it right after it crosses the limit
        	if (grace_counts[i] <= GRACE_LIMIT) {
        		grace_counts[i]++;
        	}

        	if (grace_counts[i] > GRACE_LIMIT) {
        		confirm_counts[i] = 0;
        	}

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
    // Remove comment from this for loop when DEBUGGING; this is for retrieving confirm & grace counts of individual sensors
    for (int i = 0; i<4; i++){
    	output.confirm_counts[i] = confirm_counts[i];
    	output.grace_counts[i] = grace_counts[i];
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

    // Static flag prevents the initial (0,0) state from dragging the first lock
    static bool has_history = false;

    // --- 1. CONVERT HISTORY AND CURRENT TO 3D VECTORS ---
    float last_v_x = 0.0f, last_v_y = 0.0f, last_v_z = 0.0f;
    if (has_history) {
        float lp_rad = last_pitch * (M_PI / 180.0f);
        float lr_rad = last_roll * (M_PI / 180.0f);
        last_v_x = -sinf(lp_rad) * cosf(lr_rad);
        last_v_y = cosf(lp_rad) * cosf(lr_rad);
        last_v_z = sinf(lr_rad);
    }

    float v_x[4] = {0}, v_y[4] = {0}, v_z[4] = {0};
    for (int i = 0; i < candidate_count; i++) {
        int id = candidates[i];
        float p_rad = pitches[id] * (M_PI / 180.0f);
        float r_rad = rolls[id] * (M_PI / 180.0f);

        v_x[i] = -sinf(p_rad) * cosf(r_rad);
        v_y[i] = cosf(p_rad) * cosf(r_rad);
        v_z[i] = sinf(r_rad);
    }

    // --- 2. VECTOR CONSENSUS FILTER ---
    bool use_consensus = false;
    bool in_consensus[4] = {false, false, false, false};
    const float DOT_THRESHOLD = 0.93969f;
    const float DOT_OPPOSITE_THRESHOLD = -0.99619f;

    if (candidate_count >= 2) {
        int max_agreements = 0;
        int best_consensus_idx = -1;

        for (int i = 0; i < candidate_count; i++) {
            int agreements = 0;
            for (int j = 0; j < candidate_count; j++) {
                if (i != j) {
                    float dot_product = (v_x[i] * v_x[j]) + (v_y[i] * v_y[j]) + (v_z[i] * v_z[j]);
                    if (dot_product >= DOT_THRESHOLD || dot_product <= DOT_OPPOSITE_THRESHOLD) {
                        agreements++;
                    }
                }
            }
            if (agreements > max_agreements) {
                max_agreements = agreements;
                best_consensus_idx = i;
            }
        }

        if (max_agreements >= 1) {
            use_consensus = true;
            for (int i = 0; i < candidate_count; i++) {
                if (i == best_consensus_idx) {
                    in_consensus[i] = true;
                } else {
                    float dot = (v_x[best_consensus_idx] * v_x[i]) +
                                (v_y[best_consensus_idx] * v_y[i]) +
                                (v_z[best_consensus_idx] * v_z[i]);

                    if (dot >= DOT_THRESHOLD) {
                        in_consensus[i] = true;
                    } else if (dot <= DOT_OPPOSITE_THRESHOLD) {
                        v_x[i] = -v_x[i];
                        v_y[i] = -v_y[i];
                        v_z[i] = -v_z[i];
                        in_consensus[i] = true;
                    }
                }
            }
        }
    }

    // --- 3. COMBINE VECTORS WITH 10-DEGREE HISTORY PREFERENCE ---
    float sum_v_x = 0.0f, sum_v_y = 0.0f, sum_v_z = 0.0f;
    float total_weight = 0.0f;
    int best_sensor_id = candidates[0];
    float max_weight = -1.0f;
    const float IDEAL_AREA = 384.0f; 

    for (int i = 0; i < candidate_count; i++) {
        if (use_consensus && !in_consensus[i]) continue;

        int id = candidates[i];
        
        float area_error = fabsf(areas[id] - IDEAL_AREA);
        float weight = IDEAL_AREA - area_error;
        if (weight <= 0.0f) weight = 0.001f; 

        // THE USER REQUEST: 10-DEGREE HISTORY PREFERENCE
        // cos(10 degrees) = 0.9848f
        if (has_history) {
            float hist_dot = (v_x[i] * last_v_x) + (v_y[i] * last_v_y) + (v_z[i] * last_v_z);
            if (hist_dot >= 0.9848f) {
                weight *= 2.0f; // Double the voting power of stable sensors!
            }
        }
        
        if (weight > max_weight) {
            max_weight = weight;
            best_sensor_id = id;
        }

        sum_v_x += v_x[i] * weight;
        sum_v_y += v_y[i] * weight;
        sum_v_z += v_z[i] * weight;
        
        total_weight += weight;
    }

    // --- THE POLARITY ANCHOR: FIXING THE 180-DEGREE FLIP ---
    if (has_history && total_weight > 0.0f) {
        float sum_dot = (sum_v_x * last_v_x) + (sum_v_y * last_v_y) + (sum_v_z * last_v_z);

        // A spacecraft cannot physically rotate 90+ degrees in 0.25s.
        // If the consensus group points backward, the core accidentally anchored
        // on the inverted sensor. Flip the entire sum to restore the polarity!
        if (sum_dot < 0.0f) {
            sum_v_x = -sum_v_x;
            sum_v_y = -sum_v_y;
            sum_v_z = -sum_v_z;
        }
    }

    // --- 4. EXTRACT FINAL PITCH AND ROLL ---
    if (total_weight > 0.0f) {
        output.roll = asinf(sum_v_z / total_weight) * (180.0f / M_PI);
        output.pitch = atan2f(-sum_v_x, sum_v_y) * (180.0f / M_PI);

        if (output.roll < 0) output.roll += 360.0f;
        if (output.pitch < 0) output.pitch += 360.0f;
    } else {
        output.pitch = last_pitch;
        output.roll = last_roll;
    }

    output.active_sensor_id = best_sensor_id; 
    output.confidence = candidate_count; 
    output.is_valid = true;

    last_pitch = output.pitch;
    last_roll = output.roll;
    has_history = true; // Safe to use history for the next frame
}
