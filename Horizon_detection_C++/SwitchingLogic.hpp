
#pragma once
#include "HorizonTypes.hpp"

class HorizonSensorManager {
public:
    HorizonSensorManager();
    void update(const float pitches[4], const float rolls[4], const float areas[4], const bool valids[4], HorizonOutput& output);

private:
    static constexpr int NUM_SENSORS = 4;
    static constexpr int CONFIRM_THRESH = 5;
    static constexpr int GRACE_LIMIT = 2;
    static constexpr float GATE_WIDE = 22.5f;
    static constexpr float GATE_NARROW = 3.75f;
    static constexpr int STABILITY_LOCK_TIME = 40;
    static constexpr int HYSTERESIS_LOST_TIME = 10;

    int active_id;
    float last_pitch;
    float last_roll;
    int confirm_counts[4];
    int grace_counts[4];

    bool is_acquired;
    int lock_counter;
    int lost_counter;
    float current_gate;

    void select_output(const int candidates[4], int candidate_count, const float pitches[4], const float rolls[4], const float areas[4], HorizonOutput& output);
};