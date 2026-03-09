
import math


class HorizonSensorManager:
    def __init__(self):
        # Configuration Constants 
        self.CONFIRM_THRESH = 5
        self.GRACE_LIMIT = 2
        
        # --- Dynamic Gate Thresholds ---
        self.GATE_WIDE = 22.5
        self.GATE_NARROW = 3.75
        
        # --- Hysteresis Timing ---
        # Frames to stay "locked" before tightening the gate
        self.STABILITY_LOCK_TIME = 40  
        # Frames to stay "lost" before widening the gate back out
        self.HYSTERESIS_LOST_TIME = 10 
        
        # State tracking
        self.active_id = 0
        self.last_pitch = 0.0
        self.last_roll = 0.0  
        self.confirm_counts = [0, 0, 0, 0]
        self.grace_counts = [0, 0, 0, 0]
        
        # Logic State variables
        self.is_acquired = False
        self.lock_counter = 0
        self.lost_counter = 0
        self.current_gate = self.GATE_WIDE

    def update(self, p0, r0, valid0, p1, r1, valid1, p2, r2, valid2, p3, r3, valid3):
        pitches = [p0, p1, p2, p3]
        rolls = [r0, r1, r2, r3]
        valids = [valid0, valid1, valid2, valid3]
        
        candidates = [-1, -1, -1, -1] 
        candidate_count = 0

        # Reset state if all sensors have zero trust
        if sum(self.confirm_counts) == 0:
            self.is_acquired = False
            self.lock_counter = 0
            self.lost_counter = 0
            self.current_gate = self.GATE_WIDE

        for i in range(4):
            is_valid = valids[i]
            
            # 1. PITCH GATING
            # Only apply if we have previously acquired a horizon
            if self.is_acquired and is_valid:
                if abs(pitches[i] - self.last_pitch) > self.current_gate:
                    is_valid = False

            # 2. CONFIRMATION & GRACE
            if is_valid:
                self.confirm_counts[i] += 1
                self.grace_counts[i] = 0
                if self.confirm_counts[i] >= self.CONFIRM_THRESH:
                    candidates[candidate_count] = i
                    candidate_count += 1
                    self.is_acquired = True 
            else:
                self.grace_counts[i] += 1
                if self.grace_counts[i] > self.GRACE_LIMIT:
                    self.confirm_counts[i] = 0

        # --- 3. DYNAMIC GATE WITH HYSTERESIS ---
        if candidate_count > 0:
            # We have valid data: Increment lock counter and reset lost counter
            self.lost_counter = 0
            if self.current_gate == self.GATE_WIDE:
                self.lock_counter += 1
                if self.lock_counter >= self.STABILITY_LOCK_TIME:
                    self.current_gate = self.GATE_NARROW
        else:
            # We have NO valid data: Increment lost counter and reset lock counter
            self.lock_counter = 0
            any_sensor_detecting = any(valids)

            if any_sensor_detecting:
                #Earth in view but it is rejected by gating
                # Thus narrow gate is too tight.
                if self.current_gate == self.GATE_NARROW:
                    self.lost_counter += 1
                    if self.lost_counter >= self.HYSTERESIS_LOST_TIME:
                        self.current_gate = self.GATE_WIDE
            else:
                #Blind spot so we just coast for a bit
                pass            
        return self._select_output(candidates, candidate_count, pitches, rolls)

    def _select_output(self, candidates, candidate_count, pitches, rolls):
        # (Averaging logic remains exactly the same as previous bug-fix version)
        if candidate_count == 0:
            return self.last_pitch, self.last_roll, -1, 0

        sum_pitch = 0.0
        sum_sin_roll = 0.0
        sum_cos_roll = 0.0
        
        for i in range(candidate_count):
            sensor_id = candidates[i]
            sum_pitch += pitches[sensor_id]
            r_rad = math.radians(rolls[sensor_id])
            sum_sin_roll += math.sin(r_rad)
            sum_cos_roll += math.cos(r_rad)
            
        avg_pitch = sum_pitch / candidate_count
        avg_roll = math.degrees(math.atan2(sum_sin_roll, sum_cos_roll)) % 360
        
        self.active_id = candidates[0]
        self.last_pitch = avg_pitch
        self.last_roll = avg_roll
        
        return avg_pitch, avg_roll, self.active_id, candidate_count