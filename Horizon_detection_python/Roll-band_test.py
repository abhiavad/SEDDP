

import numpy as np
import time
import math


table = np.load('conversion_table.npy').astype(np.float32)

table = table[table[:, 5].argsort()]

def search_original(v_x, v_y, area, table):
    # This simulates the original vectorized approach
    # We use columns 2, 3, 4 for the search
    diff = table[:, 2:5] - [v_x, v_y, area]
    dist_sq = np.sum(diff**2, axis=1)
    idx = np.argmin(dist_sq)
    return table[idx, 0:2]

def search_banded(v_x, v_y, area, table, tolerance=0.15):
    # 1. Estimate roll (O(1))
    approx_roll = math.atan2(v_y, v_x)
    
    # 2. Find band boundaries (O(log N) in C++, searchsorted in Python)
    start_idx = np.searchsorted(table[:, 5], approx_roll - tolerance)
    end_idx = np.searchsorted(table[:, 5], approx_roll + tolerance)
    
    if start_idx >= end_idx:
        # Fallback to full search if band is empty (unlikely with valid data)
        return search_original(v_x, v_y, area, table)
    
    # 3. Search only the small band
    band = table[start_idx:end_idx]
    diff = band[:, 2:5] - [v_x, v_y, area]
    dist_sq = np.sum(diff**2, axis=1)
    idx = np.argmin(dist_sq)
    return band[idx, 0:2], (end_idx - start_idx)

# Benchmarking
iterations = 1000
# Pick a target from the middle of the table to test
target_idx = len(table) // 2
target_vx = table[target_idx, 2]
target_vy = table[target_idx, 3]
target_area = table[target_idx, 4]

# Warm up
_ = search_original(target_vx, target_vy, target_area, table)
_ = search_banded(target_vx, target_vy, target_area, table)

# Measure Original
start_time = time.perf_counter()
for _ in range(iterations):
    _ = search_original(target_vx, target_vy, target_area, table)
end_time = time.perf_counter()
original_time = (end_time - start_time) / iterations

# Measure Banded
start_time = time.perf_counter()
for _ in range(iterations):
    _ = search_banded(target_vx, target_vy, target_area, table)
end_time = time.perf_counter()
banded_time = (end_time - start_time) / iterations

# Results
_, band_size = search_banded(target_vx, target_vy, target_area, table)

print(f"Table Size: {len(table)} entries")
print(f"Average Band Size: {band_size} entries (approx {100 * band_size / len(table):.2f}% of table)")
print(f"Original Search Time: {original_time*1e6:.2f} microseconds")
print(f"Banded Search Time:   {banded_time*1e6:.2f} microseconds")
print(f"Speedup Factor:       {original_time / banded_time:.2f}x")
print(f"Calculations skipped: {len(table) - band_size} rows")