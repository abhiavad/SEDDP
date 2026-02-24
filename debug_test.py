import numpy as np
import horizon_detection_V6 as hd 

# Load the exact same file you converted for C++
data = np.load('data/p25.0_r27.0.npy')

# Calculate using your Python functions
vec = hd.vector(data)
area = hd.integrate_angles(data)

print("--- PYTHON VERIFICATION ---")
print(f"Vector: {vec}")
print(f"Area:   {area}")

import numpy as np
# 1. Load your original Python table (NOT the .bin one)
table = np.load("conversion_table.npy")

# 2. Your C++ inputs
target_vx = -0.0640586
target_vy = -0.997946
target_area = 2.53273

# 3. Find the match in Python
diff = table[:, 2:] - [target_vx, target_vy, target_area]
dist_sq = np.sum(diff**2, axis=1)
idx = np.argmin(dist_sq)

print(f"Python Table Match -> Pitch: {table[idx,0]}, Roll: {table[idx,1]}")
print(f"Python Match Distance: {dist_sq[idx]}")