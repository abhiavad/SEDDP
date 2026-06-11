import os
import re
import numpy as np
import horizon_detection_V6 as hd

DATA_DIR = r"C:\Users\tdhus\Desktop\SEP + SEDDP\2026 - Earth Horizon Sensor for Delfi-Twin\data_package\horizon_detection\data"
rx_static = re.compile(r"p(?P<pitch>[-\d.]+)_r(?P<roll>[-\d.]+)\.npy$", re.I)

# Use sorted() to ensure deterministic order
files = sorted([f for f in os.listdir(DATA_DIR) if rx_static.fullmatch(f)])

conversion_table = []

# Load geometry once to pass into functions (C++ style)
# Assuming these are defined in your horizon_detection_V6 file
X_angles = hd.X_angles
Y_angles = hd.Y_angles
origin_rad = hd.origin_rad
threshold = hd.CONFIG_THRESHOLD

for fname in files:
    m = rx_static.fullmatch(fname)
    p_cmd = float(m["pitch"])
    r_cmd = float(m["roll"])
    
    data = np.load(os.path.join(DATA_DIR, fname))
    
    # Updated calls to use the explicit parameter passing
    v_x, v_y = hd.vector(data, threshold, X_angles, Y_angles, origin_rad)
    
    if not np.isnan(v_x):
        area = hd.integrate_angles(data, threshold, X_angles, Y_angles)
        
        # --- NEW: Pre-calculate Approximate Roll (atan2) ---
        # This is the 6th column used for the "Automatic roll-band detection"
        approx_roll = np.arctan2(v_y, v_x)
        
        conversion_table.append([p_cmd, r_cmd, v_x, v_y, area, approx_roll])

# Convert to numpy
final_table = np.array(conversion_table, dtype=np.float32)

# --- NEW: Sort by the Approximate Roll column (Index 5) ---
# The banding logic ONLY works if the table is sorted by this value.
final_table = final_table[final_table[:, 5].argsort()]

# Save the finalized, sorted table
np.save("conversion_table.npy", final_table)

print(f"Table generated with shape: {final_table.shape}")
print("Table columns: [Pitch, Roll, V_x, V_y, Area, Approx_Roll_Atan2]")

# Verification check
test_row = final_table[(final_table[:,0] == 25) & (final_table[:,1] == 27)]
print(f"Verified row for p25_r27: {test_row}")