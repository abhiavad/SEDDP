import os
import re
import numpy as np

DATA_DIR = r"data/"
rx_static = re.compile(r"p(?P<pitch>[-\d.]+)_r(?P<roll>[-\d.]+)\.npy$", re.I)

# 1. Use sorted() to ensure deterministic order
files = sorted([f for f in os.listdir(DATA_DIR) if rx_static.fullmatch(f)])

conversion_table = []

for fname in files:
    m = rx_static.fullmatch(fname)
    p_cmd = float(m["pitch"])
    r_cmd = float(m["roll"])
    
    data = np.load(os.path.join(DATA_DIR, fname))
    
    # Calculate signature using your verified functions
    import horizon_detection_V6 as hd
    vec = hd.vector(data)
    area = hd.integrate_angles(data)
    
    # Check for NaN before adding to table
    if not np.isnan(vec[0]):
        conversion_table.append([p_cmd, r_cmd, vec[0], vec[1], area])

# Convert to numpy and save
final_table = np.array(conversion_table)
np.save("conversion_table.npy", final_table)

# --- NEW: Immediate Verification ---
# Let's check the row for p25_r27 specifically
test_row = final_table[(final_table[:,0] == 25) & (final_table[:,1] == 27)]
print(f"Table data for p25_r27: {test_row}")