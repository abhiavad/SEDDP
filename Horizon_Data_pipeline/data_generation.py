# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Created on Fri May 16 11:04:14 2025

# @author: emiel & tjalling
# """


# import numpy as np
# import os
# from image_generation_V3 import simulate_earth

# # 1. Force the absolute path
# data_folder = r"C:\Users\tdhus\Desktop\SEP + SEDDP\2026 - Earth Horizon Sensor for Delfi-Twin\data_package\horizon_detection\data"
# os.makedirs(data_folder, exist_ok=True)

# # 2. Start pitch exactly at 338, end at 359 (22 steps)
# pitch = np.linspace(338, 359, num=22) 
# roll = np.linspace(0, 359, num=360) 

# new_pitch = []
# new_roll = []

# print("Resuming simulation directly from Pitch 338.0, Roll 34.0...")

# for p in pitch:
#     for r in roll:
        
#         # --- THE HARDCODED RESUME ---
#         # If we are on pitch 338, skip any rolls before 34.
#         if p == 338.0 and r < 34.0:
#             continue
#         # ----------------------------

#         filename = os.path.join(data_folder, f'p{p}_r{r}.npy')
        
#         print(f"Generating pitch {p}, roll {r}...")
        
#         data = simulate_earth(p, r, 0)
#         mask = data > 35.0
        
#         if (np.sum(mask) > 20) and (np.sum(mask) < 748):
#             np.save(filename, data)
#             new_pitch.append(p)
#             new_roll.append(r)

# print("SIMULATION FINISHED!")

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 16 11:04:14 2025

@author: emiel
"""

import os
import numpy as np
from image_generation_V4 import simulate_earth

# 1. Get the absolute path to the directory where this script lives
script_dir = os.path.dirname(os.path.abspath(__file__))

# 2. Define the path to the "data" folder next to the script
data_dir = os.path.join(script_dir, "data")

# 3. Create the "data" folder if it doesn't already exist
os.makedirs(data_dir, exist_ok=True)

pitch = np.linspace(0, 359, num=360)
roll = np.linspace(20, 119, num=100)

new_pitch = []
new_roll = []

for p in pitch:
    for r in roll:
        data = simulate_earth(p, r, 0)
        mask = data > 35.0
        
        if (np.sum(mask) > 20) and (np.sum(mask) < 748):
            # 4. Save the file using the absolute path to the data folder
            file_path = os.path.join(data_dir, f'p{p}_r{r}.npy')
            np.save(file_path, data)
            
            new_pitch.append(p)
            new_roll.append(r)

print(f"SIMULATION FINISHED! Files saved to: {data_dir}")