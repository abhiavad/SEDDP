

#this code is written to compress the large conversion_tably.npy
# so that it is able to fit on the limited storage of the STM32

import numpy as np

# Load the original table
table = np.load('conversion_table.npy')

# Define scaling factor (2^10 = 1024)
SHIFT = 12
SCALER = 1 << SHIFT  # This equals 1024

# Convert to int32 (640 KB total)
# Max angle 359 * 1024 = 367,616 (Fits safely in 32-bit signed int)
table_fixed = (table * SCALER).astype(np.int32)

# Save for the STM32
#np.save('conversion_table_pow2.npy', table_fixed)

# Save as raw binary so C++ can read the struct directly
table_fixed.tofile('conversion_table_pow2.bin')
