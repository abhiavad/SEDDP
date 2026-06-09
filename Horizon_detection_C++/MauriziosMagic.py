import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
PORT = 'COM3' # Verify your COM port
BAUD = 2500000 # Updated to 2.5 Mbps to match your CubeMX settings!

print("Connecting to STM32...")
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  
print("Reading live binary data and masks from STM32...")

# --- Setup Matplotlib Figure (2x4 Grid for 4 Sensors) ---
plt.ion()
fig, axs = plt.subplots(2, 4, figsize=(18, 7))
fig.canvas.manager.set_window_title('ADCS 4-Sensor Horizon Feed')

img_plots = []
mask_plots = []
sensor_labels = ['S0 (0x66|I2C1)', 'S1 (0x67|I2C1)', 'S2 (0x68|I2C2)', 'S3 (0x69|I2C2)']

for i in range(4):
    # Top Row: Raw Thermal
    im = axs[0, i].imshow(np.zeros((24, 32)), cmap='hot', interpolation='nearest', vmin=-20, vmax=50)
    plt.colorbar(im, ax=axs[0, i], fraction=0.046, pad=0.04)
    axs[0, i].set_title(f'{sensor_labels[i]} - Thermal')
    axs[0, i].axis('off')
    img_plots.append(im)

    # Bottom Row: Algorithm Mask
    mask = axs[1, i].imshow(np.zeros((24, 32)), cmap='gray', interpolation='nearest', vmin=0, vmax=255)
    axs[1, i].set_title(f'{sensor_labels[i]} - Mask')
    axs[1, i].axis('off')
    mask_plots.append(mask)

plt.tight_layout()
plt.show()

# --- Binary Protocol Definitions ---
HEADER = b'\xaa\xbb\xcc\xdd'

# Payload Math for 4 Sensors:
# Telemetry: 12 bytes
# Images: 4 * (3072 thermal + 768 mask) = 15360 bytes
# Total: 12 + 15360 = 15372 bytes
PAYLOAD_SIZE = 15372 

try:
    while True:
        # Wait for the synchronization header
        ser.read_until(HEADER)
        raw_data = ser.read(PAYLOAD_SIZE)
        
        if len(raw_data) == PAYLOAD_SIZE:
            
            # --- 1. UNPACK TELEMETRY (Bytes 0 to 12) ---
            telemetry_bytes = raw_data[0:12]
            telemetry = np.frombuffer(telemetry_bytes, dtype=np.float32)
            pitch, roll, active_id = telemetry[0], telemetry[1], int(telemetry[2])
            
            # --- PRINT TO TERMINAL ---
            if pitch == -999.0 or pitch == 0.0: # Depending on how you handle invalid states
                print("Status: SEARCHING | No horizon detected.")
            else:
                print(f"Status: LOCKED | Active Sensor ID: {active_id} | Pitch: {pitch:>6.2f}° | Body Roll: {roll:>6.2f}°")

            # --- 2. UNPACK ALL 4 SENSORS ---
            offset = 12
            for i in range(4):
                # Extract Thermal (3072 bytes)
                therm_bytes = raw_data[offset : offset + 3072]
                offset += 3072
                therm_array = np.frombuffer(therm_bytes, dtype=np.float32).reshape((24, 32))
                
                # Extract Mask (768 bytes)
                mask_bytes = raw_data[offset : offset + 768]
                offset += 768
                mask_array = np.frombuffer(mask_bytes, dtype=np.uint8).reshape((24, 32))

                # Update Plots
                img_plots[i].set_data(therm_array)
                
                # Auto-scale the color range dynamically based on min/max of the current frame
                current_min = np.min(therm_array)
                current_max = np.max(therm_array)
                if current_max > current_min: # Prevent warnings if the array is all zeros
                    img_plots[i].set_clim(current_min, current_max)  
                
                mask_plots[i].set_data(mask_array)
            
            # Flush the GUI events to update the window
            fig.canvas.flush_events()
            
        else:
            print(f"Dropped frame: Received {len(raw_data)} bytes, expected {PAYLOAD_SIZE}.")

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    ser.close()  
    print("Serial port closed.")