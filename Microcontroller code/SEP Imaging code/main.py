import serial
import time
import numpy as np
import matplotlib.pyplot as plt

# Configure the serial connection
ser = serial.Serial('COM8', 250000, timeout=1)
time.sleep(2)  # Wait for connection to establish

reading = 0
image_array = []
print("Reading live data from Arduino...")

plt.ion()
fig, ax = plt.subplots(figsize=(8, 6))
img_plot = ax.imshow(np.zeros((24, 32)), cmap='hot', interpolation='nearest')
cbar = plt.colorbar(img_plot, ax=ax, label='Temperature (°C)')
ax.set_title('MLX90640 image')
plt.show()


try:
    while True:
        if ser.in_waiting > 0:  # Check if data is available
            line = ser.readline().decode('utf-8').rstrip()
            # print(f"{line}")

            if line == "End of full image CSV":
                reading = 2 # start processing stuff

            if reading == 1:

                my_list = line.split(",")

                my_list = [float(stringy if stringy!='' else '20') for stringy in my_list]
                # print(my_list)
                image_array.append(my_list)

            if reading == 2:
                # print("Here is the image array:")
                np_image_array = np.array(image_array)
                # print(np_image_array)
                img_plot.set_data(np_image_array)
                img_plot.set_clim(np.min(np_image_array), np.max(np_image_array))  # adjust scale

                plt.pause(0.001)

                reading = 0

            if line == "Start of full image CSV":
                image_array = []
                reading = 1 # start putting stuff into the array


except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    ser.close()  # Always close the port
    print("Serial port closed.")