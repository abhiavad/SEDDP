import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import scipy.ndimage

def spherical_separation(a1, b1, a2, b2):
    """
    Compute the spherical angular separation between two directions given by:
    (a1, b1) and (a2, b2), using the spherical law of cosines.
    All angles are in radians.
    """
    return math.acos(math.sin(b1)*math.sin(b2) +
                     math.cos(b1)*math.cos(b2)*math.cos(a2 - a1))

def rotate_point(x, y, angle, origin=(0, 0)):
    """
    Rotate a point (x, y) around a given origin by an angle in radians.

    Parameters:
        x (float): X-coordinate of the point to rotate.
        y (float): Y-coordinate of the point to rotate.
        angle (float): Rotation angle in radians.
        origin (tuple of float, optional): The origin point (ox, oy) to rotate around. Defaults to (0, 0).

    Returns:
        The rotated point's coordinates x,y
    """
    # Unpack origin
    ox, oy = origin

    # Translate point back to origin
    x_translated = x - ox
    y_translated = y - oy

    # Perform rotation using radians directly
    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)

    x_rotated = x_translated * cos_theta - y_translated * sin_theta
    y_rotated = x_translated * sin_theta + y_translated * cos_theta

    # Translate point back
    x_new = x_rotated + ox
    y_new = y_rotated + oy

    return x_new, y_new

def simulate_earth_highres(pitch, roll, yaw, altitude_km=300.0, R_e=6378.14, earth_temp=45.0, space_temp=25.0):
    """
    Generate a 384x512 thermal image simulating Earth.
    The simulation uses upscaled fisheye calibration arrays so that each pixel has its
    own (non-uniform) angular direction.
    
    Parameters:
      pitch, roll, yaw : camera tilt offsets (in degrees)
      altitude_km : satellite altitude [km]
      R_e : Earth's radius [km]
      earth_temp : temperature assigned to Earth [°C]
      space_temp : temperature for space [°C]
      fade_width_deg : angular width over which to fade from Earth to space.
    
    Returns:
      data: a 384x512 array of simulated thermal data.
    """
    # 1) Load high resolution calibration tables.
    X_angles_deg = np.loadtxt("FOV_Files/horizontal_angles_highres_Scipy_Linear.txt", delimiter="\t")
    Y_angles_deg = np.loadtxt("FOV_Files/vertical_angles_highres_Scipy_Linear.txt", delimiter="\t")
    # Convert the calibration arrays to radians.
    X_angles = np.radians(X_angles_deg)
    Y_angles = np.radians(Y_angles_deg)

    # 2) Convert camera tilt offsets to radians.
    pitch = np.radians(pitch)
    roll = np.radians(roll)
    yaw = np.radians(yaw)
    
    # 3) Earth’s angular half-size from the satellite:
    earth_half_angle = math.asin(R_e / (R_e + altitude_km))
    
    # 4) Set the high resolution dimensions.
    high_res_height, high_res_width = 384, 512

    # 5) Create high resolution image (384x512) filled with space temperature.
    data = np.full((high_res_height, high_res_width), space_temp, dtype=float)
    
    Y_angles += roll # First, add roll to Y axis
    
    # Calculate new origin to rotate about
    origin = (X_angles[int(high_res_height/2),int(high_res_width/2)],
              Y_angles[int(high_res_height/2),int(high_res_width/2)])
    
    # Rotate the image according to roll
    X_angles, Y_angles = rotate_point(X_angles, Y_angles, pitch, origin)
    
    # 6) For each pixel, check if it's inside or outside Earth's angular radius.
    for row in range(high_res_height):
        for col in range(high_res_width):
            # a) Read local angles (azimuth, elevation) from calibration for this pixel and apply yaw.
            alpha_loc = X_angles[row, col] + yaw # azimuth (radians)
            beta_loc  = Y_angles[row, col] # elevation (radians)
            
            # b) Compute the angular distance from the camera boresight (0,0) to Earths center.
            dist_to_center = spherical_separation(0, 0, alpha_loc, beta_loc)
            
            # c) Compare dist_to_center with earth_half_angle.
            if dist_to_center < earth_half_angle:
                data[row, col] = earth_temp
            else:
                data[row, col] = space_temp
                
    return data

def downsample_image(image, new_shape):
    """
    Downsample a 2D image to new_shape by averaging non-overlapping blocks.
    Assumes that the original shape dimensions are divisible by the new shape dimensions.
    """
    old_rows, old_cols = image.shape
    new_rows, new_cols = new_shape
    factor_row = old_rows // new_rows
    factor_col = old_cols // new_cols
    # Reshape the image so that we have blocks of size (factor_row x factor_col), and average them.
    downsampled = image.reshape(new_rows, factor_row, new_cols, factor_col).mean(axis=(1, 3))
    return downsampled

def simulate_earth(pitch, roll, yaw, altitude_km=300.0, R_e=6378.14, earth_temp=45.0, space_temp=25.0, plot=False):
    # 1) Print the input camera tilt values.
    print("\n", f"Simulating Earth center with camera tilt: pitch = {pitch}°, roll = {roll}°, yaw = {yaw}°")

    # 2) Generate high-resolution (384x512) simulated thermal image.
    high_res_data = simulate_earth_highres(pitch, roll, yaw, altitude_km, R_e, earth_temp, space_temp)
    
    # 3) Downsample the high-res image to 24x32 by averaging neighboring pixels.
    low_res_data = downsample_image(high_res_data, (24, 32))

    # 4) Display both images for comparison.
    if plot:
        plt.figure(figsize=(12, 5))
        
        plt.subplot(1, 2, 1)
        plt.title("High Resolution (384x512)")
        plt.imshow(high_res_data, cmap='hot', interpolation='nearest')
        plt.colorbar()
    
        plt.subplot(1, 2, 2)
        plt.title("Downsampled to 24x32")
        plt.imshow(low_res_data, cmap='hot', interpolation='nearest')
        plt.colorbar()
    
        plt.show()

    return low_res_data

if __name__=="__main__":
    # Choose test camera tilt values.
    pitch = 0
    roll = 105
    yaw = 0
    
    data = simulate_earth(pitch, roll, yaw, plot=True)
