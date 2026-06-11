import numpy as np
import math
import matplotlib.pyplot as plt

def simulate_earth_highres(pitch, roll, yaw, altitude_km=450.0, R_e=6378.14, earth_temp=45.0, space_temp=25.0):
    """
    Generate a 384x512 thermal image simulating Earth using 3D vector rotations.
    
    Rotations are mapped to the satellite body frame:
      - Pitch: Rotation around Z-axis (width of sensor)
      - Yaw:   Rotation around Y-axis (sensor boresight)
      - Roll:  Rotation around X-axis (height of sensor)
    """
    # 1) Load high resolution calibration tables
    # (Update paths as necessary for your local environment)
    X_angles_deg = np.loadtxt(r"C:\\Users\\tdhus\\Desktop\\SEP + SEDDP\\2026 - Earth Horizon Sensor for Delfi-Twin\\data_package\\horizon_detection\\FOV_Files\\horizontal_angles_highres_Scipy_Linear.txt", delimiter="\t")
    Y_angles_deg = np.loadtxt(r"C:\\Users\\tdhus\\Desktop\\SEP + SEDDP\\2026 - Earth Horizon Sensor for Delfi-Twin\\data_package\\horizon_detection\\FOV_Files\\vertical_angles_highres_Scipy_Linear.txt", delimiter="\t")
    
    # Convert calibration grid to radians
    X_angles = np.radians(X_angles_deg)
    Y_angles = np.radians(Y_angles_deg)

    # 2) Convert input angles to radians
    pitch_rad = np.radians(pitch)
    roll_rad = np.radians(roll)
    yaw_rad = np.radians(yaw)
    
    # 3) Earth's angular half-size from satellite altitude
    earth_half_angle = math.asin(R_e / (R_e + altitude_km))
    
    # 4) Construct standard 3D rotation matrices based on requested axes
    # Roll (around X-axis / Height)
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll_rad), -math.sin(roll_rad)],
        [0, math.sin(roll_rad), math.cos(roll_rad)]
    ])
    
    # Yaw (around Y-axis / Boresight)
    Ry = np.array([
        [math.cos(yaw_rad), 0, math.sin(yaw_rad)],
        [0, 1, 0],
        [-math.sin(yaw_rad), 0, math.cos(yaw_rad)]
    ])
    
    # Pitch (around Z-axis / Width)
    Rz = np.array([
        [math.cos(pitch_rad), -math.sin(pitch_rad), 0],
        [math.sin(pitch_rad), math.cos(pitch_rad), 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix
    Rot = Rz @ Ry @ Rx 

    # 5) Project 2D pixel angles into 3D unit vectors (Body Frame)
    # y-axis = Boresight (perpendicular to sensor face)
    # x-axis = Height (spanning the 24 rows, mapped to Y_angles)
    # z-axis = Width (spanning the 32 columns, mapped to X_angles)
    ray_x = -np.sin(Y_angles)
    ray_y = np.cos(X_angles) * np.cos(Y_angles)
    ray_z = -np.sin(X_angles) * np.cos(Y_angles)
    
    # Stack into a (384, 512, 3) matrix array
    rays = np.stack([ray_x, ray_y, ray_z], axis=-1)

    # 6) Rotate all rays simultaneously into the orbit reference frame
    rotated_rays = np.dot(rays, Rot.T)
    
    # Nadir vector points straight along +Y in the nominal target reference frame
    cos_angle = rotated_rays[..., 1]
    cos_angle = np.clip(cos_angle, -1.0, 1.0) # Prevent floating point domain errors
    dist_to_center = np.arccos(cos_angle)
    
    # 7) Assign temperatures based on intersection with Earth's angular radius
    data = np.where(dist_to_center < earth_half_angle, earth_temp, space_temp)
                
    return data

def downsample_image(image, new_shape):
    """Downsample image to target array resolution (24x32) via block averaging."""
    old_rows, old_cols = image.shape
    new_rows, new_cols = new_shape
    factor_row = old_rows // new_rows
    factor_col = old_cols // new_cols
    return image.reshape(new_rows, factor_row, new_cols, factor_col).mean(axis=(1, 3))

def simulate_earth(pitch, roll, yaw, altitude_km=450.0, R_e=6378.14, earth_temp=45.0, space_temp=25.0):
    print(f"Simulating: Pitch(Z)={pitch}°, Roll(X)={roll}°, Yaw(Y)={yaw}°")
    high_res_data = simulate_earth_highres(pitch, roll, yaw, altitude_km, R_e, earth_temp, space_temp)
    low_res_data = downsample_image(high_res_data, (24, 32)) # Downsamples to MLX90642 32x24 grid
    return low_res_data





if __name__=="__main__":
    # Test Matrix 
    test_matrix = [
        (0,   0,   0,   "Test 1 - Pure Nadir"),
        (0,  20,   0,   "Test 2 - Roll 90° (Looking to Space)"),
        (0, 119,   0,   "Test 3 - Roll 180° (Looking to Space)"),
        (0,   0,  45,   "Test 4 - Boresight Yaw Spin 45°"),
        (90,  0,   0,   "Test 5 - Pitch 90° (Looking to Space)"),
        (90, 90,   0,   "Test 6 - Pitch 90° + Roll 90°")
    ]
    
    fig, axs = plt.subplots(2, 3, figsize=(16, 10))
    fig.suptitle("MLX90642 Horizon Sensor Lookup Matrix\n(y=Boresight, x=Height, z=Width)", fontsize=14, fontweight='bold')
    axs = axs.ravel()

    for idx, config in enumerate(test_matrix):
        p, r, y, label = config
        low_res = simulate_earth(pitch=p, roll=r, yaw=y)
        
        im = axs[idx].imshow(low_res, cmap='hot', interpolation='nearest', vmin=25, vmax=45)
        axs[idx].set_title(f"{label}\nP={p}°, R={r}°, Y={y}°", fontsize=10)
        axs[idx].axis('off')
        fig.colorbar(im, ax=axs[idx], fraction=0.046, pad=0.04)

    plt.tight_layout()
    plt.show()
