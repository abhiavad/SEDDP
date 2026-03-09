import numpy as np
import math
from image_generation_V3 import simulate_earth

# --- Configuration & Sensor Data (C++ Style: Global Constants/Buffers) ---
CONFIG_THRESHOLD = 35 
REFERENCE_VECTOR = (0, -1)
SENSOR_RES = (24, 32)
ORIGIN_COORD = (11, 15)

# Load sensor geometry
X_angles_deg = np.loadtxt("FOV_Files/horizontal_angles.txt", delimiter="\t") 
Y_angles_deg = np.loadtxt("FOV_Files/vertical_angles.txt", delimiter="\t") 
X_angles = np.radians(X_angles_deg).astype(np.float32)
Y_angles = np.radians(Y_angles_deg).astype(np.float32)

# Convert origin from pixel index to radian coordinates
origin_rad = np.array([X_angles[ORIGIN_COORD], Y_angles[ORIGIN_COORD]], dtype=np.float32)

# Load conversion table
conv_table = np.load("conversion_table.npy").astype(np.float32)


def vector(data, threshold, x_angles_map, y_angles_map, sensor_origin):
    """Vectorized version with explicit parameter passing (C++ style)."""
    mask = data > threshold
    valid_pixels = np.sum(mask)
    if (valid_pixels < 20) or (valid_pixels > 748):
        return np.nan, np.nan
    # C++ mindset: Operations on flat memory buffers
    valid_x = (x_angles_map[mask] - sensor_origin[0])
    valid_y = (y_angles_map[mask] - sensor_origin[1])
    sum_x = np.sum(valid_x)
    sum_y = np.sum(valid_y)
    mag = np.sqrt(sum_x**2 + sum_y**2)
    if mag == 0: 
        return np.nan, np.nan
    return sum_x / mag, sum_y / mag


def integrate_angles(data, threshold, x_angles_map, y_angles_map):
    """Vectorized integration with explicit parameter passing."""
    mask = data > threshold
    
    # In C++, gradients would be pre-calculated or done via simple subtraction
    X_gradient = np.gradient(x_angles_map, axis=1)
    Y_gradient = np.gradient(y_angles_map, axis=0)
    angle_area = np.abs(X_gradient * Y_gradient)
    
    return np.sum(angle_area[mask])


def convert_coordinates_canonical(v_x, v_y, area, table, offset_deg):
    """Canonical table search with 2D rotation matrix."""
    # --- 1. ROTATE INPUT INTO CANONICAL FRAME ---
    # We rotate the detected vector backwards by the sensor's mounting angle
    offset_rad = math.radians(offset_deg)
    # 2D Rotation Matrix application:
    v_x_can = v_x * math.cos(-offset_rad) - v_y * math.sin(-offset_rad)
    v_y_can = v_x * math.sin(-offset_rad) + v_y * math.cos(-offset_rad)

    # --- 2. PERFORM BANDED SEARCH (Standard Logic) ---
    approx_roll = math.atan2(v_y_can, v_x_can)
    TOLERANCE = 0.15 
    num_rows = table.shape[0]
    
    # Define wrap-around search bounds as before
    lower_bound, upper_bound = approx_roll - TOLERANCE, approx_roll + TOLERANCE
    lower_wrap, upper_wrap = 999.0, -999.0
    if upper_bound > math.pi:
        lower_wrap, upper_wrap = -math.pi, upper_bound - 2 * math.pi
    elif lower_bound < -math.pi:
        lower_wrap, upper_wrap = lower_bound + 2 * math.pi, math.pi

    min_dist_sq = float('inf')
    best_idx = -1
    for i in range(num_rows):
        t_roll = table[i, 5]
        if (t_roll >= lower_bound and t_roll <= upper_bound) or (t_roll >= lower_wrap and t_roll <= upper_wrap):
            dx, dy, da = table[i, 2] - v_x_can, table[i, 3] - v_y_can, table[i, 4] - area
            dist_sq = dx*dx + dy*dy + da*da
            if dist_sq < min_dist_sq:
                min_dist_sq, best_idx = dist_sq, i
                
    if best_idx == -1: return np.nan, np.nan

    # --- 3. ROTATE RESULT TO BODY FRAME ---
    table_p = table[best_idx, 0]
    table_r = table[best_idx, 1]
    
    # The table roll is relative to the sensor. Add the offset to get Body Roll.
    body_roll = (table_r + offset_deg) % 360
    
    return table_p, body_roll


# --- TEST BLOCK ---
if __name__=="__main__":
    # Simulate an Earth image
    data = simulate_earth(25, 27, 0, plot=False) 
    
    # Calculate Directional Vector
    v_x, v_y = vector(
        data, 
        CONFIG_THRESHOLD, 
        X_angles, 
        Y_angles, 
        origin_rad
    )
    
    if not np.isnan(v_x):
        # Calculate Earth Area
        area = integrate_angles(data, CONFIG_THRESHOLD, X_angles, Y_angles)
        
        # Look up using the Canonical method (assuming 0 degree offset for this test)
        p, r = convert_coordinates_canonical(v_x, v_y, area, conv_table, 0.0)
        
        print(f"Vector: ({v_x:.4f}, {v_y:.4f}), Area: {area:.6f}")
        print(f"Matched Pitch: {p:.2f}, Matched Roll: {r:.2f}")