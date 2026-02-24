import numpy as np
import cv2
import os
import glob
import re

def fit_3d_curve_wtls(points_3d, xi_tls, iterations=5):
    """
    Implements Section 2.4.2: Weighted Total Least Squares.
    Adjusts fitting based on the precision of points near FOV edges.
    """
    xi = xi_tls
    # Define design matrix E: [x^2, xy, y^2, xz, yz] (Eq 12)
    E = np.zeros((len(points_3d), 5))
    E[:, 0] = points_3d[:, 0]**2
    E[:, 1] = points_3d[:, 0] * points_3d[:, 1]
    E[:, 2] = points_3d[:, 1]**2
    E[:, 3] = points_3d[:, 0] * points_3d[:, 2]
    E[:, 4] = points_3d[:, 1] * points_3d[:, 2]
    
    y = np.ones((len(points_3d), 1))
    
    for _ in range(iterations):
        # Weighting based on Section 2.4.2: points near FOV edge are less accurate
        # We use a simplified weighting based on radial distance
        weights = 1.0 / (1.0 + np.sum(points_3d[:, :2]**2, axis=1))
        W = np.diag(weights)
        
        try:
            # Iterative update (Eq 259)
            xi = np.linalg.inv(E.T @ W @ E) @ E.T @ W @ y
        except np.linalg.LinAlgError:
            break
            
    return xi.flatten()

def get_nadir_from_xi(xi):
    """
    Reconstructs Matrix M (Eq 22) and calculates the Nadir vector.
    """
    M = np.array([
        [1 - xi[0],    -xi[1]/2.0,  -xi[3]/2.0],
        [-xi[1]/2.0,   1 - xi[2],   -xi[4]/2.0],
        [-xi[3]/2.0,   -xi[4]/2.0,   1.0]
    ])
    
    # The nadir vector is the eigenvector associated with the smallest eigenvalue
    eigenvalues, eigenvectors = np.linalg.eig(M)
    nadir = eigenvectors[:, np.argmin(np.abs(eigenvalues))]
    return nadir / np.linalg.norm(nadir)

def calculate_angles(nadir_vector):
    """
    Converts 3D Nadir unit vector to Off-Nadir (Roll) and Pitch (Azimuth).
    """
    n = nadir_vector / np.linalg.norm(nadir_vector)
    
    # Off-Nadir angle (phi) is the angle relative to the Z-axis (Eq 165)
    off_nadir_rad = np.arccos(np.clip(n[2], -1.0, 1.0))
    off_nadir_deg = np.degrees(off_nadir_rad)
    
    # Pitch is the rotation in the XY plane
    pitch_deg = np.degrees(np.arctan2(n[1], n[0])) % 360
    
    return off_nadir_deg, pitch_deg

def process_horizon_full(ir_frame, focal_length, threshold_temp=35.0, T_deg=1.0):
    # --- 1. Edge Detection (Section 2.1) ---
    binary_mask = (ir_frame > threshold_temp).astype(np.uint8) * 255
    # Verification of mask size based on your simulation criteria
    if not (20 < np.sum(ir_frame > threshold_temp) < 748):
        return None

    # Sobel for initial edge localization (Section 2.1)
    grad_x = cv2.Sobel(binary_mask, cv2.CV_64F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(binary_mask, cv2.CV_64F, 0, 1, ksize=3)
    edge_mag = np.sqrt(grad_x**2 + grad_y**2)
    v_coords, u_coords = np.where(edge_mag > 0)
    
    # Centering coordinates (assuming 32x24 sensor)
    u_centered = u_coords - 15.5
    v_centered = v_coords - 11.5
    
    # --- 2. Horizon Projection (Section 2.2) ---
    r = np.sqrt(u_centered**2 + v_centered**2)
    theta = r / focal_length # Eq 1
    
    # Map to Unit Sphere (Eq 3)
    x = np.sin(theta) * (u_centered / (r + 1e-9))
    y = np.sin(theta) * (v_centered / (r + 1e-9))
    z = np.cos(theta)
    points_3d = np.column_stack((x, y, z))
    
    if len(points_3d) < 5: return None

    # --- 3. Modified RANSAC (Section 2.3) ---
    best_inliers = points_3d
    max_inlier_count = 0
    T_rad = np.radians(T_deg)
    
    for _ in range(286): # k_max (Eq 25)
        idx = np.random.choice(len(points_3d), 5, replace=False)
        sample = points_3d[idx]
        
        # Calculate normal vector n (Eq 4)
        p = sample[0] - sample[1]
        q = sample[0] - sample[2]
        n = np.cross(p, q)
        norm_n = np.linalg.norm(n)
        if norm_n < 1e-9: continue
        n /= norm_n
        
        # Pre-verification (Step 3)
        sample_angles = np.arccos(np.clip(np.dot(sample, n), -1.0, 1.0))
        theta_m = np.mean(sample_angles)
        if np.mean(np.abs(sample_angles - theta_m)) > T_rad:
            continue
            
        # Consensus check
        all_angles = np.arccos(np.clip(np.dot(points_3d, n), -1.0, 1.0))
        inliers_mask = np.abs(all_angles - theta_m) < T_rad
        if np.sum(inliers_mask) > max_inlier_count:
            max_inlier_count = np.sum(inliers_mask)
            best_inliers = points_3d[inliers_mask]
            
        if max_inlier_count > 0.5 * len(points_3d): break

    # --- 4. 3D Curve Fitting (Section 2.4) ---
    # Initial TLS Estimation (Eq 15-17)
    ones = np.ones((len(best_inliers), 1))
    E_initial = np.zeros((len(best_inliers), 5))
    E_initial[:, 0] = best_inliers[:, 0]**2
    E_initial[:, 1] = best_inliers[:, 0] * best_inliers[:, 1]
    E_initial[:, 2] = best_inliers[:, 1]**2
    E_initial[:, 3] = best_inliers[:, 0] * best_inliers[:, 2]
    E_initial[:, 4] = best_inliers[:, 1] * best_inliers[:, 2]
    
    Ey = np.hstack((E_initial, ones))
    _, _, Vt = np.linalg.svd(Ey)
    xi_tls = -Vt[-1, :5] / Vt[-1, 5]
    
    # Execute WTLS for refinement (Section 2.4.2)
    xi_final = fit_3d_curve_wtls(best_inliers, xi_tls)
    
    nadir_vec = get_nadir_from_xi(xi_final)
    return nadir_vec

# --- Main Test Loop ---
data_folder = 'data'
f_px = 15.0 # Ensure this matches your simulation's focal length

print(f"{'Filename':<20} | {'True P':<8} | {'True R':<8} | {'Est P':<8} | {'Est R':<8} | {'Error R':<8}")
print("-" * 85)

for filepath in glob.glob(os.path.join(data_folder, "*.npy")):
    filename = os.path.basename(filepath)
    
    # Parse ground truth p and r from filename
    match = re.search(r'p([\d.]+)_r([\d.]+)', filename)
    if not match: continue
    true_p, true_r = float(match.group(1)), float(match.group(2))
    
    # Load and process
    data = np.load(filepath)
    nadir = process_horizon_full(data, f_px)
    
    if nadir is not None:
        est_r, est_p = calculate_angles(nadir)
        error_r = est_r - true_r
        print(f"{filename:<20} | {true_p:<8.1f} | {true_r:<8.1f} | {est_p:<8.2f} | {est_r:<8.2f} | {error_r:<8.4f}")