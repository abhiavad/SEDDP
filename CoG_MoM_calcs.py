import numpy as np

# --- 1. User Defined Deployment Angle ---
theta_deg = 45.0  # Try 0 for closed, 90 for flat out
theta = np.radians(theta_deg)

# --- 2. Define Masses (kg) & Dimensions (m) ---
m_bus, m_panel = 0.350, 0.050
w_bus, h_bus, d_bus = 0.050, 0.050, 0.178
t_panel, w_panel, l_panel = 0.002, 0.050, 0.178

# --- 3. Define CG Locations (X, Y, Z in meters) ---
# Original Body Components (Static)
cg_bus = np.array([0.0, 0.0, 0.074])
cg_pt = np.array([0.0, w_bus/2 + t_panel/2, d_bus/2])
cg_pb = np.array([0.0, -w_bus/2 - t_panel/2, d_bus/2])
cg_pr = np.array([w_bus/2 + t_panel/2, 0.0, d_bus/2])
cg_pl = np.array([-w_bus/2 - t_panel/2, 0.0, d_bus/2])

# Deployed Panels (Dynamic based on theta)
# Hinge is at the edge of the bus (25mm), panel CG is halfway along its length (89mm)
hinge_dist = w_bus / 2
r_cg = l_panel / 2

# FIXED: Panels fold downward (-Z) when theta is 0, and swing out as theta increases
cg_dep_top = np.array([0.0, hinge_dist + r_cg * np.sin(theta), -r_cg * np.cos(theta)])
cg_dep_bot = np.array([0.0, -hinge_dist - r_cg * np.sin(theta), -r_cg * np.cos(theta)])
cg_dep_right = np.array([hinge_dist + r_cg * np.sin(theta), 0.0, -r_cg * np.cos(theta)])
cg_dep_left = np.array([-hinge_dist - r_cg * np.sin(theta), 0.0, -r_cg * np.cos(theta)])

# --- 4. Rotation Matrices for Tensors ---
def R_x(alpha):
    return np.array([[1, 0, 0],
                     [0, np.cos(alpha), -np.sin(alpha)],
                     [0, np.sin(alpha), np.cos(alpha)]])

def R_y(beta):
    return np.array([[np.cos(beta), 0, np.sin(beta)],
                     [0, 1, 0],
                     [-np.sin(beta), 0, np.cos(beta)]])

# Base MOI for panels if they were pointing straight along the Z axis
I_base_TB = np.diag([(1/12)*m_panel*(t_panel**2 + l_panel**2), 
                     (1/12)*m_panel*(w_panel**2 + l_panel**2), 
                     (1/12)*m_panel*(w_panel**2 + t_panel**2)])

I_base_RL = np.diag([(1/12)*m_panel*(w_panel**2 + l_panel**2), 
                     (1/12)*m_panel*(t_panel**2 + l_panel**2), 
                     (1/12)*m_panel*(w_panel**2 + t_panel**2)])

# Calculate rotated local MOI tensors for the deployed panels
I_rot_top = R_x(-theta) @ I_base_TB @ R_x(-theta).T
I_rot_bot = R_x(theta) @ I_base_TB @ R_x(theta).T
I_rot_right = R_y(theta) @ I_base_RL @ R_y(theta).T
I_rot_left = R_y(-theta) @ I_base_RL @ R_y(-theta).T

# --- 5. Build System Array ---
components = [
    {'name': 'Bus', 'm': m_bus, 'cg': cg_bus, 'I_local': np.diag([(1/12)*m_bus*(h_bus**2 + d_bus**2), (1/12)*m_bus*(w_bus**2 + d_bus**2), (1/12)*m_bus*(w_bus**2 + h_bus**2)])},
    {'name': 'Body Top', 'm': m_panel, 'cg': cg_pt, 'I_local': I_base_TB},
    {'name': 'Body Bot', 'm': m_panel, 'cg': cg_pb, 'I_local': I_base_TB},
    {'name': 'Body Right', 'm': m_panel, 'cg': cg_pr, 'I_local': I_base_RL},
    {'name': 'Body Left', 'm': m_panel, 'cg': cg_pl, 'I_local': I_base_RL},
    {'name': 'Dep Top', 'm': m_panel, 'cg': cg_dep_top, 'I_local': I_rot_top},
    {'name': 'Dep Bot', 'm': m_panel, 'cg': cg_dep_bot, 'I_local': I_rot_bot},
    {'name': 'Dep Right', 'm': m_panel, 'cg': cg_dep_right, 'I_local': I_rot_right},
    {'name': 'Dep Left', 'm': m_panel, 'cg': cg_dep_left, 'I_local': I_rot_left},
]

# --- 6. Calculate System Mass and Center of Mass ---
total_m = sum(c['m'] for c in components)
system_cg = sum(c['m'] * c['cg'] for c in components) / total_m

# --- 7. Calculate System Moment of Inertia Tensor ---
I_system_cg = np.zeros((3,3))

for c in components:
    m = c['m']
    I_local = c['I_local']
    
    r = c['cg'] - system_cg
    I_offset = m * np.array([
        [r[1]**2 + r[2]**2, -r[0]*r[1], -r[0]*r[2]],
        [-r[0]*r[1], r[0]**2 + r[2]**2, -r[1]*r[2]],
        [-r[0]*r[2], -r[1]*r[2], r[0]**2 + r[1]**2]
    ])
    I_system_cg += I_local + I_offset

# --- 8. Output Results ---
print("-" * 50)
print(f"Deployment Angle: {theta_deg} degrees")
print(f"System Total Mass: {total_m:.3f} kg")
print(f"System CG (X, Y, Z in mm): [{system_cg[0]*1000:.2f}, {system_cg[1]*1000:.2f}, {system_cg[2]*1000:.2f}]")
print("-" * 50)
print("MOI Tensor about System CG (kg*m^2):")
np.set_printoptions(formatter={'float': lambda x: "{0:0.6e}".format(x)})
print(I_system_cg)
print("-" * 50)
print("TEST PERFORMED")