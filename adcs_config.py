"""
Central ADCS configuration.

Responsibilities
----------------
• Define discrete FSW timing (counter-based)
• Provide controller selection
• Centralize spacecraft, orbit, actuator, and environment parameters
• Ensure single source of truth for all tunable parameters
"""

# ==========================================================
# CONTROLLER SELECTION
# ==========================================================

"""
Options:
    "bdot"
    "detumble"
    "pd"
"""

ACTIVE_CONTROLLER = "bdot"


def get_controller_mode():
    """
    Return integer controller mode used by DipoleSelector.

    0 → BDOT
    1 → DETUMBLE
    2 → PD
    """
    mapping = {
        "bdot": 0,
        "detumble": 1,
        "pd": 2
    }

    return mapping.get(ACTIVE_CONTROLLER.lower(), 0)


# ==========================================================
# DISCRETE FSW TIMING CONFIG (LOCKED)
# ==========================================================

FSW_CONFIG = {
    "fsw_dt": 0.04,   # [s]
    "Ns": 3,           # sensing steps
    "Na": 1            # actuation steps
}


def get_fsw_dt():
    return FSW_CONFIG["fsw_dt"]


def get_Ns():
    return FSW_CONFIG["Ns"]


def get_Na():
    return FSW_CONFIG["Na"]


def get_cycle_steps():
    return FSW_CONFIG["Ns"] + FSW_CONFIG["Na"]


def get_cycle_time():
    return get_cycle_steps() * get_fsw_dt()


def get_control_scale():
    """
    Scaling factor due to duty-cycled actuation.
    """
    return get_cycle_steps() / get_Na()


# ==========================================================
# SPACECRAFT CONFIG
# ==========================================================

SPACECRAFT_CONFIG = {
    "mass": 0.75,  # kg
    "dimensions": {
        "Lx": 0.15,
        "Ly": 0.05,
        "Lz": 0.05
    },
    "initial_omega_deg": 180.0
}


def get_mass():
    return SPACECRAFT_CONFIG["mass"]


def get_dimensions():
    return SPACECRAFT_CONFIG["dimensions"]


def get_initial_omega_rad():
    import numpy as np
    return np.deg2rad(SPACECRAFT_CONFIG["initial_omega_deg"])


def get_inertia_matrix():
    """
    Compute inertia from cuboid dimensions.
    """
    dims = SPACECRAFT_CONFIG["dimensions"]
    m = SPACECRAFT_CONFIG["mass"]

    Lx, Ly, Lz = dims["Lx"], dims["Ly"], dims["Lz"]

    Ix = (1/12) * m * (Ly**2 + Lz**2)
    Iy = (1/12) * m * (Lx**2 + Lz**2)
    Iz = (1/12) * m * (Lx**2 + Ly**2)

    return [
        [Ix, 0, 0],
        [0, Iy, 0],
        [0, 0, Iz]
    ]


# ==========================================================
# ORBIT CONFIG
# ==========================================================

ORBIT_CONFIG = {
    "a": 6678e3,
    "e": 0.0,
    "i_deg": 97.7882,
    "Omega_deg": 48.2,
    "omega_deg": 347.8,
    "f_deg": 85.3
}


def get_orbit_elements():
    return ORBIT_CONFIG


# ==========================================================
# PANEL CONFIG
# ==========================================================

PANEL_CONFIG = {
    "length": 0.10,
    "width": 0.05,
    "angle_deg": 45.0
}


def get_panel_angle_rad():
    import numpy as np
    return np.deg2rad(PANEL_CONFIG["angle_deg"])


def get_panel_dimensions():
    return PANEL_CONFIG["length"], PANEL_CONFIG["width"]


# ==========================================================
# ACTUATOR CONFIG
# ==========================================================

ACTUATOR_CONFIG = {
    "max_dipole": 0.065
}


def get_max_dipole():
    return [0.065, 0.065, 0.065]

def get_bdot_gain():
    return -1.0

# ==========================================================
# DISTURBANCE CONFIG
# ==========================================================

DISTURBANCE_CONFIG = {
    "residual_dipole": [0.001, -0.001, 0.001]
}


def get_residual_dipole():
    return DISTURBANCE_CONFIG["residual_dipole"]