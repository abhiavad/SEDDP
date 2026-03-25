# adcs_state.py

_lastOmega = 0.0

def set_last_omega(omega):
    global _lastOmega
    _lastOmega = omega

def get_last_omega():
    return _lastOmega