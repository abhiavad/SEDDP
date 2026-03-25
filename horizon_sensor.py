import numpy as np
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk

class HorizonSensor(sysModel.SysModel):
    """
    Horizon sensor (minimal)
    Output: nadir_B (unit vector in body frame)
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "HorizonSensor"

        # Inputs
        self.stateInMsg = messaging.SCStatesMsgReader()
        self.attInMsg   = messaging.NavAttMsgReader()

        # Output - FIX 1: Closed the parenthesis
        self.nadirOutMsg = messaging.BodyHeadingMsg()

        # Noise toggle
        self.useNoise = False
        self.bias = np.zeros(3)
        self.noiseStd = np.zeros(3)

    def UpdateState(self, CurrentSimNanos):
        # FIX 2: Added validity check for input messages
        if not self.stateInMsg.isWritten() or not self.attInMsg.isWritten():
            return

        state = self.stateInMsg()
        att   = self.attInMsg()

        # Get position and normalize for Nadir (N-frame)
        r_N = np.asarray(state.r_BN_N).reshape(3,1)
        r_norm = np.linalg.norm(r_N)
        
        if r_norm < 1e-10:
            nadir_N = np.array([0, 0, 0])
        else:
            nadir_N = -r_N / r_norm

        # Rotate Nadir to Body frame
        sigma_BN = np.asarray(att.sigma_BN)
        C_BN = rbk.MRP2C(sigma_BN)
        nadir_B = (C_BN @ nadir_N).flatten()

        # Optional noise
        if self.useNoise:
            nadir_B += self.bias + np.random.normal(0, self.noiseStd, 3)

        # Final normalization
        norm = np.linalg.norm(nadir_B)
        if norm > 1e-10:
            nadir_B = nadir_B / norm
        else:
            nadir_B = np.zeros(3)

        # FIX 3: Correct payload attribute name for BodyHeadingMsg
        payload = messaging.BodyHeadingMsgPayload()
        payload.rHat_XB_B = nadir_B.tolist() 
        payload.timeTag = CurrentSimNanos
        
        self.nadirOutMsg.write(payload, CurrentSimNanos)