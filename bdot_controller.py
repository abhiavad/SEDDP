import numpy as np
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel

from adcs_config import get_control_scale
from adcs_config import get_max_dipole, get_bdot_gain



class BdotController(sysModel.SysModel):

    def __init__(self):
        super().__init__()
        self.ModelTag = "BdotController"

        # Inputs
        self.bdotInMsg = messaging.BodyHeadingMsgReader()
        self.bInMsg = messaging.TAMSensorBodyMsgReader()
        self.actuateInMsg = messaging.SwDataMsgReader()

        # Output
        self.cmdDipoleOutMsg = messaging.DipoleRequestBodyMsg()

        # Controller parameters
        self.I = np.eye(3)
        
        self.max_dipole = np.array(get_max_dipole())
        self.k = -1.0*get_bdot_gain()
        self.scale = get_control_scale()

    def UpdateState(self, CurrentSimNanos):

        # --------------------------
        # ACTUATION TRIGGER
        # --------------------------
        actuate = int(self.actuateInMsg().dataValue) if self.actuateInMsg.isWritten() else 0

        if actuate != 1:
            payload = messaging.DipoleRequestBodyMsgPayload()
            payload.dipole_B = [0.0, 0.0, 0.0]
            self.cmdDipoleOutMsg.write(payload, CurrentSimNanos)
            return

        # --------------------------
        # Validate inputs
        # --------------------------
        if not self.bInMsg.isWritten() or not self.bdotInMsg.isWritten():
            payload = messaging.DipoleRequestBodyMsgPayload()
            payload.dipole_B = [0.0, 0.0, 0.0]
            self.cmdDipoleOutMsg.write(payload, CurrentSimNanos)
            return

        # --------------------------
        # Read messages
        # --------------------------
        bMsg = self.bInMsg()
        bdotMsg = self.bdotInMsg()

        B = np.array(bMsg.tam_B)
        Bdot = np.array(bdotMsg.rHat_XB_B)

        B_norm_sq = np.dot(B, B)

        # --------------------------
        # Control law
        # --------------------------
        if B_norm_sq < 1e-12:
            m = np.zeros(3)
        else:
            m =  self.k * (self.I @ Bdot) / B_norm_sq

            # duty-cycle scaling (centralized)
            #m *= self.scale

            # saturation
            ratios = np.abs(m) / self.max_dipole
            max_ratio = np.max(ratios)

            if max_ratio > 1.0:
                m = m / max_ratio

        # --------------------------
        # Output
        # --------------------------
        payload = messaging.DipoleRequestBodyMsgPayload()
        payload.dipole_B = m.tolist()
        self.cmdDipoleOutMsg.write(payload, CurrentSimNanos)