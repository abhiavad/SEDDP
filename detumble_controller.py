import numpy as np

from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel

from adcs_config import get_control_scale


class DetumbleController(sysModel.SysModel):

    def __init__(self, I_matrix, k=1.0):
        super().__init__()

        self.ModelTag = "detumbleController"

        self.navStateInMsg = messaging.NavAttMsgReader()
        self.actuateInMsg = messaging.SwDataMsgReader()

        self.cmdTorqueOutMsg = messaging.CmdTorqueBodyMsg()

        self.I = np.asarray(I_matrix)
        self.k = np.clip(k, 0.0, 1.0)

    def UpdateState(self, CurrentSimNanos):

        # --------------------------
        # ACTUATION TRIGGER
        # --------------------------
        actuate = int(self.actuateInMsg().dataValue) if self.actuateInMsg.isWritten() else 0

        if actuate != 1:
            payload = messaging.CmdTorqueBodyMsgPayload()
            payload.torqueRequestBody = [0.0, 0.0, 0.0]
            payload.timeTag = CurrentSimNanos
            self.cmdTorqueOutMsg.write(payload, CurrentSimNanos)
            return

        tau = np.zeros(3)
        timeTag = CurrentSimNanos

        # --------------------------
        # Read inputs
        # --------------------------
        if self.navStateInMsg.isWritten():
            nav = self.navStateInMsg()

            omega = np.asarray(nav.omega_BN_B)

            # Angular momentum
            L = self.I @ omega

            # --------------------------
            # TIME-AGNOSTIC CONTROL LAW
            # --------------------------
            tau = -self.k * L

            # --------------------------
            # DUTY-CYCLE SCALING
            # --------------------------
            tau *= get_control_scale()

            # Preserve timing if available
            if hasattr(nav, "timeTag"):
                timeTag = nav.timeTag

        # --------------------------
        # Output
        # --------------------------
        payload = messaging.CmdTorqueBodyMsgPayload()
        payload.torqueRequestBody = tau.tolist()
        payload.timeTag = timeTag

        self.cmdTorqueOutMsg.write(payload, CurrentSimNanos)