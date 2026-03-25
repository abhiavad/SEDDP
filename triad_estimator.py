import numpy as np
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk


class TriadEstimator(sysModel.SysModel):
    """
    TRIAD attitude determination (FSW)
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "TriadEstimator"

        # Inputs
        self.b1InMsg = messaging.TAMSensorBodyMsgReader()
        self.b2InMsg = messaging.BodyHeadingMsgReader()
        self.r1InMsg = messaging.BodyHeadingMsgReader()
        self.r2InMsg = messaging.BodyHeadingMsgReader()
        self.modeInMsg = messaging.SwDataMsgReader()

        # Output
        self.attOutMsg = messaging.NavAttMsg()

        # State
        self.lastSigma = np.array([0.0, 0.0, 0.0])
        self.lastTimeTag = 0

    def _normalize(self, v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-10 else None

    def UpdateState(self, CurrentSimNanos):

        # Validate message availability
        if not all([
            self.modeInMsg.isWritten(),
            self.b1InMsg.isWritten(),
            self.b2InMsg.isWritten(),
            self.r1InMsg.isWritten(),
            self.r2InMsg.isWritten()
        ]):
            self._write_last(self._fallback_time(CurrentSimNanos))
            return

        mode = int(self.modeInMsg().dataValue)

        # Read messages
        b1_msg = self.b1InMsg()
        b2_msg = self.b2InMsg()
        r1_msg = self.r1InMsg()
        r2_msg = self.r2InMsg()

        # Extract timestamps
        msgs = (b1_msg, b2_msg, r1_msg, r2_msg)
        t_list = []

        for msg in msgs:
            if hasattr(msg, "timeTag"):
                t_list.append(msg.timeTag)
            else:
                t_list.append(CurrentSimNanos)

        t_measurement = max(t_list)

        # If not in sensing mode → hold last
        if mode != 1:
            self._write_last(self._fallback_time(CurrentSimNanos))
            return

        # Normalize vectors
        b1 = self._normalize(np.asarray(b1_msg.tam_B))
        b2 = self._normalize(np.asarray(b2_msg.rHat_XB_B))
        r1 = self._normalize(np.asarray(r1_msg.rHat_XB_B))
        r2 = self._normalize(np.asarray(r2_msg.rHat_XB_B))

        if any(v is None for v in [b1, b2, r1, r2]):
            self._write_last(self._fallback_time(CurrentSimNanos))
            return

        # Check vector independence (restored threshold)
        if (np.dot(np.cross(b1, b2), np.cross(b1, b2)) < 1e-12 or
            np.dot(np.cross(r1, r2), np.cross(r1, r2)) < 1e-12):
            self._write_last(self._fallback_time(CurrentSimNanos))
            return

        # Construct TRIAD frames
        t1_B = b1
        t2_B = self._normalize(np.cross(b1, b2))
        t3_B = (np.cross(t1_B, t2_B))

        t1_N = r1
        t2_N = self._normalize(np.cross(r1, r2))
        t3_N = (np.cross(t1_N, t2_N))

        if any(v is None for v in [t2_B, t2_N, t3_B, t3_N]):
            self._write_last(self._fallback_time(CurrentSimNanos))
            return

        # Construct DCM
        M_B = np.column_stack((t1_B, t2_B, t3_B))
        M_N = np.column_stack((t1_N, t2_N, t3_N))
        C_BN = M_B @ M_N.T

        # Re-orthonormalize
        t1 = C_BN[:,0]
        t2 = C_BN[:,1]

        inv_norm = 1.0 / np.linalg.norm(t1)
        t1 = t1 * inv_norm

        t2 = t2 - t1*np.dot(t1, t2)
        t2 = t2 / np.linalg.norm(t2)
        t3 = np.cross(t1,t2)

        C_BN = np.column_stack((t1,t2,t3))

        # Convert to MRPs
        sigma_BN = rbk.C2MRP(C_BN)
        self.lastSigma = rbk.MRPswitch(sigma_BN, 1.0)

        # Update time memory
        self.lastTimeTag = t_measurement

        # Output
        payload = messaging.NavAttMsgPayload()
        payload.sigma_BN = self.lastSigma.tolist()
        payload.omega_BN_B = [0.0, 0.0, 0.0]
        payload.timeTag = t_measurement

        self.attOutMsg.write(payload, t_measurement)

    def _fallback_time(self, CurrentSimNanos):
        return self.lastTimeTag if self.lastTimeTag > 0 else CurrentSimNanos

    def _write_last(self, timeTag):
        payload = messaging.NavAttMsgPayload()
        payload.sigma_BN = self.lastSigma.tolist()
        payload.omega_BN_B = [0.0, 0.0, 0.0]
        payload.timeTag = timeTag
        self.attOutMsg.write(payload, timeTag)