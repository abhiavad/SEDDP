import numpy as np
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk
from adcs_config import get_fsw_dt


class RateEstimator(sysModel.SysModel):
    """
    Physically consistent angular rate estimator from MRP history.
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "RateEstimator"
        self.dt = get_fsw_dt()

        # Inputs
        self.attInMsg = messaging.NavAttMsgReader()
        self.modeInMsg = messaging.SwDataMsgReader()

        # Output
        self.rateOutMsg = messaging.NavAttMsg()

        # Config
        self.bufferSize = 3
        self.sigmaBuffer = []
        self.timeBuffer = []
        self.prevMode = None
        self.I3 = np.eye(3)
        self.lastOmega = np.zeros(3)
    def Reset(self, CurrentSimNanos):
        self.dt = get_fsw_dt()

    def _skew(self, v):
        return np.array([
            [0.0, -v[2],  v[1]],
            [v[2],  0.0, -v[0]],
            [-v[1], v[0], 0.0]
        ])

    def _B_matrix(self, sigma):
        s2 = np.dot(sigma, sigma)
        return (1 - s2) * self.I3 + 2 * self._skew(sigma) + 2 * np.outer(sigma, sigma)

    def _shadow_if_needed(self, sigma):
        s2 = np.dot(sigma, sigma)
        return -sigma / s2 if s2 > 1.0 else sigma

    def UpdateState(self, CurrentSimNanos):
        # if mode ==1:
        #     return

        if not self.modeInMsg.isWritten() or not self.attInMsg.isWritten():
            payload = messaging.NavAttMsgPayload()

            if self.sigmaBuffer:
                payload.sigma_BN = self.sigmaBuffer[-1].tolist()
            else:
                payload.sigma_BN = [0.0, 0.0, 0.0]

            payload.omega_BN_B = [0.0, 0.0, 0.0]
            payload.timeTag = CurrentSimNanos

            self.rateOutMsg.write(payload, CurrentSimNanos)
            return

        mode = int(self.modeInMsg().dataValue)

        # Detect sensing start
        if mode == 1 and self.prevMode != 1:
            self.sigmaBuffer = []
            self.timeBuffer = []

        self.prevMode = mode

        att = self.attInMsg()

        # -----------------------------
        # ACTUATION PHASE → HOLD STATE
        # -----------------------------
        if mode != 1:
            payload = messaging.NavAttMsgPayload()

            if self.sigmaBuffer:
                payload.sigma_BN = self.sigmaBuffer[-1].tolist()
            else:
                payload.sigma_BN = [0.0, 0.0, 0.0]

            payload.omega_BN_B = self.lastOmega.tolist()

            if hasattr(att, "timeTag"):
                payload.timeTag = att.timeTag
            else:
                payload.timeTag = CurrentSimNanos

            self.rateOutMsg.write(payload, CurrentSimNanos)
            return

        # -----------------------------
        # SENSING PHASE
        # -----------------------------
        omega = np.zeros(3)

        sigma = np.asarray(att.sigma_BN)
        sigma = self._shadow_if_needed(sigma)

        t = CurrentSimNanos * 1e-9

        # Store buffer
        self.sigmaBuffer.append(sigma)
        self.timeBuffer.append(t)

        if len(self.sigmaBuffer) > self.bufferSize:
            self.sigmaBuffer.pop(0)
            self.timeBuffer.pop(0)

        # Not enough data → output zero rate
        if len(self.sigmaBuffer) < self.bufferSize:
            payload = messaging.NavAttMsgPayload()
            payload.sigma_BN = sigma.tolist()
            payload.omega_BN_B = self.lastOmega.tolist()

            if hasattr(att, "timeTag"):
                payload.timeTag = att.timeTag
            else:
                payload.timeTag = CurrentSimNanos

            self.rateOutMsg.write(payload, CurrentSimNanos)
            return

        # -----------------------------
        # RATE ESTIMATION (LS BLOCK)
        # ----------------------------

        t_array = np.asarray(self.timeBuffer)

        # Enforce valid monotonic time, fallback to config dt
        if np.any(np.diff(t_array) <= 1e-9):
            dt = self.dt * (len(t_array) - 1)
        else:
            dt = t_array[-1] - t_array[0]

        if dt <= 1e-9:
            dt = self.dt * (len(t_array) - 1)

        sigma_ref = self.sigmaBuffer[-1]

        sigma_rel_array = np.array([
            self._shadow_if_needed(rbk.subMRP(s, sigma_ref))
            for s in self.sigmaBuffer
        ])

        sigma_dot = (sigma_rel_array[-1] - sigma_rel_array[0]) / dt

        B = self._B_matrix(sigma_ref)

        try:
            omega = 4.0 * np.linalg.solve(B, sigma_dot)
        except np.linalg.LinAlgError:
            omega = np.zeros(3)

        self.lastOmega = omega.copy()          

        # -----------------------------
        # OUTPUT
        # -----------------------------
        payload = messaging.NavAttMsgPayload()
        payload.sigma_BN = sigma.tolist()
        payload.omega_BN_B = omega.tolist()

        if hasattr(att, "timeTag"):
            payload.timeTag = att.timeTag
        else:
            payload.timeTag = CurrentSimNanos

        self.rateOutMsg.write(payload, CurrentSimNanos)