import numpy as np

from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.utilities import RigidBodyKinematics as rbk


class AeroTorqueCalculator(sysModel.SysModel):

    def __init__(self, facets):
        super().__init__()
        self.ModelTag = "AeroTorqueCalc"

        self.atmoInMsg = messaging.AtmoPropsMsgReader()
        self.stateInMsg = messaging.SCStatesMsgReader()
        self.attInMsg   = messaging.NavAttMsgReader()
        self.facets = facets

        self.torqueOutMsg = messaging.CmdTorqueBodyMsg()

    def UpdateState(self, CurrentSimNanos):

        atmo = self.atmoInMsg()
        state = self.stateInMsg()
        att   = self.attInMsg()

        rho = atmo.neutralDensity

        v_N = np.asarray(state.v_BN_N).reshape(3,1)

        sigma_BN = np.asarray(att.sigma_BN)
        C_BN = rbk.MRP2C(sigma_BN)

        v_B = C_BN @ v_N
        v_B = v_B.flatten()
        v_norm = np.linalg.norm(v_B)

        if v_norm < 1e-10:
            # maintain message consistency (recommended fix)
            payload = messaging.CmdTorqueBodyMsgPayload()
            payload.torqueRequestBody = [0.0, 0.0, 0.0]
            self.torqueOutMsg.write(payload, CurrentSimNanos)
            return

        v_hat = v_B / v_norm

        tau_total = np.zeros(3)

        for f in self.facets:
            A = f["area"]
            Cd = f["Cd"]
            n = f["normal"]
            r = f["location"]

            cos_theta = np.dot(n, -v_hat)

            if f["two_sided"]:
                cos_eff = abs(cos_theta)
                active = cos_eff > 0
            else:
                cos_eff = cos_theta
                active = cos_theta > 0

            if active:
                F_mag = 0.5 * rho * v_norm**2 * Cd * A * cos_eff
                F = F_mag * (-v_hat)

                tau_total += np.cross(r, F)

        payload = messaging.CmdTorqueBodyMsgPayload()
        payload.torqueRequestBody = tau_total.tolist()

        self.torqueOutMsg.write(payload, CurrentSimNanos)