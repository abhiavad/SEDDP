import numpy as np
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from adcs_config import get_fsw_dt


class BdotPredictor(sysModel.SysModel):

    def __init__(self):
        super().__init__()
        self.dt = get_fsw_dt()
        self.ModelTag = "BdotPredictor"

        self.bInMsg = messaging.TAMSensorBodyMsgReader()
        self.modeInMsg = messaging.SwDataMsgReader()

        self.bOutMsg = messaging.TAMSensorBodyMsg()
        self.bdotOutMsg = messaging.BodyHeadingMsg()

        self.prevMode = None

        # Buffers (fixed size = 3)
        self.B_buffer = []
        self.t_buffer = []

        self.Bdot_est = np.zeros(3)

    def Reset(self, CurrentSimNanos):
        self.dt = get_fsw_dt()
        
    def UpdateState(self, CurrentSimNanos):

        if not self.bInMsg.isWritten() or not self.modeInMsg.isWritten():
            return

        mode = int(self.modeInMsg().dataValue)

        bMsg = self.bInMsg()
        B = np.asarray(bMsg.tam_B)
        # Use simulation time only (consistent with scheduler)
        t = CurrentSimNanos * 1e-9

        # --------------------------------------
        # START OF SENSING → RESET BUFFERS
        # --------------------------------------
        if mode == 1 and self.prevMode != 1:
            self.B_buffer = []
            self.t_buffer = []
            self.Bdot_est = np.zeros(3)

        # --------------------------------------
        # SENSING PHASE → COLLECT DATA
        # --------------------------------------
        if mode == 1:
            self.B_buffer.append(B.copy())
            self.t_buffer.append(t)

            if len(self.B_buffer) > 3:
                self.B_buffer.pop(0)
                self.t_buffer.pop(0)

            # Compute Bdot ONLY when 3 samples available
            if len(self.B_buffer) == 3:
                B1, B2, B3 = self.B_buffer
                t1, t2, t3 = self.t_buffer

                dt21 = t2 - t1
                dt32 = t3 - t2

                # Fallback to configured dt if timestamps are inconsistent
                if dt21 <= 1e-9:
                    dt21 = self.dt
                if dt32 <= 1e-9:
                    dt32 = self.dt

                Bdot1 = (B2 - B1) / dt21
                Bdot2 = (B3 - B2) / dt32
                self.Bdot_est = 0.5 * (Bdot1 + Bdot2)

        # --------------------------------------
        # OUTPUT (HOLD LAST VALUES)
        # --------------------------------------
        if len(self.B_buffer) > 0:
            B_out = self.B_buffer[-1]
        else:
            B_out = B

        payload = messaging.TAMSensorBodyMsgPayload()
        payload.tam_B = B_out.tolist()
        payload.timeTag = CurrentSimNanos
        self.bOutMsg.write(payload, CurrentSimNanos)

        bdot_payload = messaging.BodyHeadingMsgPayload()
        bdot_payload.rHat_XB_B = self.Bdot_est.tolist()
        self.bdotOutMsg.write(bdot_payload, CurrentSimNanos)

        self.prevMode = mode