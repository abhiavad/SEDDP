from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel


class DipoleSelector(sysModel.SysModel):

    def __init__(self):
        super().__init__()
        self.ModelTag = "DipoleSelector"

        # Inputs
        self.bdotDipoleInMsg = messaging.DipoleRequestBodyMsgReader()
        self.detumbleDipoleInMsg = messaging.DipoleRequestBodyMsgReader()
        self.pdDipoleInMsg = messaging.DipoleRequestBodyMsgReader()

        self.modeTypeInMsg = messaging.SwDataMsgReader()

        # Output
        self.dipoleOutMsg = messaging.DipoleRequestBodyMsg()

    def UpdateState(self, CurrentSimNanos):

        modeType = int(self.modeTypeInMsg().dataValue) if self.modeTypeInMsg.isWritten() else 0

        dipole = [0.0, 0.0, 0.0]
        timeTag = CurrentSimNanos

        if modeType == 0 and self.bdotDipoleInMsg.isWritten():
            msg = self.bdotDipoleInMsg()
            dipole = list(msg.dipole_B)

        elif modeType == 1 and self.detumbleDipoleInMsg.isWritten():
            msg = self.detumbleDipoleInMsg()
            dipole = list(msg.dipole_B)

        elif modeType == 2:
            if self.pdDipoleInMsg.isLinked() and self.pdDipoleInMsg.isWritten():
                msg = self.pdDipoleInMsg()
                dipole = msg.dipole_B
            else:
                dipole = [0.0, 0.0, 0.0]

        payload = messaging.DipoleRequestBodyMsgPayload()
        payload.dipole_B = dipole
        payload.timeTag = timeTag

        self.dipoleOutMsg.write(payload, CurrentSimNanos)