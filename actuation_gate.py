from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel


class ActuationGate(sysModel.SysModel):
    """
    ActuationGate

    Gates magnetorquer dipole commands based on scheduler state.

    Responsibilities
    ----------------
    • Allow actuation only during control window (mode == 2)
    • Allow BDOT and DETUMBLE modes
    • Disable actuation during sensing/idle phases
    • Prevent stale commands from leaking between phases
    • Remain step-size agnostic (no dependence on timestep)
    • Remain lightweight for fast long simulations
    """

    def __init__(self):
        super().__init__()
        self.ModelTag = "ActuationGate"

        # -----------------------------
        # INPUT MESSAGES
        # -----------------------------

        # Dipole command from controller (BDOT / DETUMBLE)
        self.dipoleInMsg = messaging.DipoleRequestBodyMsgReader()

        # Time-based scheduler phase
        self.modeInMsg = messaging.SwDataMsgReader()

        # State-based controller mode
        self.modeTypeInMsg = messaging.SwDataMsgReader()

        # -----------------------------
        # OUTPUT MESSAGE
        # -----------------------------

        # Dipole sent to MTB effector
        self.dipoleOutMsg = messaging.DipoleRequestBodyMsg()

        # Cached dipole to allow holding commands if a controller step is skipped
        self.lastDipole = [0.0, 0.0, 0.0]

        # Preallocated zero vector for speed
        self.zeroDipole = [0.0, 0.0, 0.0]

    def UpdateState(self, CurrentSimNanos):

        dipole = list(self.lastDipole)
        timeTag = CurrentSimNanos

        # -----------------------------
        # SAFE MESSAGE READS
        # -----------------------------
        mode = int(self.modeInMsg().dataValue) if self.modeInMsg.isWritten() else 0
        modeType = int(self.modeTypeInMsg().dataValue) if self.modeTypeInMsg.isWritten() else 0

        # -----------------------------
        # CONTROL WINDOW
        # -----------------------------
        if mode == 2:

            # Allow actuation only for BDOT or DETUMBLE
            if modeType in (0, 1):

                if self.dipoleInMsg.isWritten():

                    msg = self.dipoleInMsg()

                    # Copy to avoid referencing Basilisk message memory
                    dipole = list(msg.dipole_B)

                    # Store for potential hold if controller skips a step
                    self.lastDipole = dipole

                    if hasattr(msg, "timeTag"):
                        timeTag = msg.timeTag

                else:
                    # Hold last command if controller update missed a step
                    dipole = self.lastDipole

            else:
                # PD or other modes disable magnetorquers
                self.lastDipole = self.zeroDipole
                dipole = self.zeroDipole

        # -----------------------------
        # SENSING / IDLE WINDOW
        # -----------------------------
        else:
            # Ensure sensing phases never actuate
            self.lastDipole = self.zeroDipole
            dipole = self.zeroDipole

        # -----------------------------
        # OUTPUT MESSAGE
        # -----------------------------
        payload = messaging.DipoleRequestBodyMsgPayload()
        payload.dipole_B = dipole
        payload.timeTag = timeTag

        self.dipoleOutMsg.write(payload, CurrentSimNanos)