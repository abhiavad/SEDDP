#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

# ===============================
# CONTROLLER SELECTION
# ===============================

from adcs_config import (
    get_controller_mode,
    get_fsw_dt,
    get_dimensions,
    get_mass,
    get_inertia_matrix,
    get_initial_omega_rad,
    get_residual_dipole,
    get_max_dipole,
    get_panel_angle_rad,
    get_orbit_elements
)

import os

import csv 

import numpy as np

import matplotlib.pyplot as plt

#import general simulation support files

from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    unitTestSupport,
    # vizSupport,
)

#import simulation related support
from Basilisk.simulation import (
    spacecraft,
    extForceTorque,
    simpleNav,
    magneticFieldWMM,
    magnetometer,
    MtbEffector,
    GravityGradientEffector,
    exponentialAtmosphere,
    facetDragDynamicEffector,

)

#import FSW Algo related support, might need to be rewritten
from Basilisk.fswAlgorithms import(
    mrpFeedback,
    inertial3D,
    attTrackingError,
    tamComm,
    torque2Dipole,
    dipoleMapping,
    # mtbFeedforward
)

#import message declarations
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel 
from Basilisk.utilities import RigidBodyKinematics as rbk

#get location of supporting data
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
 
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile
from detumble_controller import DetumbleController
from aero_torque_calculator import AeroTorqueCalculator
from magnetic_disturbance import ResidualMagneticDipoleTorque
from plot_orbits import plotOrbits
from horizon_sensor import HorizonSensor
from horizon_comm import HorizonComm
from triad_estimator import TriadEstimator
from rate_estimator import RateEstimator
from mode_scheduler import ModeScheduler
from bdot_predictor import BdotPredictor
from bdot_controller import BdotController
from dipoleselector import DipoleSelector

# ===============================
# CONFIGURATION SWITCHES
# ===============================

USE_TRIAD = True              # Switch between SimpleNav and TRIAD
ENABLE_SENSOR_NOISE = True   # Toggle noise (future use)

class ReferenceVectorProvider(sysModel.SysModel):
    """
    Provides inertial reference vectors
    """

    def __init__(self):
        super().__init__()

        self.stateInMsg = messaging.SCStatesMsgReader()
        self.magInMsg = messaging.MagneticFieldMsgReader()

        # FIX 1: Change GenericVectorMsg to BodyHeadingMsg
        self.magOutMsg = messaging.BodyHeadingMsg()
        self.nadirOutMsg = messaging.BodyHeadingMsg()

    def UpdateState(self, CurrentSimNanos):
        # FIX 2: Add validity checks to prevent reading empty messages
        if not self.stateInMsg.isWritten() or not self.magInMsg.isWritten():
            return

        state = self.stateInMsg()
        mag = self.magInMsg()

        r_N = np.asarray(state.r_BN_N)
        B_N = np.asarray(mag.magField_N)

        # Calculate Nadir Reference (N-frame)
        r_norm = np.linalg.norm(r_N)
        if r_norm > 1e-10:
            nadir_N = -r_N / r_norm
        else:
            nadir_N = np.zeros(3)

        # Write magnetic field reference
        p1 = messaging.BodyHeadingMsgPayload()
        B_norm = np.linalg.norm(B_N)
        if B_norm > 1e-12:
            p1.rHat_XB_B = B_N / B_norm
        else:
            p1.rHat_XB_B = np.zeros(3)


        self.magOutMsg.write(p1, CurrentSimNanos)

        # Write nadir reference
        p2 = messaging.BodyHeadingMsgPayload()
        p2.rHat_XB_B = nadir_N
        self.nadirOutMsg.write(p2, CurrentSimNanos)

class DipoleQuantizer(sysModel.SysModel):
    """
    Intercepts continuous MTB commands and quantizes them into discrete steps.
    """
    def __init__(self, step_percentage=0.05, max_dipole=0.02):
        super().__init__()
        
        # Inputs and Outputs
        self.mtbCmdInMsg = messaging.MTBCmdMsgReader()
        self.mtbCmdOutMsg = messaging.MTBCmdMsg()
        
        # Discretization parameters
        self.max_dipole = np.asarray(max_dipole, dtype = float)
        self.step_size = self.max_dipole * step_percentage

    def UpdateState(self, CurrentSimNanos):
        # Pass zeros if the input message hasn't been written yet
        if not self.mtbCmdInMsg.isWritten():
            return
            
        in_msg = self.mtbCmdInMsg()
        dipole_cmds = np.array(in_msg.mtbDipoleCmds, dtype=float)

        # Ensure shape is (N,3)
        dipole_cmds = dipole_cmds.reshape(-1, 3)

        quantized_cmds = np.round(dipole_cmds / self.step_size) * self.step_size
        quantized_cmds = np.clip(quantized_cmds, -self.max_dipole, self.max_dipole)

        # Flatten back to original Basilisk format
        quantized_cmds = quantized_cmds.reshape(-1)
        # Write the quantized commands to the output message
        out_msg = messaging.MTBCmdMsgPayload()
        out_msg.mtbDipoleCmds = quantized_cmds.tolist()
        
        self.mtbCmdOutMsg.write(out_msg, CurrentSimNanos)

def run():
    """
    Main ADCS simulation scenario.

    Structure:
        1) Simulation + timing
        2) Dynamics models
        3) Environment models
        4) Sensors
        5) FSW estimation
        6) FSW control
        7) Logging
    """

    # =====================================================
    # SIMULATION + PROCESS SETUP
    # =====================================================

    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    fswProcessName = "fswProcess"
    fswCoreTask = "fswCoreTask"

    scSim = SimulationBaseClass.SimBaseClass()
    scSim.SetProgressBar(True)

    dynProcess = scSim.CreateNewProcess(dynProcessName)
    fswProcess = scSim.CreateNewProcess(fswProcessName)

    simulationTime = macros.sec2nano(3*86400.0)

    simStepTime = macros.sec2nano(0.02)
    fswStepTime = macros.sec2nano(get_fsw_dt())

    dynsamplingTime = simStepTime
    fswsamplingTime = fswStepTime

    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simStepTime))
    fswProcess.addTask(scSim.CreateNewTask(fswCoreTask, fswStepTime))
    # fswProcess.addTask(scSim.CreateNewTask(senseTaskName, fswStepTime))
    # fswProcess.addTask(scSim.CreateNewTask(fswCoreTask, fswStepTime))

    # =====================================================
    # SPACECRAFT PHYSICAL PARAMETERS
    # =====================================================

    # spacecraft dimensions [m]
    dims = get_dimensions()
    Lx, Ly, Lz = dims["Lx"], dims["Ly"], dims["Lz"]

    m = get_mass()

    I_matrix = np.array(get_inertia_matrix())
    Ix, Iy, Iz = I_matrix[0,0], I_matrix[1,1], I_matrix[2,2]

    omega0 = get_initial_omega_rad()

    m_residual_B = np.array(get_residual_dipole()).reshape(3,1)

    maxDipole = get_max_dipole()

    # cfg = get_config()

    # actuation_time = cfg["control_end"] - cfg["control_start"]
    # rateEstimator.timeWindow = cfg["sense_end"]
    # =====================================================
    # SPACECRAFT DYNAMICS
    # =====================================================

    delfi = spacecraft.Spacecraft()
    scSim.AddModelToTask(dynTaskName, delfi)

    # =====================================================
    # ENVIRONMENT MODELS
    # =====================================================

    atmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmo.addSpacecraftToModel(delfi.scStateOutMsg)
    scSim.AddModelToTask(dynTaskName, atmo)

    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.addSpacecraftToModel(delfi.scStateOutMsg)
    scSim.AddModelToTask(dynTaskName, magModule)

    # =====================================================
    # DISTURBANCE / DYNAMICS EFFECTORS
    # =====================================================

    dragEff = facetDragDynamicEffector.FacetDragDynamicEffector()
    scSim.AddModelToTask(dynTaskName, dragEff)

    facets = []


    # =====================================================
    # CUBOID BODY FACES
    # =====================================================
    
    A_x = Ly * Lz
    A_y = Lx * Lz
    A_z = Lx * Ly

    faces = [

        (A_x, [1,0,0],  [Lx/2,0,0]),
        (A_x, [-1,0,0], [-Lx/2,0,0]),

        (A_y, [0,1,0],  [0,Ly/2,0]),
        (A_y, [0,-1,0], [0,-Ly/2,0]),

        (A_z, [0,0,1],  [0,0,Lz/2]),
        (A_z, [0,0,-1], [0,0,-Lz/2]),
    ]

    Cd = 2.2
    for area, normal, location in faces:
        facet = {
            "area": area,
            "Cd": Cd,
            "normal": np.array(normal),
            "location": np.array(location),
            "two_sided": False
        }

        facets.append(facet)

        dragEff.addFacet(
            area,
            Cd,
            facet["normal"],
            facet["location"]
        )


    # =====================================================
    # SOLAR PANELS
    # =====================================================

    Lp = 0.10
    Wp = 0.05

    A_panel = Lp * Wp

    theta = get_panel_angle_rad()


    # panel normals

    n_panel_plusY  = np.array([ np.sin(theta),  np.cos(theta), 0.0])
    n_panel_minusY = np.array([ np.sin(theta), -np.cos(theta), 0.0])

    n_panel_plusZ  = np.array([ np.sin(theta), 0.0,  np.cos(theta)])
    n_panel_minusZ = np.array([ np.sin(theta), 0.0, -np.cos(theta)])


    # hinge geometry

    t_plusY  = np.array([-np.cos(theta),  np.sin(theta), 0.0])
    t_minusY = np.array([-np.cos(theta), -np.sin(theta), 0.0])

    t_plusZ  = np.array([-np.cos(theta), 0.0,  np.sin(theta)])
    t_minusZ = np.array([-np.cos(theta), 0.0, -np.sin(theta)])


    r_hinge_plusY  = np.array([-Lx/2,  Ly/2,  0.0])
    r_hinge_minusY = np.array([-Lx/2, -Ly/2,  0.0])

    r_hinge_plusZ  = np.array([-Lx/2, 0.0,  Lz/2])
    r_hinge_minusZ = np.array([-Lx/2, 0.0, -Lz/2])


    r_cp_plusY  = r_hinge_plusY  + (Lp/2) * t_plusY
    r_cp_minusY = r_hinge_minusY + (Lp/2) * t_minusY

    r_cp_plusZ  = r_hinge_plusZ  + (Lp/2) * t_plusZ
    r_cp_minusZ = r_hinge_minusZ + (Lp/2) * t_minusZ


    panel_facets = [

        (n_panel_plusY,  r_cp_plusY),
        (n_panel_minusY, r_cp_minusY),

        (n_panel_plusZ,  r_cp_plusZ),
        (n_panel_minusZ, r_cp_minusZ),
    ]


    for normal, location in panel_facets:

        facet = {
            "area": A_panel,
            "Cd": Cd,
            "normal": normal,
            "location": location,
            "two_sided": True
        }

        facets.append(facet)

        dragEff.addFacet(
            A_panel,
            Cd,
            normal,
            location
        )


    aeroTorque = AeroTorqueCalculator(facets)
    scSim.AddModelToTask(dynTaskName, aeroTorque)

    aeroTorqueEff = extForceTorque.ExtForceTorque()
    delfi.addDynamicEffector(aeroTorqueEff)
    scSim.AddModelToTask(dynTaskName, aeroTorqueEff)

    mag_Dist = extForceTorque.ExtForceTorque()
    delfi.addDynamicEffector(mag_Dist)
    scSim.AddModelToTask(dynTaskName, mag_Dist)

    magDist = ResidualMagneticDipoleTorque(m_residual_B)
    scSim.AddModelToTask(dynTaskName, magDist)

    # =====================================================
    # ACTUATORS
    # =====================================================

    mtbEff = MtbEffector.MtbEffector()
    delfi.addDynamicEffector(mtbEff)
    scSim.AddModelToTask(dynTaskName, mtbEff)

    # =====================================================
    # GRAVITY
    # =====================================================

    ggEff = GravityGradientEffector.GravityGradientEffector()

    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createEarth()
    planet.isCentralBody = True

    gravFactory.addBodiesTo(delfi)

    ggEff.addPlanetName(planet.planetName)
    delfi.addDynamicEffector(ggEff)
    scSim.AddModelToTask(dynTaskName, ggEff)

    # =====================================================
    # NAVIGATION + SENSORS
    # =====================================================

    delfi_NavObj = simpleNav.SimpleNav()
    scSim.AddModelToTask(dynTaskName, delfi_NavObj)

    TAM = magnetometer.Magnetometer()
    scSim.AddModelToTask(dynTaskName, TAM)

    horizon = HorizonSensor()
    scSim.AddModelToTask(dynTaskName, horizon)

    # =====================================================
    # FSW — SENSOR INTERFACES
    # =====================================================

    tamCommObj = tamComm.tamComm()
    scSim.AddModelToTask(fswCoreTask, tamCommObj)

    horizonCommObj = HorizonComm()
    scSim.AddModelToTask(fswCoreTask, horizonCommObj)

    refObj = ReferenceVectorProvider()
    scSim.AddModelToTask(fswCoreTask, refObj)

    # =====================================================
    # FSW — ESTIMATION
    # =====================================================

    triadObj = TriadEstimator()
    scSim.AddModelToTask(fswCoreTask, triadObj)

    rateEstimator = RateEstimator()
    scSim.AddModelToTask(fswCoreTask, rateEstimator)

    # =====================================================
    # MODE SCHEDULER
    # =====================================================

    modeScheduler = ModeScheduler()
    # modeScheduler.scSim = scSim
    # modeScheduler.fswCoreTask = fswCoreTask
    # modeScheduler.fswCoreTask = fswCoreTask

    scSim.AddModelToTask(fswCoreTask, modeScheduler)

    # =====================================================
    # CONTROL PIPELINE
    # =====================================================

    bdotPredictor = BdotPredictor()
    scSim.AddModelToTask(fswCoreTask, bdotPredictor)

    detumble = DetumbleController(I_matrix, k=1e-5)
    scSim.AddModelToTask(fswCoreTask, detumble)

    detumbleT2D = torque2Dipole.torque2Dipole()
    scSim.AddModelToTask(fswCoreTask, detumbleT2D)

    bdotController = BdotController()
    scSim.AddModelToTask(fswCoreTask, bdotController)

    dipoleSelector = DipoleSelector()
    scSim.AddModelToTask(fswCoreTask, dipoleSelector)

    # actuationGate = ActuationGate()
    # scSim.AddModelToTask(fswCoreTask, actuationGate)

    dipoleMappingObj = dipoleMapping.dipoleMapping()
    scSim.AddModelToTask(fswCoreTask, dipoleMappingObj)

    quantizerObj = DipoleQuantizer(
        step_percentage=0.0005,
        max_dipole=maxDipole
    )
    scSim.AddModelToTask(fswCoreTask, quantizerObj)

    # =====================================================
    # LOGGING
    # =====================================================

    mtbTorqueLog = mtbEff.logger("torqueExternalPntB_B", dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, mtbTorqueLog)

    MagDistLog = magDist.disturbanceTorqueOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, MagDistLog)

    ggLog = ggEff.gravityGradientOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, ggLog)

    dragForceLog = dragEff.logger("forceExternal_B", dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, dragForceLog)

    rhoLog = atmo.envOutMsgs[0].recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, rhoLog)

    dataRec = delfi.scStateOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, dataRec)

    magLog = magModule.envOutMsgs[0].recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, magLog)

    tamLog = TAM.tamDataOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, tamLog)

    aeroTorqueLog = aeroTorque.torqueOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, aeroTorqueLog)

    # =====================================================
    # FSW LOGGING
    # =====================================================

    tamCommLog = tamCommObj.tamOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, tamCommLog)

    horizonLog = horizonCommObj.nadirOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, horizonLog)

    mtbDipoleCmdsLog = dipoleMappingObj.dipoleRequestMtbOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, mtbDipoleCmdsLog)

    tauLog = detumble.cmdTorqueOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, tauLog)

    modeLog = modeScheduler.modeOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, modeLog)

    modeTypeLog = modeScheduler.modeTypeOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, modeTypeLog)

    rateLog = rateEstimator.rateOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, rateLog)

    trueOmegaLog = delfi.scStateOutMsg.recorder(dynsamplingTime)
    scSim.AddModelToTask(dynTaskName, trueOmegaLog)

    bdotLog = bdotPredictor.bdotOutMsg.recorder(fswsamplingTime)
    scSim.AddModelToTask(fswCoreTask, bdotLog)
    
    # =====================================================
    # ORBIT DEFINITION
    # =====================================================

    cfg = get_orbit_elements()
    oe = orbitalMotion.ClassicElements()
    oe.a = cfg["a"]
    oe.e = cfg["e"]
    oe.i = cfg["i_deg"] * macros.D2R
    oe.Omega = cfg["Omega_deg"] * macros.D2R
    oe.omega = cfg["omega_deg"] * macros.D2R
    oe.f = cfg["f_deg"] * macros.D2R


    # =====================================================
    # GRAVITY MODEL
    # =====================================================

    ggm03s__j2_only_path = get_path(DataFile.LocalGravData.GGM03S_J2_only)

    planet.useSphericalHarmonicsGravityModel(
        str(ggm03s__j2_only_path),
        2
    )

    mu = planet.mu


    # =====================================================
    # INITIAL ORBIT STATE
    # =====================================================

    rN, vN = orbitalMotion.elem2rv(mu, oe)

    # regenerate elements for consistency
    oe = orbitalMotion.rv2elem(mu, rN, vN)


    # =====================================================
    # SPACECRAFT INITIAL CONDITIONS
    # =====================================================

    delfi.hub.r_CN_NInit = rN
    delfi.hub.v_CN_NInit = vN

    delfi.hub.mHub = m

    delfi.hub.r_BcB_B = [[0.0], [0.0], [0.0]]

    delfi.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d([
        Ix, 0., 0.,
        0., Iy, 0.,
        0., 0., Iz
    ])

    delfi.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]

    delfi.hub.omega_BN_BInit = [
        omega0,
        -omega0,
        omega0
    ]


    # =====================================================
    # ORBITAL TIME PARAMETERS
    # =====================================================

    n = np.sqrt(mu / oe.a**3)
    P = 2.0 * np.pi / n


    # =====================================================
    # ATMOSPHERIC MODEL
    # =====================================================

    # Earth radius [m]
    atmo.planetRadius = orbitalMotion.REQ_EARTH * 1e3

    # atmospheric density model
    atmo.baseDensity = 3.2e-11
    atmo.scaleHeight = 70000.0

    # altitude bounds
    atmo.envMinReach = -300e3
    atmo.envMaxReach = 1000e3


    # =====================================================
    # DRAG MODEL CONNECTIONS
    # =====================================================

    dragEff.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])

    aeroTorque.atmoInMsg.subscribeTo(atmo.envOutMsgs[0])
    aeroTorque.stateInMsg.subscribeTo(delfi.scStateOutMsg)


    # =====================================================
    # SENSOR STATE INPUTS
    # =====================================================

    horizon.stateInMsg.subscribeTo(delfi.scStateOutMsg)
    TAM.stateInMsg.subscribeTo(delfi.scStateOutMsg)
    refObj.stateInMsg.subscribeTo(delfi.scStateOutMsg)

    delfi_NavObj.scStateInMsg.subscribeTo(delfi.scStateOutMsg)

    aeroTorque.attInMsg.subscribeTo(delfi_NavObj.attOutMsg)
    horizon.attInMsg.subscribeTo(delfi_NavObj.attOutMsg)

    magDist.attNavInMsg.subscribeTo(delfi_NavObj.attOutMsg)


    # =====================================================
    # DISTURBANCE TORQUE CONNECTION
    # =====================================================

    aeroTorqueEff.cmdTorqueInMsg.subscribeTo(aeroTorque.torqueOutMsg)


    # =====================================================
    # MAGNETIC FIELD MODEL
    # =====================================================

    epochMsg = unitTestSupport.timeStringToGregorianUTCMsg(
        "2019 June 27, 10:23:0.0 (UTC)"
    )

    wmm_path = get_path(DataFile.MagneticFieldData.WMM)

    magModule.configureWMMFile(str(wmm_path))
    magModule.epochInMsg.subscribeTo(epochMsg)

    mtbEff.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    TAM.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    refObj.magInMsg.subscribeTo(magModule.envOutMsgs[0])
    magDist.magInMsg.subscribeTo(magModule.envOutMsgs[0])

    mag_Dist.cmdTorqueInMsg.subscribeTo(
        magDist.disturbanceTorqueOutMsg
    )


    # =====================================================
    # MAGNETORQUER CONFIGURATION
    # =====================================================

    mtbConfigParams = messaging.MTBArrayConfigMsgPayload()

    mtbConfigParams.numMTB = 3

    mtbConfigParams.GtMatrix_B = [
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.
    ]

    

    maxDipole_scalar = float(maxDipole[0]) if isinstance(maxDipole, (list, np.ndarray)) else float(maxDipole)

    mtbConfigParams.maxMtbDipoles = [
        maxDipole_scalar
    ] * mtbConfigParams.numMTB

    mtbParamsInMsg = messaging.MTBArrayConfigMsg().write(
        mtbConfigParams
    )

    mtbEff.mtbParamsInMsg.subscribeTo(mtbParamsInMsg)

 
    # =====================================================
    # SENSOR CONFIGURATION
    # =====================================================

    # Horizon sensor noise
    horizon.useNoise = ENABLE_SENSOR_NOISE

    # Magnetometer configuration
    TAM.scaleFactor = 1.0
    TAM.senNoiseStd = [0.0, 0.0, 0.0]


    # =====================================================
    # SENSOR → FSW INTERFACES
    # =====================================================

    # TAM communication
    tamCommObj.dcm_BS = [
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.
    ]

    tamCommObj.tamInMsg.subscribeTo(
        TAM.tamDataOutMsg
    )

    # Horizon communication
    horizonCommObj.nadirInMsg.subscribeTo(
        horizon.nadirOutMsg
    )


    # =====================================================
    # TRIAD ATTITUDE ESTIMATOR
    # =====================================================

    triadObj.b1InMsg.subscribeTo(
        tamCommObj.tamOutMsg
    )

    triadObj.b2InMsg.subscribeTo(
        horizonCommObj.nadirOutMsg
    )

    triadObj.r1InMsg.subscribeTo(
        refObj.magOutMsg
    )

    triadObj.r2InMsg.subscribeTo(
        refObj.nadirOutMsg
    )

    triadObj.modeInMsg.subscribeTo(
        modeScheduler.modeOutMsg
    )


    # =====================================================
    # RATE ESTIMATOR
    # =====================================================

    

    # rateEstimator.timeWindow = cfg["sense_end"]

    rateEstimator.attInMsg.subscribeTo(
        triadObj.attOutMsg
    )

    rateEstimator.modeInMsg.subscribeTo(
        modeScheduler.modeOutMsg
    )


    # =====================================================
    # MODE SCHEDULER INPUT
    # =====================================================

    # modeScheduler.rateInMsg.subscribeTo(rateEstimator.rateOutMsg)


    # =====================================================
    # BDOT PREDICTOR
    # =====================================================

    bdotPredictor.bInMsg.subscribeTo(
        tamCommObj.tamOutMsg
    )

    bdotPredictor.modeInMsg.subscribeTo(
        modeScheduler.modeOutMsg
    )

    # bdotPredictor.modeTypeInMsg.subscribeTo(
    #     modeScheduler.modeTypeOutMsg
    # )


    # =====================================================
    # DETUMBLE CONTROLLER
    # =====================================================

    detumble.navStateInMsg.subscribeTo(
        rateEstimator.rateOutMsg
    )


    # =====================================================
    # DETUMBLE TORQUE → DIPOLE
    # =====================================================

    detumbleT2D.tauRequestInMsg.subscribeTo(
        detumble.cmdTorqueOutMsg
    )

    detumbleT2D.tamSensorBodyInMsg.subscribeTo(
        bdotPredictor.bOutMsg
    )


    # =====================================================
    # BDOT CONTROLLER
    # =====================================================

    bdotController.bInMsg.subscribeTo(
        bdotPredictor.bOutMsg
    )

    bdotController.bdotInMsg.subscribeTo(
        bdotPredictor.bdotOutMsg
    )

    # =====================================================
    # ACTUATION TRIGGER CONNECTION (CRITICAL)
    #=====================================================

    bdotController.actuateInMsg.subscribeTo(
        modeScheduler.actuateOutMsg
    )

    detumble.actuateInMsg.subscribeTo(
        modeScheduler.actuateOutMsg
    ) 

    # bdotController.modeTypeInMsg.subscribeTo(
    #     modeScheduler.modeTypeOutMsg
    # )

    # bdotController.modeInMsg.subscribeTo(
    #     modeScheduler.modeOutMsg
    # )


    # =====================================================
    # CONTROLLER SELECTION
    # =====================================================

    dipoleSelector.modeTypeInMsg.subscribeTo(
        modeScheduler.modeTypeOutMsg
    )

    # ---------------------------------
    # SIMPLE CONTROLLER SWITCH
    # ---------------------------------

    if get_controller_mode() == 0:

        dipoleSelector.bdotDipoleInMsg.subscribeTo(
            bdotController.cmdDipoleOutMsg
        )

    elif get_controller_mode() == 1:

        dipoleSelector.detumbleDipoleInMsg.subscribeTo(
            detumbleT2D.dipoleRequestOutMsg
        )

    # elif CONTROL_MODE == "PD":

    #     dipoleSelector.pdDipoleInMsg.subscribeTo(
    #         pdTorque2Dipole.dipoleRequestOutMsg
    #     )


    # =====================================================
    # ACTUATION GATE
    # =====================================================

    # actuationGate.modeInMsg.subscribeTo(
    #     modeScheduler.modeOutMsg
    # )

    # actuationGate.modeTypeInMsg.subscribeTo(
    #     modeScheduler.modeTypeOutMsg
    # )

    # actuationGate.dipoleInMsg.subscribeTo(
    #     dipoleSelector.dipoleOutMsg
    # )


    # =====================================================
    # DIPOLE → MTB MAPPING
    # =====================================================

    dipoleMappingObj.steeringMatrix = [
        1., 0., 0.,
        0., 1., 0.,
        0., 0., 1.
    ]

    dipoleMappingObj.dipoleRequestBodyInMsg.subscribeTo(
        dipoleSelector.dipoleOutMsg
    )

    dipoleMappingObj.mtbArrayConfigParamsInMsg.subscribeTo(
        mtbParamsInMsg
    )


    # =====================================================
    # MTB QUANTIZATION
    # =====================================================

    quantizerObj.mtbCmdInMsg.subscribeTo(
        dipoleMappingObj.dipoleRequestMtbOutMsg
    )

    mtbEff.mtbCmdInMsg.subscribeTo(
        quantizerObj.mtbCmdOutMsg
    )


    # =====================================================
    # SIMULATION EXECUTION
    # =====================================================

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()


    # =====================================================
    # EXTRACT LOGGED DATA
    # =====================================================

    # FSW time base
    times = rateLog.times() * 1e-9

    mode = np.array(modeLog.dataValue)
    modeType = np.array(modeTypeLog.dataValue)

    omega = np.array(rateLog.omega_BN_B)
    omega_norm = np.linalg.norm(omega, axis=1)

    dipole = np.array(mtbDipoleCmdsLog.mtbDipoleCmds)
    dipole_norm = np.linalg.norm(dipole, axis=1)

    torque = np.array(tauLog.torqueRequestBody)

    # dynamics time base
    dynTimes = magLog.times() * 1e-9

    # magnetic field
    B_dyn = np.array(magLog.magField_N)
    B_norm_dyn = np.linalg.norm(B_dyn, axis=1)

    # BDOT estimate
    bdot = np.array(bdotLog.rHat_XB_B)
    bdot_norm = np.linalg.norm(bdot, axis=1)

    # actual MTB torque
    mtbTorque_dyn = np.array(mtbTorqueLog.torqueExternalPntB_B)

    trueOmega_dyn = np.array(trueOmegaLog.omega_BN_B)


    # =====================================================
    # RESAMPLING UTILITIES
    # =====================================================

    def resample_dyn(data_dyn):

        return np.vstack([
            np.interp(times, dynTimes, data_dyn[:,0]),
            np.interp(times, dynTimes, data_dyn[:,1]),
            np.interp(times, dynTimes, data_dyn[:,2])
        ]).T


    def resample_dyn_scalar(data_dyn):

        return np.interp(times, dynTimes, data_dyn)


    # =====================================================
    # RESAMPLE DYNAMICS DATA TO FSW TIMELINE
    # =====================================================

    B_dyn_resampled = resample_dyn(B_dyn)
    B_norm = resample_dyn_scalar(B_norm_dyn)

    mtbTorque = resample_dyn(mtbTorque_dyn)

    trueOmega = resample_dyn(trueOmega_dyn)
    trueOmega_norm = np.linalg.norm(trueOmega, axis=1)


    # =====================================================
    # BUILD DATA MATRIX
    # =====================================================

    data = np.column_stack((

        times,
        mode,
        modeType,

        omega[:,0],
        omega[:,1],
        omega[:,2],
        omega_norm,

        B_norm,

        bdot[:,0],
        bdot[:,1],
        bdot[:,2],
        bdot_norm,

        dipole[:,0],
        dipole[:,1],
        dipole[:,2],
        dipole_norm,

        torque[:,0],
        torque[:,1],
        torque[:,2],

        mtbTorque[:,0],
        mtbTorque[:,1],
        mtbTorque[:,2],

        # -----------------------------
        # TRUE OMEGA (ADD THIS BLOCK)
        # -----------------------------
        trueOmega[:,0],
        trueOmega[:,1],
        trueOmega[:,2],
        trueOmega_norm

    ))


    # =====================================================
    # CSV EXPORT
    # =====================================================

    header = (
    "time_s,"
    "mode,"
    "modeType,"
    "omega_x_rad_s,omega_y_rad_s,omega_z_rad_s,omega_norm_rad_s,"
    "B_norm_T,"
    "bdot_x_T_s,bdot_y_T_s,bdot_z_T_s,bdot_norm_T_s,"
    "dipole_x_A_m2,dipole_y_A_m2,dipole_z_A_m2,dipole_norm_A_m2,"
    "torque_x_Nm,torque_y_Nm,torque_z_Nm,"
    "mtb_torque_x_Nm,mtb_torque_y_Nm,mtb_torque_z_Nm,"
    "true_omega_x_rad_s,true_omega_y_rad_s,true_omega_z_rad_s,true_omega_norm_rad_s"
    )

    np.savetxt(
        "adcs_log.csv",
        data,
        delimiter=",",
        header=header,
        comments=""
    )


    # =====================================================
    # QUICK DIAGNOSTICS
    # =====================================================

    print("Mean density:", np.mean(rhoLog.neutralDensity))

    print(
        "Mean drag force:",
        np.mean(np.linalg.norm(dragForceLog.forceExternal_B, axis=1))
    )

    print(
        "Mean drag torque:",
        np.mean(np.linalg.norm(aeroTorqueLog.torqueRequestBody, axis=1))
    )

    print("Mode sample:", modeLog.dataValue[:20])
    print("ModeType sample:", modeTypeLog.dataValue[:20])

    omega_diag = np.sqrt(np.sum(rateLog.omega_BN_B**2, axis=1))
    print("Omega norm sample:", omega_diag[:20])

    print("ModeType (first 20):", modeTypeLog.dataValue[:20])


    # =====================================================
    # ORBIT DATA FOR PLOTTING
    # =====================================================

    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N

    dynTime = dataRec.times()
    fswTime = tamCommLog.times()

    magData = magLog.magField_N


    print("NaN in r:", np.isnan(posData).any())
    print("NaN in v:", np.isnan(velData).any())

    print(
        "Commanded torque sample:",
        tauLog.torqueRequestBody[0:5]
    )

    print(
        "Mean magnetic field magnitude:",
        np.mean(np.linalg.norm(magData, axis=1))
    )


    # =====================================================
    # PLOTTING
    # =====================================================

    figureList, finalDiff = plotOrbits(
        fswTime,
        dynTime,
        posData,
        velData,
        oe,
        mu,
        P,
        dataRec,
        magData,
        mtbDipoleCmdsLog,
        MagDistLog,
        ggLog,
        dragForceLog,
        aeroTorqueLog,
        fileName
    )

    plt.figure()

    labels = ['x', 'y', 'z']

    for i in range(3):
        plt.subplot(3,1,i+1)
        plt.plot(times, omega[:,i], label=f"Est ω_{labels[i]}")
        plt.plot(times, trueOmega[:,i], '--', label=f"True ω_{labels[i]}")
        plt.ylabel("rad/s")
        plt.legend()
        plt.grid()

    plt.xlabel("Time [s]")
    plt.suptitle("Axis-wise Angular Rate Comparison")

    plt.show()
    plt.close("all")

    return finalDiff, figureList


    # =====================================================
    # SCRIPT ENTRY POINT
    # =====================================================

if __name__ == "__main__":
    run()