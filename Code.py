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
import os

import numpy as np
np.set_printoptions(precision=16)

import matplotlib.pyplot as plt

#import general simulation support files

from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    unitTestSupport,
    vizSupport,
)

#import simulation related support
from Basilisk.simulation import (
    spacecraft,
    extForceTorque,
    simpleNav,
)

#import FSW Algo related support, might need to be rewritten
from Basilisk.fswAlgorithms import(
    mrpFeedback,
    inertial3D,
    attTrackingError,
)

#import message declarations
from Basilisk.architecture import messaging

#get location of supporting data
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
 
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

def run():
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        useUnmodeledTorque (bool): Specify if an external torque should be included
        useIntGain (bool): Specify if the feedback control uses an integral feedback term
        useKnownTorque (bool): Specify if the external torque is feed forward in the control
        useCMsg (bool): Specify if the C-based stand-alone messages should be used

    """

    #create sim variable names
    simTaskName = "simTask"
    simProcessName ="simProcess"

    #sim module as empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #progress bar to check if code is hung
    scSim.SetProgressBar(True)

    #sim process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    #set sim time
    simulationTime = macros.sec2nano(1200.0)
    simulationTimeStep = macros.sec2nano(0.01)
    numDataPoints = 400
    samplingTime = unitTestSupport.samplingTime(simulationTime,simulationTimeStep, numDataPoints)

    # dynamics task with specidied integration update time
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

     #set up sim task/obj, initialize the spacecraft object, dyn sim is setup using spacecraft module
    KEBAB = spacecraft.Spacecraft() # Kinematics and Estimation for Body Attitude Block
    KEBAB.ModelTag ="Kebab"
    #add kebab to sim process
    scSim.AddModelToTask(simTaskName, KEBAB)
    
    #define the simulation inertia    
    I= [1.4063E-02, 0., 0.,
        0., 2.8125E-03, 0.,
        0., 0., 2.8125E-03]
    KEBAB.hub.mHub=0.75 #in Kg
    KEBAB.hub.r_BcB_B= [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    KEBAB.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)
    
    #setup the orbit using classical orbit elements
    oe=orbitalMotion.ClassicElements()
    oe.a=7000*1000 #meters
    oe.e=0.0001
    oe.i=33.3*macros.D2R
    oe.Omega = 48.2 * macros.D2R #what is this?
    oe.omega = 347.8 * macros.D2R
    oe.f =85.3* macros.D2R 
        
    KEBAB.hub.sigma_BNInit=[[0.1], [0.2], [-0.3]]  # sigma_BN_B
    KEBAB.hub.omega_BN_BInit=[[0.001], [-0.01], [0.03]]  # rad/s - omega_BN_B

    #setting up gravity body, creating gravity body factor class. creating an empty gravitational body list
    gravFactory=simIncludeGravBody.gravBodyFactory()
    #connect grav body to spacecraft
    gravFactory.addBodiesTo(KEBAB)

    planet = gravFactory.createEarth()
    planet.isCentralBody =True
    
    ggm03s__j2_only_path = get_path(DataFile.LocalGravData.GGM03S_J2_only)
    planet.useSphericalHarmonicsGravityModel(str(ggm03s__j2_only_path),2) #2 indicates only first 2 harmonics. 
    mu=planet.mu
    
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe= orbitalMotion.rv2elem(mu, rN, vN) #doing this to have consistent orbit

    #initial spacecraft conditions
    KEBAB.hub.r_CN_NInit = rN # in m
    KEBAB.hub.v_CN_NInit = vN # in m/s
    
    #Calc time based parameters
    n = np.sqrt(mu/(oe.a)**3)
    P = 2.0 * np.pi/n
    
    # #setup the extForceTorque module
    # #the control torque is read in through the messaging system
    
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag= "externalDisturbance"
    extFTObject.extTorquePntB_B = [[1e-7], [-1e-7], [5e-8]]
    KEBAB.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)

    # #add the simple Navigation sensor module. This sets SC attitude, rate, position, vel nav message
    
    KEBAB_NavObj = simpleNav.SimpleNav()
    KEBAB_NavObj.ModelTag ="SimpleNavigation"
    scSim.AddModelToTask(simTaskName,KEBAB_NavObj)
    KEBAB_NavObj.scStateInMsg.subscribeTo(KEBAB.scStateOutMsg)

    # #setup FSW algo tasks

    # #inertial3D guidance module
    
    KEBAB_inertial3DObj= inertial3D.inertial3D()
    KEBAB_inertial3DObj.ModelTag="inertial3D"
    scSim.AddModelToTask(simTaskName, KEBAB_inertial3DObj)
    KEBAB_inertial3DObj.sigma_R0N=[0.,0.,0.] #desired inertial orientation

    # #att tracking error eval module
    
    attError=attTrackingError.attTrackingError()
    attError.ModelTag="attErrorInertial3D"
    scSim.AddModelToTask(simTaskName,attError)
    attError.attNavInMsg.subscribeTo(KEBAB_NavObj.attOutMsg)
    attError.attRefInMsg.subscribeTo(KEBAB_inertial3DObj.attRefOutMsg)

    # #setup MRP Feedback control Module
    
    mrpControl=mrpFeedback.mrpFeedback()
    mrpControl.ModelTag="mrpFeedback"
    scSim.AddModelToTask(simTaskName,mrpControl)
    mrpControl.K=0.01
    mrpControl.Ki = -1
    mrpControl.P = 0.0
    mrpControl.integralLimit=2./mrpControl.Ki *0.1
    mrpControl.guidInMsg.subscribeTo(attError.attGuidOutMsg)

    # #mrp feedback algo required the vehicle inertia tensor
    
    configData=messaging.VehicleConfigMsgPayload(ISCPntB_B=I)
    configDataMsg = messaging.VehicleConfigMsg()
    configDataMsg.write(configData)
    mrpControl.vehConfigInMsg.subscribeTo(configDataMsg)
    
    # #conect the messages to the module

    extFTObject.cmdTorqueInMsg.subscribeTo(mrpControl.cmdTorqueOutMsg)

    # #setup data logging before sim is initialized
  
    dataRec = KEBAB.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)
    
    attErrorLog= attError.attGuidOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName,attErrorLog)
    
    mrpLog = mrpControl.cmdTorqueOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName,mrpLog)
   
    #initialize the sim
    scSim.InitializeSimulation()

    #set sim stop time
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #retrive logged data
    posData = dataRec.r_BN_N
    velData = dataRec.v_BN_N
    timeAxis=dataRec.times()
    attTime=attErrorLog.times()
    
    print("NaN in r:", np.isnan(posData).any())
    print("NaN in v:", np.isnan(velData).any())

   
    figureList, finalDiff = plotOrbits(attTime, timeAxis, posData, velData, oe, mu, P, attErrorLog, mrpLog, dataRec )

    plt.show()

    plt.close("all")
    return finalDiff, figureList
    

def plotOrbits(attTime, timeAxis, posData, velData, oe, mu,P, attErrorLog, mrpLog, dataRec):
    plt.close("all")
    # attTime=attErrorLog.times()
    plt.figure(1)
    fig = plt.gcf()
    ax=fig.gca()
    ax.ticklabel_format(useOffset=False, style ="plain")
    finalDiff= 0.0
        
    for idx in range(3):
        plt.plot(timeAxis*macros.NANO2SEC/P, posData[:,idx]/1000.0, color=unitTestSupport.getLineColor(idx,3), label= "$r_{BN,"+str(idx) +"}$")

    plt.legend(loc="lower right")
    plt.xlabel("Time [orbits]")
    plt.ylabel("Inertial Position [km]")
    figureList={}
    pltName=fileName + "1"
    figureList[pltName] = plt.figure(1)

    plt.figure(2) 
    fig=plt.gcf()
    ax=fig.gca()
    ax.ticklabel_format(useOffset=False, style ="plain")
    smaData=[]
    for idx in range(0, len(posData)):
        oeData=orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        smaData.append(oeData.a /1000.0)
    plt.plot(timeAxis*macros.NANO2SEC/P, smaData, color ="#aa0000")
    plt.xlabel("time [orbit]")
    plt.ylabel("SMA [km]")

    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)
    


    plt.figure(3)
    for idx in range(3):
        plt.plot(attTime * macros.NANO2MIN, attErrorLog.sigma_BR[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\sigma_' + str(idx) + '$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Attitude Error $\sigma_{B/R}$')
    pltName = fileName + "3"
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    for idx in range(3):
        plt.plot(attTime * macros.NANO2MIN, mrpLog.torqueRequestBody[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$L_{r,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel(r'Control Torque $L_r$ [Nm]')
    pltName = fileName + "4"
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    for idx in range(3):
        plt.plot(attTime * macros.NANO2MIN, attErrorLog.omega_BR_B[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\omega_{BR,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Rate Tracking Error [rad/s] ')

    plt.figure(6)
    for idx in range(3):
        plt.plot(attTime * macros.NANO2MIN, dataRec.omega_BN_B[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km]')

    return figureList, finalDiff

if __name__ == "__main__":
    run()
