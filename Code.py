import os
from copy import copy

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


from Basilisk.simulation import spacecraft

KEBAB = spacecraft.Spacecraft()

# Kinematics and Estimation for Body Attitude Block


from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    orbitalMotion,
    simIncludeGravBody,
    unitTestSupport,
    vizSupport,
)
 
from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile

def run():
    #create sim variable names
    simTaskName = "simTask"
    simProcessName ="simProcess"

    #sim module as empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #progress bar to check if code is hung
    scSim.SetProgressBar(True)

    #sim process
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # dynamics task with specidied integration update time
    simulationTimeStep = macros.sec2nano(1.0)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    #set up sim task/obj, initialize the spacecraft object, dyn sim is setup using spacecraft module
    KEBAB.ModelTag ="Kebab"

    #add kebab to sim process
    scSim.AddModelToTask(simTaskName, KEBAB)

    #setting up gravity body, creating gravity body factor class. creating an empty gravitational body list
    gravFactory=simIncludeGravBody.gravBodyFactory()

    planet = gravFactory.createEarth()
    planet.isCentralBody =True
    
    ggm03s__j2_only_path = get_path(DataFile.LocalGravData.GGM03S_J2_only)
    planet.useSphericalHarmonicsGravityModel(str(ggm03s__j2_only_path),2) #2 indicates only first 2 harmonics. 

    mu = planet.mu
    #connect grav body to spacecraft
    gravFactory.addBodiesTo(KEBAB)
    oe=orbitalMotion.ClassicElements()
    oe.a=7000*1000 #meters
    oe.e=0.0001
    oe.i=33.3*macros.D2R
    oe.Omega = 48.2 * macros.D2R #what is this?
    oe.omega = 347.8 * macros.D2R
    oe.f =85.3* macros.D2R 
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe= orbitalMotion.rv2elem(mu, rN, vN) #doing this to have consistent orbit

    #initial spacecraft conditions
    KEBAB.hub.r_CN_NInit = rN # in m
    KEBAB.hub.v_CN_NInit = vN # in m/s

    #set sim time
    n = np.sqrt(mu/(oe.a)**3)
    P = 2.0 * np.pi/n

    simulationTime = macros.sec2nano(3.0 * P)
    numDataPoints = 400

    samplingTime = unitTestSupport.samplingTime(simulationTime,simulationTimeStep, numDataPoints)
    
    #create a logging task obj of sc output message at desired down smapling ratio
    dataRec = KEBAB.scStateOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, dataRec)

    #initialize the sim
    scSim.InitializeSimulation()

    #set sim stop time
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #retrive logged data
    posData = dataRec.r_BN_N
    velData=dataRec.v_BN_N

    np.set_printoptions(precision=16)

    figureList, finalDiff = plotOrbits(dataRec.times(), posData, velData, oe, mu, P)

    plt.show()

    plt.close("all")
    return finalDiff, figureList
    

def plotOrbits(timeAxis, posData, velData, oe, mu,P):
        plt.close("all")
        plt.figure(1)
        fig = plt.gcf()
        ax=fig.gca()
        ax.ticklabel_format(useOffset=False, style ="plain")
        finalDiff= 0.0
        
        for idx in range(3):
            plt.plot(timeAxis*macros.NANO2SEC/P, posData[:,idx]/1000.0, color=unitTestSupport.getLineColor(idx,3), label= "$r_{BN"+str(idx) +"}$",)

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
        return figureList, finalDiff

if __name__ == "__main__":
    run()
