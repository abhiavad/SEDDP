import os
from copy import copy

import matplotlib.pyplot as plt
import numpy as np
from Basilisk import __path__

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])
print(fileName)

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
