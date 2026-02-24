#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 19 09:11:34 2025

@author: emiel
"""


import horizon_detection_V6 as hd
import numpy as np
import re
import os
import matplotlib.pyplot as plt

DATA_DIR = r"static_10.44_5_8/"
rx_static = re.compile(r"p(?P<pitch>[-\d.]+)_r(?P<roll>[-\d.]+)\.csv$", re.I)

# Sensor dimensions
SENSOR_WIDTH = 32
SENSOR_HEIGHT = 24

files = [f for f in os.listdir(DATA_DIR) if rx_static.fullmatch(f)]
if not files:
    raise SystemExit("No static test files (p…_r….csv) found!")

pitch_vals = sorted({float(rx_static.fullmatch(f)["pitch"]) for f in files})
roll_vals  = sorted({float(rx_static.fullmatch(f)["roll" ]) for f in files})

result = np.zeros((len(files), 4))

for i,fname in enumerate(files):
    m      = rx_static.fullmatch(fname)
    p_cmd  = float(m["pitch"])
    r_cmd  = float(m["roll"])
    fpath  = os.path.join(DATA_DIR, fname)
    
    image = np.loadtxt(fpath, skiprows=1, delimiter=',')
    data = image[30,:768]
    data = data.reshape((SENSOR_HEIGHT, SENSOR_WIDTH))
    
    vec = hd.vector(data)
    area = hd.integrate_angles(data)
    p, r = hd.convert_coordinates(vec,area)
    p -= 90
    result[i,:] = np.array([p_cmd, r_cmd, p, r])

sorted_roll = result[result[:, 1].argsort()]
p_err = sorted_roll[:,0] - sorted_roll[:,2]
r_err = sorted_roll[:,1] - sorted_roll[:,3]

plt.figure(dpi=150.0)
plt.plot(sorted_roll[:,1], p_err, ".", label="Pitch error")
plt.plot(sorted_roll[:,1], r_err, ".", label="Roll error")
plt.xlabel("Roll in degrees")
plt.ylabel("Error in degrees")
plt.ylim(-20,20)
plt.legend()
plt.title("Error in pitch and roll from static tests")
