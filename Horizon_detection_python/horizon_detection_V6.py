#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 17 10:32:21 2025

@author: emiel
"""

import numpy as np
from image_generation_V3 import simulate_earth

# Parameters
threshold = 35 # Degrees celsius
reference_vector = (0,-1) # Vector pointing down in sensor boresight
sensor = (24, 32)
origin = (11, 15)

# Load X and Y angles from text files.
X_angles_deg = np.loadtxt("FOV_Files/horizontal_angles.txt", delimiter="\t") # Use 32x24
Y_angles_deg = np.loadtxt("FOV_Files/vertical_angles.txt", delimiter="\t") # Use 32x24

# Convert to radians.
X_angles = np.radians(X_angles_deg)
Y_angles = np.radians(Y_angles_deg)

# Load conversion table
conv_table = np.load("conversion_table.npy")

# Convert from pixel coordinates to fov angle coordinates in radians
origin = np.array([X_angles[origin], Y_angles[origin]])

def vector(data):
    mask = data > threshold
    
    # Sensor sees too little or too much of earth
    if (np.sum(mask) < 20) or (np.sum(mask) > 748):
        return np.nan, np.nan
    
    vectors = []
    for x in range(sensor[0]):
        for y in range(sensor[1]):
            if mask[x,y]:
                x_angle = X_angles[x,y] - origin[0]
                y_angle = Y_angles[x,y] - origin[1]
                vectors.append((x_angle, y_angle))
    
    unit_vector = np.sum(np.array(vectors), axis=0)
    unit_vector /= np.sqrt(unit_vector[0]**2 + unit_vector[1]**2)
    
    return unit_vector

def integrate_angles(data):
    data_mask = data > threshold
    angle_sum = 0
    
    X_gradient = np.gradient(X_angles, axis=1)
    Y_gradient = np.gradient(Y_angles, axis=0)
    
    angle_area = np.abs(X_gradient*Y_gradient)
    
    angle_sum = np.sum(angle_area[data_mask])
    
    return angle_sum

def convert_coordinates(vector, area):
    diff = conv_table[:,2:] - [vector[0], vector[1], area]
    diff = np.sqrt(diff[:,0]**2 + diff[:,1]**2 + diff[:,2]**2)
    index = np.argmin(diff)
    pitch, roll = conv_table[index, :2]
    
    return pitch, roll

if __name__=="__main__":
    data = simulate_earth(25, 27, 0, plot=True)
    vec = vector(data)
    area = integrate_angles(data)
    p, r = convert_coordinates(vec,area)
    print(f"Vector: {vec}, area: {area}")
    print(f"Pitch: {p}, Roll: {r}")
