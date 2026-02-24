#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May 16 11:04:14 2025

@author: emiel
"""

import numpy as np
from image_generation_V3 import simulate_earth

pitch = np.linspace(0, 359, num=360)
roll = np.linspace(20, 119, num=100)

new_pitch = []
new_roll = []

for p in pitch:
    for r in roll:
        data = simulate_earth(p, r, 0)
        mask = data > 35.0
        if (np.sum(mask) > 20) and (np.sum(mask) < 748):
            np.save(f'data/p{p}_r{r}.npy', data)
            new_pitch.append(p)
            new_roll.append(r)

