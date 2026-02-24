#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 22 10:02:24 2025

@author: emiel
"""

import numpy as np
tbl = np.load("conversion_table.npy")

for i in range(tbl.shape[0]):
    print(f'{tbl[i,2]},',end='')
