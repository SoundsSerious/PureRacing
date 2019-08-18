# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""
from matplotlib.pylab import *

dL = 0.51
dcg = 0.09
fG = 9.81
mSim = 38.82

demandedPitch = deg2rad(10)
requiredTime = 0.25

demandedAngularAccel = (demandedPitch * 2) / requiredTime

I = array([[23.40,-0.04,-0], # 1st row
            [-0.04,5.78,-1.67], # 2nd row
            [-0,-1.67,23.27]] # 3rd row
                      )

A = array([demandedAngularAccel,0,0])

M = matmul(I,A)

print(M)

fInertia = M[0]/dL
fNormal = fG * dcg

forceX = (fInertia + fNormal) / dL

print(forceX)

