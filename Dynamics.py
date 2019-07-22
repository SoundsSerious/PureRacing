# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 17:46:12 2019

@author: Nico
"""

from matplotlib.pylab import *
from numpy.linalg import  *
from scipy import integrate as scint

#Geometry
pivotToSimCG_Y = 0.68
pivotToSimCG_Z = 0.1
centerlineToActuator_X = 0.32
pivotToTbar_Z = 0.51
humanToPivotY = 0.35
humanToPivotZ = -0.17

Rl = array([centerlineToActuator_X,0,pivotToTbar_Z])
Rr = array([-centerlineToActuator_X,0,pivotToTbar_Z])
Rp = array([0,humanToPivotY,humanToPivotZ])
Rg = array([0,pivotToSimCG_Y,pivotToSimCG_Z])

gear_r = 0.15
gear_n = 0.2
gear_l = gear_r / gear_n

#Mass 
gravity = 9.81
massSimulator = 40.32
massPerson = 75.5

I_Chassis = array([[23.40,-0.04,-0], # 1st row
            [-0.04,5.78,-1.67], # 2nd row
            [-0,-1.67,23.27]] # 3rd row
                      )

I_User = array([[25.44,0.14,-0.04], # 1st row
            [0.14,9.44,6.41], # 2nd row
            [-0.04,6.41,17.77]] # 3rd row
                      )

I=I_Chassis + I_User

#Forces and Moments
#forceActuator = array([0,440.0,0])
forceSimGravity = array([0,-gravity*massSimulator,0])
forceUserGravity = array([0,-gravity*massPerson,0])

#momentActuator_L = cross(forceActuator,Rl)
#momentActuator_R = cross(forceActuator,Rr)
momentSimGrav = cross(forceSimGravity,Rg)
momentUserGrav = cross(forceUserGravity,Rp)

maxTorque = 80.0  #Nm
gearRatio = 53.


#momentTotal = momentActuator_L + momentActuator_R + momentSimGrav + momentUserGrav

def motorTorque(u):
    #u should be from -1 to 1
    if u >= 1.0:
        return maxTorque
    elif u<= -1.0:
        return -maxTorque
    else:
        return u*maxTorque
    
def gearForce(u,theta):    
    r_x = gear_r*sin(theta)
    r_y = gear_r*cos(theta)
    
    alpha_rad = arcsin(r_x / gear_l)
    l_y = gear_l*cos(alpha_rad)
    
    ll = l_y + r_y
    
    Fo= F * cos(alpha_rad) * sin(alpha_rad + theta_rad)
    
    T = Fo * r
    
    return T
    


#Solve
dwdt = solve(I,momentTotal)





