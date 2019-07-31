# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 17:46:12 2019

@author: Nico
"""

from matplotlib.pylab import *
from numpy.linalg import  *
from scipy import integrate as scint

from scipy.integrate import *
from crankdynamics import *
from pitchrollcommand import *

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
forceActuator = array([0,440.0,0])
forceSimGravity = array([0,-gravity*massSimulator,0])
forceUserGravity = array([0,-gravity*massPerson,0])

momentActuator_L = cross(forceActuator,Rl)
momentActuator_R = cross(forceActuator,Rr)
momentSimGrav = cross(forceSimGravity,Rg)
momentUserGrav = cross(forceUserGravity,Rp)

#maxTorque = 80.0  #Nm
#gearRatio = 53.

#STatic forces
def static_forces(beta):
    Bstatic = array([pivotToSimCG_Y*beta *massSimulator*gravity + humanToPivotY * beta * massPerson * gravity,
               pivotToSimCG_Z*massSimulator*gravity + humanToPivotZ * massPerson * gravity])
    Astatic = array([[-centerlineToActuator_X, centerlineToActuator_X],
                     [pivotToTbar_Z, pivotToTbar_Z]])
    Astatic_inv = inv(Astatic)
    return dot(Astatic_inv,Bstatic)


momentTotal = momentActuator_L + momentActuator_R + momentSimGrav + momentUserGrav

#Solve
#dwdt = solve(I,momentTotal)
pitch_roll_cntl = PitchRollController(gamma=deg2rad(-5),beta=gamma=deg2rad(10))

crank_left = CrankConfig()
l_theta_controller = CrankController()
l_x_controller = DisplacementController()

crank_right = CrankConfig()
r_theta_controller = CrankController()
r_x_controller = DisplacementController()


def solve_system_dynamics(t,y,x_controller = None,theta_controller=None):
    omega_l,theta_l, omega_r,theta_r, gam_dot,beta_dot,gam,beta = y

    x_l_dmd,x_r_dmd = pitch_roll_cntl.X

    x_l = crank_left.x(theta_l)
    l_x_controller.demanded = x_l_dmd
    l_x_controller.update( t, x_l)
    l_theta_controller.demanded = l_x_controller.u
    l_theta_controller.update(t, theta_l, omega_l)

    x_r = crank_left.x(theta_r)
    l_x_controller.demanded = x_r_dmd
    r_x_controller.update( t, x_r)
    r_theta_controller.demanded = r_x_controller.u
    r_theta_controller.update(t, theta_r, omega_r)

    Static_Forces = static_forces(beta)
    Fl_static,Fr_static = Static_Forces[0],Static_Forces[1]
    F_l = Fl_static
    F_r = Fr_static

    Tresitance_l = left_crank.Tresistance(theta_l,F_l)
    T_l = l_theta_controller.torque
    Tresitance_r = crank.Tresistance(theta_r,F_r)
    T_r = r_theta_controller.torque

    domgegadt_l = (T_l - crank_left.Cf * omega_l - Tresitance_l) / crank_left.I
    domgegadt_r = (T_r - crank_right.Cf * omega_r - Tresitance_r) / crank_right.I

    return [domgegadt_l,omega_l,domgegadt_r,omega_r]
