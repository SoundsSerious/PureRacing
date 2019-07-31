# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 18:47:47 2019

@author: Nico
"""

from matplotlib.pylab import *

theta = linspace(-90,90,360)
theta_rad = deg2rad(theta)

L = 0.1
R = 0.1
n = R/L

Q2 = (n**2.)*L/4
Q3 = L*(1.0 - n**2./4.)

@vectorize
def normalFromTorque(theta,Torque = 1.0):
    v1 = R*sin(theta)
    v2 = 1.+(R*cos(theta))/(Q3+Q2*cos(2*theta))
    return Torque * (1./(v1*v2))

normal = normalFromTorque(theta_rad)

figure()
plot(theta,normal,label = 'Normal / Torque')
grid()
legend()
xlabel('Degrees')
ylabel('Normal/Torque')
semilogy()
