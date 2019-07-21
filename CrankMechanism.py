#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 23:48:24 2019

@author: kevinrussell
"""

from matplotlib.pylab import *

r = 0.15
n = 0.2
l = r / n

theta = linspace(-180,180,36)
theta_rad = deg2rad(theta)

Sim_L = 0.5
Sim_W = 0.3


@vectorize
def calculate_x_pos(theta_rad):
    r_x = r*sin(theta_rad)
    r_y = r*cos(theta_rad)
    
    alpha_rad = arcsin(r_x / l)
    l_y = l*cos(alpha_rad)
    
    ll = l_y + r_y
    
    F = 1.0
    Fo= F * cos(alpha_rad) * sin(alpha_rad + theta_rad)
    
    T = Fo * r
    
    return ll

@vectorize
def calculate_torque(theta_rad,F=1.0):
    r_x = r*sin(theta_rad)
    r_y = r*cos(theta_rad)
    
    alpha_rad = arcsin(r_x / l)
    l_y = l*cos(alpha_rad)
    
    ll = l_y + r_y
    
    Fo= F * cos(alpha_rad) * sin(alpha_rad + theta_rad)
    
    T = Fo * r
    
    return T


@vectorize
def calculate_gamma(theta1,theta2):
    L1 = calculate_x_pos(theta1)
    L2 = calculate_x_pos(theta2)
    
    X1 = L1 - L1.min()
    X2 = L2 - L2.min()
    Xc = (X1 + X2) / 2.0
    
    gamma = arctan(Xc / Sim_L)
    return gamma

@vectorize
def calculate_beta(theta1,theta2):
    L1 = calculate_x_pos(theta1)
    L2 = calculate_x_pos(theta2)
    
    X1 = L1 - L1.min()
    X2 = L2 - L2.min()
    Xc = (X1 + X2) / 2.0
    
    beta = arctan((X1-Xc)/Sim_W)
    return beta


T1,T2 = meshgrid(theta_rad,theta_rad)


BB = calculate_beta(T1,T2)
GG = calculate_gamma(T1,T2)

L1 = calculate_x_pos(theta1)
L2 = calculate_x_pos(theta2)

X1 = L1 - L1.min()
X2 = L2 - L2.min()
Xc = (X1 + X2) / 2.0

beta = arctan((X1-Xc)/Sim_W)
gamma = arctan(Xc / Sim_L)

T = calculate_torque(theta_rad)
L = calculate_x_pos(theta_rad)
x = L - L.min()

contourf(rad2deg(T1),rad2deg(T2),rad2deg(beta));colorbar();title('Beta')
figure()
contourf(rad2deg(T1),rad2deg(T2),rad2deg(gamma));colorbar();title('Gamma')

print 'Gamma Range: {}->{}'.format(rad2deg(gamma).min(),rad2deg(gamma).max())
print 'Beta Range: {}->{}'.format(rad2deg(beta).min(),rad2deg(beta).max())

figure()
plot(theta, T,label = 'T/F')
plot(theta, x,label = 'x')
grid()
legend()

print 'Max x: {}'.format(x.max())
print 'Max T/F: {}'.format(abs(T).max())

