#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 23 12:59:32 2019

@author: kevinrussell
"""
from matplotlib.pylab import *
from ottermatics import *
from scipy.integrate import ode, odeint



class CrankController(Configuration):

    max_torque = 50.0

    gain = 7.0
    Kp = 1.
    _Ki = 0.25
    Kd = 0.5

    _demanded = 0.0
    dmd_max = pi
    dmd_min = -pi

    error = 0.0
    ep = 0.0
    ei = 0.0
    ed = 0.0

    u = 0.0 #Output Value

    last_update = 0.
    update_frequency = 10. #hz
    update_interval = (1.0 / update_frequency)

    def update(self,time,current_value):
        dt = time - self.last_update
        if dt >= self.update_interval:

            last_error = self.error
            self.error = self.demanded - current_value

            self.ei += self.error*self.Ki  * dt
            self.ed = (self.error - last_error) / dt
            self.ep = self.error

            print time, self.ei, self.error

            self.u = self.gain*(self.ep*self.Kp +  self.ei +  self.ed * self.Kd)
            self.last_update = time

    @property
    def demanded(self):
        return self._demanded

    @demanded.setter
    def demanded(self,inputvalue):
        self._demanded = min(max(inputvalue,self.dmd_min),self.dmd_max)

    def set_demanded_min_max(self,min,max):
        self.dmd_max = max
        self.dmd_min = min

    @property
    def torque(self):
        return min(max(self.u,-self.max_torque),self.max_torque)

    @property
    def Ki(self):
        return self._Ki
        if self.u < self.max_torque and self.u > -self.max_torque:
            return self._Ki
        return 0.0

class CrankConfig(Configuration):

    r = 0.1
    l = 0.2
    Cf = 0.5
    I = 1.0

    def x(self,theta):
        y1 = self.r*cos(theta)
        y2 = (self.l**2.0 + self.r**2.0*sin(theta)**2.0)**0.5
        return y1 + y2

    def v(self,theta):
        v1 = -self.r * sin(theta)
        v2 = -(self.r**2.0 * sin(theta) * cos(theta)) / \
                (self.l**2.0 + self.r**2.0*sin(theta)**2.0)**0.5
        return v1+v2

    def alpha(self, theta):
        dx_r = self.r * sin(theta)
        return arcsin(dx_r/self.l)

    def torqueQforce(self,theta):
        alpha = self.alpha(theta)
        return self.r * cos(alpha) * sin(alpha + theta)


crank = CrankConfig()
controller = CrankController()

solutions = collections.OrderedDict()
Y0 = [0.0,pi*0.1]
dt = 1.0 / 100.0
Tend = 20.0

def solvecrank(t,y):
    omega,theta = y

    controller.update(t, theta)

    x = crank.x(theta)
    tqf = crank.torqueQforce(theta)

    Fresistance = (x - 0.1)*5.0
    Tresitance = tqf * Fresistance
    T = controller.torque

    domgegadt = (T - crank.Cf * omega - Tresitance) / crank.I

    return [domgegadt,omega]

stepset = False

SLV = ode(solvecrank).set_integrator('dopri5')
SLV.set_initial_value(Y0, 0.0)

while SLV.successful() and SLV.t < Tend:
    nexttime = SLV.t + dt

    print(nexttime, SLV.integrate(nexttime))

    omega, theta = SLV.y

    if SLV.t > 11.0 and not stepset:
       controller.demanded = pi * 0.9

    U = controller.u
    ei = controller.ei
    ep = controller.ep
    ed = controller.ed

    tht_dmd = controller.demanded

    x = crank.x(theta)
    tqf = crank.torqueQforce(theta)

    Fresistance = (x - 0.5)*5.0
    Tresitance = tqf * Fresistance
    T = controller.torque

    sol = (theta,omega,tht_dmd,U,ep,ei,ed,Tresitance,T,x)
    solutions[SLV.t] = sol

T = array([t for t,sols in solutions.items()])
SOLS = array([sols for t,sols in solutions.items()])

for i,field in enumerate('theta,omega,tht_dmd,U,ep,ei,ed,TorqueResist,TorqueIn,x'.split(',')):
    tempvar = SOLS[:,i]
    exec('{}=tempvar'.format(field))

close('all')

figure(figsize=(12,6) )
plot(T,theta,label='th act')
plot(T,tht_dmd,label='th dmd')
# plot(T,ep,label='p',alpha=0.2)
# plot(T,ei,label='i',alpha=0.2)
# plot(T,ed,label='d',alpha=0.2)
# plot(T,U,label='u',alpha=0.2)
legend()
grid()
show()

figure(figsize=(12,6) )
plot(T,TorqueIn,label='Tin')
plot(T,TorqueResist,label='Tf')
legend()
grid()
show()

# figure(figsize=(12,6) )
# plot(T,theta,label='Theta')
# plot(T,omega,label='Omega')
# legend()
# grid()
# show()




# theta = linspace(-pi,pi)
# f_v = vectorize(crank.v)
# f_x = vectorize(crank.x)
# f_a = vectorize(crank.alpha)
# f_tqf=vectorize(crank.torqueQforce)
#
# v = f_v(theta)
# x = f_x(theta)
# a = f_a(theta)
# t = f_tqf(theta)


# figure()
# title('Mechanism Profile')
# plot(theta,v,label='v')
# plot(theta,x,label='x')
# plot(theta,a/(pi),label='a')
# plot(theta,t,label='tqf')
# grid()
# legend()
# show()
