#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 23 12:59:32 2019

@author: kevinrussell
"""
from matplotlib.pylab import *
from ottermatics import *
from scipy.integrate import ode, odeint

class CrankConfig(Configuration):

    r = 0.1
    l = 0.15

    I_crank = 1.0

    def x(self,theta):
        y1 = self.r*cos(theta)
        y2 = (self.l**2.0 + self.r**2.0*sin(theta)**2.0)**0.5
        return y1 + y2

    def v(self,theta, omega):
        alpha = self.alpha(theta)
        v1 = -self.r * sin(theta+alpha) * omega / cos(alpha)
        return v1

    def alpha(self, theta):
        dx_r = self.r * sin(theta)
        return arcsin(dx_r/self.l)

    def torqueQforce(self,theta):
        alpha = self.alpha(theta)
        return self.r * cos(alpha) * sin(alpha + theta)

    def theta_acceleration(self,theta,Tin,Fin):
        tqf = self.torqueQforce(theta) * Fin
        dT = Tin - tqf
        dthetadtdt = dT / self.I_crank
        return dthetadtdt

class CrankController(Configuration):

    MotorTorqueMax = 50 #Nm

    Kp = 1.
    Ki = 1.7
    Kd = 0.25

    _demanded = 0.2
    dmd_max = 0.3
    dmd_min = 0.1

    error = 0.0
    ep = 0.0
    ei = 0.0
    ed = 0.0

    u = 0.0 #Output Value
    gain = 2.50

    last_update = -999.
    update_frequency = 60. #hz
    update_interval = (1.0 / update_frequency)

    def update(self,time,current_value):
        dt = time - self.last_update
        if dt >= self.update_interval:

        #if time != self.last_update:
            last_error = self.error
            self.error = self.demanded - current_value

            self.ei += self.error * self.Ki * dt
            self.ed = self.Kd *(self.error - last_error) / dt
            self.ep = self.error * self.Kp

            #Prevent Integral Windup
            #if self.ei  > self.dmd_max: self.ei = self.dmd_max
            #if self.ei  < 0: self.ei = 0
            #elif self.ei  < self.dmd_min: self.ei = self.dmd_min
            #elif self.ei  < self.dmd_max: self.ei = -self.dmd_max

            self.u = self.ep +  self.ei +  self.ed
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

    def TorqueOutput(self,theta,Fi):
        Tresistance = crank.torqueQforce(theta) * Fi
        dT_target = self.u * self.MotorTorqueMax * self.gain * self.error
        Toutput = dT_target + Tresistance
        return min(max(Toutput,-self.MotorTorqueMax),self.MotorTorqueMax)

controller = CrankController()
crank = CrankConfig()

solutions = collections.OrderedDict()
Y0 = [0.0,pi*0.1]
dt = 10.0 / 100.0
Tend = 25.0

def solvecrank(t,y):
    omega,theta = y

    x_current = crank.x(theta)

    Fi = 100.0#-x_current * 100.0

    controller.update(t,x_current)
    Ti = controller.TorqueOutput(theta,Fi)

    domgegadt = crank.theta_acceleration(theta,Ti,Fi) - omega * 0.1

    return [domgegadt,omega]

stepset = False

SLV = ode(solvecrank).set_integrator('dopri5')
SLV.set_initial_value(Y0, 0.0)

while SLV.successful() and SLV.t < Tend:
    nexttime = SLV.t + dt
    print(nexttime, SLV.integrate(nexttime))

    omega, theta = SLV.y
    x_current = crank.x(theta)
    #Fi = -x_current * 10.0
    Fi = 100.0

    TorqueIn = controller.TorqueOutput(theta,Fi)
    TorqueResist = crank.torqueQforce(theta) * Fi

    #if SLV.t > 5.0 and not stepset:
    #    controller.demanded = 0.25

    U = controller.u
    ei = controller.ei
    ep = controller.ep
    ed = controller.ed

    x_dmd = controller.demanded

    sol = (theta,omega,x_current,x_dmd,TorqueIn,TorqueResist,U,ep,ei,ed)
    solutions[SLV.t] = sol

T = array([t for t,sols in solutions.items()])
SOLS = array([sols for t,sols in solutions.items()])

for i,field in enumerate('theta,omega,x_current,x_dmd,TorqueIn,TorqueResist,U,ep,ei,ed'.split(',')):
    tempvar = SOLS[:,i]
    exec('{}=tempvar'.format(field))

close('all')

figure(figsize=(12,6) )
plot(T,x_current,label='current')
plot(T,x_dmd,label='dmd')
plot(T,ep,label='p')
plot(T,ei,label='i')
plot(T,ed,label='d')
plot(T,U,label='u')
legend()
grid()
show()

figure(figsize=(12,6) )
plot(T,TorqueIn,label='Tin')
plot(T,TorqueResist,label='Tf')
legend()
grid()
show()

figure(figsize=(12,6) )
plot(T,theta,label='Tin')
plot(T,omega,label='Th')
legend()
grid()
show()

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
