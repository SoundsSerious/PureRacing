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

    r = 0.0445
    l = 0.0762
    Cf = 0.05
    I = 0.1

    x_offset = 0
    axis_offset = 0.0

    def x(self,theta):
        y1 = self.r*cos(theta)
        y2 = (self.l**2.0 - self.r**2.0*sin(theta)**2.0)**0.5
        return y1 + y2 - self.x_offset

    def v(self,theta, omega):
        alpha = self.alpha(theta)
        v = -self.r * sin(theta+alpha) * omega / cos(alpha)
        return v

    def alpha(self, theta):
        dx_r = self.r * sin(theta)
        return arcsin(dx_r/self.l)

    def torqueQforce(self,theta):
        alpha = self.alpha(theta)
        return self.r * cos(alpha) * sin(alpha + theta)

    def Tresistance(self,theta,force):
        tqf = self.torqueQforce(theta)
        return  tqf * force

    @property
    def max_x(self):
        return self.r + self.l - self.x_offset

    @property
    def min_x(self):
        return self.l - self.r - self.x_offset


crank = CrankConfig()

class CrankController(Configuration):

    stall_torque = 70.0
    max_speed = 40 * 2. * pi / 60.

    torque_margin = 20.0
    speed_limit = (stall_torque-torque_margin) * (max_speed) /  stall_torque


    gain = 5.0
    Kp = 1.25
    _Ki = 0.25
    Kd = 0.2
    gain_dampening = 1.0

    _demanded = 0.0
    dmd_max = pi
    dmd_min = -pi

    last2error = 0.0
    last_error = 0.0
    error = 0.0
    ep = 0.0
    ei = 0.0
    ed = 0.0

    u = 0.0 #Output Value
    omega_current = 0
    theta_current = 0

    last_update = 0.
    update_frequency = 500. #hz
    update_interval = (1.0 / update_frequency)

    d_lowpass = 0.9
    o_lowpass = 1.0

    def update(self,time,current_value, omega_value):
        dt = time - self.last_update
        if dt >= self.update_interval:
            self.omega_current = omega_value*self.o_lowpass + self.omega_current*(1.0-self.o_lowpass)
            self.theta_current = current_value#*self.o_lowpass + self.theta_current*(1.0-self.o_lowpass)

            self.last2error = self.last_error
            self.last_error = self.error
            self.error = self.demanded - self.theta_current

            self.ei += self.error*self.Ki  * dt
            self.ed = self.ed*(1.0-self.d_lowpass) + self.d_lowpass*(self.error - self.last_error) / dt
            self.ep = self.error

            #if sign(self.ei) != sign(self.ep):
            #    self.ei = 0.0#self.ei * 0.9

            #print time, self.ei, self.error

            #Gain Dampening:
            self.gain_dampening = max(1.0, self.omega_current / self.speed_limit)

            self.u = self.gain*(self.ep*self.Kp +  self.ei +  self.ed * self.Kd) #\\// self.gain_dampening
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
    def max_torque(self):
        if abs(self.omega_current) > self.max_speed:
            return 0.
        return self.stall_torque - abs(self.omega_current)*(self.stall_torque / self.max_speed)

    @property
    def torque(self):
        return min(max(self.u,-self.max_torque),self.max_torque)

    @property
    def overspeed_torque(self):
        return 0.0
        if abs(self.omega_current) < self.max_speed:
            return 0.0
        else:
            scalar_val =  self.stall_torque - self.omega_current * (self.stall_torque / self.max_speed)
            vector_val = -1.*scalar_val * sign(self.omega_current)
            return vector_val

    @property
    def Ki(self):
        #return self._Ki
        if self.u < self.max_torque and self.u > -self.max_torque:
            return self._Ki
        return 0.0

    @property
    def speed_limit(self):
        return (self.stall_torque-self.torque_margin) * (self.max_speed) / self.stall_torque

class DisplacementController(Configuration):

    gain = -35.0
    Kp = 1.5
    _Ki = 0.75
    Kd = -0.05

    _demanded = crank.max_x - 0.05
    dmd_max = crank.max_x
    dmd_min = crank.min_x

    last2error = 0.0
    last_error = 0.0
    error = 0.0
    ep = 0.0
    ei = 0.0
    ed = 0.0

    u = 0.0 #Output Value

    last_update = 0.
    update_frequency = 500. #hz
    update_interval = (1.0 / update_frequency)

    u_lowpass = 1.0

    def update(self,time,current_value):
        dt = time - self.last_update
        if dt >= self.update_interval:

            self.last2error = self.last_error
            self.last_error = self.error
            self.error = self.demanded - current_value

            self.ei += self.error*self.Ki  * dt
            self.ed = (self.error - self.last_error) / dt
            self.ep = self.error

            #if sign(self.ei) != sign(self.ep):
            #    self.ei = 0.0# self.ei * 0.9
            # print time, self.ei, self.error

            self.u = self.u * (1.-self.u_lowpass)  + \
                    self.u_lowpass*self.gain*(self.ep*self.Kp +  self.ei +  self.ed * self.Kd)
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
    def Ki(self):
        #return self._Ki
        if self.u < pi and self.u > -pi:
            return self._Ki
        return 0.0

def solvecrank(t,y,x_controller = None,theta_controller=None):
    omega,theta = y

    x = crank.x(theta)

    x_controller.update( t, x)
    theta_controller.demanded = x_controller.u
    theta_controller.update(t, theta, omega)

    Fresistance = (x - 0.1)*1000.0 + 100

    Tresitance = crank.Tresistance(theta,Fresistance)
    T = theta_controller.torque

    domgegadt = (T - crank.Cf * omega - Tresitance) / crank.I

    return [domgegadt,omega]

if __name__ == '__main__':
    theta_controller = CrankController()
    x_controller = DisplacementController()

    solutions = collections.OrderedDict()
    Y0 = [0.0,0.0]
    dt = 1.0 / 100.0
    Tend = 30.0



    stepset = False

    SLV = ode(solvecrank).set_integrator('dopri5')
    SLV.set_initial_value(Y0, 0.0).set_f_params(x_controller,theta_controller)

    while SLV.successful() and SLV.t < Tend:
        nexttime = SLV.t + dt

        print(nexttime, SLV.integrate(nexttime))

        omega, theta = SLV.y

        if SLV.t > 11.0 and not stepset:
           x_controller.demanded = crank.max_x-0.01

        if SLV.t > 21.0 and not stepset:
           x_controller.demanded = crank.min_x+0.01

        Ut = theta_controller.u
        # eit = theta_controller.ei
        # ept = theta_controller.ep
        # edt = theta_controller.ed

        tht_dmd = theta_controller.demanded
        x_dmd = x_controller.demanded

        x = crank.x(theta)
        tqf = crank.torqueQforce(theta)

        Fresistance = (x - 0.1)*1000.0 + 100

        Tresistance = crank.Tresistance(theta,Fresistance)
        T = theta_controller.torque

        sol = (theta,omega,tht_dmd,Ut,Tresistance,T,x,x_dmd)
        solutions[SLV.t] = sol

    T = array([t for t,sols in solutions.items()])
    SOLS = array([sols for t,sols in solutions.items()])

    for i,field in enumerate('theta,omega,tht_dmd,Ut,TorqueResist,TorqueIn,X,Xdmd'.split(',')):
        tempvar = SOLS[:,i]
        exec('{}=tempvar'.format(field))

    close('all')

    #figure(figsize=(12,6) )
    fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
    ax1.plot(T,theta,label='th act')
    ax1.plot(T,tht_dmd,label='th dmd')
    # plot(T,ep,label='p',alpha=0.2)
    # plot(T,ei,label='i',alpha=0.2)
    # plot(T,ed,label='d',alpha=0.2)
    # plot(T,U,label='u',alpha=0.2)
    ax1.legend()
    ax1.grid()
    ax2.plot(T,X,label='x')
    ax2.plot(T,Xdmd,label='x dmd')
    ax2.legend()
    ax2.grid()
    show()

    fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
    ax1.plot(T,theta,label='Theta')
    ax1.plot(T,omega,label='Omega')
    ax1.plot(T,ones(T.shape)*4.71,label='Omg Max')
    ax1.plot(T,-ones(T.shape)*4.71,label='Omg Max')
    ax1.plot(T,ones(T.shape)*4.71*0.75,label='Omg 75%')
    ax1.plot(T,-ones(T.shape)*4.71*0.75,label='Omg 75%')
    ax1.legend()
    ax1.grid()
    ax2.plot(T,TorqueIn,label='Tin')
    ax2.plot(T,TorqueResist,label='Tf')
    ax2.legend()
    ax2.grid()
    show()


    theta = linspace(-pi,pi)
    #f_v = vectorize(crank.v)
    f_x = vectorize(crank.x)
    f_a = vectorize(crank.alpha)
    f_tqf=vectorize(crank.torqueQforce)

    #v = f_v(theta)
    x = f_x(theta)
    a = f_a(theta)
    t = f_tqf(theta)


    figure()
    title('Mechanism Profile')
    #plot(theta,v,label='v')
    plot(theta,x,label='x')
    plot(theta,a/(pi),label='a')
    plot(theta,t,label='Torque/Force')
    grid()
    legend()
    show()
