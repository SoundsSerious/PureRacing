# -*- coding: utf-8 -*-
"""
Created on Sun Jul 14 17:46:12 2019

@author: Nico & Kevo
"""

from matplotlib.pylab import *
from numpy.linalg import  *
from scipy import integrate as scint

from scipy.integrate import *
from crankdynamics import *
from platformconfig import *


Y0 = [0.0,pi*0.9,0.0,pi*0.9]



#platform = PlatformControl(gamma=deg2rad(-5),beta=deg2rad(10))
class PlatformDesign(Configuration):

    X_GAIN = 25.
    T_GAIN = 35.
    Kd_x = 0.001
    Kd_t = 0.05

    R_crank = 0.046 #0.046
    L_crank = 0.0762
    x_offset = L_crank#0.0762
    y_offset = 0.0 #Not Yet Provisioned

    omgQVmax = 50

    include_springs = False
    spring_constant = 500.0
    unsprung_length = 0.1

    platform = None
    crank_left, crank_right = None, None
    l_x_controller, r_x_controller = None, None
    l_theta_controller, r_theta_controller = None, None

    #Motor Parameters
    stall_torque = 70.0
    max_speed = 40 * 2. * pi / 60.

    torque_margin = 20.0
    speed_limit = (stall_torque-torque_margin) * (max_speed) / stall_torque

    #Loop Variabls - don't fuq wid dis
    domgegadt_l, domgegadt_r = 0., 0.
    omega_l, omega_r = 0., 0.
    theta_l, theta_r = 0., 0.
    x_l, x_r = 0., 0.
    x_l_dmd, x_r_dmd = 0., 0.

    Fl_static, Fr_static = 0., 0.
    Fl_dyn, Fr_dyn = 0., 0.
    F_l, F_r = 0., 0.
    Fsl,Fsr = 0.,0.
    Tresitance_l,Tresitance_r = 0.,0.
    T_l, T_r = 0.,0.

    #Simulation
    solutions = None
    dt = 1.0 / 400.0
    Tend = 30.
    Min_Time_Warmup = 5.0
    T_warmup = 0.0
    warmup_set = False
    Nit = 50

    #Output
    DATA = None

    def __init__(self,**kwargs):
        super(PlatformDesign,self).__init__(**kwargs)
        self.inititalize()

    def inititalize(self):
        self.platform = PlatformControl(gamma_goal=0,beta_goal=0,x_offset = self.x_offset,\
                                        gear_l = self.L_crank, gear_r = self.R_crank,\
                                        spring_constant = self.spring_constant, \
                                        unsprung_length = self.unsprung_length)

        self.crank_left = CrankConfig(r=self.platform.gear_r,l=self.platform.gear_l,\
                                                        x_offset = self.x_offset)
        self.l_x_controller = DisplacementController(gain=self.X_GAIN,dmd_max=self.crank_left.max_x,\
                                            dmd_min=self.crank_left.min_x,Kd = self.Kd_x)
        self.l_theta_controller = CrankController(gain=self.T_GAIN,Kd = self.Kd_t,\
                                        stall_torque = self.stall_torque, max_speed = self.max_speed)


        self.crank_right = CrankConfig(r= self.platform.gear_r, l= self.platform.gear_l,\
                                                            x_offset = self.x_offset)
        self.r_x_controller = DisplacementController(gain=self.X_GAIN,dmd_max=self.crank_right.max_x,\
                                                    dmd_min=self.crank_right.min_x,Kd = self.Kd_x)
        self.r_theta_controller = CrankController(gain=self.T_GAIN,Kd = self.Kd_t,\
                                        stall_torque = self.stall_torque, max_speed = self.max_speed)

        x_l = self.crank_left.x(Y0[1])
        x_r = self.crank_right.x(Y0[3])
        self.platform.update(0,x_l,x_r)


    def generate_plots(self):
        self.force_plots()
        self.simulation_controls_plot()
        #self.torque_plots()
        self.torque_speed_plots()
        #self.simulation_x_control_varaibles()
        #self.simulation_t_control_varaibles()

    def force_plots(self):
        fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
        fig.suptitle('{} Force Plots'.format(self.name))

        ax1.plot(self.T,self.DATA['Fl_static'], label='Static')
        ax1.plot(self.T,self.DATA['Fl_dyn'], label='Dynamic')
        ax1.plot(self.T,self.DATA['F_l'], label='Total')
        if self.include_springs == True:
            ax1.plot(self.T,self.DATA['Fsl'], label='Springs')
        ax1.legend()
        ax1.grid()
        ax1.set_title(self.name+' Left Side')

        ax2.plot(self.T,self.DATA['Fr_static'], label='Static')
        ax2.plot(self.T,self.DATA['Fr_dyn'], label='Dynamic')
        ax2.plot(self.T,self.DATA['F_r'], label='Total')
        if self.include_springs == True:
            ax2.plot(self.T,self.DATA['Fsr'], label='Springs')
        ax2.legend()
        ax2.grid()
        ax2.set_title(self.name+' Right Side')
        show()

    def torque_plots(self):
        fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
        fig.suptitle('{} Torque Plots'.format(self.name))

        ax1.plot(self.T,self.DATA['Tresitance_l'], label='Resistance')
        ax1.plot(self.T,self.DATA['T_l'], label='Input')
        ax1.plot(self.T,self.DATA['Tmax_l'],label='Max Torque')
        ax1.legend()
        ax1.grid()
        ax1.set_title(self.name+' Left Side')

        ax2.plot(self.T,self.DATA['Tresitance_r'], label='Resistance')
        ax2.plot(self.T,self.DATA['T_r'], label='Input')
        ax2.plot(self.T,self.DATA['Tmax_r'],label='Max Torque')
        ax2.legend()
        ax2.grid()
        ax2.set_title(self.name+' Right Side')
        show()

    def simulation_controls_plot(self):
        fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
        fig.suptitle('{} Run Simulation'.format(self.name))

        ax1.plot(self.T,rad2deg(self.DATA['gam']),label='Pitch act')
        ax1.plot(self.T,rad2deg(self.DATA['gam_dmd']),label='Pitch dmd')
        ax1.plot(self.T,rad2deg(self.DATA['beta']),label='Roll')
        ax1.plot(self.T,rad2deg(self.DATA['beta_dmd']),label='Roll dmd')
        ax1.legend()
        ax1.grid()
        ax1.set_title(self.name+' Pitch And Roll')

        ax2.plot(self.T,ones(self.T.shape)*self.platform.x_max,'k--',label='maxlim')
        ax2.plot(self.T,self.DATA['x_l'],label='xl act')
        ax2.plot(self.T,self.DATA['xl_dmd'],label='xl dmd')
        ax2.plot(self.T,self.DATA['x_r'],'--',label='xr act')
        ax2.plot(self.T,self.DATA['xr_dmd'],'--',label='xr dmd')
        ax2.plot(self.T,ones(self.T.shape)*self.platform.x_min,'k--',label='minlim')
        ax2.legend()
        ax2.grid()
        ax2.set_title(self.name+' Actuator Position')

        show()

    def simulation_x_control_varaibles(self):
        fig,(ax1,ax2) = subplots(2,sharex=True,figsize=(12,10))
        fig.suptitle('{} X Control Plots'.format(self.name))

        ax1.plot(self.T,self.DATA['pid_xl_u'], label='U')
        ax1.plot(self.T,self.DATA['pid_xl_ei'], label='P')
        ax1.plot(self.T,self.DATA['pid_xl_ep'], label='I')
        ax1.plot(self.T,self.DATA['pid_xl_ed'], label='D')
        ax1.legend()
        ax1.grid()
        ax1.set_title(self.name+' Left Side')

        ax2.plot(self.T,self.DATA['pid_xr_u'], label='U')
        ax2.plot(self.T,self.DATA['pid_xr_ei'], label='P')
        ax2.plot(self.T,self.DATA['pid_xr_ep'], label='I')
        ax2.plot(self.T,self.DATA['pid_xr_ed'], label='D')
        ax2.legend()
        ax2.grid()
        ax2.set_title(self.name+' Right Side')
        show()

    def simulation_t_control_varaibles(self):
        fig,(ax1,ax2,ax3) = subplots(3,sharex=True,figsize=(12,10))
        fig.suptitle('{} Theta Control Plots'.format(self.name))

        ax1.plot(self.T,self.DATA['pid_tl_u'], label='U')
        ax1.plot(self.T,self.DATA['pid_tl_ei'], label='I')
        ax1.plot(self.T,self.DATA['pid_tl_ep'], label='P')
        ax1.plot(self.T,self.DATA['pid_tl_ed'], label='D')
        ax1.legend()
        ax1.grid()
        ax1.set_title(self.name+' Left Side')

        ax2.plot(self.T,self.DATA['pid_tr_u'], label='U')
        ax2.plot(self.T,self.DATA['pid_tr_ei'], label='I')
        ax2.plot(self.T,self.DATA['pid_tr_ep'], label='P')
        ax2.plot(self.T,self.DATA['pid_tr_ed'], label='D')
        ax2.legend()
        ax2.grid()
        ax2.set_title(self.name+' Right Side')

        ax3.plot(self.T,self.DATA['theta_l'], label='ThetaL')
        ax3.plot(self.T,self.DATA['theta_l_dmd'], label='ThetaLDmd')
        ax3.plot(self.T,self.DATA['theta_r'], label='ThetaR')
        ax3.plot(self.T,self.DATA['theta_r_dmd'], label='ThetaRDmd')
        ax3.legend()
        ax3.grid()
        ax3.set_title(self.name+'Angles')
        show()

    def torque_speed_plots(self):
        fig,ax1 = subplots(figsize=(12,10))
        fig.suptitle('{} Torque Speed Plot'.format(self.name))

        ax1.plot(abs(self.DATA['omega_l'])*9.5493,abs(self.DATA['T_l']), label='Left Side')
        ax1.plot(abs(self.DATA['omega_r'])*9.5493, abs(self.DATA['T_r']),label='Right Side')
        ax1.legend()
        ax1.grid()
        ax1.set_xlabel('RPM')
        ax1.set_ylabel('Torque')


    def solve_system_dynamics(self,t,y):
        self.omega_l,self.theta_l, self.omega_r,self.theta_r = y

        self.x_l_dmd, self.x_r_dmd = self.platform.X_dmd

        self.x_l = self.crank_left.x(self.theta_l)
        self.l_x_controller.demanded = self.x_l_dmd
        self.l_x_controller.update( t, self.x_l)
        self.l_theta_controller.demanded = self.l_x_controller.u
        self.l_theta_controller.update(t, self.theta_l, self.omega_l)

        self.x_r = self.crank_right.x(self.theta_r)
        self.r_x_controller.demanded = self.x_r_dmd
        self.r_x_controller.update( t, self.x_r)
        self.r_theta_controller.demanded = self.r_x_controller.u
        self.r_theta_controller.update(t, self.theta_r, self.omega_r)

        #print t,x_l,x_r,omega_l,omega_r, platform.last_update
        #print t,platform.last_update, r_x_controller.last_update, r_theta_controller.last_update

        self.platform.update(t,self.x_l,self.x_r)

        #Output Section
        self.Xspring_l = self.platform.Xspring_l
        self.Xspring_r = self.platform.Xspring_r
        self.Fl_static,self.Fr_static = self.platform.static_forces
        self.Fl_dyn, self.Fr_dyn = self.platform.dynamic_forces
        self.Fsl, self.Fsr = self.platform.spring_forces

        self.pid_xl_ei = self.l_x_controller.ei
        self.pid_xl_ed = self.l_x_controller.ed * self.l_x_controller.Kd
        self.pid_xl_ep = self.l_x_controller.ep * self.l_x_controller.Kp
        self.pid_xl_u  = self.l_x_controller.u / self.l_x_controller.gain

        self.pid_xr_ei = self.r_x_controller.ei
        self.pid_xr_ed = self.r_x_controller.ed * self.r_x_controller.Kd
        self.pid_xr_ep = self.r_x_controller.ep * self.r_x_controller.Kp
        self.pid_xr_u  = self.r_x_controller.u / self.r_x_controller.gain

        self.pid_tl_ei = self.l_theta_controller.ei
        self.pid_tl_ed = self.l_theta_controller.ed * self.l_theta_controller.Kd
        self.pid_tl_ep = self.l_theta_controller.ep * self.l_theta_controller.Kp
        self.pid_tl_u  = self.l_theta_controller.u / self.l_theta_controller.gain
        self.theta_l_dmd = self.l_theta_controller.demanded

        self.pid_tr_ei = self.r_theta_controller.ei
        self.pid_tr_ed = self.r_theta_controller.ed * self.r_theta_controller.Kd
        self.pid_tr_ep = self.r_theta_controller.ep * self.r_theta_controller.Kp
        self.pid_tr_u  = self.r_theta_controller.u / self.r_theta_controller.gain
        self.theta_r_dmd = self.r_theta_controller.demanded

        #Net Forces -- Add Dynamics
        if t < self.Min_Time_Warmup:
            self.Fl_dyn = self.Fl_dyn*min(1.0,max(t-2.5/self.Min_Time_Warmup,0.0))
            self.Fr_dyn = self.Fr_dyn*min(1.0,max(t-2.5/self.Min_Time_Warmup,0.0))
            self.F_l = self.Fl_static + self.Fl_dyn
            self.F_r = self.Fr_static + self.Fr_dyn
        elif not self.include_springs:
            self.F_l = self.Fl_static + self.Fl_dyn
            self.F_r = self.Fr_static + self.Fr_dyn
        elif self.include_springs:
            self.F_l = self.Fl_static + self.Fl_dyn + self.Fsl
            self.F_r = self.Fr_static + self.Fr_dyn + self.Fsl


        self.Tresitance_l = self.crank_left.Tresistance(self.theta_l,self.F_l)
        self.T_l = self.l_theta_controller.torque
        self.Tmax_l = self.l_theta_controller.max_torque
        self.Tresitance_r = self.crank_right.Tresistance(self.theta_r,self.F_r)
        self.T_r = self.r_theta_controller.torque
        self.Tmax_r = self.r_theta_controller.max_torque

        #Crank Dynamics
        self.domgegadt_l = (self.T_l - self.crank_left.Cf * self.omega_l - self.Tresitance_l \
                                     - self.l_theta_controller.overspeed_torque) / self.crank_left.I
        self.domgegadt_r = (self.T_r - self.crank_right.Cf * self.omega_r - self.Tresitance_r\
                                     - self.r_theta_controller.overspeed_torque) / self.crank_right.I

        #Platform Dynamics

        return [self.domgegadt_l, self.omega_l, self.domgegadt_r, self.omega_r]

    def run_pitch_test(self,t):
        #Pitch Run
        if t > 10:
            self.platform.gamma_goal=deg2rad(-12.)
        if t > 20:
            self.platform.gamma_goal=deg2rad(12.0)
        if t > 30:
            self.platform.gamma_goal=deg2rad(-12.)
        if t > 40:
            self.platform.gamma_goal=deg2rad(12)
        if t > 50:
            self.platform.beta_goal = deg2rad(-3)
            self.platform.gamma_goal= deg2rad(5)

    def run_roll_test(self,t):
        #Roll Run
        if t> 10:
            self.platform.beta_goal = deg2rad(-10)
        if t > 20:
            self.platform.beta_goal = deg2rad(0)
        if t > 30:
            self.platform.beta_goal = deg2rad(10)
        if t > 40:
            self.platform.beta_goal = deg2rad(0)
        if t > 50:
            self.platform.gamma_goal= deg2rad(5)
            self.platform.beta_goal = deg2rad(5)

    def run_simulation(self, run_program):
        self.solutions = collections.OrderedDict()
        print 'Runing Simulation for {}'.format(self.name)
        #Reset From Prior Run
        self.warmup_set = False
        self.T_warmup = 0.
        self.DATA = {}

        SLV = ode(self.solve_system_dynamics).set_integrator('dropri5')
        SLV.set_initial_value(Y0, 0.0)

        while SLV.successful() and SLV.t < self.Tend+self.T_warmup:
            nexttime = SLV.t + self.dt

            out = SLV.integrate(nexttime)

            if not self.warmup_set and all([ abs(array([c.ei,c.ed,c.ep])).max() \
                            < 0.1 for c in [self.l_x_controller,self.r_x_controller]]) \
                            and SLV.t > self.Min_Time_Warmup:
                self.warmup_set = True
                self.T_warmup = SLV.t
                out = SLV.integrate(nexttime)
                print 'Warmed Up',self.T_warmup

            elif not self.warmup_set and SLV.t > self.Min_Time_Warmup:
                if SLV.t % 0.1 < 2E-3:
                    print 'Warming up',SLV.t
                if SLV.t > self.Tend:
                    print 'Did Not Initialize :('
            elif self.warmup_set and SLV.t > self.Min_Time_Warmup: #Run Program & Output
                t = SLV.t-self.T_warmup

                run_program(t)
                self.solutions[SLV.t] = self.standard_output

                if SLV.t % 0.1 < 2E-3:
                    print SLV.t, rad2deg(self.platform.beta), rad2deg(self.platform.gamma)


            elif SLV.t < self.Min_Time_Warmup:
                if SLV.t % 0.1 < 2E-3: print 'Warmup Period ', SLV.t
                #print SLV.t



        if not SLV.successful():
            print 'Error: {}'.format( SLV.get_return_code() )

        self.apply_solvervariables()

    @property
    def standard_output(self):

        return self.omega_l,self.theta_l, self.omega_r,self.theta_r, self.platform.gamma,self.platform.beta, \
              self.platform.gamma_dmd, self.platform.beta_dmd,self.x_l_dmd,self.x_r_dmd,self.x_l,self.x_r, \
              self.Fl_static, self.Fr_static, self.Fl_dyn, self.Fr_dyn,self.Fsl,self.Fsr,self.F_l, self.F_r,\
              self.Tresitance_l,self.Tresitance_r,self.T_l,self.T_r,self.Xspring_l,self.Xspring_r,\
              self.Tmax_l,self.Tmax_r,self.pid_xl_ei,self.pid_xl_ed,self.pid_xl_ep,self.pid_xl_u, \
              self.pid_xr_ei,self.pid_xr_ed,self.pid_xr_ep,self.pid_xr_u,self.pid_tl_ei, \
              self.pid_tl_ed,self.pid_tl_ep,self.pid_tl_u,self.pid_tr_ei,self.pid_tr_ed, \
              self.pid_tr_ep,self.pid_tr_u,self.theta_l_dmd,self.theta_r_dmd

    @property
    def standard_labels(self):
        return 'omega_l,theta_l,omega_r,theta_r,gam,beta,gam_dmd,beta_dmd,xl_dmd,\
                xr_dmd,x_l,x_r,Fl_static,Fr_static,Fl_dyn,Fr_dyn,Fsl,Fsr,F_l,F_r,\
                Tresitance_l,Tresitance_r,T_l,T_r,Xspring_l,Xspring_r,Tmax_r,\
                Tmax_l,pid_xl_ei,pid_xl_ed,pid_xl_ep,pid_xl_u,pid_xr_ei,pid_xr_ed,\
                pid_xr_ep,pid_xr_u,pid_tl_ei,pid_tl_ed,pid_tl_ep,pid_tl_u,pid_tr_ei,\
                pid_tr_ed,pid_tr_ep,pid_tr_u,theta_l_dmd,theta_r_dmd'.split(',')

    @property
    def T(self):
        return array([t for t,sols in self.solutions.items()])-self.T_warmup

    @property
    def SOLS(self):
        return array([sols for t,sols in self.solutions.items()])

    def apply_solvervariables(self):
        for i,field in enumerate(self.standard_labels):
            try:
                self.DATA[field.strip()] = self.SOLS[:,i].astype('float')
            except:
                self.DATA[field.strip()] = self.SOLS[:,i]



if __name__ == '__main__':
    close('all')


    BOSCH_GAIN_X = -30.
    BOSCH_GAIN_T = 20.
    bosch = PlatformDesign(name = 'Bosch Motor Pitch', stall_torque = 70.0, max_speed = 75 * 2. * pi / 60.,\
                            T_GAIN = BOSCH_GAIN_T , X_GAIN = BOSCH_GAIN_X, Kd_t = 0.01 , Kd_x = 0.0)
    bosch.run_simulation( bosch.run_pitch_test )
    bosch.generate_plots()

    bosch_roll = PlatformDesign(name = 'Bosch Motor Roll', stall_torque = 70.0, max_speed = 75 * 2. * pi / 60.,\
                            T_GAIN = BOSCH_GAIN_T , X_GAIN = BOSCH_GAIN_X, Kd_t = 0.01 , Kd_x = 0.0)
    bosch_roll.run_simulation( bosch_roll.run_roll_test )
    bosch_roll.generate_plots()


    # for st in linspace(10,60+1,4):
    #     bst = PlatformDesign(name = 'Bosch Motor {:3.0f}'.format(st), stall_torque = st, max_speed = 75 * 2. * pi / 60.,\
    #                             T_GAIN = BOSCH_GAIN_T , X_GAIN = BOSCH_GAIN_X, Kd_t = 0.01 , Kd_x = 0.0)
    #     bst.run_simulation( bst.run_pitch_test )
    #     bst.generate_plots()

    boschs = PlatformDesign(name = 'Bosch Motor w/ Springs Pitch', include_springs = True,
                           stall_torque = 70.0, max_speed = 75 * 2. * pi / 60.,\
                           T_GAIN = BOSCH_GAIN_T , X_GAIN = BOSCH_GAIN_X, Kd_t = 0.0 , Kd_x = 0.00)
    boschs.run_simulation( boschs.run_pitch_test )
    boschs.generate_plots()

    boschs_roll = PlatformDesign(name = 'Bosch Motor w/ Springs Roll', include_springs = True,
                           stall_torque = 70.0, max_speed = 75 * 2. * pi / 60.,\
                           T_GAIN = BOSCH_GAIN_T , X_GAIN = BOSCH_GAIN_X, Kd_t = 0.0 , Kd_x = 0.00)
    boschs_roll.run_simulation( boschs_roll.run_roll_test )
    boschs_roll.generate_plots()
    #
    #
    # for st in arange(10,60+1,5):
    #     with boschs.difference(stall_torque=st,name = bosch.name + ' ST:{:2.0f}'.format(st)) as bst:
    #         bst.run_simulation( bosch.run_pitch_test )
    #         bst.generate_plots()



    # DOGA_GAIN_X = -60
    # DOGA_GAIN_T = 35
    # doga = PlatformDesign(name = 'Doga Motor', stall_torque = 120.0, max_speed = 40 * 2. * pi / 60.,
    #                     T_GAIN = DOGA_GAIN_T , X_GAIN = DOGA_GAIN_X, Kd_t = 0.0 , Kd_x = 0.0)
    # doga.run_simulation( doga.run_pitch_test )
    # doga.generate_plots()

    # dogas = PlatformDesign(name = 'Doga Motor w/ Springs',include_springs=True, \
    #                         stall_torque = 120.0, max_speed = 40 * 2. * pi / 60.,
    #                     T_GAIN = DOGA_GAIN_X , X_GAIN = DOGA_GAIN_X, Kd_t = 0.0, Kd_x = 0.0)
    # dogas.run_simulation( dogas.run_pitch_test )
    # dogas.generate_plots()


    # KING_GAIN_X = 60.
    # KING_GAIN_T = 35.
    # king = PlatformDesign(name = 'King Motor',stall_torque = 98.0,max_speed = 40 * 2. * pi / 60.,\
    #                     T_GAIN = KING_GAIN_T , X_GAIN = KING_GAIN_X, Kd_t = 0.1 , Kd_x = 0.05)
    # king.run_simulation( king.run_pitch_test )
    # king.generate_plots()
    #
    # kings=PlatformDesign(name = 'King Motor w/ Springs',include_springs=True, \
    #                     stall_torque = 98.0, max_speed = 40 * 2. * pi / 60.,\
    #                     T_GAIN = KING_GAIN_T , X_GAIN = KING_GAIN_X, Kd_t = 0.1 , Kd_x = 0.05)
    # kings.run_simulation( kings.run_pitch_test )
    # kings.generate_plots()




















# def crank_out_force(self,crank, theta, omega, Tin, Tout, Fin):
#     v = crank.v(theta,omega)
#     if v == 0 or abs(omega / v) >self.omgQVmax:
#         F = Fin
#         return F
#     return (Tout - Tin) * omega / v + Fin
# #Geometry
# pivotToSimCG_Y = 0.68
# pivotToSimCG_Z = 0.1
# centerlineToActuator_X = 0.32
# pivotToTbar_Z = 0.51
# humanToPivotY = 0.35
# humanToPivotZ = -0.17
#
# Rl = array([centerlineToActuator_X,0,pivotToTbar_Z])
# Rr = array([-centerlineToActuator_X,0,pivotToTbar_Z])
# Rp = array([0,humanToPivotY,humanToPivotZ])
# Rg = array([0,pivotToSimCG_Y,pivotToSimCG_Z])
#
# #Mass
# gravity = 9.81
# massSimulator = 40.32
# massPerson = 75.5
#
# I_Chassis = array([[23.40,-0.04,-0], # 1st row
#             [-0.04,5.78,-1.67], # 2nd row
#             [-0,-1.67,23.27]] # 3rd row
#                       )
#
# I_User = array([[25.44,0.14,-0.04], # 1st row
#             [0.14,9.44,6.41], # 2nd row
#             [-0.04,6.41,17.77]] # 3rd row
#                       )
#
# I=I_Chassis + I_User

#Forces and Moments
# forceActuator = array([0,440.0,0])
# forceSimGravity = array([0,-gravity*massSimulator,0])
# forceUserGravity = array([0,-gravity*massPerson,0])
#
# momentActuator_L = cross(forceActuator,Rl)
# momentActuator_R = cross(forceActuator,Rr)
# momentSimGrav = cross(forceSimGravity,Rg)
# momentUserGrav = cross(forceUserGravity,Rp)

#maxTorque = 80.0  #Nm
#gearRatio = 53.


# momentTotal = momentActuator_L + momentActuator_R + momentSimGrav + momentUserGrav

#Solve
# netmomentActuator_L_sat = cross(array([0,Fl_static,0]),platform.Rl)
# netmomentActuator_L_con = cross(array([0,Fcon_l,0]),platform.Rl)
# netmomentActuator_L_net = cross(array([0,Fcrank_l_o,0]),platform.Rl)
# netmomentActuator_L = netmomentActuator_L_net + netmomentActuator_L_con + netmomentActuator_L_sat
#
# netmomentActuator_R_sat = cross(array([0,Fr_static,0]),platform.Rr)
# netmomentActuator_R_con = cross(array([0,Fcon_r,0]),platform.Rr)
# netmomentActuator_R_net = cross(array([0,Fcrank_r_o,0]),platform.Rr)
# netmomentActuator_R = netmomentActuator_R_net + netmomentActuator_R_con + netmomentActuator_R_sat
#
# momentTotal = netmomentActuator_L + netmomentActuator_R
# dwdt = solve(platform.I,momentTotal)
#
# platform.alpha = dwdt
#
# dbetadtdt = dwdt[0]
# dgamdtdt = dwdt[2]
#
# return [domgegadt_l,omega_l,domgegadt_r,omega_r, dgamdtdt, dbetadtdt, gam_dot, beta_dot]    #
