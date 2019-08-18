from ottermatics import *
from matplotlib.pylab import *
from numpy.linalg import  *
from scipy import integrate as scint


#B = gamma , beta
#X = x1, x2

class PlatformControl(Configuration):

    #Geometry
    W = 0.32*2.0
    L= 0.51
    pivotToSimCG_Y = 0.68
    pivotToSimCG_Z = 0.1

    humanToPivotY = 0.35
    humanToPivotZ = -0.17

    spring_upper_L = -0.581
    spring_upper_W = 0.189

    spring_lower_L = -0.630
    spring_lower_W = 0.298
    spring_lower_H = -0.188

    gear_r = 0.08#0.0445
    gear_l = 0.12#0.0762

    spring_constant = 1.0
    unsprung_length = 0.2

    x_offset = 0.12

    _xl = gear_r + gear_l - x_offset
    _xr = -gear_r + gear_l - x_offset

    A = array(   [[1.0 / (L*2.0), 1.0 / (L*2.0)],
                     [-1.0 / W, 1.0 / W]          ])
    Ainv = inv(A)

    max_pitch_rate = 960 * pi / 180.0 / 60.0 # rpm to rad/s
    max_roll_rate = 960 * pi / 180.0 / 60.0 # rpm to rad/s
    gamma_goal = 0
    beta_goal = 0
    gamma_dmd = 0
    beta_dmd = 0

    omega = array([0,0,0])
    alpha = array([0,0,0])

    #Geometry
    Rl = array([-W/2.,0,L])
    Rr = array([W/2.,0,L])
    Rp = array([0,humanToPivotY,humanToPivotZ])
    Rg = array([0,pivotToSimCG_Y,pivotToSimCG_Z])

    Rsll = array([-spring_lower_W, spring_lower_H, spring_lower_L])
    Rslr = array([spring_lower_W, spring_lower_H, spring_lower_L])

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

    Iinv = inv(I)

    Astatic = array([[-W/2, W/2],
                     [L, L]])
    Astatic_inv = inv(Astatic)

    last_update = 0
    low_pass_alpha = 0.01
    #update_frequency = 1000. #hz
    #update_interval = (1.0 / update_frequency)



    @property
    def B_dmd(self):
        return [self.gamma_dmd,self.beta_dmd]

    @property
    def B(self):
       X_ =  dot(self.A, array([self.xl,self.xr]) )
       gamma = X_[0]
       beta = X_[1]
       return [gamma,beta]

    @property
    def gamma(self):
        return self.B[0]

    @property
    def beta(self):
        return self.B[1]

    @property
    def xl(self):
        return self._xl

    @xl.setter
    def xl(self,in_x):
        self._xl = min(max(in_x,self.x_min),self.x_max)

    @property
    def xr(self):
        return self._xr

    @xr.setter
    def xr(self,in_x):
        self._xr = min(max(in_x,self.x_min),self.x_max)

    @property
    def x_max(self):
        return self.gear_r + self.gear_l - self.x_offset

    @property
    def x_min(self):
        return -self.gear_r  + self.gear_l - self.x_offset

    @property
    def Rsul(self):
        return array([self.spring_upper_W*cos(self.beta), self.spring_upper_L*sin(self.gamma) +\
                    self.spring_upper_W*sin(self.beta),self.spring_upper_L*cos(self.gamma)])

    @property
    def Rsur(self):
        return array([-self.spring_upper_W*cos(self.beta), self.spring_upper_L*sin(self.gamma) -\
                        self.spring_upper_W*sin(self.beta),self.spring_upper_L*cos(self.gamma)])

    @property
    def Xspring_l(self):
        return self.Rsul - self.Rsll

    @property
    def Xspring_r(self):
        return self.Rsur - self.Rslr

    @property
    def Fspring_l(self):
        xl = self.Xspring_l
        Lxl = sqrt(xl.dot(xl))
        nxl = xl / Lxl
        F = nxl * (Lxl - self.unsprung_length) * self.spring_constant
        return F

    @property
    def Fspring_r(self):
        xr = self.Xspring_r
        Lxr = sqrt(xr.dot(xr))
        nxr = xr / Lxr
        F = nxr * (Lxr - self.unsprung_length) * self.spring_constant
        return F

    @property
    def X_dmd(self):
        X_ =  dot(self.Ainv, self.B_dmd)
        x1 = X_[0] #- self.x_offset
        x2 = X_[1] #- self.x_offset
        b_x1_min = x1 < self.x_min
        b_x1_max = x1 > self.x_max
        b_x2_max = x2 > self.x_max
        b_x2_min = x2 < self.x_min
        minmax = lambda x: min(max(x,self.x_min),self.x_max)
        if any([b_x1_max,b_x1_min]) and any([b_x2_max,b_x2_min]):
            #We're On Two Limits:
            # print 'Were on two limits g{:3.4f}|b{:3.4f}'.format(self.gamma_dmd,self.beta_dmd)
            return [self.x_max if b_x1_max else self.x_min, self.x_max if b_x2_max else self.x_min]
        elif any([b_x1_max,b_x1_min]) or any([b_x2_max,b_x2_min]):
            if any([b_x1_max,b_x1_min]):
                x1_limval = self.x_max if b_x1_max else self.x_min
                x2_gam = minmax((self.gamma_dmd - x1_limval/(self.L*2.)) * self.L * 2.)
                x2_beta = minmax((self.beta_dmd + x1_limval/self.W) * self.W)
                x2_avg = (x2_gam + x2_beta)/2.0 #Avergae requests on the limit
                # print 'Were on x1  limits g{:3.4f}|b{:3.4f}: {:3.4f},{:3.4f}->{:3.4f}'\
                #                    .format(self.gamma_dmd,self.beta_dmd,x2_gam,x2_beta,x2_avg)
                return [ x1_limval , x2_avg]
            elif any([b_x2_max,b_x2_min]):

                x2_limval = self.x_max if b_x2_max else self.x_min
                x1_gam = minmax((self.gamma_dmd - x2_limval/(self.L*2.)) * self.L * 2.)
                x1_beta = minmax((self.beta_dmd + x2_limval/self.W) * self.W)

                x1_avg = (x1_gam + x1_beta)/2.0 #Avergae requests on the limit
                # print 'Were on x2  limits g{:3.4f}|b{:3.4f}: {:3.4f},{:3.4f}->{:3.4f}'\
                #                    .format(self.gamma_dmd,self.beta_dmd,x1_gam,x1_beta,x1_avg)
                return [ x1_avg , x2_limval]
        return [x1,x2]

    @property
    def static_forces(self):
        Bstatic = array([self.pivotToSimCG_Y*self.beta *self.massSimulator*self.gravity + \
                        self.humanToPivotY * self.beta * self.massPerson * self.gravity,
                        self.pivotToSimCG_Z*self.massSimulator*self.gravity + \
                        self.humanToPivotZ * self.massPerson * self.gravity])
        return dot(self.Astatic_inv,Bstatic)

    @property
    def dynamic_forces(self):
        DynamicMoments = dot(self.I,self.alpha)
        Mgamma,My,Mbeta = DynamicMoments
        B = [Mbeta,Mgamma]

        #VelMoments = cross(self.omega,cross(self.I,self.omega))

        return dot(self.Astatic_inv,B)

    @property
    def spring_forces(self):
        Ml = cross(self.Rsul, self.Fspring_l)
        Mr = cross(self.Rsur, self.Fspring_r)
        M = Ml + Mr
        Mg = M[0]
        Mb = M[2]
        Bspring = [Mb,Mg]
        return dot(self.Astatic_inv,Bspring)

    def update_demanded_values(self,dt):
        #Bypass Code
        #self.beta_dmd = self.beta_goal
        #self.gamma_dmd = self.gamma_goal
        #return
        if dt > 0.0:
            dB = self.beta_goal - self.beta_dmd
            dG = self.gamma_goal - self.gamma_dmd
            #dA = sqrt(dB**2.0 + dG**2.0)
            #fB = dB / dA
            #fG = dG / dA
            self.beta_dmd += (dB if abs(dt*self.max_roll_rate) > abs(dB) \
                                 else (dt*self.max_roll_rate*sign(dB)))
            self.gamma_dmd += (dG if abs(dt*self.max_pitch_rate) > abs(dG) \
                                 else (dt*self.max_pitch_rate*sign(dG)))

    def update(self,time,xl,xr):
        self.dt = time - self.last_update
        self.update_demanded_values(self.dt)
        if self.dt >= 1E-4:
            self.lasttheta = array([self.gamma, 0, self.beta])
            self.lastomega = self.omega
            self.xl = xl
            self.xr = xr
            self.newtheta = array([self.gamma, 0, self.beta])
            self.omega = self.omega*(1.0-self.low_pass_alpha)+\
                         self.low_pass_alpha*(self.newtheta - self.lasttheta)/self.dt
            self.alpha = self.alpha*(1.0-self.low_pass_alpha)+\
                         self.low_pass_alpha*(self.omega - self.lastomega)/self.dt
            self.last_update = time






if __name__ == '__main__':
    c = PlatformControl()
    Y = []
    X = linspace(-30,30)
    for gdeg in X:
        with c.difference(gamma_dmd = deg2rad(-1), beta_dmd = deg2rad(gdeg)):
          Y.append([nan,nan])
          Y[-1][0] = c.X_dmd[0]
          Y[-1][1] = c.X_dmd[1]
    Y = array(Y)
    figure()
    plot(X,Y[:,0],label='x1')
    plot(X,Y[:,1],label='x2')
    legend()
    grid()
    show()
