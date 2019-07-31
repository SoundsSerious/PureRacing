from ottermatics import *
from matplotlib.pylab import *
from numpy.linalg import  *
from scipy import integrate as scint

#Geometry
W = 0.32*2.0
L= 0.51

gear_r = 0.15
gear_l = 0.2

x_max = gear_r
x_min = -gear_r

A = array(   [[1.0 / (L*2.0), 1.0 / (L*2.0)],
                 [-1.0 / W, 1.0 / W]          ])
Ainv = inv(A)
#B = gamma , beta
#X = x1, x2

class PitchRollController(Configuration):

    gamma = 0
    beta = 0

    @property
    def B(self):
        return [self.gamma,self.beta]

    @property
    def X(self):
        X_ =  dot(Ainv, self.B)
        x1 = X_[0]
        x2 = X_[1]
        b_x1_min = x1 < x_min
        b_x1_max = x1 > x_max
        b_x2_max = x2 > x_max
        b_x2_min = x2 < x_min
        minmax = lambda x: min(max(x,x_min),x_max)
        if any([b_x1_max,b_x1_min]) and any([b_x2_max,b_x2_min]):
            #We're On Two Limits:
            print 'Were on two limits'
            return [x_max if b_x1_max else x_min, x_max if b_x2_max else x_min]
        elif any([b_x1_max,b_x1_min]) or any([b_x2_max,b_x2_min]):
            if any([b_x1_max,b_x1_min]):
                x1_limval = x_max if b_x1_max else x_min
                x2_gam = minmax((self.gamma - x1_limval/(L*2.)) * L * 2.)
                x2_beta = minmax((self.beta + x1_limval/W) * W)
                x2_avg = (x2_gam + x2_beta)/2.0 #Avergae requests on the limit
                print 'Were on x1 limits: {:3.4f},{:3.4f}->{:3.4f}'\
                                    .format(x2_gam,x2_beta,x2_avg)
                return [ x1_limval , x2_avg]
            elif any([b_x2_max,b_x2_min]):

                x2_limval = x_max if b_x2_max else x_min
                x1_gam = minmax((self.gamma - x2_limval/(L*2.)) * L * 2.)
                x1_beta = minmax((self.beta + x2_limval/W) * W)

                x1_avg = (x1_gam + x1_beta)/2.0 #Avergae requests on the limit
                print 'Were on x2 limits: {:3.4f},{:3.4f}->{:3.4f}'\
                                    .format(x1_gam,x1_beta,x1_avg)
                return [ x1_avg , x2_limval]
        return [x1,x2]


c = PitchRollController()

if __name__ == '__main__':
    Y = []
    X = linspace(-30,30)
    for gdeg in X:
        with c.difference(gamma = deg2rad(gdeg), beta = deg2rad(-5)):
          Y.append([nan,nan])
          Y[-1][0] = c.X[0]
          Y[-1][1] = c.X[1]
    Y = array(Y)
    figure()
    plot(X,Y[:,0],label='x1')
    plot(X,Y[:,1],label='x2')
    legend()
    grid()
    show()
