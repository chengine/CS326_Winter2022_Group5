import numpy as np
from utils import wrap_to_pi

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1, k2, k3):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3

    def calculate(self, x_goal, x):

        self.x_g, self.y_g, self.th_g = x_goal
        x, y, th = x

        rho = np.sqrt((x-self.x_g)**2+(y-self.y_g)**2)
        delta = wrap_to_pi(np.arctan2(self.y_g-y, self.x_g-x)-self.th_g)
        alpha = wrap_to_pi(delta-(th-self.th_g))

        V = self.k1*rho*np.cos(alpha)
        om = self.k2*alpha + self.k1*np.sinc(alpha/np.pi)*np.cos(alpha)*(alpha+self.k3*delta)
        
        return V, om
