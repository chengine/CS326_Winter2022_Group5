import numpy as np

class Ramsete():

    def __init__(self, b, zeta):

        self.b = b
        self.zeta = zeta

    def calculate(self, x_goal, v_goal, w_goal, x):

        x_diff = x_goal - x

        theta_err = (x_diff[2] + np.pi) % (2 * np.pi) - np.pi
        x_err = np.cos(x[2]) * x_diff[0] + np.sin(x[2]) * x_diff[1]
        y_err = -np.sin(x[2]) * x_diff[0] + np.cos(x[2]) * x_diff[1]

        k = 2 * self.zeta * np.sqrt(w_goal ** 2 + self.b * v_goal ** 2)
        v = v_goal * np.cos(theta_err) + k * x_err
        w = w_goal + k * theta_err + self.b * v_goal * np.sinc(theta_err) * y_err

        return v, w