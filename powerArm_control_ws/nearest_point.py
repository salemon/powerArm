import numpy as np
from scipy.optimize import minimize
import time

class nearest_point():
    def __init__(self):
        self.init_pose = np.array([1.2841, -0.15968, 1.0249])
        self.sample_left = np.array([1.2841, -0.15968, 1.0249])
        self.sample_right = np.array([1.2835, 0.16549, 1.0249])
        self.r = (self.sample_right[1] - self.sample_left[1]) / 2.0
        self.cur_pose = self.sample_left

        self.cons = ({'type': 'ineq', 'fun': lambda t: 2 * np.pi - t},
        {'type': 'ineq', 'fun': lambda t: t})
    
    def curve_parametric(self, t):
        x = 1.2841
        y = self.init_pose[1] + self.r * (1 - np.cos(t))
        z = self.init_pose[2] + self.r * np.sin(t)
        
        return np.array([x, y[0], z[0]])

    def distance_squared(self, t):
        curve_point = self.curve_parametric(t)
        diff_vector = curve_point - self.cur_pose

        return diff_vector @ diff_vector


# ic = impedance_control()

# result = minimize(ic.distance_squared, 0, method='SLSQP', constraints = ic.cons)

# print(ic.curve_parametric(result.x))
# print(result.x)
