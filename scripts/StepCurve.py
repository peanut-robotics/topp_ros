#! /usr/bin/env python
"""
Class that creates a step curve 
Designed to be used as sampling function for toppra's JointVelocityConstraintVarying class

@author: Mrunal
"""

import numpy as np
from math import floor

class StepCurve(object):
    def __init__(self, x_data, y_data):
        self.x_data = x_data
        self.y_data = y_data
        self.n = len(self.y_data)
        self.dof = len(self.y_data[0])

        assert len(self.x_data) == len(self.y_data)
        
    def __call__(self, x):
        """
        :param x: float
        :return np.array of size dof by 2 

        Data returned simulates a step function which changes value 
        at every time defined by x_data
        """
        if x < self.x_data[0]:
            print("Cannot sample point at x: {}".format(x))
            return None
            
        # Find leftmost point closest to x
        x_left = min(self.n - 1,int(floor(x)))
        y = np.zeros((self.dof,2))
        for i in range(self.dof):
            y[i,:] = self.y_data[x_left][i]
        
        return y

if __name__ == "__main__":
    x_data = [0,1,2]
    y_data = [
                [[-1, 1], [-10, 10], [-100, 100], [-1000, 1000]],
                [[-2, 2], [-20, 20], [-200, 200], [-2000, 2000]],
                [[-3, 3], [-30, 30], [-300, 300], [-3000, 3000]]
            ]

    curve = StepCurve(x_data, y_data)
    print(curve(0.1))