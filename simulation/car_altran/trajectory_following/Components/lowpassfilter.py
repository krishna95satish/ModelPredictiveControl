import math
import numpy as np
# online low pass filter
"""
This function provides a lowpass filter working according to the following formula:
y_1 = K*(u_1-IV)+IV
y_i = K*(u_i-y_i-1)+y_i-1
K = attenuation factor (The value of K must be in the range [0...1])
u = input value
y = output value
IV = inital value
i = timestep
"""
class LowpassConst(object):
    def __init__(self, K=1.0, IV=0.0):
        self._K = K
        self._IV = IV
    
    def run_step(self, u_i):
        y_i = self._K*(u_i-self._IV)+self._IV
        self._IV = y_i
        return y_i