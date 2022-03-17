import math
import numpy as np
# Laengsregelung
# PID Regler - Soll-Beschleunigung

# Parameter
K_P_V = 5 # Proportional term based on error of speed
K_D_V = 0 # Differential term based on error of speed
K_I_V = 0.2 # Integral term based on error of speed
K_P_X = 0.5 # Proportional term based on error of distance


class Laengsregelung(object): 
    """
    Longitudinal control according to PID using error of speed and longitduinal distance.
    """
    def __init__(self):
        self._K_P_V = K_P_V
        self._K_D_V = K_D_V
        self._K_I_V = K_I_V
        self._K_P_X = K_P_X
        self.error_vel = 0.0
        self.error_integral_vel = 0.0
        self.error_derivative_vel = 0.0

    def lrgTrg(self, Delta_x, Delta_v):
        previous_error_vel = self.error_vel
        self.error_vel = Delta_v
        self.error_integral_vel = np.clip(self.error_integral_vel + self.error_vel, -40.0/3.6, 40.0/3.6)
        self.error_derivative_vel = self.error_vel - previous_error_vel
        a_fr = Delta_x * self._K_P_X + Delta_v * self._K_P_V + self.error_integral_vel * self._K_I_V + self.error_derivative_vel * self._K_D_V
        return a_fr

    def lrgDeviation(self, Xref, Yref, Headingref, Vref, Xcurr, Ycurr, Vcurr):
        Delta_x = (Xref - Xcurr) * math.cos(Headingref) + (Yref - Ycurr) * math.sin(Headingref)
        Delta_v = Vref - Vcurr
        return Delta_x, Delta_v

    def lrgBfr(self, Xref, Yref, Headingref, Vref, Xcurr, Ycurr, Vcurr):
        Delta_x, Delta_v = self.lrgDeviation(Xref, Yref, Headingref, Vref, Xcurr, Ycurr, Vcurr)
        a_fr = self.lrgTrg(Delta_x, Delta_v)
        return a_fr