import math
import numpy as np
# Querregelung
# PID Regler - Soll-Kruemmung

# Parameter
K_P_Y = -0.3 # Proportional term based on error of lateral distance
K_D_Y = 0.0 # Differential term based on error of lateral distance
K_I_Y = 0.0 # Integral term based on error of lateral distance
K_P_A = 0.0 # Proportional term based on error of heading


class Querregelung(object): 
    """
    Lateral control according to PID using error of lateral distance and heading.
    """
    def __init__(self):
        self._K_P_Y = K_P_Y
        self._K_D_Y = K_D_Y
        self._K_I_Y = K_I_Y
        self._K_P_A = K_P_A
        self.error_y = 0.0
        self.error_integral_y = 0.0
        self.error_derivative_y = 0.0

    def qrgTrg(self, Delta_y, Delta_heading):
        previous_error_y = self.error_y
        self.error_y = Delta_y
        self.error_integral_y = np.clip(self.error_integral_y+ self.error_y, -40.0, 40.0)
        self.error_derivative_y = self.error_y - previous_error_y
        Curvature = Delta_y * self._K_P_Y + Delta_heading * self._K_P_A + self.error_integral_y * self._K_I_Y + self.error_derivative_y* self._K_D_Y
        return Curvature

    def qrgDeviation(self, Xref, Yref, Headingref, Xcurr, Ycurr, Headingcurr):
        Delta_y = (Xref - Xcurr) * math.sin(Headingref) + (Ycurr - Yref) * math.cos(Headingref)
        Delta_heading = Headingcurr - Headingref
        return Delta_y, Delta_heading

    def qrgBfr(self, Xref, Yref, Headingref, Xcurr, Ycurr, Headingcurr):
        Delta_y, Delta_heading = self.qrgDeviation(Xref, Yref, Headingref, Xcurr, Ycurr, Headingcurr)
        Curvature = self.qrgTrg(Delta_y, Delta_heading)
        return Curvature