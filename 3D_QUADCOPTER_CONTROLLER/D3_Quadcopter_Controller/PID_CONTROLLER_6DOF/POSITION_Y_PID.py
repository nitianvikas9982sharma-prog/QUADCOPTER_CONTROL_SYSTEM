import math
import numpy as np

# ===========================
# POSITION Y-PID (OUTER LOOP)
# ===========================
class ypid:
    # Taking inputs that are constant over time
    # ============
    # CONSTRUCTOR 
    # ============
    def __init__(self, K_y, max_roll_angle, min_roll_angle, dt=0.01):
        self.kp = K_y[0]
        self.ki = K_y[1]
        self.kd = K_y[2]

        # Pitch saturation limits in radians (e.g., +15 deg to -15 deg)
        self.max_roll = max_roll_angle
        self.min_roll = min_roll_angle
        self.dt = dt

        
    # ============
    # IDEAL_PID
    # ============
    def normal_pid(self,desired_y,actual_y,previous_error,integral_error):
        # ============
        # Error terms
        # ============
        self.error = desired_y - actual_y
        self.integral = integral_error + self.error * self.dt
        self.derivative = (self.error - previous_error) / self.dt

        # ==================================
        # VIRTUAL ACCELERATION COMMAND (u_y)
        # ==================================
        return self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
        