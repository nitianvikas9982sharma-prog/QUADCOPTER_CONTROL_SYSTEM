import math
import numpy as np

# ===========================
# POSITION X-PID (OUTER LOOP)
# ===========================
class xpid:
    # Taking inputs that are constant over time
    # ============
    # CONSTRUCTOR 
    # ============
    def __init__(self, K_x, max_pitch_angle, min_pitch_angle, dt=0.01):
        self.kp = K_x[0]
        self.ki = K_x[1]
        self.kd = K_x[2]

        # Pitch saturation limits in radians (e.g., +15 deg to -15 deg)
        self.max_pitch = max_pitch_angle
        self.min_pitch = min_pitch_angle
        self.dt = dt
  
    # ============
    # IDEAL_PID
    # ============
    def normal_pid(self,desired_x,actual_x,previous_error,integral_error):
        # ============
        # Error terms
        # ============
        self.error = desired_x - actual_x
        self.integral = integral_error + self.error * self.dt
        self.derivative = (self.error - previous_error) / self.dt

        # ==================================
        # VIRTUAL ACCELERATION COMMAND (u_x)
        # ==================================
        return self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
      