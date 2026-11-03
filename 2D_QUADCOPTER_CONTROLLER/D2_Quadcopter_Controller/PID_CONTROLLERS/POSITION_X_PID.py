import math

class xzpid:
    def __init__(self, desired_x, actual_x, previous_xerror, integral_xerror,
                  max_tilt_deg=40.0,K_x=[1.5, 0.05, 2.4],dt=0.01):

        self.kp=K_x[0]
        self.ki=K_x[1]
        self.kd=K_x[2]
        self.dt=dt

        self.max_tilt=math.radians(max_tilt_deg)
        self.min_tilt= -self.max_tilt
        #=======================
        # Error calculation
        #=======================
        self.error = desired_x - actual_x
        self.integral = integral_xerror + self.error * self.dt
        self.derivative = (self.error - previous_xerror) / self.dt
        #======================
        # Virtual acceleration 
        #======================
        self.u_x = (self.kp * self.error+self.ki * self.integral +self.kd * self.derivative)

    def angleconvetor(self):
        g = 9.81
        #======================================
        # Virtual acceleration to desired pitch
        #======================================
        self.theta_des = self.u_x / g

        #======================================
        # Pitch angle saturation
        #======================================
        self.theta_actual = max(min(self.theta_des, self.max_tilt),self.min_tilt)
        return self.theta_actual
