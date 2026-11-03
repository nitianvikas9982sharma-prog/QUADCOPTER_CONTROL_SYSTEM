import math
from .POSITION_X_PID import  xpid
from .POSITION_Y_PID import  ypid
class xypid:
     
      def __init__(self,desired_x,actual_x,previous_xerror,integral_xerror,
                        desired_y,actual_y,previous_yerror,integral_yerror):
        
        
        s1=xpid([1.5,0.05,2.4],math.radians(40),math.radians(-40) ,0.01)    
        s2=ypid([1.5,0.05,2.4],math.radians(40),math.radians(-40),0.01)

        self.u_x=s1.normal_pid(desired_x,actual_x,previous_xerror,integral_xerror)
        self.u_y=s2.normal_pid(desired_y,actual_y,previous_yerror,integral_yerror)

        self.xerror=s1.error
        self.yerror=s2.error

        self.xintegral=s1.integral
        self.yintegral=s2.integral

      def angleconvetor(self,yaw_angle): 
        g=9.81
        max_pitch=max_roll=math.radians(40)     # tilt limit
        min_pitch=min_roll=-math.radians(40)


        # ==============================================
        # INERTIAL FRAME TO DESIRED PITCH & ROLL ANGLES 
        # ==============================================
        # Pitch: theta_d = u_x * cos(psi) + u_y * sin(psi)
        self.theta_des = ((self.u_x * math.cos(yaw_angle)) + (self.u_y * math.sin(yaw_angle)))/g

        # Roll: phi_d = u_x * sin(psi) - u_y * cos(psi)
        self.phi_des = ((self.u_x * math.sin(yaw_angle)) - (self.u_y * math.cos(yaw_angle)))/g

        # Apply pitch limit saturation
        self.theta_desired_actual = max(min(self.theta_des, max_pitch), min_pitch)

        # Apply roll limit saturation
        self.phi_desired_actual = max(min(self.phi_des, max_roll), min_roll)

        return self.phi_desired_actual,self.theta_desired_actual
        