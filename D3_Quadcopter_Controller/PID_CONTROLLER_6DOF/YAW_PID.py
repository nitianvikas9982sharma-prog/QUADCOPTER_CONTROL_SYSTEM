import math
# ===================
# YAW ANGLE WRAPPING
# ===================
def warp_pi(angle):
    return (angle+math.pi)%(2*math.pi)-math.pi

class yawpid:
    #taking input which are not going to change which time
    #============
    #Construtcor 
    #============
    def __init__(self,K_yaw=[8.0,0.1,4.0],max_yaw_torque=0.185,min_yaw_torque=-0.185,Izz=8.801e-3,dt=0.01):
        self.kp=K_yaw[0]
        self.ki=K_yaw[1]
        self.kd=K_yaw[2]

        self.max_tou=max_yaw_torque
        self.min_tou=min_yaw_torque
        self.Izz=Izz
        self.dt=dt
    #===========
    #IDEAL_PID
    #===========
    def normalpid(self,desired_psi,actual_psi,previous_error,integral_error):

        #============
        #Error terms
        #============
        self.error=warp_pi(desired_psi-actual_psi)
        self.integral=integral_error+self.error*(self.dt)
        self.derivative=(self.error-previous_error)/(self.dt)

        #===========
        #YAW_TORQUE
        #===========
        self.torque=(self.kp*self.error+self.ki*self.integral+self.kd*self.derivative)*self.Izz
        self.torque_actual= max(min(self.torque,self.max_tou),self.min_tou)
        return self.torque_actual

    #==================
    #ANTI_WINDU>>CLAMP
    #==================
    def conditionalclamp(self):
        #torque=torque by pid
        #torque_actual is after passing the saturation  box
          

        # Upper saturation
        if self.torque > self.max_tou:
            if self.error > 0:  #Error is pushing torque further upward
                self.integral -= self.error * self.dt

        # Lower saturation
        elif self.torque < self.min_tou:
            if self.error < 0:  # Error is pushing torque further downward
               self.integral -= self.error * self.dt

        return self.integral  
    
    #===================== 
    #ANTI_WINDUP>>BACK_CAL
    #=====================
    def backcalculation(self,kaw):
        correction=kaw*(self.torque_actual-self.torque)
        self.integral=self.integral+correction*self.dt
        return self.integral

     #=================
     # MAX_RATE_TORQUE
     #=================   
    def torque_rate_limit(self,previous_torque,rate_max=3.7):

        max_change=rate_max*self.dt
        change=self.torque_actual-previous_torque

        if change>max_change:
            change = max_change
        elif change < -max_change:
            change = -max_change

        torque_command = previous_torque + change
        return torque_command 