#====================
#PITCH PID
#====================
class pitchpid:
    def __init__(self,max_tou_pitch=0.543,min_tou_pitch=-0.543,K_pitch=[64.0,0.5,11.0],Iyy=4.856e-3,dt=0.01):
        self.kp=K_pitch[0]
        self.ki=K_pitch[1]
        self.kd=K_pitch[2]
        self.min_tou=min_tou_pitch
        self.max_tou=max_tou_pitch
        self.Iyy=Iyy
        self.dt=dt

    #================
    #NORMAL PID
    #===============
    def normalpid(self,desired_theta,actual_theta,integral_error,previous_error):
        self.error=desired_theta-actual_theta
        self.integral=integral_error+self.error*self.dt
        self.derivative=(self.error-previous_error)/self.dt

        #==============
        #PITCH_TORQUE
        #==============
        self.torque=(self.kp*self.error+self.ki*self.integral+self.kd*self.derivative)*self.Iyy

        #==============
        #SATURATION_TOU
        #==============
        self.torque_actual=max(min(self.max_tou,self.torque),self.min_tou)

    #==================
    #ANTI-WINDU>>CLAMP
    #==================
    def conditionalclamp(self):
        #torque=torque by pid
        #torque_actual is after passing the saturation box

        # Upper saturation
        if self.torque > self.max_tou:
            if self.error > 0:
                self.integral -= self.error * self.dt

        # Lower saturation
        elif self.torque < self.min_tou:
            if self.error < 0:
                self.integral -= self.error * self.dt

        return self.integral

    #=====================
    #ANTI-WINDUP>>BACK_CAL
    #=====================
    def backcalculation(self,kaw):
        correction=kaw*(self.torque_actual-self.torque)
        self.integral=self.integral+correction*self.dt
        return self.integral

    #=================
    # MAX_RATE_TORQUE
    #=================
    def torque_rate_limit(self,previous_torque,rate_max=10.9):

        max_change=rate_max*self.dt
        change=self.torque_actual-previous_torque

        if change>max_change:
            change = max_change
        elif change < -max_change:
            change = -max_change

        torque_command = previous_torque + change
        return torque_command

