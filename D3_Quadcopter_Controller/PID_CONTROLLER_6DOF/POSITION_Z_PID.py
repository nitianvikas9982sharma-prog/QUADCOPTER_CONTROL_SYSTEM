#====================
# Z PID
#====================
import math
class zpid:
    def __init__(self,max_thrust_z=9.66,min_thrust_z=0,K_z=[8.0,0.3,6.0],m=0.468,dt=0.01):

        self.kp=K_z[0]
        self.ki=K_z[1]
        self.kd=K_z[2]

        self.min_thrust=min_thrust_z
        self.max_thrust=max_thrust_z

        self.m=m
        self.g=9.81
        self.dt=dt

    #================
    # NORMAL PID
    #===============
    def normalpid(self,desired_z,actual_z,integral_error,previous_error,phi,theta):

        self.error=desired_z-actual_z
        self.integral=integral_error+self.error*self.dt
        self.derivative=(self.error-previous_error)/self.dt

        #=======================
        # Z ACCELERATION COMMAND
        #=======================
        self.acceleration=(self.kp*self.error+self.ki*self.integral+self.kd*self.derivative)

        #================
        # THRUST
        #================
        self.thrust=(self.g+self.acceleration)*self.m/max(math.cos(phi)*math.cos(theta),0.5)

        #==================
        # SATURATION THRUST
        #==================
        self.thrust_actual=max(min(self.max_thrust,self.thrust),self.min_thrust)

    #=====================
    # ANTI-WINDUP >> CLAMP
    #=====================
    def conditionalclamp(self):

        # thrust = thrust calculated by PID
        # thrust_actual = thrust after saturation

        # Upper saturation
        if self.thrust > self.max_thrust:

            if self.error > 0:
                self.integral -= self.error*self.dt

        # Lower saturation
        elif self.thrust < self.min_thrust:

            if self.error < 0:
                self.integral -= self.error*self.dt

        return self.integral

    #=====================
    # ANTI-WINDUP >> BACK_CAL
    #=====================
    def backcalculation(self,kaw):

        correction=kaw*(self.thrust_actual-self.thrust)

        self.integral=self.integral+correction*self.dt

        return self.integral

    #=================
    # MAX_RATE_THRUST
    #=================
    def thrust_rate_limit(self,previous_thrust,rate_max=96.6):

        max_change=rate_max*self.dt

        change=self.thrust_actual-previous_thrust

        if change>max_change:
            change=max_change

        elif change < -max_change:
            change=-max_change

        thrust_command=previous_thrust+change

        return thrust_command