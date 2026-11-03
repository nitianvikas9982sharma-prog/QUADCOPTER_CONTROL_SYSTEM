import math
import numpy as np
import control as ct

# ===========================
# POSITION Z (INNER_LOOP)
# ===========================
class zlqr:
    # Taking inputs that are constant over time
    # ===============
    # CONSTRUCTOR 
    # ===============
    def __init__(self,max_thrust,min_thrust):
        #m*zddot=T-m*g=delT
        #xdot=A*x+B*u
        #u=delta
        self.dt=0.01
        self.max_thrust=max_thrust
        self.min_thrust=min_thrust
        self.m=0.468
        self.g=9.81
        self.A=np.array([[0.0,1.0],[0.0,0.0]])
        self.B=np.array([[0.0],[1/self.m]])

        #===============================================
        #for optimise cost function we have to tune Q,R
        #===============================================
        self.Q=np.array([[1.0,0.0],[0.0,2.0]])
        self.R=np.array([1])
        self.k,_,_=ct.lqr(self.A,self.B,self.Q,self.R)

    
    # ============
    # IDEAL_LQR
    # ============
    def normal_lqr(self,desired_z,actual_z):
           
            # ===============
            # OUTPUT THRUST
            # ===============
            self.thrust=(-self.k@(actual_z-desired_z)).item()+self.m*self.g

            #==================
            # SATURATION THRUST
            #==================
            self.thrust_actual=max(min(self.max_thrust,self.thrust),self.min_thrust)
            return self.thrust_actual

    #=================
    # MAX_RATE_THRUST
    #=================
    def thrust_rate_limit(self,previous_thrust,rate_max=40):
    
            max_change=rate_max*self.dt
    
            change=self.thrust_actual-previous_thrust
    
            if change>max_change:
                change=max_change
    
            elif change < -max_change:
                change=-max_change
    
            thrust_command=previous_thrust+change
    
            return thrust_command