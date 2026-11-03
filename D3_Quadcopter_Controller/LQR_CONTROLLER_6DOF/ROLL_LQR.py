import math
import numpy as np
import control as ct

# ===========================
# ATTITUDE ROLL CONTROL
# ===========================
class rolllqr:
    # Taking inputs that are constant over time
    # ===============
    # CONSTRUCTOR 
    # ===============
    def __init__(self,max_torque,min_torque):
    
    # Roll LQR [phi, phi_dot] -> torque_xx ---
        
        self.dt=0.01
        self.max_torque=max_torque
        self.min_torque=min_torque
        self.m=0.468
        self.I_xx=4.85e-3
        self.g=9.81
        self.A=np.array([[0.0,1.0],[0.0,0.0]])
        self.B=np.array([[0.0],[1/self.I_xx]])

        #===============================================
        #for optimise cost function we have to tune Q,R
        #===============================================
        self.Q=np.diag([1.0,0.01])
        self.R=np.array([1.0])
        self.k,_,_=ct.lqr(self.A,self.B,self.Q,self.R)

    
    # ============
    # IDEAL_LQR
    # ============
    def normal_lqr(self,phi_des,phi):
           
            # ===============
            # OUTPUT THRUST
            # ===============
            self.tau_xx=(-self.k@(phi-phi_des)).item()

            #==================
            # SATURATION THRUST
            #==================
            self.tau_x_actual=max(min(self.max_torque,self.tau_xx),self.min_torque)
            return self.tau_x_actual

    #====================
    # MAX_RATE_TORQUE_XX
    #====================
    def torque_rate_limit(self,previous_torque,rate_max=40):
    
            max_change=rate_max*self.dt
    
            change=self.tau_x_actual-previous_torque
    
            if change>max_change:
                change=max_change
    
            elif change < -max_change:
                change=-max_change
    
            torque_command=previous_torque+change
    
            return torque_command