import math
import numpy as np
import control as ct

# ===========================
# ATTITUDE YAW CONTROL
# ===========================
class yawlqr:
    # Taking inputs that are constant over time
    # ===============
    # CONSTRUCTOR 
    # ===============
    def __init__(self,max_torque,min_torque):
    
    # Yaw LQR [psi, psi_dot] -> torque_zz ---
        
        self.dt=0.01
        self.max_torque=max_torque
        self.min_torque=min_torque
        self.m=0.468
        self.I_zz=8.81e-3
        self.g=9.81
        self.A=np.array([[0.0,1.0],[0.0,0.0]])
        self.B=np.array([[0.0],[1/self.I_zz]])

        #===============================================
        #for optimise cost function we have to tune Q,R
        #===============================================
        self.Q=np.diag([0.5,0.01])
        self.R=np.array([1.0])
        self.k,_,_=ct.lqr(self.A,self.B,self.Q,self.R)

    
    # ============
    # IDEAL_LQR
    # ============
    def normal_lqr(self,psi_des,psi):
           
            # ===============
            # OUTPUT THRUST
            # ===============
            self.tau_zz=(-self.k@(psi-psi_des)).item()

            #==================
            # SATURATION THRUST
            #==================
            self.tau_z_actual=max(min(self.max_torque,self.tau_zz),self.min_torque)
            return self.tau_z_actual

    #====================
    # MAX_RATE_TORQUE_ZZ
    #====================
    def torque_rate_limit(self,previous_torque,rate_max=40):
    
            max_change=rate_max*self.dt
    
            change=self.tau_z_actual-previous_torque
    
            if change>max_change:
                change=max_change
    
            elif change < -max_change:
                change=-max_change
    
            torque_command=previous_torque+change
    
            return torque_command