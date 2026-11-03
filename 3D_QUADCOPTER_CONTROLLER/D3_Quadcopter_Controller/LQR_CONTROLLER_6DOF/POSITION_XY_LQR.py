import math
import numpy as np
import control as ct

# ===========================
# POSITION Z (INNER_LOOP)
# ===========================
class xylqr:
    # Taking inputs that are constant over time
    # ===============
    # CONSTRUCTOR 
    # ===============
    def __init__(self):
        #xdot=A*x+B*u
        #u for xlqr is theta ,u for ylqr is phi
        self.dt=0.01
        self.m=0.468
        self.g=9.81
        self.Ax=np.array([[0.0,1.0],[0.0,0.0]])
        self.Bx=np.array([[0.0],[self.g]])

        self.Ay=np.array([[0.0,1.0],[0.0,0.0]])
        self.By=np.array([[0.0],[-self.g]])

        #===============================================
        #for optimise cost function we have to tune Q,R
        #===============================================
        self.Qx=np.array([[0.05,0.0],[0.0,0.05]])
        self.Rx=np.array([1])

        self.Qy=np.array([[0.05,0.0],[0.0,0.05]])
        self.Ry=np.array([1])

        self.kx,_,_=ct.lqr(self.Ax,self.Bx,self.Qx,self.Rx)
        self.ky,_,_=ct.lqr(self.Ay,self.By,self.Qy,self.Ry)

    
    # ============
    # IDEAL_LQR
    # ============
    def normal_lqr(self,desired_x,actual_x,desired_y,actual_y):
           
            # =================
            # OUTPUT THETA,PHI
            # =================
            self.theta=(-self.kx@(actual_x-desired_x)).item()
            self.phi=(-self.ky@(actual_y-desired_y)).item()
            return self.phi ,self.theta