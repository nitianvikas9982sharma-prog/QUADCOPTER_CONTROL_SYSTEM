""""
┌─────────────────────────────────┐
│(xd, yd, zd) [Desired Trajectory]│       
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│Position Error(x_err,y_err,z_err)│       
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│     Outer Loop Position PID     │
└─────────────────────────────────┘
               ├──► Total Thrust (u1 / Fz) 
               ▼
┌─────────────────────────────────┐
│ Inverse Kinematics / Mapping    │       
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│ Desired Attitude (ϕd, θd, ψd)   │       
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│Attitude Error(ϕ_err,θ_err,ψ_err)│       
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│    Inner Loop Attitude PID      │
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│  Control Torques (τx, τy, τz)   │
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│  Motor Mixer Matrix /Allocation │ 
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│ Motor Speeds (ω1, ω2, ω3, ω4)   |
└─────────────────────────────────┘
               ▼
┌─────────────────────────────────┐
│   12_parameters_update box      |
└─────────────────────────────────┘
"""
import sys
import os
# ============================================================
# PROJECT PATH
# ============================================================
sys.path.append(
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..')
    )
)
import math
import numpy as np
from PID_CONTROLLER_6DOF.PITCH_PID import pitchpid
from PID_CONTROLLER_6DOF.ROLL_PID import rollpid
from PID_CONTROLLER_6DOF.YAW_PID import yawpid
from PID_CONTROLLER_6DOF.POSITION_Z_PID import zpid
from PID_CONTROLLER_6DOF.zout_XY_PID import  xypid
from PID_CONTROLLER_6DOF.POSITION_X_PID import  xpid
from PID_CONTROLLER_6DOF.POSITION_Y_PID import  ypid
from MOTOR_MIXER import motor_mixer
from I2_parameters import parameters12


class pidmain:
    def __init__(self,inital_position,final_position,total_time):
        #================
        #parmaters
        #================
        g = 9.81         # Gravity (m/s^2)
        m = 0.468        # Mass (kg)
        dt=0.01
        steps = int(total_time / dt)  # 1000 timestamps
        time_vec = np.linspace(0, total_time, steps)

        inital_X=[inital_position[0],inital_position[1],inital_position[2],0,0,0,0,0,0,0,0,0]
        final_X=[final_position[0],final_position[1],final_position[2],0,0,0,0,0,0,0,0,0]


        present_X=inital_X
        X_hist=[]
        X_hist.append(inital_X)

        #====================================
        #Inital errors
        #====================================
        previous_xerror=final_X[0]-inital_X[0]
        previous_yerror=final_X[1]-inital_X[1]
        previous_zerror=final_X[2]-inital_X[2]

        previous_phierror=final_X[6]-inital_X[6]
        previous_thetaerror=final_X[7]-inital_X[7]
        previous_psierror=final_X[8]-inital_X[8]

        #=====================
        #Inital integral error
        #=====================
        integral_xerror=0
        integral_yerror=0
        integral_zerror=0

        integral_phierror=0
        integral_thetaerror=0
        integral_psierror=0

        #======================
        #previous thurst,torque
        #======================
        previous_rolltorque=0
        previous_pitchtorque=0
        previous_yawtorque=0
        previous_thrust=m*g


        for i in range(steps-1):
            #=======================================================================================
        
            #-------------------------
            # x,y to desired phi,theta
            #-------------------------
            sxy=xypid(final_X[0],present_X[0],previous_xerror,integral_xerror,
                    final_X[1],present_X[1],previous_yerror,integral_yerror)
            d_phi,d_theta=sxy.angleconvetor(present_X[8])

            previous_xerror=sxy.xerror
            previous_yerror=sxy.yerror

            integral_xerror=sxy.xintegral
            integral_yerror=sxy.yintegral
            #=======================================================================================
            
            #-------------------
            #phi to tou about x
            #-------------------
            sroll=rollpid()
            sroll.normalpid(desired_phi=d_phi,actual_phi=present_X[6],
                        integral_error=integral_phierror,previous_error=previous_phierror)

            torque_x=sroll.torque_rate_limit(previous_rolltorque)

            previous_rolltorque=torque_x
            previous_phierror=sroll.error
            integral_phierror=sroll.conditionalclamp()
            #========================================================================================

            #---------------------
            #theta to tou about y
            #---------------------
            spitch=pitchpid()
            spitch.normalpid(desired_theta=d_theta,actual_theta=present_X[7],
                            integral_error=integral_thetaerror,previous_error=previous_thetaerror)

            torque_y=spitch.torque_rate_limit(previous_pitchtorque)

            previous_pitchtorque=torque_y
            previous_thetaerror=spitch.error
            integral_thetaerror=spitch.conditionalclamp()
            #========================================================================================

            #---------------------
            #psi to tou about z
            #---------------------
            syaw=yawpid()
            syaw.normalpid(desired_psi=final_X[8],actual_psi=present_X[8],
                        integral_error=integral_psierror,previous_error=previous_psierror)

            torque_z=syaw.torque_rate_limit(previous_yawtorque)

            previous_yawtorque=torque_z
            previous_psierror=syaw.error
            integral_psierror=syaw.conditionalclamp()
            #========================================================================================

            #---------------
            # Z TO THRUST
            #---------------
            sz=zpid()
            sz.normalpid(desired_z=final_X[2],actual_z=present_X[2],integral_error=integral_zerror
                        ,previous_error=previous_zerror,phi=present_X[6],theta=present_X[7])

            thrust=sz.thrust_rate_limit(previous_thrust)

            previous_thrust=thrust
            previous_zerror=sz.error
            integral_zerror=sz.conditionalclamp()
            #========================================================================================
            #-------------------------------
            #[T,Tou>>x,y,z] to [w1,w2,w3,w4]
            #-------------------------------
            u,error=motor_mixer(thrust,torque_x,torque_y,torque_z)

            #=======================================================================================
            #-----------------------
            #current_X to updated_X
            #-----------------------
            sp=parameters12(present_X,u)
            updated_X=sp.X
            present_X=updated_X 
            X_hist.append(updated_X)
        self.X_hist=X_hist
        self.time_vec=time_vec

#=========================================================================================

if __name__ == '__main__':
    #================
    #INITAL_POSITION
    #================
    inital_position=[float(x) for x in input("Enter the inital location of drone :").split()]
    final_position=[float(x) for x in input("Enter the final location of drone :").split()]

    s=pidmain(inital_position,final_position,10)
    X_hist=s.X_hist
    time_vec=s.time_vec

    #===============
    # 3D_Visulation
    #===============
    from Visulation_3D import DroneVisualizer as DV
    DV(X_hist, time_vec).run()

    
    #=================================
    # 6DOF_PLOTS=[x,y,z,phi,theta,psi]
    #=================================
    from PLOT_6DOF import plot_flight_states as plot
    plot(X_hist,time_vec)

#=========================================================================================







    



    
   




    
    
    











