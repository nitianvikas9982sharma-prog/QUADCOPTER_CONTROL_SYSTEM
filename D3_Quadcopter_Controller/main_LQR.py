import numpy as np
from LQR_CONTROLLER_6DOF.POSITION_XY_LQR import xylqr 
from LQR_CONTROLLER_6DOF.ROLL_LQR import rolllqr
from LQR_CONTROLLER_6DOF.PITCH_LQR import pitchlqr
from LQR_CONTROLLER_6DOF.POSITION_Z_LQR import zlqr
from LQR_CONTROLLER_6DOF.YAW_LQR import yawlqr
from MOTOR_MIXER import  motor_mixer
from I2_parameters import parameters12

class lqrmain:
    def __init__(self,inital_position,final_position,total_time):
        #================
        #parmaters
        #================
        g = 9.81         # Gravity (m/s^2)
        m = 0.468        # Mass (kg)
        dt=0.01
        steps = int(total_time / dt)  # 1000 timestamps
        time_vec = np.linspace(0, total_time, steps)

        #=================
        #INITAL_VELOCITIES
        #=================
        inital_vx,final_vx=0.0,0.0
        inital_vy,final_vy=0.0,0.0
        inital_vz,final_vz=0.0,0.0

        #=================
        #INITAL_STATES
        #=================
        initalstate_x=np.array([[inital_position[0]],[inital_vx]])
        initalstate_y=np.array([[inital_position[1]],[inital_vy]])
        initalstate_z=np.array([[inital_position[2]],[inital_vz]])

        #=================
        #FINAL_STATES
        #=================
        finalstate_x=np.array([[final_position[0]],[final_vx]])
        finalstate_y=np.array([[final_position[1]],[final_vy]])
        finalstate_z=np.array([[final_position[2]],[final_vz]])

        #===============================
        #X_hist,U=[T,touxx,touyy,touzz]
        #===============================
        X=[inital_position[0],inital_position[1],inital_position[2],inital_vx,inital_vy,inital_vz,0,0,0,0,0,0]
        X_hist=[X]

        initalstate_phi=np.array([[0],[0]])
        initalstate_theta=np.array([[0],[0]])
        initalstate_psi=np.array([[0],[0]])


        sxy=xylqr()
        sz=zlqr(9.66,0)
        spsi=yawlqr(0.185,-0.185)
        sroll=rolllqr(0.543,-0.543)
        spitch=pitchlqr(0.543,-0.543)
        
        for  i in range(steps-1):

            phi_des,theta_des=sxy.normal_lqr(finalstate_x,initalstate_x,finalstate_y,initalstate_y)

            max_tilt=0.7   # rad (~40 deg), keeps small-angle model valid
            phi_des=float(np.clip(phi_des,-max_tilt,max_tilt))
            theta_des=float(np.clip(theta_des,-max_tilt,max_tilt))

            finalstate_phi=np.array([[phi_des],[0]])
            
            finalstate_theta=np.array([[theta_des],[0]])

            tou_x=sroll.normal_lqr(finalstate_phi,initalstate_phi)

            tou_y=spitch.normal_lqr(finalstate_theta,initalstate_theta)

            thrust=sz.normal_lqr(finalstate_z,initalstate_z)

            finalstate_psi=np.array([[0],[0]])

            tou_z=spsi.normal_lqr(finalstate_psi,initalstate_psi)

            W,_=motor_mixer(thrust, tou_x, tou_y, tou_z)

            ss=parameters12(X,W)
            X_update=ss.X

            X=X_update
            X_hist.append(list(X))   # copy, so history entries can't alias one object

            initalstate_x=np.array([[X[0]],[X[3]]])
            initalstate_y=np.array([[X[1]],[X[4]]])
            initalstate_z=np.array([[X[2]],[X[5]]])
            initalstate_phi=np.array([[X[6]],[X[9]]])
            initalstate_theta=np.array([[X[7]],[X[10]]])
            initalstate_psi=np.array([[X[8]],[X[11]]])
        self.X_hist=X_hist
        self.time_vec=time_vec


if __name__ == '__main__':
    #================
    #INITAL_POSITION
    #================
    inital_position=[float(x) for x in input("Enter the inital location of drone :").split()]
    final_position=[float(x) for x in input("Enter the final location of drone :").split()]

    s=lqrmain(inital_position,final_position,10)
    X_hist=s.X_hist
    time_vec=s.time_vec

    #===============
    # 3D_Visulation
    #===============
    from Visulation_3D import DroneVisualizer as DV
    DV(X_hist, time_vec).run()

    from PLOT_6DOF import plot_flight_states as plo
    #=================================
    # 6DOF_PLOTS=[x,y,z,phi,theta,psi]
    #=================================
    plo(X_hist,time_vec)