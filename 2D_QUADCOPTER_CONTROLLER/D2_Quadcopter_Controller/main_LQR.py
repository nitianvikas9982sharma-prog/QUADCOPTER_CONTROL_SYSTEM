"""
TASK 2 - 2D (x-z plane) quadcopter with LQR

desired x    --> xlqr(POSITION_X_LQR.py) --> desired_theta                                            
desired_theta--> pitchlqr (PITCH_LQR.py) --> tau_y
desired z    --> zlqr(POSITION_Z_LQR.py) --> thrust T    
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
import numpy as np
from D2_Quadcopter_Controller.LQR_CONTROLLERS.POSITION_X_LQR import xlqr
from D2_Quadcopter_Controller.LQR_CONTROLLERS.PITCH_LQR import pitchlqr
from D2_Quadcopter_Controller.LQR_CONTROLLERS.POSITION_Z_LQR import zlqr
from Motor_Mixer_2d import motor_mixer
from Plant_2D import parameters2d

class lqrmain:
    def __init__(self, initial_position, final_position, total_time):

        dt = 0.01       
        steps = int(round(total_time/dt))
        time_vec = np.arange(steps) * dt

        # ======================
        # INITIAL/FINAL STATES
        # ======================
        initial_vx,initial_vz=0.0,0.0
        final_vx,final_vz =0.0,0.0

        state_x=np.array([[initial_position[0]],[initial_vx]])
        state_z = np.array([[initial_position[1]],[initial_vz]])
        state_theta = np.array([[0.0],[0.0]])

        finalstate_x=np.array([[final_position[0]],[final_vx]])
        finalstate_z=np.array([[final_position[1]],[final_vz]])

        # X=[x,z,xdot,zdot,theta,thetadot]
        X =[float(initial_position[0]),float(initial_position[1]),initial_vx,initial_vz,0.0,0.0]
        X_hist=[list(X)]

        self.thrust_hist,self.torque_hist =[],[]
        self.theta_cmd_hist,self.mixer_errors =[],[]

        sx = xlqr()
        sz = zlqr()
        spitch = pitchlqr()
        Max_pitch = 0.7   #in radian   40degree                


        for i in range(steps - 1):

            #x->desired pitch
            theta_des = sx.normal_lqr(finalstate_x,state_x)
            theta_des = float(np.clip(theta_des, -Max_pitch, Max_pitch))
            finalstate_theta = np.array([[theta_des], [0.0]])

            #theta->torque about y
            tau_y = spitch.normal_lqr(finalstate_theta, state_theta)

            #z -> thrust
            thrust = sz.normal_lqr(finalstate_z, state_z)

            #[T,tau_y]->[w1, w2]
            W, mixer_error = motor_mixer(thrust, tau_y)

            #current_X ->updated_X
            ss = parameters2d(X, W, dt)
            X = ss.X

            X_hist.append(list(X))
            self.thrust_hist.append(thrust)
            self.torque_hist.append(tau_y)
            self.theta_cmd_hist.append(theta_des)
            self.mixer_errors.append(mixer_error)

            state_x = np.array([[X[0]], [X[2]]])
            state_z = np.array([[X[1]], [X[3]]])
            state_theta = np.array([[X[4]], [X[5]]])

       
        self.X_hist = np.asarray(X_hist)                   # (N, 6)
        self.time_vec = time_vec
        self.thrust_hist = np.asarray(self.thrust_hist)
        self.torque_hist = np.asarray(self.torque_hist)
        self.theta_cmd_hist = np.asarray(self.theta_cmd_hist)
        self.mixer_errors = np.asarray(self.mixer_errors)
        self.desired = np.tile([final_position[0], final_position[1]],
                               (len(time_vec), 1))         # (N, 2)


if __name__ == '__main__':
    initial_position = [float(v) for v in input("Enter the initial location of drone [x z] : ").split()]
    final_position = [float(v) for v in input("Enter the final location of drone [x z] : ").split()]

    s = lqrmain(initial_position, final_position, 10)

    # ================
    # 2D visualisation
    # ================
    from D2_Quadcopter_Controller.Visulation_2D import DroneVisualizer2D as DV
    DV(s.X_hist, s.time_vec, desired=s.desired).run()

    # =======================================
    # PLOTS = [x, z, theta, x-z path, inputs]
    # =======================================
    from Plot_2D import plot_flight_states_2d as plo
    plo(s.X_hist, s.time_vec,)