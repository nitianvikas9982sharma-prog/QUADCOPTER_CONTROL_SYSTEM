# ===================
# PID MAIN
# ===================
import sys
import os

sys.path.append(
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..')
    )
)
from Plant_2D import parameters2d
from Motor_Mixer_2d import motor_mixer
from D2_Quadcopter_Controller.PID_CONTROLLERS.PITCH_PID import pitchpid
from D2_Quadcopter_Controller.PID_CONTROLLERS.POSITION_Z_PID import zpid
from D2_Quadcopter_Controller.PID_CONTROLLERS.POSITION_X_PID import xzpid

#===================
# PARAMETERS
#===================
g=9.81
mass=0.468
dt=0.01


class pidmain:
    def __init__(self, initial_position, final_position, total_time=10.0):

        # ===================
        # SIMULATION SETTINGS
        # ===================
        steps = int(total_time / dt)
        self.time_vec = [i * dt for i in range(steps)]

        # State: [x, z, x_dot, z_dot, theta, theta_dot]
        present_X = [initial_position[0],initial_position[1],0.0, 0.0, 0.0, 0.0]
        X_hist = [present_X.copy()]

        #===================
        # INITIAL ERRORS
        #===================
        previous_xerror=final_position[0]-present_X[0]
        previous_zerror=final_position[1]-present_X[1]
        previous_thetaerror=0.0

        #=========================
        # INITIAL INTEGRAL ERRORS
        #=========================
        integral_xerror=0.0
        integral_zerror=0.0
        integral_thetaerror=0.0

        #=========================
        # PREVIOUS CONTROL INPUTS
        #=========================
        previous_pitchtorque=0.0
        previous_thrust=mass*g

        #=========================
        # HISTORIES
        #=========================
        self.command_history = []
        self.mixer_errors = []

        #=========================
        # MAIN SIMULATION LOOP
        #=========================
        for i in range(steps - 1):
            # -------------------------
            # X -> DESIRED PITCH
            # -------------------------
            sx = xzpid( final_position[0],present_X[0],previous_xerror, integral_xerror)
            desired_theta = sx.angleconvetor()

            previous_xerror = sx.error
            integral_xerror = sx.integral
            self.command_history.append(desired_theta)

            # -------------------------
            # PITCH -> TORQUE ABOUT Y
            # -------------------------
            spitch = pitchpid()
            spitch.normalpid(desired_theta,present_X[4],integral_thetaerror,previous_thetaerror)
            torque_y = spitch.torque_rate_limit(previous_pitchtorque)

            previous_pitchtorque = torque_y
            previous_thetaerror = spitch.error
            integral_thetaerror = spitch.conditionalclamp()

            # -------------------------
            # Z -> TOTAL THRUST
            # -------------------------
            sz = zpid()
            sz.normalpid(final_position[1],present_X[1],integral_zerror,previous_zerror,0.0,present_X[4])
            thrust = sz.thrust_rate_limit(previous_thrust)

            previous_thrust=thrust
            previous_zerror=sz.error
            integral_zerror=sz.conditionalclamp()

            # -------------------------
            # THRUST + TORQUE -> MOTORS
            # -------------------------
            u,mixer_error=motor_mixer(thrust,torque_y)
            self.mixer_errors.append(mixer_error)

            # -------------------------
            # PLANT -> UPDATED STATE
            # -------------------------
            plant = parameters2d(present_X, u)
            present_X = list(plant.X)
            X_hist.append(present_X.copy())

        # =========================
        # SAVE RESULTS
        # =========================
        self.X_hist = X_hist
        self.command_history = self.command_history

        self.desired = [[final_position[0],final_position[1]]for i in range(steps)]


# ===================
# MAIN EXECUTION
# ===================
if __name__ == '__main__':

    initial_position = [ float(v) for v in input("Enter initial location [x z]: ").split()]

    final_position = [float(v) for v in input("Enter final location [x z]: ").split()
]

    if len(initial_position) != 2 or len(final_position) != 2:
        raise ValueError("Enter exactly two values: x z")

    s = pidmain(initial_position,final_position,total_time=10.0)

    # ===================
    # 2D VISUALIZATION
    # ===================
    from D2_Quadcopter_Controller.Visulation_2D import DroneVisualizer2D as DV
    DV(s.X_hist,s.time_vec,desired=s.desired).run()

    # ===================
    # PLOTS
    # ===================
    from Plot_2D import plot_flight_states_2d as plo
    plo(s.X_hist, s.time_vec)
