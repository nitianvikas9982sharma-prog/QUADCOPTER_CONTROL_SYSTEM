import numpy as np
from Altitude_PID import zpid

class main:
    def __init__(self,inital_z,final_z,simulation_time):
        #==========================
        #PARAMETERS
        #==========================
        mass=0.468
        g=9.81
        dt=0.01  #sensor respond gap time
        steps = int(simulation_time / dt) 
        self.time_vec = np.linspace(0, simulation_time, steps)

        #=======================
        #INITAL_TERMS
        #=======================
        previous_zerror=final_z-inital_z
        integral_error=0
        previous_thrust=mass*g
        present_z=inital_z
        present_v=0 #m/s

        Thrust_hist=[]
        Thrust_hist.append(previous_thrust)
        Z_hist=[]
        Z_hist.append(present_z)

    
        for i in range(steps-1):
            sz=zpid()
            sz.normalpid(final_z,present_z,integral_error,previous_zerror)
            integral_error=sz.conditionalclamp()
            previous_zerror=sz.error
            previous_thrust=sz.thrust_rate_limit(previous_thrust)

            #===========================
            #Euler_integral=>thrust_to_z
            #===========================
            a=(previous_thrust-mass*g)/mass
            present_v=present_v+a*dt
            present_z=present_z+present_v*dt

            #============================
            #STORING_Z_AND_THRUST
            #============================
            Thrust_hist.append(previous_thrust)
            Z_hist.append(present_z)
        self.Z_hist=Z_hist
        self.Thrust_hist=Thrust_hist
#===========================================================

if __name__ == '__main__':

    total_time=30
    #===========================
    #INPUT TAKING FROM USER
    #===========================
    inital_z=float(input("Enter the inital_location :"))
    final_z=float(input("Enter the final_location :"))

    sim=main(inital_z,final_z,total_time)
    Z_hist=sim.Z_hist
    Thrust_hist=sim.Thrust_hist
    time_vec=sim.time_vec

    #=============================
    #GRAPH=>Z_hist and Thrust_hist
    #=============================
    from Graph_plot import ZPlot
    ZPlot(Z_hist, Thrust_hist, time_vec,final_z)

    #============================
    #3D_Visulation
    #============================
    from Animation import DroneAnimation 
    animation=DroneAnimation(Z_hist, final_z, time_vec)
    animation.show()



















        


            

