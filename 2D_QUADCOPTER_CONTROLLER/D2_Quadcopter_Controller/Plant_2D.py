import numpy as np
import math as m


class parameters2d:
    """
    Planar (x-z) quadcopter plant.
    State  X = [x,z,xdot,zdot,theta,thetadot]
    Input  W = [w1, w2]   rotor-pair speeds
           w1 = rear pair  angular velocity
           w2 = front pair angular velocity
    """
    def __init__(self,X,W,dt=0.01):

        #=========================
        # PARAMETERS
        #=========================
        mass=0.468
        g=9.81
        l=0.25
        l_proj=l*(np.sqrt(2)/2.0)

        Iy=4.856e-3
        k_t=2.980e-6

        #=========================
        # ROTOR SPEEDS
        #=========================
        w1,w2=W
        w1_sq=w1**2
        w2_sq=w2**2

        #=========================
        # TOTAL THRUST AND PITCH TORQUE
        #=========================
        thrust=2.0*k_t*(w1_sq+w2_sq)
        tau_th=2.0*k_t*l_proj*(w1_sq-w2_sq)

        #=========================
        # STATES
        #=========================
        x,z,xdot,zdot,theta,q=X

        #==========================
        # TRANSLATIONAL ACCELERATION
        #==========================
        xddot=(thrust/mass)*m.sin(theta)
        zddot=(thrust/mass)*m.cos(theta)-g

        #================
        # UPDATE VELOCITY
        #================
        xdot=xdot+xddot*dt
        zdot=zdot+zddot*dt

        #================
        # UPDATE POSITION
        #================
        x=x+xdot*dt
        z=z+zdot*dt

        #===========================
        # PITCH ANGULAR ACCELERATION
        #===========================
        qdot=tau_th/Iy
        q=q+qdot*dt      #for x,z plane thetadot=q
        theta=theta+q*dt

        #============
        # FINAL STATE
        #============
        self.X =[float(x), float(z),
                float(xdot),float(zdot),
                float(theta),float(q)]