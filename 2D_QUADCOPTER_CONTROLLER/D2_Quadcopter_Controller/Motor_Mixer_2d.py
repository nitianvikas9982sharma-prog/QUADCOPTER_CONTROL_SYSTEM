import numpy as np

def motor_mixer(thrust, torque_y,k_t=2.980e-6,arm_length=0.25, max_speed=900.0, min_speed=0.0):
    #=========================================================================
    """
    Planar mixer: [thrust, pitch torque] -> [w1, w2]

    w1 = rear pair (-x), w2 = front pair (+x).
    Rows match the plant in I2_parameters_2D.py:

    thrust = 2*k_t*(w1^2 + w2^2)
    tau_y= 2*k_t*d*(w2^2 - w1^2)

    [T]=[2k_t           2k_t][w1^2]
    [Tou]=[2*d*k_t   2*d*k_t][w2^2]
    """
    #=========================================================================
    # M is coefficient matrix of  above equation
    #=========================================================================

    d = arm_length / np.sqrt(2)          # thou is _|_ distance into thurst
    b = np.array([thrust, torque_y], dtype=float)
   
    M = np.array([
                 [2 * k_t    ,   2 * k_t   ],
                 [2 * k_t * d,  -2 * k_t * d], ])

    #==================================================================
    #Finding  suitable soution(w1^2,w2^2) where b=[thrust,tou]
    #==================================================================
    u_squared = np.linalg.solve(M, b)

    #==================================================================
    #Checking where OMEGA in under saturation or not
    #==================================================================
    u_squared_clipped = np.clip(u_squared, min_speed ** 2, max_speed ** 2) 
    motor_speeds = list(np.sqrt(u_squared_clipped))      #[w1, w2]

    #==================================================================
    #it find error between actual OMEGA and after Saturation box OMEGA
    #==================================================================
    residual_error = np.linalg.norm(b - M @ u_squared_clipped)
    
    return motor_speeds, residual_error