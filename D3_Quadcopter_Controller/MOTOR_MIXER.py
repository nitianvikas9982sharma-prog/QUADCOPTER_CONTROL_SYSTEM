import numpy as np

def motor_mixer(thrust, torque_x, torque_y, torque_z,
                k_t=2.980e-6, k_d=1.140e-7,      # SAME as I2_parameters.py (b, d)
                arm_length=0.25, max_speed=900.0, min_speed=0.0):
    b = np.array([thrust, torque_x, torque_y, torque_z], dtype=float)

    d = arm_length / np.sqrt(2)          # same as l_proj in plant
    # Rows EXACTLY match the plant equations in I2_parameters.py
    #   thrust  = b*(w1+w2+w3+w4)
    #   tau_phi = b*d*(-w1-w2+w3+w4)
    #   tau_th  = b*d*( w1-w2+w3-w4)
    #   tau_psi = d_*( w1-w2-w3+w4)
    M = np.array([
        [ k_t,     k_t,     k_t,     k_t    ],
        [-k_t*d,  -k_t*d,   k_t*d,   k_t*d  ],
        [ k_t*d,  -k_t*d,   k_t*d,  -k_t*d  ],
        [ k_d,    -k_d,    -k_d,     k_d    ],
    ])

    u_squared = np.linalg.solve(M, b)                       # M is square -> direct solve
    u_squared_clipped = np.clip(u_squared, min_speed**2, max_speed**2)
    motor_speeds = list(np.sqrt(u_squared_clipped))         # w1,w2,w3,w4 (all >= 0)

    residual_error = np.linalg.norm(b - M @ u_squared_clipped)
    return motor_speeds, residual_error
