import numpy as np
import math as m

class parameters12:
    def __init__(self,X,W,dt=0.01):
        
        # =========================
        # PARAMETERS
        # =========================
        mass = 0.468
        g = 9.81
        l = 0.25
        l_proj = l * (np.sqrt(2) / 2.0)

        Ix = 4.856e-3
        Iy = 4.856e-3
        Iz = 8.801e-3
        Im = 3.357e-5

        b = 2.980e-6 #
        d = 1.140e-7 #

        # =========================
        # ROTOR SPEEDS
        # =========================
        w1, w2, w3, w4 = W

        w1_sq = w1**2
        w2_sq = w2**2
        w3_sq = w3**2
        w4_sq = w4**2

        # =========================
        # TOTAL THRUST
        # =========================
        thrust = b * (w1_sq + w2_sq + w3_sq + w4_sq)

        # =========================
        # TORQUES
        # =========================

        # Roll torque
        tau_phi = b * l_proj * (-w1_sq - w2_sq + w3_sq + w4_sq)

        # Pitch torque
        tau_th = b * l_proj * ( w1_sq - w2_sq + w3_sq - w4_sq)

        # Yaw
        tau_psi = d * ( w1_sq - w2_sq - w3_sq + w4_sq )

        # =========================
        # GYROSCOPIC TERM
        # =========================
        Ws = w1 - w2 - w3 + w4

        T = [thrust, tau_phi, tau_th, tau_psi]

        # =========================
        # EULER ANGLES
        # =========================
        phi = X[6]
        theta=X[7]
        psi = X[8]

        # =========================
        # ROTATION MATRIX
        # =========================

        r1 = np.array([m.cos(psi) * m.cos(theta),
                       m.cos(psi) * m.sin(theta) * m.sin(phi)- m.sin(psi) * m.cos(theta),
                       m.cos(psi) * m.sin(theta) * m.cos(phi)+ m.sin(psi) * m.sin(phi)])
        
        r2 = np.array([m.sin(psi) * m.cos(theta),
                       m.sin(psi) * m.sin(theta) * m.sin(phi) + m.cos(psi) * m.cos(theta),
                       m.sin(psi) * m.sin(theta) * m.cos(phi)- m.cos(psi) * m.sin(phi)])

        r3 = np.array([-m.sin(theta),
                       m.cos(theta) * m.sin(phi),
                       m.cos(theta) * m.cos(phi)])

        R = np.array([r1, r2, r3])

        # ==========================
        # TRANSLATIONAL ACCELERATION
        # ==========================
        
        acce = (R @ np.array([[0], [0], [T[0] / mass]])- np.array([[0], [0], [g]]))
        xddot = acce[0, 0]
        yddot = acce[1, 0]
        zddot = acce[2, 0]

        # ================
        # UPDATE VELOCITY
        # ================
        xdot = X[3] + xddot * dt
        ydot = X[4] + yddot * dt
        zdot = X[5] + zddot * dt

        # ================
        # UPDATE POSITION
        # ================
        x = X[0] + xdot * dt
        y = X[1] + ydot * dt
        z = X[2] + zdot * dt

        # ====================================
        # EULER RATE -> BODY ANGULAR VELOCITY
        # ====================================
        W_matrix =np.array([[1,0,-m.sin(theta)],
                           [0,m.cos(phi),m.cos(theta)*m.sin(phi)],
                           [0,-m.sin(phi),m.cos(theta)*m.cos(phi)]])
        
        # [p q r] = W_matrix [phidot thetadot psidot]
        omega = W_matrix @ np.array([[X[9]],[X[10]],[X[11]]])
        p = omega[0, 0]
        q = omega[1, 0]
        r = omega[2, 0]

        # =========================
        # BODY ANGULAR ACCELERATION
        # =========================
        pdot=((Iy-Iz)*q*r+T[1]-q*Im*Ws)/Ix
        qdot=((Iz-Ix)*p*r+T[2]+Im*Ws*p)/Iy
        rdot=((Ix-Iy)*q*p+T[3])/Iz
       
        # ============
        # UPDATE p,q,r
        # ============
        p = p + pdot * dt
        q = q + qdot * dt
        r = r + rdot * dt

        # ===============================================
        # BODY ANGULAR VELOCITY -> EULER ANGULAR VELOCITY
        # ===============================================
        W_inv = np.linalg.inv(W_matrix)

        euler_dot = W_inv @ np.array([[p],[q],[r]])
        phidot = euler_dot[0,0]
        thetadot = euler_dot[1,0]
        psidot = euler_dot[2,0]

        # ===================
        # UPDATE EULER ANGLES
        # ===================
        phi = phi + phidot * dt
        theta = theta + thetadot * dt
        psi = psi + psidot * dt

        # ============
        # FINAL STATE
        # ============
        self.X =[float(x), float(y), float(z),
                 float(xdot), float(ydot), float(zdot),
                 float(phi), float(theta), float(psi),
                 float(phidot),float(thetadot),float(psidot)
                 ]
        