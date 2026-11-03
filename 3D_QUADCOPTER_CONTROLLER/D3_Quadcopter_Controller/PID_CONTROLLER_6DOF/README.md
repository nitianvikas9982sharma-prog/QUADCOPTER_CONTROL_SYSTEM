# 6-DOF Quadcopter Control — Cascade PID

## 1. Project Overview
    This project implements a **6-DOF quadcopter control system** using cascaded PID controllers.

    The objective is to control the drone's position and orientation in three-dimensional space using its six degrees of freedom: translation along the X, Y, and Z axes, and rotation about the roll, pitch, and yaw axes.

## 3. Cascade PID Control Strategy
    The control architecture consists of outer and inner control loops for position and attitude regulation.

### 3.1 Altitude Control — Outer Loop
    The position controllers calculate the error between the desired and actual positions along the X, Y.
    Their outputs are desired ANGLES to the corresponding axes.

    MAX_THRUST=9.66 N
    MIN_THRUST=0 N
    THRUST_RATE_LIMIT=96.6 N/s

    **PID Tuning Parameters:**
                | (K_p)  | (K_i)  | (K_d)  |
    | X Position |  1.5   |  0.05. |  2.4   |
    | Y Position |  1.5   |  0.05  |  2.4   |
    | Z Position |  8.0   |  0.3   |  6.0   |

### 3.2 ATITUDE Control — Inner  Loop
    The Atitude controllers compare the desired and actual values to calculate the required  control commands.

    ROLL_TOU_MAX=0.543 N.m
    ROLL_TOU_MIN=-0.543 N.m

    PITCH_TOU_MAX=0.543 N.m
    PITCH_TOU_MIN=-0.543 N.m

    YAW_TOU_MAX=0.185 N.m
    YAW_TOU_MIN=-0.185 N.m

    **PID Tuning Parameters:**
                | (K_p)  | (K_i)  | (K_d)  |
    |  ROLL      |  64.0  |  0.5   |  11.0  |
    |  PITCH     |  64.0  |  0.5   |  11.0  |
    |  YAW       |  8.0   |  0.1   |  4.0   |

## 4. Thrust, Torque and Motor Mixing
    The controller outputs are converted into total thrust and body torque commands. The motor mixer then distributes the required control effort among the four motors.

    The motor commands are constrained by the configured actuator limits to ensure feasible thrust and torque generation.

## 5. Simulation and Visualization
    Dedicated visualization and plotting modules can be used to analyze:
    - 3D trajectory tracking.
    - Target versus actual position.

## 6. Project Goal
    To implement and analyze a complete **6-DOF cascaded PID control system**, integrating position control, velocity regulation, attitude stabilization, angular rate control, motor mixing, and three-dimensional simulation.