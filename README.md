# Quadcopter Control

## Objective
The primary objective of this repository is to implement and compare autonomous control systems for a quadcopter across **1D, 2D, and 3D space**. The framework focuses on precise trajectory tracking and position following while providing a benchmark for handling external disturbances, unmodeled dynamics, and stability requirements.

---

## Making the Plant

### Aim
The plant dynamics model calculates the next state vector of the quadcopter given its current state and individual **motor angular speeds** ($\omega_1, \omega_2, \omega_3, \omega_4$).

$$\dot{x} = f(x, \omega)$$

* **State Vector ($x$):** Positions ($x, y, z$), translational velocities ($\dot{x}, \dot{y}, \dot{z}$), Euler angles ($\phi, \theta, \psi$), and body angular velocities ($p, q, r$).
* **Control Input Vector ($\omega$):** Motor rotational speeds $\omega = [\omega_1, \omega_2, \omega_3, \omega_4]^T$.

> **Constant Parameters & System Equations**
>
> ![Quadcopter Parameters & Formulas](https://github.com/user-attachments/assets/fa451b71-5820-4c53-9a2f-14a48a7801f3)
>
> *Figure 1: Physical parameters (mass, moments of inertia, arm lengths, thrust/drag coefficients).*



---

## Making the Controller

### 1. PID Controller (Proportional-Integral-Derivative)

#### Theory
The PID controller uses feedback error $e(t) = r(t) - y(t)$ to generate the required control input without requiring an explicit physical model of the quadcopter:

$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(t') \, dt' + K_d \frac{de(t)}{dt}$$

* **Proportional ($K_p$):** Produces a control output proportional to the current error. Increases system responsiveness and reduces rise time, but high values can cause overshoots and instability.
* **Integral ($K_i$):** Accumulates past errors over time to eliminate **steady-state error**.
* **Derivative ($K_d$):** Responds to the rate of error change. Acts as a damper to reduce overshoot, improve transient response, and optimize **settling time**.

> **Block Diagram**
>
> ![PID Controller Block Diagram](https://github.com/user-attachments/assets/961e2494-6c60-4b7e-b7d1-b9f0f46d4dc3)
>
> *Figure 2: Closed-loop PID control architecture for quadcopter position and attitude tracking.*

#### Results & Visualizations

##### 1D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 1D Plot 1](https://github.com/user-attachments/assets/9f15d806-896b-4bd7-abd2-804beffcb7d2) | ![PID 1D Simulation GIF 1](https://github.com/user-attachments/assets/9bc6fca9-df8b-4d5f-b419-59679a1614fc) |


##### 2D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 2D Plot 1](https://github.com/user-attachments/assets/b28e7e59-caa0-40b0-ae9c-31e6c61c8339) | ![PID 2D Simulation GIF 1](https://github.com/user-attachments/assets/e9070307-4caa-4983-9f56-768a16462769) |


##### 3D Quadcopter Control
| Simulation Plots | Dynamic Trajectory GIFs |
| :---: | :---: |
| ![PID 3D Plot 1](https://github.com/user-attachments/assets/edbbfbf2-7a49-42d0-92a5-91efc3996927) | ![PID 3D Simulation GIF 1](https://github.com/user-attachments/assets/2a4c2ebc-d9d2-4c73-b93b-7ce0d322572e) |


---

### 2. LQR Controller (Linear Quadratic Regulator)

#### Theory
LQR is an optimal control strategy designed for state-space representations ($\dot{x} = Ax + Bu$). It computes an optimal state-feedback gain matrix $K$ that minimizes a continuous-time quadratic cost function:

$$J = \int_{0}^{\infty} \left( x^T Q x + u^T R u \right) dt$$

Where:
* $Q \ge 0$ is the positive semi-definite state weighting matrix (penalizes tracking errors).
* $R > 0$ is the positive definite control effort weighting matrix (penalizes excessive actuator input).

The optimal control law is given by:

$$u = -Kx$$

The state-feedback gain matrix $K$ is computed as:

$$K = R^{-1} B^T P$$

Where $P$ is the unique positive-definite solution to the **Continuous Algebraic Riccati Equation (CARE)**:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

> **Block Diagram**
>
> ![LQR Controller Block Diagram](https://github.com/user-attachments/assets/763aa732-e332-4e71-b380-d1c6146237fe)
>
> *Figure 3: Optimal state-feedback LQR control architecture.*

#### Results & Visualizations

##### 3D Quadcopter Control
| Dynamic Trajectory GIFs |
| :---: | :---: |
| ![LQR 3D Simulation GIF 1](https://github.com/user-attachments/assets/17478e32-6353-4b31-9c82-c60da27884d1) |



---

## Making Motor Mixer

### Objective
The Motor Mixer serves as the interface between high-level control commands (desired total thrust $F$ and moment/torque vector $\tau = [\tau_x, \tau_y, \tau_z]^T$) and the low-level physical actuators, determining individual motor speeds ($\omega_1^2, \omega_2^2, \omega_3^2, \omega_4^2$).

The mapping from squared motor speeds to wrench inputs is given by:

$$\begin{bmatrix} F \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix} = M \begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix}$$

To compute the required motor speeds from high-level outputs, the inverse relationship is evaluated. For over-determined or sensitive control conditions, the **Normal Equation** (least squares approach) is applied to minimize error and actuator output variance:

$$\omega^2 = (M^T M)^{-1} M^T \begin{bmatrix} F \\ \tau_x \\ \tau_y \\ \tau_z \end{bmatrix}$$



## Comparison: PID vs LQR

| Feature | PID Controller | LQR Controller |
| :--- | :--- | :--- |
| **Model Dependency** | Model-free; does not require physical dynamics equations. | Model-based; requires linear state-space matrices ($A, B$). |
| **Architecture** | Single-Input Single-Output (SISO) loops tuned independently. | Multi-Input Multi-Output (MIMO) optimal full-state feedback. |
| **Efficiency** | Tuned heuristically or manually (risk of overshoot/actuator saturation). | Systematically optimizes performance vs. control effort via $Q$ and $R$ matrices. |
| **Coupling Handling** | Can struggle with strong dynamic coupling in 3D motions. | Inherently handles cross-coupling across translational and rotational axes. |

### Comparative Results
![PID vs LQR Comparison Plot](https://github.com/user-attachments/assets/bec375a1-7059-426b-940f-58704f4eb4db)

![PID vs LQR Comparison Simulation GIF](https://github.com/user-attachments/assets/c4eb2206-003e-41c2-9bb3-c50312202860)


### References Used (Research Papers)
[ASCEAS.1943-5525.0001336 (1).pdf](https://github.com/user-attachments/files/33455191/ASCEAS.1943-5525.0001336.1.pdf)

[Control Strategies for 6-DOF Quadcopter UAVs_ Cascade PID Stabili (1).pdf](https://github.com/user-attachments/files/33455142/Control.Strategies.for.6-DOF.Quadcopter.UAVs_.Cascade.PID.Stabili.1.pdf)
