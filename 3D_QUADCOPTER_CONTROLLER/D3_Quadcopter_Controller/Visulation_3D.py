import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class DroneVisualizer:

    def __init__(self, X_hist, time_vec, arm=0.25): 
        self.X = np.asarray(X_hist, dtype=float)
        self.t = np.asarray(time_vec, dtype=float)

        self.arm = arm

        # -----------------------------------------
        # Check input
        # -----------------------------------------

        if self.X.ndim != 2:
            raise ValueError(
                "X_hist must be a 2D array"
            )

        if self.X.shape[1] != 12:
            raise ValueError(
                "Each state must contain 12 parameters"
            )

        if len(self.X) != len(self.t):
            raise ValueError(
                "X_hist and time_vec must have same length"
            )

        # -----------------------------------------
        # Extract states
        # -----------------------------------------

        # Position
        self.x = self.X[:, 0]
        self.y = self.X[:, 1]
        self.z = self.X[:, 2]

        # Euler angles
        self.phi = self.X[:, 6]
        self.theta = self.X[:, 7]
        self.psi = self.X[:, 8]

        self.n = len(self.t)

    # =================================================
    # Rotation matrix
    # =================================================

    @staticmethod
    def rotation_matrix(phi, theta, psi):

        cp = np.cos(phi)
        sp = np.sin(phi)

        ct = np.cos(theta)
        st = np.sin(theta)

        cy = np.cos(psi)
        sy = np.sin(psi)

        R = np.array([

            [
                cy * ct,
                cy * st * sp - sy * cp,
                cy * st * cp + sy * sp
            ],

            [
                sy * ct,
                sy * st * sp + cy * cp,
                sy * st * cp - cy * sp
            ],

            [
                -st,
                ct * sp,
                ct * cp
            ]

        ])

        return R

    # =================================================
    # Run visualization
    # =================================================

    def run(self):

        L = self.arm

        # -----------------------------------------
        # Create figure
        # -----------------------------------------

        fig = plt.figure(figsize=(12, 9))

        ax = fig.add_subplot(
            111,
            projection="3d"
        )

        # -----------------------------------------
        # Axis limits
        # -----------------------------------------

        margin = 1.0

        ax.set_xlim(
            self.x.min() - margin,
            self.x.max() + margin
        )

        ax.set_ylim(
            self.y.min() - margin,
            self.y.max() + margin
        )

        ax.set_zlim(
            max(0, self.z.min() - margin),
            self.z.max() + margin
        )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")

        ax.set_title(
            "3D QUADCOPTER FLIGHT"
        )

        # -----------------------------------------
        # Drone body
        # -----------------------------------------

        arm1, = ax.plot(
            [], [], [],
            linewidth=6
        )

        arm2, = ax.plot(
            [], [], [],
            linewidth=6
        )

        motors, = ax.plot(
            [], [], [],
            "o",
            markersize=9
        )

        # -----------------------------------------
        # Flight trajectory
        # -----------------------------------------

        trajectory, = ax.plot(
            [], [], [],
            "--",
            linewidth=2,
            label="Trajectory"
        )

        # -----------------------------------------
        # Ground projection
        # -----------------------------------------

        ground, = ax.plot(
            [], [], [],
            ":",
            linewidth=1.5
        )

        # -----------------------------------------
        # Information box
        # -----------------------------------------

        info = ax.text2D(
            0.02,
            0.75,
            "",
            transform=ax.transAxes,
            fontsize=11,
            bbox=dict(
                boxstyle="round",
                facecolor="white",
                edgecolor="gray",
                alpha=0.9
            )
        )

        # =================================================
        # Update function
        # =================================================

        def update(i):

            # -----------------------------------------
            # Current position
            # -----------------------------------------

            position = np.array([
                self.x[i],
                self.y[i],
                self.z[i]
            ])

            # -----------------------------------------
            # Current orientation
            # -----------------------------------------

            phi = self.phi[i]
            theta = self.theta[i]
            psi = self.psi[i]

            R = self.rotation_matrix(
                phi,
                theta,
                psi
            )

            # =================================================
            # DRONE ARMS IN BODY FRAME
            # =================================================

            arm_x = np.array([
                [-L, 0, 0],
                [ L, 0, 0]
            ])

            arm_y = np.array([
                [0, -L, 0],
                [0,  L, 0]
            ])

            # -----------------------------------------
            # Convert body → world
            # -----------------------------------------

            arm_x_world = (
                R @ arm_x.T
            ).T + position

            arm_y_world = (
                R @ arm_y.T
            ).T + position

            # -----------------------------------------
            # Draw arms
            # -----------------------------------------

            arm1.set_data(
                arm_x_world[:, 0],
                arm_x_world[:, 1]
            )

            arm1.set_3d_properties(
                arm_x_world[:, 2]
            )

            arm2.set_data(
                arm_y_world[:, 0],
                arm_y_world[:, 1]
            )

            arm2.set_3d_properties(
                arm_y_world[:, 2]
            )

            # =================================================
            # MOTORS
            # =================================================

            motor_body = np.array([
                [-L, 0, 0],
                [ L, 0, 0],
                [0, -L, 0],
                [0,  L, 0]
            ])

            motor_world = (
                R @ motor_body.T
            ).T + position

            motors.set_data(
                motor_world[:, 0],
                motor_world[:, 1]
            )

            motors.set_3d_properties(
                motor_world[:, 2]
            )

            # =================================================
            # TRAJECTORY
            # =================================================

            trajectory.set_data(
                self.x[:i + 1],
                self.y[:i + 1]
            )

            trajectory.set_3d_properties(
                self.z[:i + 1]
            )

            # =================================================
            # GROUND PROJECTION
            # =================================================

            ground.set_data(
                self.x[:i + 1],
                self.y[:i + 1]
            )

            ground.set_3d_properties(
                np.zeros(i + 1)
            )

            # =================================================
            # INFORMATION
            # =================================================

            info.set_text(

                f"TIME : {self.t[i]:.2f} s\n\n"

                f"Position\n"
                f"X = {self.x[i]:.2f} m\n"
                f"Y = {self.y[i]:.2f} m\n"
                f"Z = {self.z[i]:.2f} m\n\n"

                f"Orientation\n"
                f"Roll  φ = "
                f"{np.degrees(phi):.2f}°\n"

                f"Pitch θ = "
                f"{np.degrees(theta):.2f}°\n"

                f"Yaw   ψ = "
                f"{np.degrees(psi):.2f}°"
            )

            return (
                arm1,
                arm2,
                motors,
                trajectory,
                ground,
                info
            )

        # =================================================
        # Animation
        # =================================================

        self.animation = FuncAnimation(

            fig,

            update,

            frames=self.n,

            interval=(
                self.t[1] - self.t[0]
            ) * 1000,

            blit=False,

            repeat=True
        )

        # Camera

        ax.view_init(
            elev=25,
            azim=-60
        )

        ax.legend()

        plt.tight_layout()

        plt.show()