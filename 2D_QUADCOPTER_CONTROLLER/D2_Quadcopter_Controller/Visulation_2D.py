import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class DroneVisualizer2D:
    """
    Animated side view (x horizontal, z vertical) of the planar quadcopter.

    X_hist : (N, 6) array  [x z xdot zdot theta thetadot]
    desired: optional (N, 2) array of the desired [x z] path
    waypoints: optional (M, 2) array of waypoints to mark
    """

    def __init__(self, X_hist, time_vec, arm=0.3, desired=None, waypoints=None):
        self.X = np.asarray(X_hist, dtype=float)
        self.t = np.asarray(time_vec, dtype=float)
        self.arm = arm
        self.desired = None if desired is None else np.asarray(desired, dtype=float)
        self.waypoints = None if waypoints is None else np.asarray(waypoints, dtype=float)

        if self.X.ndim != 2:
            raise ValueError("X_hist must be a 2D array")
        if self.X.shape[1] != 6:
            raise ValueError("Each state must contain 6 parameters")
        if len(self.X) != len(self.t):
            raise ValueError("X_hist and time_vec must have same length")

        self.x = self.X[:, 0]
        self.z = self.X[:, 1]
        self.theta = self.X[:, 4]
        self.n = len(self.t)

    # body (bx forward/+x, bz up) -> world (x, z); positive theta tilts thrust toward +x
    @staticmethod
    def body_to_world(bx, bz, x, z, theta):
        c, s = np.cos(theta), np.sin(theta)
        return x + bx * c + bz * s, z - bx * s + bz * c

    def run(self):
        L = self.arm

        fig, ax = plt.subplots(figsize=(11, 7))

        xs = [self.x]
        zs = [self.z]
        if self.desired is not None:
            xs.append(self.desired[:, 0])
            zs.append(self.desired[:, 1])
        if self.waypoints is not None:
            xs.append(self.waypoints[:, 0])
            zs.append(self.waypoints[:, 1])
        xmin, xmax = np.min(np.concatenate(xs)), np.max(np.concatenate(xs))
        zmin, zmax = np.min(np.concatenate(zs)), np.max(np.concatenate(zs))
        margin = 1.0

        ax.set_xlim(xmin - margin, xmax + margin)
        ax.set_ylim(min(0.0, zmin) - margin, zmax + margin)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Z (m)")
        ax.set_title("2D QUADCOPTER FLIGHT (x-z plane)")
        ax.grid(True)
        ax.axhline(0.0, color="gray", lw=1)

        if self.desired is not None:
            ax.plot(self.desired[:, 0], self.desired[:, 1], ":", lw=1.5,
                    color="tab:orange", label="Desired")
        if self.waypoints is not None:
            ax.plot(self.waypoints[:, 0], self.waypoints[:, 1], "k*",
                    markersize=12, label="Waypoints")

        body, = ax.plot([], [], lw=6, color="tab:blue")
        up, = ax.plot([], [], lw=3, color="tab:red")             # body z axis (thrust direction)
        motors, = ax.plot([], [], "o", markersize=9, color="black")
        trajectory, = ax.plot([], [], "--", lw=2, color="tab:green", label="Trajectory")

        info = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=11,
                       va="top",
                       bbox=dict(boxstyle="round", facecolor="white",
                                 edgecolor="gray", alpha=0.9))

        def update(i):
            x, z, th = self.x[i], self.z[i], self.theta[i]

            bx, bz = self.body_to_world(np.array([-L, L]), np.zeros(2), x, z, th)
            body.set_data(bx, bz)
            motors.set_data(bx, bz)

            ux, uz = self.body_to_world(np.array([0.0, 0.0]),
                                        np.array([0.0, 0.5 * L]), x, z, th)
            up.set_data(ux, uz)

            trajectory.set_data(self.x[:i + 1], self.z[:i + 1])

            info.set_text(
                f"TIME : {self.t[i]:.2f} s\n"
                f"X = {x:.2f} m\n"
                f"Z = {z:.2f} m\n"
                f"Pitch θ = {np.degrees(th):.2f}°"
            )
            return body, up, motors, trajectory, info

        self.animation = FuncAnimation(
            fig, update, frames=self.n,
            interval=(self.t[1] - self.t[0]) * 1000,
            blit=False, repeat=True)

        ax.legend(loc="upper right")
        plt.tight_layout()
        plt.show()