import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def rotation_matrix(phi, theta, psi):
    cp, sp = np.cos(phi), np.sin(phi)
    ct, st = np.cos(theta), np.sin(theta)
    cy, sy = np.cos(psi), np.sin(psi)
    return np.array([
        [cy * ct, cy * st * sp - sy * cp, cy * st * cp + sy * sp],
        [sy * ct, sy * st * sp + cy * cp, sy * st * cp - cy * sp],
        [-st,     ct * sp,                ct * cp],])


class CompareVisualizer:
    """PID (orange) and LQR (blue) flights in the same 3D scene + time plots."""

    def __init__(self, X_pid, X_lqr, time_vec, arm=0.25):
        X_pid = np.asarray(X_pid, float)
        X_lqr = np.asarray(X_lqr, float)
        t = np.asarray(time_vec, float)

        for name, X in (("X_pid", X_pid), ("X_lqr", X_lqr)):
            if X.ndim != 2 or X.shape[1] != 12:
                raise ValueError(f"{name} must have shape (N, 12)")
            if not np.all(np.isfinite(X)):
                raise ValueError(f"{name} contains NaN/inf (that simulation diverged)")

        n = min(len(X_pid), len(X_lqr), len(t))
        if len({len(X_pid), len(X_lqr), len(t)}) > 1:
            print(f"Lengths differ, using the first {n} samples of each")

        self.n, self.t, self.arm = n, t[:n], arm
        self.runs = {"PID": (X_pid[:n], "tab:orange"),
                     "LQR": (X_lqr[:n], "tab:blue")}

    # ------------------------------------------------------------
    # Static plots: position and angles vs time
    # ------------------------------------------------------------
    def plot(self):
        idx = [0, 1, 2, 6, 7, 8]
        labels = ["x (m)", "y (m)", "z (m)", "roll φ (deg)", "pitch θ (deg)", "yaw ψ (deg)"]

        fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
        for ax, i, label in zip(axes.ravel(), idx, labels):
            for name, (X, c) in self.runs.items():
                y = np.degrees(X[:, i]) if i >= 6 else X[:, i]
                ax.plot(self.t, y, color=c, label=name)
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
        for ax in axes[1]:
            ax.set_xlabel("time (s)")
        axes[0, 0].legend()
        fig.suptitle("PID vs LQR")
        plt.tight_layout()
        plt.show()

    # ------------------------------------------------------------
    # 3D animation of both drones
    # ------------------------------------------------------------
    def run(self):
        L, margin = self.arm, 1.0
        allX = np.vstack([X for X, _ in self.runs.values()])

        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlim(allX[:, 0].min() - margin, allX[:, 0].max() + margin)
        ax.set_ylim(allX[:, 1].min() - margin, allX[:, 1].max() + margin)
        ax.set_zlim(max(0, allX[:, 2].min() - margin), allX[:, 2].max() + margin)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("PID vs LQR")

        motors_body = np.array([[-L, 0, 0], [L, 0, 0], [0, -L, 0], [0, L, 0]])

        art = {}
        for name, (X, c) in self.runs.items():
            arm1, = ax.plot([], [], [], color=c, linewidth=5)
            arm2, = ax.plot([], [], [], color=c, linewidth=5)
            mot, = ax.plot([], [], [], "o", color=c, markersize=8)
            traj, = ax.plot([], [], [], "--", color=c, linewidth=2, label=name)
            shadow, = ax.plot([], [], [], ":", color=c, linewidth=1.2)
            art[name] = (arm1, arm2, mot, traj, shadow)

        info = ax.text2D(0.02, 0.70, "", transform=ax.transAxes, fontsize=10,
                         bbox=dict(boxstyle="round", facecolor="white",
                                   edgecolor="gray", alpha=0.9))

        def update(i):
            text = [f"TIME : {self.t[i]:.2f} s", ""]
            changed = [info]
            for name, (X, c) in self.runs.items():
                pos = X[i, :3]
                phi, theta, psi = X[i, 6:9]
                w = (rotation_matrix(phi, theta, psi) @ motors_body.T).T + pos

                arm1, arm2, mot, traj, shadow = art[name]
                arm1.set_data_3d(w[0:2, 0], w[0:2, 1], w[0:2, 2])
                arm2.set_data_3d(w[2:4, 0], w[2:4, 1], w[2:4, 2])
                mot.set_data_3d(w[:, 0], w[:, 1], w[:, 2])
                traj.set_data_3d(X[:i + 1, 0], X[:i + 1, 1], X[:i + 1, 2])
                shadow.set_data_3d(X[:i + 1, 0], X[:i + 1, 1], np.zeros(i + 1))
                changed += [arm1, arm2, mot, traj, shadow]

                text += [f"{name}",
                         f"  x={pos[0]:.2f}  y={pos[1]:.2f}  z={pos[2]:.2f} m",
                         f"  roll={np.degrees(phi):.1f}  pitch={np.degrees(theta):.1f}  yaw={np.degrees(psi):.1f} deg",
                         ""]

            gap = np.linalg.norm(self.runs["PID"][0][i, :3] - self.runs["LQR"][0][i, :3])
            text.append(f"distance PID-LQR = {gap:.2f} m")
            info.set_text("\n".join(text))
            return changed

        self.animation = FuncAnimation(fig, update, frames=self.n,
                                       interval=(self.t[1] - self.t[0]) * 1000,
                                       blit=False, repeat=True)

        ax.view_init(elev=25, azim=-60)
        ax.legend()
        plt.tight_layout()
        plt.show()