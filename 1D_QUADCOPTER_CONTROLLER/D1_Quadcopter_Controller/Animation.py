
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class DroneAnimation:

    ANIMATION_TIME = 20.0

    def __init__(self, Z_hist,Z_target,time_vec):

        self.time_vec = time_vec
        self.altitude = Z_hist
        self.altitude_position = Z_target

        self.total_frames = len(Z_hist)

        self.frame_interval = (
            self.ANIMATION_TIME * 1000 / self.total_frames
        )

        self.speed_factor = (
            (time_vec[-1] - time_vec[0]) / self.ANIMATION_TIME
        )

        self.fig, self.ax = plt.subplots(figsize=(10, 7))

        margin = max(
            1,
            0.1 * abs(np.max(Z_hist) - np.min(Z_hist))
        )

        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(
            np.min(Z_hist) - margin,
            np.max(Z_hist) + margin
        )

        self.ax.set_xlabel("Horizontal Position (m)")
        self.ax.set_ylabel("Altitude (m)")
        self.ax.set_title("Quadcopter 1D Altitude Simulation")
        self.ax.grid(True)

        self.ax.axhline(
            Z_target,
            linestyle="--",
            linewidth=2,
            label=f"Target Altitude = {Z_target} m"
        )

        self.drone, = self.ax.plot(
            [], [], marker="o", markersize=14,
            linestyle="None", label="Quadcopter"
        )

        self.trajectory, = self.ax.plot(
            [], [], linewidth=2, label="Altitude Trajectory"
        )

        self.info = self.ax.text(
            0.02, 0.95, "",
            transform=self.ax.transAxes,
            verticalalignment="top"
        )

        self.ax.legend()

    def init(self):

        self.drone.set_data([], [])
        self.trajectory.set_data([], [])
        self.info.set_text("")

        return self.drone, self.trajectory, self.info

    def update(self, frame):

        z = self.altitude[frame]
        t = self.time_vec[frame]

        self.drone.set_data([0], [z])

        self.trajectory.set_data(
            np.zeros(frame + 1),
            self.altitude[:frame + 1]
        )

        self.info.set_text(
            f"Simulation Time : {t:.2f} s\n"
            f"Altitude        : {z:.2f} m\n"
            f"Animation Speed : {self.speed_factor:.1f}x"
        )

        return self.drone, self.trajectory, self.info

    def show(self):

        self.animation = FuncAnimation(
            self.fig,
            self.update,
            frames=self.total_frames,
            init_func=self.init,
            interval=self.frame_interval,
            blit=True,
            repeat=False
        )

        plt.show()
