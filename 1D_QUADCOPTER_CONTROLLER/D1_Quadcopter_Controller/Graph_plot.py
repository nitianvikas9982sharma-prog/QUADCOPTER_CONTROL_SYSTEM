
import matplotlib.pyplot as plt

class ZPlot:
    def __init__(self, Z_hist, Thrust_hist, time_vec, z_target):

        plt.subplot(2, 1, 1)
        plt.plot(time_vec, Z_hist[:len(time_vec)], linewidth=2, label="Altitude")
        plt.axhline(y=z_target, linestyle="-.", label="Altitude Target")
        plt.title("Altitude vs Time")
        plt.xlabel("Time")
        plt.ylabel("Altitude")
        plt.legend()
        plt.grid()

        plt.subplot(2, 1, 2)
        plt.plot(time_vec, Thrust_hist[:len(time_vec)], linewidth=2, label="Thrust")
        plt.title("Thrust vs Time")
        plt.xlabel("Time")
        plt.ylabel("Thrust")
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()
