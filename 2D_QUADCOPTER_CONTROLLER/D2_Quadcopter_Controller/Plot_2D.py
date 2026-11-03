import matplotlib.pyplot as plt

def plot_flight_states_2d(X_hist, time_vec):

    x = [row[0] for row in X_hist]
    z = [row[1] for row in X_hist]
    theta = [row[5] for row in X_hist]

    fig, ax = plt.subplots(2, 2, figsize=(10, 6))

    ax[0, 0].plot(time_vec, x)
    ax[0, 0].set_title("X Position (m)")

    ax[0, 1].plot(time_vec, z)
    ax[0, 1].set_title("Z Position (m)")

    ax[1, 0].plot(time_vec, theta)
    ax[1, 0].set_title("Pitch Angle (rad)")

    ax[1, 1].plot(x, z)
    ax[1, 1].set_title("Flight Path (X-Z)")
    ax[1, 1].set_xlabel("X (m)")
    ax[1, 1].set_ylabel("Z (m)")
    ax[1, 1].set_aspect("equal", adjustable="datalim")

    for a in ax.flat:
        a.grid(True)

    plt.tight_layout()
    plt.show()

    return fig, ax