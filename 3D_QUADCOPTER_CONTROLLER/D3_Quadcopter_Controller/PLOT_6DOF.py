import matplotlib.pyplot as plt

def plot_flight_states(states,time_vec):   #states = X_hist
    x,y,z,phi,theta,psi =[row[0] for row in states], [row[1] for row in states],[row[2] for row in states],[row[6] for row in states],[row[7] for row in states],[row[8] for row in states]

    # Create 3x2 subplot grid
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    # Left Column: Positions (x, y, z)
    axes[0, 0].plot(time_vec, x, 'b-')
    axes[0, 0].set_title("Position x (m)")
    axes[0, 0].grid(True)

    axes[1, 0].plot(time_vec, y, 'b-')
    axes[1, 0].set_title("Position y (m)")
    axes[1, 0].grid(True)

    axes[2, 0].plot(time_vec, z, 'b-', lw=2)
    axes[2, 0].set_title("Position z (Height) (m)")
    axes[2, 0].grid(True)

    # Right Column: Orientations (Roll, Pitch, Yaw)
    axes[0, 1].plot(time_vec, phi, 'g-')
    axes[0, 1].set_title("Roll Angle $\phi$ (rad)")
    axes[0, 1].grid(True)

    axes[1, 1].plot(time_vec, theta, 'g-')
    axes[1, 1].set_title("Pitch Angle $\theta$ (rad)")
    axes[1, 1].grid(True)

    axes[2, 1].plot(time_vec, psi, 'g-')
    axes[2, 1].set_title("Yaw Angle $\psi$ (rad)")
    axes[2, 1].grid(True)

    # Apply x-axis label to all subplots
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
    return fig, axes