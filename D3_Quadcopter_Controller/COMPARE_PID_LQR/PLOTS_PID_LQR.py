import matplotlib.pyplot as plt
import matplotlib.lines as mlines

def plot_flight_states(states_1, states_2,time_vec):   #states = X_hist

    x_1, y_1, z_1, phi_1, theta_1, psi_1 = [row[0] for row in states_1], [row[1] for row in states_1],[row[2] for row in states_1],[row[6] for row in states_1],[row[7] for row in states_1],[row[8] for row in states_1]
    x_2, y_2, z_2, phi_2, theta_2, psi_2 = [row[0] for row in states_2], [row[1] for row in states_2],[row[2] for row in states_2],[row[6] for row in states_2],[row[7] for row in states_2],[row[8] for row in states_2]

    # Create 3x2 subplot grid
    fig, axes = plt.subplots(3, 2, figsize=(12, 10))
    
    # Left Column: Positions (x, y, z)
    axes[0, 0].plot(time_vec, x_1, 'b-')
    axes[0, 0].plot(time_vec, x_2, 'r-')
    axes[0, 0].set_title("Position x (m)")
    axes[0, 0].grid(True)

    axes[1, 0].plot(time_vec, y_1, 'b-')
    axes[1, 0].plot(time_vec, y_2, 'r-')
    axes[1, 0].set_title("Position y (m)")
    axes[1, 0].grid(True)

    axes[2, 0].plot(time_vec, z_1, 'b-', lw=2)
    axes[2, 0].plot(time_vec, z_2, 'r-', lw=2)
    axes[2, 0].set_title("Position z (Height) (m)")
    axes[2, 0].grid(True)

    # Right Column: Orientations (Roll, Pitch, Yaw)
    axes[0, 1].plot(time_vec, phi_1, 'b-')
    axes[0, 1].plot(time_vec, phi_2, 'r-')
    axes[0, 1].set_title("Roll Angle $ phi$ (rad)")
    axes[0, 1].grid(True)

    axes[1, 1].plot(time_vec, theta_1, 'b-')
    axes[1, 1].plot(time_vec, theta_2, 'r-')
    axes[1, 1].set_title("Pitch Angle $ theta$ (rad)")
    axes[1, 1].grid(True)

    axes[2, 1].plot(time_vec, psi_1, 'b-')
    axes[2, 1].plot(time_vec, psi_2, 'b-')
    axes[2, 1].set_title("Yaw Angle $ psi$ (rad)")
    axes[2, 1].grid(True)

    # Apply x-axis label to all subplots
    for ax in axes.flat:
        ax.set_xlabel("Time (s)")
        
    pid_line = mlines.Line2D([], [], color='blue', linestyle='-', label='PID')
    lqr_line = mlines.Line2D([], [], color='red', linestyle='-', label='LQR')

    fig.legend(handles=[pid_line, lqr_line], loc='upper center', bbox_to_anchor=(0.5, 0.98), ncol=2, frameon=True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
    return fig, axes