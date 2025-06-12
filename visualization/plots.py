import matplotlib.pyplot as plt
import numpy as np
import os

def plot_attitude(t, y):
    # y: shape (12, N) or (N, 12), [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    if y.shape[0] == 12:
        phi, theta, psi = y[6], y[7], y[8]
    else:
        phi, theta, psi = y[:,6], y[:,7], y[:,8]
    plt.figure()
    plt.plot(t, np.rad2deg(phi), label='Roll (deg)')
    plt.plot(t, np.rad2deg(theta), label='Pitch (deg)')
    plt.plot(t, np.rad2deg(psi), label='Yaw (deg)')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.title('Attitude vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join('results', 'attitude_vs_time.png'))
    plt.close()

def plot_altitude(t, y):
    # y: shape (12, N) or (N, 12)
    if y.shape[0] == 12:
        z = y[2]
    else:
        z = y[:,2]
    plt.figure()
    plt.plot(t, z, label='Altitude (z)')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.title('Altitude vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join('results', 'altitude_vs_time.png'))
    plt.close()

def plot_torques(torques_history):
    t = torques_history[:, 0]
    torques = torques_history[:, 1:].T  # shape (3, N)
    plt.figure()
    plt.plot(t, torques[0], label='Torque Roll')
    plt.plot(t, torques[1], label='Torque Pitch')
    plt.plot(t, torques[2], label='Torque Yaw')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title('Control Torques vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join('results', 'torques_vs_time.png'))
    plt.close()

def plot_montecarlo_altitude(all_t, all_z):
    plt.figure()
    first = True
    for t, z in zip(all_t, all_z):
        if first:
            plt.plot(t, z, color='blue', alpha=0.1, label='Individual Runs')
            first = False
        else:
            plt.plot(t, z, color='blue', alpha=0.1)
    # Compute statistics if all runs have the same time base
    if all(len(t) == len(all_t[0]) for t in all_t):
        z_stack = np.stack(all_z)
        mean_z = np.mean(z_stack, axis=0)
        std_z = np.std(z_stack, axis=0)
        plt.plot(all_t[0], mean_z, color='black', label='Mean Altitude')
        plt.fill_between(all_t[0], mean_z-std_z, mean_z+std_z, color='gray', alpha=0.3, label='±1 Std Dev')
    plt.xlabel('Time [s]')
    plt.ylabel('Altitude [m]')
    plt.title('Monte Carlo Altitude vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join('results', 'montecarlo_altitude.png'))
    plt.close()

def plot_montecarlo_attitude(all_t, all_pitch):
    plt.figure()
    first = True
    for t, pitch in zip(all_t, all_pitch):
        if first:
            plt.plot(t, pitch, color='orange', alpha=0.1, label='Individual Runs')
            first = False
        else:
            plt.plot(t, pitch, color='orange', alpha=0.1)
    # Compute statistics if all runs have the same time base
    if all(len(t) == len(all_t[0]) for t in all_t):
        pitch_stack = np.stack(all_pitch)
        mean_pitch = np.mean(pitch_stack, axis=0)
        std_pitch = np.std(pitch_stack, axis=0)
        plt.plot(all_t[0], mean_pitch, color='black', label='Mean Pitch')
        plt.fill_between(all_t[0], mean_pitch-std_pitch, mean_pitch+std_pitch, color='gray', alpha=0.3, label='±1 Std Dev')
    plt.xlabel('Time [s]')
    plt.ylabel('Pitch [deg]')
    plt.title('Monte Carlo Pitch vs Time')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join('results', 'montecarlo_pitch.png'))
    plt.close()

def plot_metric_hist(data, xlabel, title, filename):
    plt.figure()
    plt.hist(data, bins=20, color='green', alpha=0.7)
    plt.xlabel(xlabel)
    plt.ylabel('Count')
    plt.title(title)
    plt.grid(True)
    plt.savefig(os.path.join('results', filename))
    plt.close()
