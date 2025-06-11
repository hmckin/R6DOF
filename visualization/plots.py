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
