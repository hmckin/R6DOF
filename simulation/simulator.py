import numpy as np
from models.dynamics import rocket_ode
from config import INITIAL_CONDITIONS, TIME_STEP, TOTAL_TIME
from control.pid import PID

def guidance_law(t):
    """
    Define target [roll, pitch, yaw] over time.
    For example: pitch over from 0 to 10 degrees in first 5 seconds, then hold.
    """
    pitch_target = np.deg2rad(10) if t > 5 else np.deg2rad(0)
    return np.array([0.0, pitch_target, 0.0])  # [roll, pitch, yaw]

def simulate_closed_loop():
    # Initial state: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    state = np.array(
        INITIAL_CONDITIONS.position +
        INITIAL_CONDITIONS.velocity +
        INITIAL_CONDITIONS.orientation +
        INITIAL_CONDITIONS.angular_velocity,
        dtype=float
    )
    t = 0.0
    history = []
    N = int(TOTAL_TIME / TIME_STEP)

    # PID controllers for roll, pitch, yaw
    pid_roll = PID(kp=10, ki=0.1, kd=2)
    pid_pitch = PID(kp=10, ki=0.1, kd=2)
    pid_yaw = PID(kp=10, ki=0.1, kd=2)

    for i in range(N):
        # --- Guidance law: get target attitude ---
        target_angles = guidance_law(t)

        # --- Control: compute attitude error and PID output ---
        current_angles = state[6:9]  # [phi, theta, psi]
        error = target_angles - current_angles
        # Wrap error to [-pi, pi] for each angle
        error = (error + np.pi) % (2 * np.pi) - np.pi
        # PID outputs (torques)
        torque = np.array([
            pid_roll.update(error[0], TIME_STEP),
            pid_pitch.update(error[1], TIME_STEP),
            pid_yaw.update(error[2], TIME_STEP)
        ])

        # --- Dynamics: propagate state with control torque ---
        dstate = rocket_ode(t, state, torque)
        # Euler integration, would use Runge-Kutta for better accuracy
        state = state + dstate * TIME_STEP
        t += TIME_STEP
        history.append(np.concatenate(([t], state, target_angles, torque)))

    history = np.array(history)
    return history
