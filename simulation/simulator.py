import numpy as np
from scipy.integrate import solve_ivp
from models.dynamics import rocket_ode
from config import INITIAL_CONDITIONS, TOTAL_TIME, TIME_STEP
from control.pid import PID

def guidance_law(t):
    """
    Define target [roll, pitch, yaw] over time.
    For example: pitch over from 0 to 10 degrees in first 5 seconds, then hold.
    """
    pitch_target = np.deg2rad(10) if t > 5 else np.deg2rad(0)
    return np.array([0.0, pitch_target, 0.0])  # [roll, pitch, yaw]

def simulate_closed_loop():
    # Initial state vector: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    state0 = np.array(
        INITIAL_CONDITIONS.position +
        INITIAL_CONDITIONS.velocity +
        INITIAL_CONDITIONS.orientation +
        INITIAL_CONDITIONS.angular_velocity,
        dtype=float
    )

    # PID controllers
    pid_roll = PID(kp=10, ki=0.1, kd=2)
    pid_pitch = PID(kp=10, ki=0.1, kd=2)
    pid_yaw = PID(kp=10, ki=0.1, kd=2)

    # To track previous errors for derivative control
    prev_error = np.zeros(3)
    integral_error = np.zeros(3)

    def closed_loop_dynamics(t, state):
        nonlocal prev_error, integral_error

        # --- Guidance target ---
        target_angles = guidance_law(t)
        current_angles = state[6:9]  # phi, theta, psi

        # --- PID control ---
        pid_roll.setpoint = target_angles[0]
        pid_pitch.setpoint = target_angles[1]
        pid_yaw.setpoint = target_angles[2]
        torque = np.array([
            pid_roll.update(current_angles[0], TIME_STEP),
            pid_pitch.update(current_angles[1], TIME_STEP),
            pid_yaw.update(current_angles[2], TIME_STEP)
        ])

        # --- Rocket dynamics ---
        return rocket_ode(t, state, torque)

    # Solve using solve_ivp
    sol = solve_ivp(
        fun=closed_loop_dynamics,
        t_span=(0, TOTAL_TIME),
        y0=state0,
        method='RK45',
        dense_output=True,
        max_step=0.05  # Optional: controls resolution
    )

    return sol

