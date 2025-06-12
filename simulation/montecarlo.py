import numpy as np
from copy import deepcopy
from simulation.simulator import simulate_closed_loop
from config import INITIAL_CONDITIONS, THRUST_PROFILE, ROCKET, PID_GAINS, TIME_STEP


def monte_carlo_simulation(num_runs=100, seed=None):
    """Run Monte Carlo simulations with random variations.

    Returns list of dicts with keys: 'sol', 'torques', 'metrics'.
    Metrics include final_pitch_error (deg), overshoot (deg), control_effort (Nm·s).
    """
    if seed is not None:
        np.random.seed(seed)

    results = []
    base_thrust = THRUST_PROFILE.thrust
    base_drag = ROCKET.drag_coefficient

    for _ in range(num_runs):
        # Randomize initial orientation and angular velocity
        random_orientation = list(np.random.uniform(-0.05, 0.05, 3))  # rad
        random_ang_vel = list(np.random.uniform(-0.1, 0.1, 3))  # rad/s
        init_state = (
            INITIAL_CONDITIONS.position +
            INITIAL_CONDITIONS.velocity +
            random_orientation +
            random_ang_vel
        )

        # Randomize thrust (±10%) and drag (±10%)
        thrust_rand = base_thrust * np.random.uniform(0.9, 1.1)
        drag_rand = base_drag * np.random.uniform(0.9, 1.1)

        # Randomize PID gains (±20%)
        gains_rand = deepcopy(PID_GAINS)
        for axis in gains_rand.values():
            axis['kp'] *= np.random.uniform(0.8, 1.2)
            axis['ki'] *= np.random.uniform(0.8, 1.2)
            axis['kd'] *= np.random.uniform(0.8, 1.2)

        # Run simulation
        sol, torques_hist = simulate_closed_loop(
            initial_state=init_state,
            gains=gains_rand,
            thrust=thrust_rand,
            drag=drag_rand
        )

        # Compute metrics
        # Final pitch error (deg)
        final_pitch = np.rad2deg(sol.y[7, -1])
        target_pitch = 10.0  # deg
        final_error = abs(final_pitch - target_pitch)
        # Overshoot (max deviation above target)
        max_pitch = np.rad2deg(np.max(sol.y[7]))
        overshoot = max(0, max_pitch - target_pitch)
        # Control effort: sum(|torque|) * dt
        effort = np.sum(np.abs(torques_hist[:, 1:])) * TIME_STEP

        metrics = {
            'final_error_deg': final_error,
            'overshoot_deg': overshoot,
            'control_effort': effort
        }

        results.append({'sol': sol, 'torques': torques_hist, 'metrics': metrics})

    return results
