import numpy as np
from simulation.simulator import simulate_closed_loop
from config import INITIAL_CONDITIONS


def monte_carlo_simulation(num_runs=100, seed=None):
    if seed is not None:
        np.random.seed(seed)
    results = []
    for i in range(num_runs):
        # Randomize initial conditions (example: orientation and angular velocity)
        init_cond = INITIAL_CONDITIONS
        random_orientation = list(np.random.uniform(-0.05, 0.05, 3))  # radians
        random_ang_vel = list(np.random.uniform(-0.1, 0.1, 3))  # rad/s
        # Optionally randomize position, velocity, mass, etc.
        # Can also deepcopy and modify the Rocket if needed
        init_state = (
            init_cond.position +
            init_cond.velocity +
            random_orientation +
            random_ang_vel
        )

        sol, torques_history = simulate_closed_loop(initial_state=init_state)
        results.append((sol, torques_history))
    return results
