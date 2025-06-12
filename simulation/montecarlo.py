import numpy as np
from simulation.simulator import simulate_closed_loop
from config import INITIAL_CONDITIONS

# Example: randomize initial orientation and angular velocity

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
        # You can also deepcopy and modify the Rocket if needed
        # For now, just orientation and angular velocity
        # Patch the simulator to accept these ICs, or set globally if needed
        init_state = (
            init_cond.position +
            init_cond.velocity +
            random_orientation +
            random_ang_vel
        )
        # You may need to modify simulate_closed_loop to accept initial state as argument
        sol, torques_history = simulate_closed_loop(initial_state=init_state)
        results.append((sol, torques_history))
    return results
