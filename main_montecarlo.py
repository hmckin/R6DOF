import numpy as np
import os
from simulation.montecarlo import monte_carlo_simulation
from visualization.plots import plot_montecarlo_attitude, plot_montecarlo_altitude

print("Starting Monte Carlo 6DOF rocket simulation...")

num_runs = 100
results = monte_carlo_simulation(num_runs=num_runs)

all_t = []
all_z = []
all_pitch = []
for sol, _ in results:
    if hasattr(sol, 't') and hasattr(sol, 'y'):
        t = sol.t
        y = sol.y
    else:
        t = sol[:, 0]
        y = sol[:, 1:13].T
    all_t.append(t)
    all_z.append(y[2])  # Altitude
    all_pitch.append(np.rad2deg(y[7]))  # Pitch in degrees

os.makedirs('results', exist_ok=True)
plot_montecarlo_altitude(all_t, all_z)
plot_montecarlo_attitude(all_t, all_pitch)

print("Monte Carlo plots saved to results directory.")
print("Simulation complete.") 