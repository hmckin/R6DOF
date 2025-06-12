import numpy as np
import os
from simulation.montecarlo import monte_carlo_simulation
from visualization.plots import plot_montecarlo_attitude, plot_montecarlo_altitude, plot_metric_hist

print("Starting Monte Carlo 6DOF rocket simulation...")

num_runs = 100
results = monte_carlo_simulation(num_runs=num_runs)

all_t = []
all_z = []
all_pitch = []
errors = []
overshoots = []
control_efforts = []
for res in results:
    sol = res['sol']
    metrics = res['metrics']
    if hasattr(sol, 't') and hasattr(sol, 'y'):
        t = sol.t
        y = sol.y
    else:
        t = sol[:, 0]
        y = sol[:, 1:13].T
    all_t.append(t)
    all_z.append(y[2])  # Altitude
    all_pitch.append(np.rad2deg(y[7]))  # Pitch in degrees
    errors.append(metrics['final_error_deg'])
    overshoots.append(metrics['overshoot_deg'])
    control_efforts.append(metrics['control_effort'])

os.makedirs('results', exist_ok=True)
plot_montecarlo_altitude(all_t, all_z)
plot_montecarlo_attitude(all_t, all_pitch)
plot_metric_hist(errors, 'Final Pitch Error [deg]', 'Monte Carlo Final Pitch Error', 'montecarlo_pitch_error_hist.png')
plot_metric_hist(overshoots, 'Pitch Overshoot [deg]', 'Monte Carlo Pitch Overshoot', 'montecarlo_pitch_overshoot_hist.png')
plot_metric_hist(control_efforts, 'Control Effort [Nm·s]', 'Monte Carlo Control Effort', 'montecarlo_control_effort_hist.png')

print("Monte Carlo plots saved to results directory.")
print("Simulation complete.") 