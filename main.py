import numpy as np
import os
from simulation.simulator import simulate_closed_loop
from visualization.plots import plot_attitude, plot_altitude, plot_torques

print("Starting closed-loop 6DOF rocket simulation...")

# Run closed-loop simulation
total_result = simulate_closed_loop()
sol = total_result[0]
torques_history = total_result[1]

# Extract time and state
if hasattr(sol, 't') and hasattr(sol, 'y'):
    t = sol.t
    y = sol.y
else:
    t = sol[:, 0]
    y = sol[:, 1:13].T  # fallback for array output

# Ensure results directory exists
os.makedirs('results', exist_ok=True)

# Plot and save attitude (roll, pitch, yaw)
plot_attitude(t, y)
# Plot and save altitude
plot_altitude(t, y)
# Plot and save control torques
plot_torques(torques_history)

print("Plots saved to results directory.")
print("Simulation complete.")
