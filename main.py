import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from config import INITIAL_CONDITIONS, TIME_STEP, TOTAL_TIME
from models.dynamics import rocket_ode
import os

print("Starting 6DOF rocket simulation...")

# Initial state: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
state0 = (
    INITIAL_CONDITIONS.position +
    INITIAL_CONDITIONS.velocity +
    INITIAL_CONDITIONS.orientation +
    INITIAL_CONDITIONS.angular_velocity
)

# Time span
t_span = (0, TOTAL_TIME)
t_eval = np.arange(0, TOTAL_TIME, TIME_STEP)

# Run simulation
sol = solve_ivp(
    rocket_ode,
    t_span,
    state0,
    t_eval=t_eval,
    rtol=1e-8,
    atol=1e-8
)

# Check if rocket lifted off (z increases from 0)
lifted_off = np.any(sol.y[2] > 0.1)  # z > 0.1 m
max_altitude = np.max(sol.y[2])
final_altitude = sol.y[2, -1]
flight_time = sol.t[-1]

if lifted_off:
    print("Rocket lifted off!")
else:
    print("Rocket did NOT lift off.")
print(f"Maximum altitude reached: {max_altitude:.2f} m")
print(f"Final altitude: {final_altitude:.2f} m")
print(f"Total flight time simulated: {flight_time:.2f} s")

# Plot altitude vs time
plt.figure(figsize=(8, 5))
plt.plot(sol.t, sol.y[2], label='Altitude (z)')
plt.xlabel('Time [s]')
plt.ylabel('Altitude [m]')
plt.title('Rocket Altitude vs Time')
plt.grid(True)
plt.legend()

# Ensure results directory exists
os.makedirs('results', exist_ok=True)

# Save plot
plot_path = os.path.join('results', 'altitude_vs_time.png')
plt.savefig(plot_path)
print(f"Plot saved to {plot_path}")

# Save raw data
np.savez(os.path.join('results', 'simulation_data.npz'), t=sol.t, state=sol.y)
print("Simulation data saved to results/simulation_data.npz")
print("Simulation complete.")
