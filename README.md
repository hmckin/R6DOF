# Rocket6DOF-Sim

A modular Python simulation framework for modeling, control, and analysis of a 6-degree-of-freedom (6DOF) rocket. The project is structured for extensibility, clarity, and rigorous testing, supporting features such as Monte Carlo analysis and 3D visualization.

---

## Project Overview

This repository simulates the flight dynamics of a rigid-body rocket in six degrees of freedom (6DOF), implements closed-loop control for attitude stabilization, and provides tools for trajectory guidance, Monte Carlo testing, and result visualization.

---

## Features

- **6DOF Rigid Body Dynamics**: Simulates translational and rotational motion under external forces and torques.
- **PID Attitude Control**: Stabilizes pitch, yaw, and roll using classical control techniques.
- **Trajectory Guidance**: Follows user-defined attitude profiles.
- **Monte Carlo Analysis**: Evaluates robustness under randomized conditions (e.g., thrust, mass, wind).
- **Visualization**: Generates 2D plots and optional 3D animations of flight and control performance.

---

## Repository Structure

 ```plaintext
main.py                    # Entry point for running simulations
config.py                  # Simulation constants and configuration

models/
  dynamics.py              # 6DOF equations of motion
  vehicle.py               # Rocket physical properties and state

control/
  pid.py                   # PID controller implementation

simulation/
  simulator.py             # Integrates dynamics and control
  montecarlo.py            # Monte Carlo test runner

visualization/
  plots.py                 # 2D plotting utilities
  animation.py             # 3D animation (optional)

results/                   # Output plots and data
tests/                     # Unit tests

requirements.txt           # Python dependencies
README.md                  # Project documentation
.gitignore                 # Standard Python ignores
```
---

## Quick Start

1. **Install dependencies**  
   (Recommended: use the provided virtual environment)
   ```sh
   pip install -r requirements.txt
   ```

2. **Configure simulation**  
   Edit `config.py` to set rocket parameters, initial conditions, and simulation options.

3. **Run a simulation**  
   ```sh
   python main.py
   ```

4. **View results**  
   Output plots and data will be saved in the `results/` directory.

---

## Governing Equations & Assumptions

### 6DOF Rigid Body Dynamics

The rocket is modeled as a rigid body with six degrees of freedom:

- **Translational Motion** (Newton's 2nd Law):

  ```
  F = m * dv/dt
  ```

  where `F` is the sum of external forces, `m` is mass, and `v` is velocity.

- **Rotational Motion** (Euler's Equations):

  ```
  I * dω/dt + ω × (I * ω) = M
  ```

  where `I` is the inertia tensor, `ω` is angular velocity, and `M` is the sum of external moments.

- **Attitude Representation**:  
  Euler angles (roll, pitch, yaw) are used for simplicity; quaternions may be added for robustness.

- **Numerical Integration**:  
  All ODEs are integrated using `scipy.integrate.solve_ivp`.

### Assumptions

- The rocket is a rigid body (no structural flex).
- Mass and inertia may be constant or updated as fuel is burned (configurable).
- Aerodynamic forces and moments are simplified or parameterized.
- Wind and thrust uncertainties are modeled as random variables in Monte Carlo analysis.
- Gravity is constant and acts in the -Z direction.

---

## Project Phases

1. **Dynamics Core**: Implement 6DOF physics and vehicle model.
2. **Control System**: Add PID controllers for attitude stabilization.
3. **Guidance System**: Track desired attitude/trajectory profiles.
4. **Monte Carlo Testing**: Assess robustness to uncertainties.
5. **Visualization & Reporting**: Generate plots and animations for analysis.

---

## Testing

Unit tests are provided in the `tests/` directory and can be run with:
```sh
pytest
```

---

## License

This project is open source and available under the MIT License.
