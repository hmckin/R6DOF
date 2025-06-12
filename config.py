from dataclasses import dataclass
from typing import List
from models.vehicle import Rocket

# Physical constants
GRAVITY = 9.80665  # m/s^2, standard gravity
AIR_DENSITY = 1.225  # kg/m^3, sea level standard

# Simulation parameters
TIME_STEP = 0.01  # s
TOTAL_TIME = 60.0  # s

@dataclass
class ThrustProfile:
    thrust: float  # N, constant thrust for now
    # For a time-varying thrust, use a function or a list of (time, thrust) pairs

@dataclass
class InitialConditions:
    position: List[float]  # m, [x, y, z]
    velocity: List[float]  # m/s, [vx, vy, vz]
    orientation: List[float]  # rad, [roll, pitch, yaw]
    angular_velocity: List[float]  # rad/s, [wx, wy, wz]

# Example rocket configuration
ROCKET = Rocket(
    mass=50.0,  # kg
    moment_of_inertia=[10.0, 10.0, 1.0],  # kg*m^2
    center_of_mass=[0.0, 0.0, 2.0],  # m
    center_of_pressure=[0.0, 0.0, 2.5],  # m
    drag_coefficient=0.75,  # dimensionless
    lift_coefficient=0.1,  # dimensionless
    reference_area=0.0314,  # m^2 (e.g., 0.2 m diameter)
)

THRUST_PROFILE = ThrustProfile(
    thrust=1500.0,  # N
)

INITIAL_CONDITIONS = InitialConditions(
    position=[0.0, 0.0, 0.0],  # m
    velocity=[0.0, 0.0, 0.0],  # m/s
    orientation=[0.0, 0.0, 0.0],  # rad
    angular_velocity=[0.0, 0.0, 0.0],  # rad/s
)

# PID gains for roll, pitch, yaw
PID_GAINS = {
    'roll':  {'kp': 10.0, 'ki': 0.1, 'kd': 2.0},
    'pitch': {'kp': 10.0, 'ki': 0.1, 'kd': 2.0},
    'yaw':   {'kp': 10.0, 'ki': 0.1, 'kd': 2.0},
}
