import numpy as np
from config import ROCKET, THRUST_PROFILE, GRAVITY, AIR_DENSITY

def rotation_matrix_euler(angles):
    """
    Compute rotation matrix from body to inertial frame given Euler angles (roll, pitch, yaw).
    angles: [phi, theta, psi] in radians
    Returns 3x3 rotation matrix
    """
    phi, theta, psi = angles
    cphi, sphi = np.cos(phi), np.sin(phi)
    cth, sth = np.cos(theta), np.sin(theta)
    cpsi, spsi = np.cos(psi), np.sin(psi)
    # ZYX order
    R = np.array([
        [cpsi*cth, cpsi*sth*sphi - spsi*cphi, cpsi*sth*cphi + spsi*sphi],
        [spsi*cth, spsi*sth*sphi + cpsi*cphi, spsi*sth*cphi - cpsi*sphi],
        [-sth,     cth*sphi,                 cth*cphi]
    ])
    return R

def euler_angle_rates(angles, omega):
    """
    Convert body angular velocity to Euler angle rates.
    angles: [phi, theta, psi]
    omega: [wx, wy, wz] in body frame
    Returns [dphi, dtheta, dpsi]
    """
    phi, theta, _ = angles
    wx, wy, wz = omega
    t1 = np.array([
        [1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
        [0, np.cos(phi),             -np.sin(phi)],
        [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]
    ])
    return t1 @ omega

def rocket_ode(t, state, torque=None, thrust=None, drag_coeff=None):
    """
    ODE function for 6DOF rocket dynamics.
    state: [x, y, z, vx, vy, vz, phi, theta, psi, wx, wy, wz]
    torque: np.ndarray, shape (3,), control torque in body frame (e.g., from PID controller)
    thrust: float, thrust in N
    drag_coeff: float, drag coefficient
    Returns dstate/dt
    """
    # Unpack state
    x, y, z = state[0:3]
    vx, vy, vz = state[3:6]
    phi, theta, psi = state[6:9]
    wx, wy, wz = state[9:12]
    
    # Translational dynamics
    mass = ROCKET.mass
    # Use provided overrides or fall back to global config values
    if thrust is None:
        thrust = THRUST_PROFILE.thrust  # N

    if drag_coeff is None:
        drag_coeff = ROCKET.drag_coefficient  # dimensionless

    # Thrust in body frame (assume along +z)
    thrust_body = np.array([0, 0, thrust])
    # Convert thrust to inertial frame
    R = rotation_matrix_euler([phi, theta, psi])
    thrust_inertial = R @ thrust_body
    # Gravity in inertial frame
    gravity = np.array([0, 0, -mass * GRAVITY])

    # Simple quadratic drag opposite to velocity (in inertial frame)
    vel_vec = np.array([vx, vy, vz])
    speed = np.linalg.norm(vel_vec)
    if speed > 0:
        drag_dir = -vel_vec / speed
    else:
        drag_dir = np.zeros(3)
    drag_mag = 0.5 * AIR_DENSITY * speed**2 * drag_coeff * ROCKET.reference_area
    drag_force = drag_mag * drag_dir

    # Net force
    force = thrust_inertial + gravity + drag_force
    # Acceleration
    accel = force / mass
    
    # Rotational dynamics (Euler's equations)
    I = np.array(ROCKET.moment_of_inertia)
    omega = np.array([wx, wy, wz])
    # Use provided torque or default to zero
    if torque is None:
        torque = np.zeros(3)
    domega = np.zeros(3)
    domega[0] = (torque[0] - (I[2] - I[1]) * wy * wz) / I[0]
    domega[1] = (torque[1] - (I[0] - I[2]) * wx * wz) / I[1]
    domega[2] = (torque[2] - (I[1] - I[0]) * wx * wy) / I[2]
    
    # Euler angle rates
    dangles = euler_angle_rates([phi, theta, psi], omega)
    
    # Pack derivatives
    dstate = np.zeros_like(state)
    dstate[0:3] = [vx, vy, vz]
    dstate[3:6] = accel
    dstate[6:9] = dangles
    dstate[9:12] = domega
    return dstate
