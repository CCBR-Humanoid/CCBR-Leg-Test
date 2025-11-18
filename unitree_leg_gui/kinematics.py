# kinematics.py
import numpy as np

# leg geometry (mm)  +x fwd, +y left, +z DOWN
l1, l2, l3 = 90.0, 205.0, 240.0

def fk_leg(theta1, theta2, theta3, s_y=+1):
    """
    Forward kinematics (output-side radians) -> (x,y,z) mm, also returns planar z'
    """
    z_p = l2*np.cos(theta2) + l3*np.cos(theta2 + theta3)
    x   = l2*np.sin(theta2) + l3*np.sin(theta2 + theta3)
    y   = s_y*l1*np.cos(theta1) - z_p*np.sin(theta1)
    z   = s_y*l1*np.sin(theta1) + z_p*np.cos(theta1)
    return np.array([x, y, z]), z_p

def ik_leg(x, y, z, s_y=+1):
    """
    Inverse kinematics (mm) -> list of (t1,t2,t3) in output-side radians.
    Returns possibly multiple elbow-up/down solutions.
    """
    sols = []
    r_yz = np.hypot(y, z)
    if r_yz < l1 - 1e-9:
        return sols
    zprime_mag = np.sqrt(max(r_yz**2 - l1**2, 0.0))
    for z_p in (zprime_mag, -zprime_mag):
        theta1 = np.arctan2(z, y) - np.arctan2(z_p, s_y*l1)
        r = np.hypot(x, z_p)
        c3 = (r*r - l2*l2 - l3*l3) / (2.0*l2*l3)
        if c3 < -1.0 or c3 > 1.0:
            continue
        for knee_sign in (1.0, -1.0):  # elbow-down/up
            theta3 = knee_sign * np.arccos(np.clip(c3, -1.0, 1.0))
            theta2 = np.arctan2(x, z_p) - np.arctan2(l3*np.sin(theta3), l2 + l3*np.cos(theta3))
            sols.append((theta1, theta2, theta3))
    return sols

def pick_closest_solution(prev_angles, candidates):
    if not candidates:
        return None
    if prev_angles is None:
        elbow_down = [c for c in candidates if c[2] >= 0]
        return elbow_down[0] if elbow_down else candidates[0]
    # Use a weighted angular distance to pick the closest solution.
    # Historically sudden flips were observed when the roll (theta1)
    # wrapped and caused a candidate with a small roll difference but
    # large pitch/knee change to be chosen. For smoother leg motion
    # prioritize continuity of pitch and knee (indices 1 and 2).
    pa = np.array(prev_angles)
    best, best_dist = None, float("inf")
    weights = np.array([0.0, 1.0, 1.0])  # ignore roll when comparing
    # Hysteresis penalty to prefer keeping the same knee sign (elbow up/down)
    # This avoids sudden switches when the IK solver returns both elbow solutions
    # and the roll/other small differences make the alternative briefly closer.
    try:
        prev_knee_sign = 1 if pa[2] >= 0 else -1
    except Exception:
        prev_knee_sign = None
    penalty = 0.6  # radians; tune down to make switching easier
    for c in candidates:
        ca = np.array(c)
        # principal difference in [-pi, pi]
        d = (ca - pa + np.pi) % (2*np.pi) - np.pi
        dist = np.linalg.norm(d * weights)
        # apply penalty if knee sign changes
        try:
            cand_knee_sign = 1 if ca[2] >= 0 else -1
        except Exception:
            cand_knee_sign = None
        if prev_knee_sign is not None and cand_knee_sign != prev_knee_sign:
            dist += penalty
        if dist < best_dist:
            best_dist, best = dist, c
    return best

def clamp_to_workspace(x, y, z, s_y=+1):
    """
    Simple workspace clamp so IK stays solvable.
    You can tune these bounds based on your actual leg.
    """
    # Previously we clamped to rough workspace bounds here. Clamping removed
    # per repository preference — return requested coordinates unchanged.
    return (x, y, z)

def calculate_torque(tau, kp, kd, q_des, q, dq_des, dq):
    """
    Calculate the expected output torque of the motor rotor.

    Parameters:
        tau (float): Feedforward torque of the motor rotor (N.m).
        kp (float): Proportional coefficient of motor position error (stiffness coefficient).
        kd (float): Proportional coefficient of motor speed error (damping coefficient).
        q_des (float): Expected angular position of the motor rotor (rad).
        q (float): Current angular position of the motor rotor (rad).
        dq_des (float): Expected rotor angular velocity (rad/s).
        dq (float): Current rotor angular velocity (rad/s).

    Returns:
        float: The calculated torque (N.m).
    """
    torque = tau + kp * (q_des - q) + kd * (dq_des - dq)
    return torque
