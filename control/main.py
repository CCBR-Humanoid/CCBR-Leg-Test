# main.py
"""
Entry point for Unitree leg control.

- Adds ../lib to sys.path so `unitree_actuator_sdk` is importable.
- Connects to the Unitree GO-M8010-6 motors.
- Zeroes the leg by driving all joints to 0 rad.
- Lets you command Cartesian (x, y, z) targets via IK using a PD loop
  with Kp = 0.01, Kd = 0.01 on position/velocity.
"""

import os
import sys
import time
import math
import numpy as np

# --------------------------------------------------------------------
# Make sure ../lib (Unitree SDK) and this folder are on sys.path
# --------------------------------------------------------------------
THIS_DIR = os.path.dirname(__file__)
SDK_PATH = os.path.join(THIS_DIR, "..", "lib")

if SDK_PATH not in sys.path:
    sys.path.append(SDK_PATH)
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

# Now these imports should work
import config
import kinematics
from unitree_actuator_sdk import SerialPort, MotorCmd, MotorData


# ------------ PD gains (per your request) ------------
KP_POS = 0.01   # position gain
KD_VEL = 0.01   # velocity gain


# ------------ Helpers to convert joint <-> motor angles ------------

def joint_to_motor_angle(joint_angle_rad: float, motor_index: int) -> float:
    """
    Convert joint angle (output side, rad) to motor rotor angle (rad).
    Uses gear ratio and direction from config.
    """
    gear = config.GEAR[motor_index]
    direction = config.DIR[motor_index]
    return joint_angle_rad * gear * direction


def motor_to_joint_angle(motor_angle_rad: float, motor_index: int) -> float:
    """
    Convert motor rotor angle (rad) to joint angle (output side, rad).
    """
    gear = config.GEAR[motor_index]
    direction = config.DIR[motor_index]
    return motor_angle_rad * direction / gear


# ------------ Low-level send helper ------------

def send_pd_command(port, motor_id: int, q_des_joint: float, dq_des_joint: float = 0.0):
    """
    Send a single PD command to one motor, where q_des_joint and dq_des_joint
    are in *joint/output* radians and rad/s.

    This converts them to rotor side and fills MotorCmd.
    """
    cmd = MotorCmd()
    data = MotorData()

    # match your working example
    cmd.motorType = config.MOTOR_TYPE
    data.motorType = config.MOTOR_TYPE
    cmd.mode = config.CONTROL_MODE

    cmd.id = motor_id

    # output -> rotor
    motor_index = config.MOTOR_IDS.index(motor_id)
    q_des_rotor = joint_to_motor_angle(q_des_joint, motor_index)
    dq_des_rotor = joint_to_motor_angle(dq_des_joint, motor_index)

    cmd.q = q_des_rotor
    cmd.dq = dq_des_rotor
    cmd.kp = KP_POS
    cmd.kd = KD_VEL
    cmd.tau = 0.0  # no torque feedforward for now

    port.sendRecv(cmd, data)
    return data


def read_current_joint_angles(port) -> np.ndarray:
    """
    Query current rotor positions and convert to joint angles (rad).
    Assumes all motors are already enabled and in CONTROL_MODE.
    """
    q_joint = np.zeros(3)
    for i, mid in enumerate(config.MOTOR_IDS):
        cmd = MotorCmd()
        data = MotorData()

        cmd.motorType = config.MOTOR_TYPE
        data.motorType = config.MOTOR_TYPE
        cmd.mode = config.CONTROL_MODE

        cmd.id = mid
        # "idle" command to just read state
        cmd.q = 0.0
        cmd.dq = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.tau = 0.0

        port.sendRecv(cmd, data)
        q_joint[i] = motor_to_joint_angle(data.q, i)
    return q_joint


# ------------ Zeroing routine ------------

def zero_leg(port, duration_s: float = 5.0, s_y: int = +1):
    """
    Drive all three joints to 0 rad using PD control for duration_s seconds.

    Zero pose is defined as:
        theta1 = theta2 = theta3 = 0 (joint/output angles)
    Also prints the FK XYZ at the end.
    """
    print("=== Zeroing leg ===")
    print("Driving joints to (theta1, theta2, theta3) = (0, 0, 0) rad")
    print(f"Holding this for {duration_s:.1f} seconds with PD gains:")
    print(f"    Kp = {KP_POS}, Kd = {KD_VEL}")
    print("Make sure the leg can safely move to this pose!")

    steps = int(duration_s / config.DT)
    target = np.zeros(3)

    for _ in range(steps):
        for i, mid in enumerate(config.MOTOR_IDS):
            send_pd_command(port, mid, target[i], 0.0)
        time.sleep(config.DT)

    # After zeroing, read back and print FK XYZ
    q_final = read_current_joint_angles(port)
    pos_final, _ = kinematics.fk_leg(q_final[0], q_final[1], q_final[2], s_y=s_y)
    print("Zeroing phase done.")
    print("Measured joint angles after zeroing [rad]:", q_final)
    print("FK foot position at this pose [mm]: x={:.1f}, y={:.1f}, z={:.1f}".format(
        pos_final[0], pos_final[1], pos_final[2]
    ))
    print()


# ------------ IK-based target routine ------------

def move_leg_to_cartesian(port, x_mm: float, y_mm: float, z_mm: float, s_y: int = +1,
                          move_time_s: float = 2.0):
    """
    Solve IK for (x, y, z) in mm, pick a nice solution, and smoothly move the leg there
    over move_time_s seconds using a simple linear interpolation in joint space.
    """
    print(f"Target foot position: x={x_mm:.1f} mm, y={y_mm:.1f} mm, z={z_mm:.1f} mm")

    # Inverse kinematics (may return multiple solutions)
    candidates = kinematics.ik_leg(x_mm, y_mm, z_mm, s_y=s_y)
    if not candidates:
        print("  [IK] No valid IK solutions for this target.")
        return

    # Get current joint angles to choose closest IK solution
    q_current = read_current_joint_angles(port)
    best = kinematics.pick_closest_solution(q_current, candidates)
    if best is None:
        print("  [IK] Could not select an IK solution.")
        return

    q_target = np.array(best)
    print("  IK solution (theta1, theta2, theta3) [rad]:", q_target)

    # Simple linear interpolation in joint space
    n_steps = max(1, int(move_time_s / config.DT))
    for k in range(n_steps):
        alpha = (k + 1) / n_steps
        q_cmd = (1.0 - alpha) * q_current + alpha * q_target
        for i, mid in enumerate(config.MOTOR_IDS):
            send_pd_command(port, mid, q_cmd[i], 0.0)
        time.sleep(config.DT)

    print("  Move complete.\n")


# ------------ Main entry ------------

def main():
    device = "/dev/ttyUSB0"
    s_y = +1  # left leg convention for FK/IK

    print("Opening SerialPort:", device)
    port = SerialPort(device)

    try:
        # --- Show zero-pose XYZ and wait for Enter before zeroing ---
        zero_angles = np.zeros(3)
        pos_zero, _ = kinematics.fk_leg(0.0, 0.0, 0.0, s_y=s_y)
        print("=== Zero pose preview ===")
        print("Zero joint angles (theta1=theta2=theta3=0 rad).")
        print("FK foot position for zero pose [mm]: "
              "x={:.1f}, y={:.1f}, z={:.1f}".format(
                  pos_zero[0], pos_zero[1], pos_zero[2]
              ))
        input("Press Enter to start zeroing the leg to this pose...")

        # 1) Zero the leg
        zero_leg(port, duration_s=5.0, s_y=s_y)

        # 2) CLI loop: ask for Cartesian targets
        print("=== IK control loop ===")
        print("Enter foot positions in mm (x y z), or 'q' to quit.")
        print("Coordinate frame: +x forward, +y left, +z DOWN.")
        print("Using left-leg sign s_y = +1.\n")

        while True:
            line = input("Target (x y z) in mm (or 'q'): ").strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            try:
                x_str, y_str, z_str = line.split()
                x = float(x_str)
                y = float(y_str)
                z = float(z_str)
            except ValueError:
                print("  Please enter three numbers, e.g. '150 0 -200'")
                continue

            move_leg_to_cartesian(port, x, y, z, s_y=s_y, move_time_s=2.0)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        # Relax motors
        print("Sending relax command (zero gains, zero torque)...")
        for mid in config.MOTOR_IDS:
            cmd = MotorCmd()
            data = MotorData()

            cmd.motorType = config.MOTOR_TYPE
            data.motorType = config.MOTOR_TYPE
            cmd.mode = config.CONTROL_MODE

            cmd.id = mid
            cmd.q = 0.0
            cmd.dq = 0.0
            cmd.kp = 0.0
            cmd.kd = 0.0
            cmd.tau = 0.0
            port.sendRecv(cmd, data)
        print("Done. Bye!")


if __name__ == "__main__":
    main()
