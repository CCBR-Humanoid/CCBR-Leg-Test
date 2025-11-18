# config.py
import math
from unitree_actuator_sdk import MotorType, MotorMode, queryMotorMode

# Which motors we have on the leg
MOTOR_IDS = [0, 1, 2]  # 0: hip roll, 1: hip pitch, 2: knee
MOTOR_TYPE = MotorType.GO_M8010_6
CONTROL_MODE = queryMotorMode(MOTOR_TYPE, MotorMode.FOC)

# Gear ratios rotor->joint
GEAR = {
    0: 6.33,
    1: 6.33,
    2: 6.33 * (30.0 / 18.0),  # knee extra pulley
}

# Directions (+1 normal, -1 inverted)
DIR  = {
    0: +1.0,
    1: +1.0,
    2: +1.0,
}

# Base PD gains (we’ll override per phase in gui_leg)
KP = 0.1
KD = 0.1

# Control loop period (s)
DT = 0.0005  # 2000 Hz

# Max joint velocity we allow (output side) – used only for safety clamps if needed
VEL_LIMIT_OUTPUT = math.radians(720)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))
