# Unitree Actuator SDK

This repository provides a compact SDK and examples for controlling Unitree actuators, including the GO-M8010-6, A1, and B1 motors. It includes both C++ and Python examples, as well as a GUI for higher-level control.

## Quick Start

### Requirements
- Supported motors: GO-M8010-6, A1, B1
- GCC version:
  - x86 platform: GCC >= 5.4.0
  - ARM platform: GCC >= 7.5.0

Check your GCC version with:
```bash
gcc --version
```

### Build Instructions
1. Create a build directory and compile the code:
```bash
mkdir build
cd build
cmake ..
make
```

2. After a successful build, example executables will be available in the `build/` folder.

### Running Examples
#### C++ Examples
Run the compiled examples with `sudo`:
```bash
sudo ./example_a1_motor
```

#### Python Examples
Navigate to the `python/` folder and run the Python scripts with `sudo`:
```bash
sudo python3 example_a1_motor.py
```

#### GUI Application
To run the GUI for controlling the robotic leg:
```bash
cd unitree_leg_gui
python3 main.py
```

## Code Overview

### C++ Examples
The C++ examples demonstrate low-level motor control. For instance, the following snippet from `example_a1_motor.cpp` shows how to configure and send commands to the motor:
```c++
cmd.motorType = MotorType::A1;
data.motorType = MotorType::A1;
cmd.mode  = queryMotorMode(MotorType::A1, MotorMode::FOC);
cmd.id    = 0;
cmd.kp    = 0.0;
cmd.kd    = 2;
cmd.q     = 0.0;
cmd.dq    = -6.28 * queryGearRatio(MotorType::A1);
cmd.tau   = 0.0;
serial.sendRecv(&cmd, &data);
```
**Note:** Commands are for the **rotor** side. Ensure proper conversion if working with the **output** side.

### Python Examples
The Python examples use the `unitree_actuator_sdk` module to interact with the motors. For example, `example_a1_motor.py` demonstrates basic motor control.

### GUI Application
The GUI, located in `unitree_leg_gui/`, provides a higher-level interface for controlling the robotic leg. It includes features such as:
- Live feedback of joint angles, end-effector position, and motor torques.
- Modes for inverse kinematics (IK) and direct angle control.
- Jump motion control using predefined scripts.

#### Key Files
- `gui_leg.py`: Main GUI logic.
- `kinematics.py`: Handles forward and inverse kinematics calculations.
- `config.py`: Centralized configuration for motor parameters and limits.
- `jump_control.py`: Implements jump motion logic.

## Notes
- The codebase is modular and designed for maintainability. Configuration constants are centralized in `config.py`.
- Ensure proper permissions for accessing `/dev/ttyUSB0` (e.g., `sudo` or `chmod`).
- Default serial baud rates and timeouts are critical for motor stability. Avoid modifying them unless necessary.

For detailed documentation, refer to the comments within the code files.
