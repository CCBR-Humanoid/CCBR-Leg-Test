# CCBR Leg Test

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

### Running the GUI Application
The `unitree_leg_gui` folder contains a Python-based graphical user interface (GUI) for controlling a robotic leg. This GUI provides a higher-level interface compared to the C++ and Python examples, allowing for real-time control and feedback.

#### Steps to Run the GUI
1. Navigate to the `unitree_leg_gui` directory:
```bash
cd unitree_leg_gui
```

2. Run the `main.py` script:
```bash
python3 main.py
```

3. The GUI window will open, providing controls for:
   - **Live Feedback**: Displays joint angles, end-effector position, motor torques, and motor status (temperature, errors).
   - **Control Modes**: Switch between inverse kinematics (IK) and direct angle control.
   - **Jump Motion**: Execute predefined jump scripts with adjustable timing parameters.

#### Key Features of the GUI
- **Live Feedback**: Continuously updates joint angles, end-effector position, and motor torques in real-time.
- **Control Modes**:
  - **IK Mode**: Specify the desired position of the end-effector (x, y, z) in millimeters. The GUI calculates the required joint angles using inverse kinematics.
  - **Angles Mode**: Directly control the joint angles (θ1, θ2, θ3) in radians.
- **Jump Motion**: Execute scripted jump motions with adjustable parameters for crouch, hold, and explode phases.

#### Code Overview
- `gui_leg.py`: The main entry point for the GUI. Handles user interactions, motor commands, and live feedback updates.
- `kinematics.py`: Provides forward and inverse kinematics calculations for the robotic leg.
- `config.py`: Centralized configuration for motor parameters, limits, and default values.
- `jump_control.py`: Implements the logic for scripted jump motions.

### Notes
- Ensure proper permissions for accessing `/dev/ttyUSB0` (e.g., `sudo` or `chmod`).
- Default serial baud rates and timeouts are critical for motor stability. Avoid modifying them unless necessary.

For detailed documentation, refer to the comments within the code files.
