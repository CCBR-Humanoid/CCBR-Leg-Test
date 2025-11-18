## Quick orientation

This repository is a compact SDK and set of examples for Unitree actuators (A1, B1, GO_M8010_6). It contains:
- native C++ SDK and examples (built with CMake) — see `CMakeLists.txt` and `example/`.
- a pybind11-based Python module (`unitree_actuator_sdk`) built from `thirdparty/python_wrapper/wrapper.cpp` (CMake target in `CMakeLists.txt`).
- Python example scripts in `python/` and a small Tk GUI in `unitree_leg_gui/` that demonstrates higher-level control (IK, jumps).
- low-level serial/IO code in `include/serialPort/SerialPort.h` and `include/IOPort/IOPort.h` and motor message definitions in `include/unitreeMotor/`.
- CLI motor utilities under `motor_tools/` (prebuilt arm/x86 binaries and README with usage).

If you are an AI coding agent: focus on those files when you need to understand how data flows and how to run things.

## Build & run (explicit)
- Build native (x86/ARM):
  - mkdir build && cd build
  - cmake ..
  - make
  - Examples appear in `build/` (e.g. `build/example_a1_motor`).
- The project links against the prebuilt shared libs in `lib/` (CMake detects aarch64 to pick `libUnitreeMotorSDK_Arm64.so` vs `libUnitreeMotorSDK_Linux64.so`).
- Python module: CMake creates a pybind extension named `unitree_actuator_sdk` from `thirdparty/python_wrapper/wrapper.cpp` (target added via `pybind11_add_module` in `CMakeLists.txt`). After build, import from Python via `sys.path` to `lib/` or by installing the built module.

## Runtime notes & developer workflows
- Many examples and python scripts open `/dev/ttyUSB0` using `SerialPort` and expect high baudrates (4M is default in `SerialPort` constructor). Tests typically require permission to access the serial device — either run with `sudo` or `chmod` the device (see `motor_tools/README.md`).
- The `motor_tools/` folder provides helper binaries (`unisp`, `changeid`, `swboot`, `swmotor`, `cancelboot`) and usage examples; they are executed directly (may require `chmod +x` and `sudo`).
- The C++ examples and python examples populate and send `MotorCmd` structs and read `MotorData` (see `include/unitreeMotor/unitreeMotor.h`). When editing control logic, follow existing patterns where:
  - `cmd.motorType` and `data.motorType` are set together
  - `cmd.mode` uses `queryMotorMode(MotorType, MotorMode)` helpers
  - commands are for the rotor; the README documents rotor<->output conversions (kp/kd scaling and gear ratio considerations). See `example_a1_motor.cpp` and `example_a1_motor_output.cpp` for concrete conversions.

## Important code patterns and integration points
- Serial IO: `SerialPort` implements `IOPort` and provides `sendRecv(MotorCmd*, MotorData*)` and vector versions. Treat it as the single transport layer between examples / GUI and motors.
- Messages: `include/unitreeMotor/*` contains message layouts for each motor family (A1/B1/GO_M8010_6). Use `queryGearRatio()` and `queryMotorMode()` helpers rather than hardcoding magic numbers.
- Python binding: `pybind11_add_module(unitree_actuator_sdk thirdparty/python_wrapper/wrapper.cpp)` — if you update the wrapper, update the CMake target and re-run CMake. Python examples import `unitree_actuator_sdk` and construct `SerialPort`, `MotorCmd`, `MotorData` directly.
- GUI: `unitree_leg_gui/gui_leg.py` demonstrates a readable control loop pattern: warmup with many `sendRecv` calls, lightweight UI thread plus a control thread that sends commands via `SerialPort`. It is a practical example for safe, periodic motor interactions.

## Safety & platform-specific details AI agents should preserve
- Default serial baudrates and timeouts are important for motor stability. Avoid changing them unless necessary and verified with hardware.
- The project contains prebuilt native libraries (`lib/`). CMake chooses the right .so for aarch64. Keep that detection logic if you add platforms.
- Many examples expect root or device permissions; do not remove sudo/device permission guidance.

## Quick examples for the agent to reference
- Where to look when adding or changing control code:
  - low-level transport: `include/serialPort/SerialPort.h` and corresponding implementation files
  - data structures: `include/unitreeMotor/unitreeMotor.h`
  - C++ example usage: `example/example_a1_motor.cpp`, `example/example_a1_motor_output.cpp`
  - Python usage: `python/example_a1_motor.py`, `python/motor_test.py`
  - GUI demonstration: `unitree_leg_gui/gui_leg.py` (shows warmup pattern, control loop, torque-only send example)

## What NOT to change without hardware verification
- message framing, baudrate defaults, timeout values, and the rotor-vs-output scaling math in README and examples. These are hardware-dependent and tested in examples.

If anything in this file is unclear or you want me to add more examples (e.g., exact places to run unit tests, or a small CI target), tell me which area to expand and I will iterate.
