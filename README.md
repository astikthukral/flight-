HexaCopter Flight Controller Firmware (ESP32)


Overview
This repository contains the flight controller firmware for a Hexacopter UAV built around the ESP32 microcontroller. The codebase has undergone multiple iterations and improvements focusing on stability, sensor integration, and calibration routines to achieve reliable and responsive flight control.

Features
Sensor fusion and calibration
Implements a multi-stage sensor calibration process (IMU, magnetometer, barometer) with adjustable timing to ensure accurate flight data.

Real-time flight control algorithms
PID-based attitude control tuned for hexacopter dynamics.

Modular architecture
Clean separation of sensor reading, calibration, control loops, and communication.

Support for multiple flight modes
Manual, Stabilize, Altitude Hold, and Autonomous waypoint navigation (work in progress).

Telemetry and logging
Real-time data output via serial or wireless for analysis and debugging.

Hardware Requirements
Component	Description
ESP32 Dev Board	Primary flight controller
MPU9250	9-axis IMU sensor
Barometer	Altitude measurement
ESCs (6x)	Electronic speed controllers
Motors (6x)	Brushless motors
Power Supply	LiPo Battery
RC Receiver	Manual input (

Calibration Procedure
Proper sensor calibration is critical for flight stability:

IMU Calibration
On startup, the system performs a 5-10 second gyroscope and accelerometer calibration to determine bias offsets.

Magnetometer Calibration
Requires slow rotation around all axes; calibration time ~15-30 seconds.

Barometer Offset
Calibrated at rest to establish a baseline altitude.
