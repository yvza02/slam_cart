Gyroscope Library (gyro_lib)
=================================

A lightweight MPU6050 gyroscope library for angle tracking using I2C on Raspberry Pi.

Designed for robotics applications such as:
- Rotation control
- Heading estimation
- Precise turning using angle feedback

--------------------------------------------------

OVERVIEW

This library reads angular velocity (Z-axis) from MPU6050 and integrates it over time to produce rotation angle.

It uses:
- Continuous background thread
- Offset calibration
- Noise filtering (deadband)

--------------------------------------------------

FEATURES

- I2C communication (SMBus)
- Auto retry on init
- Gyro calibration (offset correction)
- Continuous angle tracking
- Thread-safe angle reading
- Noise filtering using deadband

--------------------------------------------------

FUNCTIONS

init()
    Initialize MPU6050 sensor

calibrate(samples=500)
    Calculate offset (keep robot still)

reset()
    Reset accumulated angle to 0

start()
    Start background angle tracking

stop()
    Stop tracking thread

get_angle()
    Get current angle (integer degrees)

--------------------------------------------------

HOW IT WORKS

1. Read raw gyro Z-axis data
2. Convert to degrees/sec
3. Subtract offset (calibration)
4. Apply deadband filtering
5. Integrate over time:
   angle += gyro_rate * dt

--------------------------------------------------

USAGE

import gyro_lib

gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()

while True:
    angle = gyro_lib.get_angle()
    print("Angle:", angle)

--------------------------------------------------

IMPORTANT NOTES

- Always calibrate before use
- Keep robot completely still during calibration
- Call start() before reading angle
- Angle is relative (not absolute heading)
- Uses only Z-axis (yaw rotation)

--------------------------------------------------

CONFIGURATION

MPU_ADDR = 0x68
DEADBAND = 0.5

- DEADBAND filters small noise values
- Adjust if gyro is too sensitive

--------------------------------------------------

ERROR HANDLING

- Auto retries during init
- Handles I2C read errors gracefully
- Continues running even with intermittent failures

--------------------------------------------------

THREADING

- Runs in background thread
- Non-blocking
- Safe access using locks

--------------------------------------------------

SYSTEM FLOW

INIT → CALIBRATE → START → READ ANGLE → STOP

--------------------------------------------------

APPLICATION EXAMPLE

# turn robot 90 degrees
gyro_lib.reset()

while gyro_lib.get_angle() < 90:
    robot.left(10)

robot.stop()

--------------------------------------------------

Created by Spec-Tech

"powerfull than magic call it engineering"
