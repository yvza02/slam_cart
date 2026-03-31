Robot Serial Library (robot_serial_lib)
==========================================

A continuous command-based motor control library using serial communication.

Designed for robotics systems where a Raspberry Pi (or PC) sends motion commands
to a microcontroller (e.g., Arduino, ESP32).

--------------------------------------------------

OVERVIEW

This library sends motor commands in the format:

M,<forward>,<turn>

Example:
M,20,0      → forward
M,0,10      → turn right
M,0,0       → stop

Commands are sent continuously at ~20Hz to ensure smooth motor control.

--------------------------------------------------

FEATURES

- Continuous command streaming (20Hz)
- Thread-based non-blocking control
- Simple differential drive interface
- Clean API (forward, backward, left, right, stop)
- Thread-safe command updates

--------------------------------------------------

CLASS

RobotSerial()

--------------------------------------------------

INITIALIZATION

robot = RobotSerial(
    port="/dev/ttyUSB0",
    baud=115200,
    timeout=1
)

--------------------------------------------------

HOW IT WORKS

1. Opens serial connection
2. Starts background thread
3. Continuously sends last command:
   M,<forward>,<turn>
4. Robot keeps moving while commands are being sent

--------------------------------------------------

CORE FUNCTION

send(forward, turn)

- forward → forward/backward speed
- turn    → turning value

--------------------------------------------------

HIGH-LEVEL FUNCTIONS

forward(speed)
backward(speed)
left(speed)
right(speed)
stop()

--------------------------------------------------

USAGE EXAMPLE

from robot_serial_lib import RobotSerial

robot = RobotSerial("/dev/ttyUSB0")

robot.forward(20)
time.sleep(2)

robot.left(10)
time.sleep(1)

robot.stop()

robot.close()

--------------------------------------------------

IMPORTANT NOTES

- Commands must be sent continuously → handled automatically
- If sending stops → robot should stop (failsafe)
- Ensure correct serial port
- Compatible with Arduino / ESP32 motor controllers

--------------------------------------------------

COMMAND FORMAT

"M,<forward>,<turn>\n"

Example values:
forward = 20 → forward motion
forward = -20 → backward motion
turn = 10 → right turn
turn = -10 → left turn

--------------------------------------------------

THREADING

- Runs background thread
- Sends command every 50ms (~20Hz)
- Thread-safe using lock

--------------------------------------------------

SYSTEM FLOW

APP → RobotSerial → Serial → Microcontroller → Motor Driver

--------------------------------------------------

DESIGNED FOR

- Differential drive robots
- Remote motor control
- Raspberry Pi robotics systems
- Serial-based motor drivers

--------------------------------------------------

Created by Spec-Tech

"powerfull than magic call it engineering"
