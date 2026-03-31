Avoidance Library (avoidance_lib)
====================================

A deterministic obstacle avoidance and re-alignment module designed for robotics navigation using LiDAR, gyroscope, and AprilTag vision.

--------------------------------------------------

OVERVIEW

This module performs a fixed sequence of actions when an obstacle is detected:

1. STOP
2. TURN LEFT (gyro-based angle)
3. MOVE FORWARD (time-based)
4. TURN RIGHT (return angle)
5. REALIGN using AprilTag
6. RETURN control to main program

--------------------------------------------------

FUNCTION

avoid_and_realign(robot, target_id, trigger_front_mm, front_value, ...)

--------------------------------------------------

TRIGGER CONDITION

The routine executes ONLY when:

front_value < trigger_front_mm

Otherwise:
- No action is taken
- Function returns False

--------------------------------------------------

PARAMETERS

robot
    Robot interface (must support stop, left, right, forward)

target_id
    AprilTag ID used for re-alignment

trigger_front_mm
    Distance threshold to trigger avoidance

front_value
    Current front LiDAR distance

--------------------------------------------------

OPTIONAL PARAMETERS

turn_angle = 45
return_angle = -20
turn_speed = 10
forward_speed = 10
forward_time = 15
align_speed = 5
align_left_thr = -20
align_right_thr = 20

--------------------------------------------------

REALIGN LOGIC

Uses AprilTag offset:

offset < left threshold  → turn left  
offset > right threshold → turn right  
within threshold         → stop (aligned)

If tag is not detected:
- robot pauses and retries

--------------------------------------------------

RETURN VALUE

True  → avoidance executed  
False → no obstacle detected  

--------------------------------------------------

USAGE EXAMPLE

front = lidar["FRONT"]

did_avoid = avoid_and_realign(
    robot=robot,
    target_id=1,
    trigger_front_mm=500,
    front_value=front
)

if did_avoid:
    print("Avoidance complete")

--------------------------------------------------

REQUIREMENTS

- gyro_lib must be initialized and running
- apriltag_lib camera must be started
- robot must support continuous command interface

--------------------------------------------------

SYSTEM FLOW

NORMAL → OBSTACLE → AVOID → REALIGN → RESUME

--------------------------------------------------

DESIGN NOTES

- Blocking function (runs until finished)
- Deterministic behavior
- Designed for simple integration into mission loops

--------------------------------------------------

Created by Spec-Tech

"powerfull than magic call it engineering"
