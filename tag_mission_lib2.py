Tag Mission Library (tag_mission_lib)
==========================================

A complete autonomous mission controller for AprilTag-based robot navigation.

This library integrates:
- AprilTag vision (target detection)
- LiDAR obstacle avoidance
- Gyroscope orientation control
- Motor control via RobotSerial

--------------------------------------------------

OVERVIEW

This module provides a full mission flow:

SEARCH → CENTER → APPROACH → AVOID → FINAL ALIGN → DOCK → TURN BACK

Designed for autonomous robotics systems requiring reliable target navigation.

--------------------------------------------------

MAIN FUNCTION

run_tag_mission(robot, target_id)

--------------------------------------------------

MISSION FLOW

1. SEARCH
   - Scan left and right using gyro
   - Detect target AprilTag

2. FIRST CENTERING
   - Align robot to tag center (pixel offset)

3. APPROACH (to ~0.9m)
   - Move toward target
   - Adjust heading continuously

4. OBSTACLE DETECTION
   - Uses LiDAR (FRONT, LEFT, RIGHT)
   - Trigger avoidance if blocked

5. AVOIDANCE
   - Calls avoidance_lib or avoidance_lib_left
   - Re-align after avoidance

6. FINAL CENTERING
   - Fine alignment before docking

7. FINAL APPROACH (to ~0.5m)
   - Slow forward movement
   - Ensures accurate stopping distance

8. TURN 180°
   - Rotate robot after docking

--------------------------------------------------

REQUIREMENTS

Before calling:

gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.start()

robot = RobotSerial("/dev/ttyUSB0")

--------------------------------------------------

USAGE

from tag_mission_lib import run_tag_mission

run_tag_mission(robot, target_id=1)

--------------------------------------------------

DEPENDENCIES

- gyro_lib
- apriltag_lib
- lidar_obstacle_lib
- avoidance_lib
- avoidance_lib_left
- RobotSerial

--------------------------------------------------

LIDAR INTEGRATION

Uses YDLidarObstacle:

- FRONT distance for collision detection
- LEFT / RIGHT for avoidance decision
- Uses filtering and hysteresis

--------------------------------------------------

AVOIDANCE LOGIC

- If FRONT < 500 mm → trigger
- If LEFT clear → avoid right path
- If RIGHT clear → avoid left path
- Calls appropriate avoidance routine

--------------------------------------------------

ALIGNMENT LOGIC

Uses AprilTag offset:

offset < -threshold → turn left  
offset > threshold  → turn right  
within range        → aligned  

--------------------------------------------------

DISTANCE CONTROL

Uses pose estimation:

- Stop at ~0.9m (approach phase)
- Stop at ~0.5m (final docking)

--------------------------------------------------

IMPORTANT NOTES

- Camera is started automatically (first run)
- Function is blocking (runs until mission completes)
- Designed for continuous loop integration
- Uses retries for detection stability

--------------------------------------------------

SYSTEM FLOW

SEARCH → ALIGN → MOVE → AVOID → ALIGN → DOCK → TURN

--------------------------------------------------

DESIGNED FOR

- Autonomous robots
- AprilTag docking systems
- Smart navigation robots
- AI vision robotics

--------------------------------------------------

Created by Spec-Tech

"powerfull than magic call it engineering"
