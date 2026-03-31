LiDAR Obstacle Library (lidar_obstacle_lib)
=============================================

A high-performance YDLIDAR obstacle detection library designed for robotics navigation.

This library processes raw LiDAR data and converts it into usable sector-based distance information.

--------------------------------------------------

OVERVIEW

The library reads LiDAR packets and extracts distance data into 3 main sectors:

- FRONT
- LEFT
- RIGHT

It includes advanced features for stable robotics operation:
- Non-blocking serial reading
- Accumulation window filtering
- Hold-last-valid data
- Hysteresis (anti-jitter)
- Sector-based obstacle detection

--------------------------------------------------

FEATURES

- Real-time LiDAR parsing (AA55 protocol)
- Sector-based distance output
- Noise filtering using accumulation
- Stable readings using hold mechanism
- Hysteresis to prevent rapid switching
- Configurable sectors
- Non-blocking operation (poll-based)

--------------------------------------------------

CLASS

YDLidarObstacle()

--------------------------------------------------

INITIALIZATION PARAMETERS

port="/dev/ttyUSB0"
baud=128000
obstacle_mm=2000
release_mm=2200
heading_offset_deg=180.0
triangulation=True
apply_angle_correction=False
accumulate_ms=120
far_limit_mm=3000
hold_ms=300

--------------------------------------------------

HOW IT WORKS

1. Reads raw LiDAR packets
2. Extracts angle + distance points
3. Assigns points into sectors
4. Accumulates readings over time window
5. Applies filtering:
   - median filtering
   - hold-last-valid
   - hysteresis
6. Outputs stable sector distances

--------------------------------------------------

SECTORS

Default:

FRONT = (355° to 5°)
LEFT  = (265° to 275°)
RIGHT = (85° to 95°)

You can redefine sectors if needed.

--------------------------------------------------

USAGE

from lidar_obstacle_lib import YDLidarObstacle

lidar = YDLidarObstacle()
lidar.start()

while True:
    result = lidar.poll()

    if result:
        front = result["FRONT"]
        left  = result["LEFT"]
        right = result["RIGHT"]

        print(front, left, right)

--------------------------------------------------

OUTPUT FORMAT

{
    "LEFT": distance_mm,
    "FRONT": distance_mm,
    "RIGHT": distance_mm,
    "blocked": {
        "LEFT": True/False,
        "FRONT": True/False,
        "RIGHT": True/False
    }
}

--------------------------------------------------

HYSTERESIS LOGIC

- obstacle_mm → trigger ON
- release_mm  → trigger OFF

Prevents rapid switching (motor jitter)

--------------------------------------------------

HOLD MECHANISM

If a sector temporarily has no data:
- Keeps last valid value for hold_ms duration
- Prevents flickering (INF values)

--------------------------------------------------

IMPORTANT NOTES

- Call start() before poll()
- poll() is non-blocking → call in loop
- Returns None until accumulation window completes
- Designed for continuous real-time operation

--------------------------------------------------

SYSTEM FLOW

LiDAR → Packet → Sector → Filter → Stable Output

--------------------------------------------------

DESIGNED FOR

- Obstacle avoidance
- Autonomous navigation
- Robotics mapping
- Real-time control systems

--------------------------------------------------

Created by Spec-Tech

"powerfull than magic call it engineering"
