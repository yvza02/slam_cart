# slam_cart

Python code and utilities for a small “slam cart” / mobile robot that combines:

- **Motor control over serial** (ESP32/MCU receives `M,<forward>,<turn>` commands)
- **AprilTag vision** (OpenCV + `pupil_apriltags`) for target detection/centering/docking
- **Gyro heading tracking** (MPU6050 over I2C) for angle-based turning
- **LiDAR sector obstacle sensing** (YDLIDAR AA55 packet stream) for FRONT/LEFT/RIGHT ranges

This repo currently contains a mix of **working scripts**, **embedded “library code” stored in `*_README.txt` files**, and a few **misnamed/placeholder modules**. This README is written to be honest about what is runnable *as-is* and what needs renaming/cleanup.

---

## Quick start (sanity checks)

These are the fastest ways to verify your camera and dependencies.

### 1) AprilTag detection (camera + OpenCV)

```bash
python vision_try.py
```

- Press `Esc` to exit.
- Requires: `opencv-python`, `pupil-apriltags`.

### 2) AprilTag distance demo (uses `apriltag_lib` API)

```bash
python tag_mission_lib.py
```

This script calls `apriltag_lib.get_tag_pose_distance()` and shows frames via `apriltag_lib.show_frame()`.

Note: in the current repo state, `apriltag_lib.py` does **not** contain the expected Python source (see “Repo gotchas”). Use `apriltag_lib_old.py` as the real implementation.

---

## Hardware assumptions

The code strongly suggests a Raspberry Pi / Linux robot build:

- **Serial motor controller** (ESP32/Arduino/MCU) on `/dev/ttyUSB0` (or `/dev/ttyUSB1` in some scripts)
- **MPU6050** gyro on I2C bus (`smbus`), address `0x68`
- **USB camera** accessed as index `0`
- **YDLIDAR** device outputting AA55 packets at `128000` baud

If you run on Windows, you’ll likely need to change:

- Serial port from `/dev/ttyUSB0` → `COM3` (or similar)
- I2C gyro code won’t work unless you have an I2C adapter + compatible library

---

## Software requirements

- Python 3.8+ recommended
- Packages (typical):

```bash
pip install pyserial opencv-python pupil-apriltags
```

On Raspberry Pi for I2C gyro support you’ll also typically need:

- `smbus` (often via OS packages: `python3-smbus` / `i2c-tools`)
- I2C enabled in the OS (`raspi-config`)

---

## What to run (scripts)

### Manual driving GUI

File: `mpu.py`

```bash
python mpu.py
```

- Tkinter window for keyboard control (`WASD`, `Q/E` speed, `Space` stop)
- Uses `RobotSerial`

Important: `mpu.py` expects diagonal helper methods like `forward_left()` / `forward_right()` on `RobotSerial`. The reference `RobotSerial` implementation in `robot_serial_lib_README.txt` does **not** currently define those helpers, so you may need to add them (see “Repo gotchas”).

### AprilTag detector demo

File: `vision_try.py`

```bash
python vision_try.py
```

Pure OpenCV + `pupil_apriltags` demo that prints detections and draws tag outlines.

### Gyro angle print

File: `RES_GUI.py`

```bash
python RES_GUI.py
```

Prints `gyro_lib.get_angle()` in a loop.

Note: `gyro_lib.py` in this repo is not the real gyro implementation; see `gyro_lib_old.py`.

### AprilTag “approach/dock” routine

File: `manual.py`

```bash
python manual.py
```

Implements a multi-stage routine:

1. Scan left/right until the target AprilTag is found
2. Center on the tag using pixel offset
3. Approach to ~0.9 m using pose distance
4. Re-center, then approach to ~0.4–0.5 m
5. Turn ~180° using gyro angle

This script depends on working `RobotSerial`, `gyro_lib`, and `apriltag_lib` modules.

### Tag mission runner (multiple tag IDs)

File: `robot_serial_lib.py` (currently a *script*, despite the name)

This file creates a robot instance and runs a mission for multiple IDs via `tag_mission_lib.run_tag_mission(...)`.

---

## “Library” code locations

Several of the core libraries are currently stored in `*_README.txt` files (they contain real Python code):

- `robot_serial_lib_README.txt` — contains a `RobotSerial` implementation using `pyserial` + a background sender thread
- `lidar_obstacle_lib_README.txt` — contains `YDLidarObstacle` which parses AA55 LiDAR packets into sectors + hysteresis
- `tag_mission_lib_README.txt` — contains a larger `run_tag_mission(...)` flow (vision + LiDAR + avoidance)

There are also “old” versions that look like the real modules:

- `apriltag_lib_old.py` — real AprilTag vision module (camera control + detection + pose distance)
- `gyro_lib_old.py` — real MPU6050 gyro module (I2C + background integration thread)

---

## Repo gotchas (current state)

If you run into `ImportError` / `AttributeError`, it’s likely due to these issues:

1. `main.py` is currently **plain-text documentation**, not an entrypoint.
2. `apriltag_lib.py` is **not readable Python source** in this repo snapshot.
	 - Use `apriltag_lib_old.py` as the actual implementation.
3. `gyro_lib.py` and `lidar_check.py` contain **documentation text**, not the expected Python code.
	 - Use `gyro_lib_old.py` for gyro.
4. `robot_serial_lib.py` is currently a **mission runner script**, not the `RobotSerial` class.
	 - The `RobotSerial` class implementation is in `robot_serial_lib_README.txt`.
5. `lidar_obstacle_lib.py` is currently a **LiDAR test script**, while `YDLidarObstacle` lives in `lidar_obstacle_lib_README.txt`.
6. `tag_mission_lib.py` is currently a **camera distance demo**, while `run_tag_mission(...)` lives in `tag_mission_lib_README.txt` / `test_stryt.py`.

## Configuration points (what you’ll most likely edit)

- **Serial port**: `/dev/ttyUSB0` vs `/dev/ttyUSB1` vs Windows `COMx`
	- Appears in: `mpu.py`, `manual.py`, `robot_serial_lib_README.txt`, `tag_mission_lib_README.txt`
- **Camera index**: `0` (USB camera)
	- Appears in: `vision_try.py`, `apriltag_lib_old.py`, `manual.py`, `tag_mission_lib_README.txt`
- **AprilTag params**:
	- `tag_size_m` (defaults used: `0.20`)
	- `fx` / `fy` and `cal_factor` used for distance estimation
- **LiDAR params** (`YDLidarObstacle`):
	- `baud=128000`
	- sector angles (FRONT/LEFT/RIGHT)
	- `obstacle_mm` / `release_mm` hysteresis thresholds

---

## Troubleshooting

- **Camera opens but no tags detected**
	- Confirm you’re using the right tag family (`tag36h11` is used in `vision_try.py`).
	- Ensure good lighting and enough tag size in frame.

- **`ModuleNotFoundError` / missing functions**
	- See “Repo gotchas” and use the `*_old.py` / `*_README.txt` code as source of truth.

- **Serial errors (`Permission denied` / port not found)**
	- Linux: check device path (`/dev/ttyUSB0`), group permissions, and that nothing else is using the port.
	- Windows: switch to `COMx` and ensure drivers are installed.

- **MPU6050 / I2C errors**
	- Ensure I2C is enabled and wiring is correct.
	- Address `0x68` is assumed.

---

## Safety

These scripts can command motors immediately.

- Test with wheels off the ground first.
- Keep an emergency stop ready (power switch / unplug / kill process).
- Start with low speeds and short timeouts.
