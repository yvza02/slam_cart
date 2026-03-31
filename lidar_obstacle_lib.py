from lidar_obstacle_lib import YDLidarObstacle
import time
import math

lidar = YDLidarObstacle(
    port="/dev/ttyUSB0",   # change if needed
    baud=128000,
    obstacle_mm=2000,
    release_mm=2200,
    heading_offset_deg=180.0,
    triangulation=True,
    apply_angle_correction=False,
    sample_mode="NO_INTENSITY",
    accumulate_ms=120,
    far_limit_mm=3000,
    hold_ms=300
)

try:
    lidar.start()
    print("LiDAR started")

    while True:
        res = lidar.poll()
        if res is not None:
            left = res["LEFT"]
            front = res["FRONT"]
            right = res["RIGHT"]
            b = res["blocked"]

            def fmt(v):
                return "INF" if not math.isfinite(v) else f"{v:.0f}"

            print(
                f"LEFT:{fmt(left)}  FRONT:{fmt(front)}  RIGHT:{fmt(right)}   "
                f"| blocked L:{b['LEFT']} F:{b['FRONT']} R:{b['RIGHT']}"
            )

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    lidar.stop()
    print("LiDAR stopped")