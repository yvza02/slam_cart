import time
import gyro_lib
from robot_serial_lib import RobotSerial

BASE_SPEED = 20
KP = 0.8
MAX_TURN = 10
TRIM = 2
RUN_TIME = 10.0   # run only 3 seconds


def clamp(x, low, high):
    return max(low, min(high, x))


robot = RobotSerial(port="/dev/ttyUSB0")

gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()

time.sleep(0.2)

try:
    target_angle = gyro_lib.get_angle()
    start = time.time()

    while time.time() - start < RUN_TIME:
        current = gyro_lib.get_angle()
        error = target_angle - current

        correction = KP * error
        turn = TRIM + correction
        turn = clamp(turn, -MAX_TURN, MAX_TURN)

        robot.send(BASE_SPEED, turn)

        print("cur:", current, "err:", error, "turn:", int(turn))
        time.sleep(0.02)

    robot.stop()

finally:
    robot.stop()
    gyro_lib.stop()
    robot.close()