from robot_serial_lib import RobotSerial
import gyro_lib
import tag_mission_lib
import time


gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()

robot = RobotSerial(port="/dev/ttyUSB0")

tag_mission_lib.run_tag_mission(robot, target_id=1)
gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()
time.sleep(10)
tag_mission_lib.run_tag_mission(robot, target_id=2)
gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()
time.sleep(10)
tag_mission_lib.run_tag_mission(robot, target_id=0)