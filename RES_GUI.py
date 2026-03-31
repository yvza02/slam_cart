import time
import gyro_lib

gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()

try:
    while True:
        ang = gyro_lib.get_angle()
        print(ang)
        time.sleep(0.1)

except KeyboardInterrupt:
    gyro_lib.stop()