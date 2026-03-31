import time
import gyro_lib


def clamp(x, low, high):
    if x < low:
        return low
    if x > high:
        return high
    return x


class StraightController:
    def __init__(self, robot, kp=3.0, max_turn=30):
        self.robot = robot
        self.kp = kp
        self.max_turn = max_turn
        self.target_angle = 0.0
        self.started = False

    def start_gyro(self):
        gyro_lib.init()
        gyro_lib.calibrate()
        gyro_lib.reset()
        gyro_lib.start()
        time.sleep(0.2)
        self.target_angle = gyro_lib.get_angle()
        self.started = True
        print("Straight target angle =", self.target_angle)

    def reset_target(self):
        self.target_angle = gyro_lib.get_angle()
        print("New target angle =", self.target_angle)

    def forward(self, speed):
        if not self.started:
            raise RuntimeError("Call start_gyro() first")

        current_angle = gyro_lib.get_angle()
        error = self.target_angle - current_angle

        turn = self.kp * error
        turn = clamp(turn, -self.max_turn, self.max_turn)

        self.robot.send(speed, turn)

        return {
            "target": self.target_angle,
            "current": current_angle,
            "error": error,
            "turn": int(turn),
            "speed": int(speed)
        }

    def stop(self):
        self.robot.stop()

    def close(self):
        try:
            self.robot.stop()
        except Exception:
            pass
        gyro_lib.stop()