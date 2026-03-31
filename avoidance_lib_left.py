import time
import gyro_lib
import apriltag_lib


def avoid_and_realign(robot, target_id,
                      trigger_front_mm,
                      front_value,
                      turn_angle=45,
                      return_angle=-20,
                      turn_speed=10,
                      forward_speed=10,
                      forward_time=15,
                      align_speed=5,
                      align_left_thr=-20,
                      align_right_thr=20):
    """
    Fixed avoidance sequence:
    1. stop
    2. turn left until turn_angle
    3. move forward for forward_time
    4. turn right until return_angle
    5. re-align to tag
    6. return True if avoidance was executed

    Parameters:
        robot          : RobotSerial object
        target_id      : AprilTag/QR target id
        trigger_front_mm : threshold to trigger avoidance
        front_value    : current front lidar distance
    """

    if front_value >= trigger_front_mm:
        return False

    robot.stop()
    time.sleep(0.1)

    gyro_lib.reset()
    seq = 1

    while 1:
        ang_avoid = gyro_lib.get_angle()
        print("obstacle", ang_avoid)

        if seq == 1:
            if ang_avoid < turn_angle:
                robot.left(turn_speed)
            else:
                robot.stop()
                seq = 2

        elif seq == 2:
            robot.forward(forward_speed)
            time.sleep(forward_time)
            robot.stop()
            seq = 3

        elif seq == 3:
            if ang_avoid > return_angle:
                robot.right(turn_speed)
            else:
                robot.stop()

                # re-align to tag
                while 1:
                    center_a, offset_a = apriltag_lib.get_tag_offset(target_id)
                    print("realign:", center_a, offset_a)

                    if center_a:
                        if offset_a < align_left_thr:
                            robot.left(align_speed)
                            time.sleep(0.05)
                            robot.stop()

                        elif offset_a > align_right_thr:
                            robot.right(align_speed)
                            time.sleep(0.05)
                            robot.stop()

                        else:
                            robot.stop()
                            time.sleep(1)
                            print("done")
                            break
                    else:
                        robot.stop()
                        time.sleep(0.05)

                break

        time.sleep(0.05)
    
    return True