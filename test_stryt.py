import time
import gyro_lib
import apriltag_lib
from lidar_obstacle_lib import YDLidarObstacle
import math

lidar = YDLidarObstacle(port="/dev/ttyUSB1")
lidar.start()


def run_tag_mission(robot, target_id, start_camera_once=True):
    """
    Reusable mission flow.
    Required:
        robot
        target_id

    Assumes:
        - gyro_lib.init(), calibrate(), start() already done outside
          OR call setup_system() below
        - robot is already created from RobotSerial
    """

    dir = 1
    first_run = 1

    # =========================
    # SEARCH TARGET
    # =========================
    while 1:
        if first_run:
            apriltag_lib.start_camera(0, 1280, 720)
            first_run = 0

        found_target1 = apriltag_lib.is_tag_found(0)
        ang_target_look = gyro_lib.get_angle()
        print(found_target1)

        if found_target1 == 0:
            print("not found")

            if dir == 1:
                if ang_target_look > -20:
                    robot.right(10)
                    time.sleep(0.1)
                    robot.stop()
                else:
                    robot.stop()
                    dir = -1

            elif dir == -1:
                if ang_target_look < 20:
                    robot.left(10)
                    time.sleep(0.1)
                    robot.stop()
                else:
                    robot.stop()
                    dir = 0

            else:
                if ang_target_look > 0:
                    robot.right(10)
                    time.sleep(0.1)
                    robot.stop()
                print("done")

        else:
            print("found")
            break

    # =========================
    # FIRST CENTERING
    # =========================
    while 1:
        center1, c_offset = apriltag_lib.get_tag_offset(target_id)
        print(center1, c_offset)

        if center1:
            if c_offset < -6:
                robot.left(5)
                time.sleep(0.05)
                robot.stop()
                time.sleep(0.1)

            elif c_offset > -1:
                robot.right(5)
                time.sleep(0.05)
                robot.stop()
                time.sleep(0.1)

            else:
                robot.stop()
                break
        else:
            robot.stop()

    # =========================
    # APPROACH TO 0.9m
    # =========================
    while 1:
        dist_f, dist = apriltag_lib.get_tag_pose_distance(
            target_id,
            tag_size_m=0.20,
            fx=510.0,
            fy=510.0,
            cal_factor=1.94
        )
        res = lidar.poll()
        if res:
            left = res["LEFT"]
            front = res["FRONT"]
            right = res["RIGHT"]

            # optional: replace INF with large number
            if not math.isfinite(left):
                left = 9999
            if not math.isfinite(front):
                front = 9999
            if not math.isfinite(right):
                right = 9999

            # now you can use variables directly
            print(left, front, right)

        off_f, c_offset_h = apriltag_lib.get_tag_offset(target_id)

        print("approaching dist =", dist)
        print("found =", off_f, "offset =", c_offset_h)

        if not off_f:
            robot.stop()
            time.sleep(0.05)
            if front < 500:
                gyro_lib.reset()
                seq = 1
                
                while 1:
                    ang_avoid = gyro_lib.get_angle()
                
                    gyro_lib.reset()
                    seq = 1

                    while 1:
                        ang_avoid = gyro_lib.get_angle()
                        print("obstacle")
                        print(ang_avoid)

                        if seq == 1:
                            if ang_avoid < 45:
                                robot.left(10)
                            else:
                                robot.stop()
                                seq = 2

                        elif seq == 2:
                            robot.forward(10)
                            time.sleep(15)
                            robot.stop()
                            seq = 3

                        elif seq == 3:
                            if ang_avoid > -20:
                                robot.right(10)
                            else:
                                robot.forward(10)
                                time.sleep(5)
                                robot.stop()
                                print("done")
                                break

                        time.sleep(0.05)
                    
            continue

        if dist_f and dist <= 0.9:
            robot.stop()
            time.sleep(0.3)

            dist_f1, dist1 = apriltag_lib.get_tag_pose_distance(
                target_id,
                tag_size_m=0.20,
                fx=510.0,
                fy=510.0,
                cal_factor=1.94
            )

            if dist_f1 and dist1 <= 0.9:
                robot.stop()
                break
            else:
                continue

        if c_offset_h < -100:
            robot.send(10, -6)

        elif c_offset_h > 100:
            robot.send(10, 6)

        else:
            if dist_f and dist > 0.9:
                robot.send(15, 0)
            else:
                robot.stop()

        time.sleep(0.05)

    # =========================
    # FINAL CENTERING
    # =========================
    while 1:
        center2, c_offset2 = apriltag_lib.get_tag_offset(target_id)
        print("final centering ", center2, c_offset)

        if center2:
            if c_offset2 < -6:
                robot.left(5)
                time.sleep(0.05)
                robot.stop()
                time.sleep(0.1)

            elif c_offset2 > 6:
                robot.right(5)
                time.sleep(0.05)
                robot.stop()
                time.sleep(0.1)

            else:
                robot.stop()
                break
        else:
            robot.stop()

    # =========================
    # FINAL APPROACH TO 0.4m
    # =========================
    while 1:
        dist_f, dist = apriltag_lib.get_tag_pose_distance(
            target_id,
            tag_size_m=0.20,
            fx=510.0,
            fy=510.0,
            cal_factor=1.94
        )

        print("final dist = ", dist, "final found = ", dist_f)

        if dist_f:
            if dist > 0.4:
                robot.forward(10)
            else:
                robot.stop()
                time.sleep(1)

                dist_f1, dist1 = apriltag_lib.get_tag_pose_distance(
                    target_id,
                    tag_size_m=0.20,
                    fx=510.0,
                    fy=510.0,
                    cal_factor=1.94
                )

                if dist1 <= 0.4:
                    robot.stop()
                    gyro_lib.reset()
                    break
        else:
            time.sleep(0.1)
            robot.stop()

    # =========================
    # TURN 180
    # =========================
    while 1:
        ang_target_look = gyro_lib.get_angle()

        if ang_target_look > -180:
            robot.right(10)
            time.sleep(0.1)
            robot.stop()
        else:
            robot.stop()
            break

    print("done")
    robot.stop()
    apriltag_lib.stop_camera()

    return 1