import time
import gyro_lib
import apriltag_lib
from lidar_obstacle_lib import YDLidarObstacle
import math
import avoidance_lib
import avoidance_lib_left

lidar = YDLidarObstacle(port="/dev/ttyUSB0")
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
    finding_count = 0
    while 1:
        if first_run:
            apriltag_lib.start_camera(0, 1280, 720)
            first_run = 0

        found_target1 = apriltag_lib.is_tag_found(target_id)
        ang_target_look = gyro_lib.get_angle()
        #print(found_target1)

        if found_target1 == 0:
            #print("not found")

            if dir == 1:
                if ang_target_look > -90:
                    print("moving right")
                    robot.right(5)
                    time.sleep(0.1)
                    robot.stop()
                else:
                    robot.stop()
                    dir = -1

            elif dir == -1:
                if ang_target_look < 90:
                    print("moving left")
                    robot.left(5)
                    time.sleep(0.1)
                    robot.stop()
                else:
                    robot.stop()
                    # 5x scanning
                    print("count finding", finding_count)
                    if finding_count >= 5:
                        dir = 0
                        print("==============can't find apriltag================")
                    else:
                        finding_count = finding_count + 1
                        dir = 1

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

            elif c_offset > 6:
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
    front_close_count = 0
    while 1:
        dist_f, dist = apriltag_lib.get_tag_pose_distance(
            target_id,
            tag_size_m=0.20,
            fx=510.0,
            fy=510.0,
            cal_factor=1.94
        )

        res = lidar.poll()
        if not res:
            left = 9999
            front = 9999
            right = 9999
            print("lidar poll returned None")
        else:
            left = res["LEFT"]
            front = res["FRONT"]
            right = res["RIGHT"]

            if not math.isfinite(left):
                left = 9999
            if not math.isfinite(front):
                front = 9999
            if not math.isfinite(right):
                right = 9999

        print(left, front, right)
        print("counter",front_close_count)

        off_f, c_offset_h = apriltag_lib.get_tag_offset(target_id)

        #print("approaching dist =", dist)
        #print("found =", off_f, "offset =", c_offset_h)

        if front < 500:
            front_close_count += 1
        else:
            front_close_count = 0

        did_avoid = False
        did_avoid_left = False

        if front_close_count >= 3:
            time.sleep(5)
            in_counter = 0
            while 1:
                res1 = lidar.poll()
                if not res1:
                    front_in = 9999
                    left_in = 9999
                    right_in = 9999
                else:
                    #front_in = res1["FRONT"]
                    left_in = res1["LEFT"]
                    front_in = res1["FRONT"]
                    right_in = res1["RIGHT"]
                    if not math.isfinite(front_in):
                        front_in = 9999
                    if not math.isfinite(left_in):
                        left_in = 9999
                    if not math.isfinite(right_in):
                        right_in = 9999
                        
                if front_in <= 500:
                    in_counter = in_counter + 1
                else:
                    did_avoid = False
                    break

                if in_counter >= 1:
                    print("left",left_in ,"right",right_in)
                    time.sleep(5)
                    
                    if left_in >= 1100:
                        print("right in====================")
                        did_avoid = avoidance_lib.avoid_and_realign(
                            robot=robot,
                            target_id=target_id,
                            trigger_front_mm=500,
                            front_value=front
                        )
                        break
                    elif right_in >= 1100:
                        print("left in++====================")
                        
                        did_avoid_left = avoidance_lib_left.avoid_and_realign_left(
                            robot=robot,
                            target_id=target_id,
                            trigger_front_mm=500,
                            front_value=front
                        )
                        break
                        
                print(front_in)
                time.sleep(0.1)
            front_close_count = 0

        if did_avoid or did_avoid_left:
            continue




        if not off_f:
            robot.stop()
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

        if c_offset_h < -80:
            robot.send(10, -6)

        elif c_offset_h > 80:
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
    swiping_dir = 1
    while 1:
        dist_f, dist = apriltag_lib.get_tag_pose_distance(
            target_id,
            tag_size_m=0.20,
            fx=510.0,
            fy=510.0,
            cal_factor=1.94
        )

        #print("final dist = ", dist, "final found = ", dist_f)

        if dist_f:
            if dist > 0.5:
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

                if dist1 <= 0.5:
                    robot.stop()
                    gyro_lib.reset()
                    break
        else:
            if swiping_dir == 1:  
               robot.left(5)
               swiping_dir = 0
               print("align left")
               
            else:
                robot.right(5)
                swiping_dir = 1
                print("align right")
            time.sleep(0.2)
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