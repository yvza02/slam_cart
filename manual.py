from robot_serial_lib import RobotSerial
import time
import gyro_lib
import cv2
import apriltag_lib
gyro_lib.init()
gyro_lib.calibrate()
gyro_lib.reset()
gyro_lib.start()

robot = RobotSerial(port="/dev/ttyUSB0")
apriltag_lib.start_camera(0, 1280, 720)


target_id = 0
dir = 1
first_run = 1
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
        
   
while 1:
    center1,c_offset = apriltag_lib.get_tag_offset(target_id)
    print(center1,c_offset)
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
while 1:
    dist_f, dist = apriltag_lib.get_tag_pose_distance(
        target_id,
        tag_size_m=0.20,
        fx=510.0,
        fy=510.0,
        cal_factor=1.94
    )

    off_f, c_offset_h = apriltag_lib.get_tag_offset(target_id)

    print("approaching dist =", dist)
    print("found =", off_f, "offset =", c_offset_h)

    if not off_f:
        robot.stop()
        time.sleep(0.05)
        continue

    if dist_f and dist <= 0.9:
        robot.stop()
        time.sleep(0.3)

        # confirm stop distance
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
        #robot.left(5)

    elif c_offset_h > 100:
        robot.send(10, 6)
        #robot.right(5)

    else:
        if dist_f and dist > 0.9:
            robot.send(15, 0)
            #robot.forward(15)
        else:
            robot.stop()

    time.sleep(0.05)
while 1:
    center2,c_offset2 = apriltag_lib.get_tag_offset(target_id)
    print("final centering ",center2,c_offset)
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
    

while 1:
    dist_f,dist = apriltag_lib.get_tag_pose_distance(target_id,
        tag_size_m=0.20,
        fx=510.0,
        fy=510.0,
        cal_factor=1.94)
    print("final dist = ",dist, "final found = ",dist_f)
    if dist_f:
        if dist > 0.4:
            robot.forward(10)
        else:
            robot.stop()
            time.sleep(1)
            dist_f1,dist1 = apriltag_lib.get_tag_pose_distance(target_id,
                tag_size_m=0.20,
                fx=510.0,
                fy=510.0,
                cal_factor=1.94)
            if dist1 <= 0.4:
                 robot.stop()
                 gyro_lib.reset()
                 break
    else:
        #robot.backward(10)
        time.sleep(0.1)
        robot.stop()
    
    
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
first_run = 0
apriltag_lib.stop_camera()


