import apriltag_lib
import cv2

apriltag_lib.start_camera(0, 1280, 720)

while True:
    ok, dist = apriltag_lib.get_tag_pose_distance(
        0,
        tag_size_m=0.20,
        fx=510.0,
        fy=510.0,
        cal_factor=1.94
    )

    if ok:
        print("distance_m =", dist)
    else:
        print("pose distance not available")

    apriltag_lib.show_frame("cam")

    if cv2.waitKey(1) & 0xFF == 27:
        break

apriltag_lib.stop_camera()
cv2.destroyAllWindows()