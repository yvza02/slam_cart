# apriltag_lib.py
import cv2
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1
)

cap = None
last_frame = None
last_gray = None
last_detections = []

# made by spectech
# "powerfull than magic call it engineering"

def start_camera(cam_index=0, w=1280, h=720):
    global cap
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    if not cap.isOpened():
        raise RuntimeError("Camera not opened")


def stop_camera():
    global cap
    if cap:
        cap.release()
        cap = None


def update_frame():
    global cap, last_frame, last_gray
    if cap is None:
        return 0, None

    ok, frame = cap.read()
    if not ok:
        return 0, None

    last_frame = frame
    last_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return 1, frame


def detect_tags_simple():
    global last_gray, last_detections

    if last_gray is None:
        return []

    last_detections = detector.detect(last_gray)
    return last_detections


def detect_tags_pose(tag_size_m=0.20, fx=510.0, fy=510.0):
    global last_frame, last_gray, last_detections

    if last_frame is None or last_gray is None:
        return []

    h, w = last_frame.shape[:2]
    camera_params = (float(fx), float(fy), w / 2.0, h / 2.0)

    last_detections = detector.detect(
        last_gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=float(tag_size_m)
    )
    return last_detections


def get_tag(target_id, tag_size_m=0.20, fx=510.0, fy=510.0, cal_factor=1.94):
    """
    Returns:
        None
        or
        {
            "id": int,
            "offset_px": float,
            "distance_m": float,
            "cx": float,
            "cy": float
        }
    """
    ok, frame = update_frame()
    if not ok:
        return None

    detections = detect_tags_pose(tag_size_m=tag_size_m, fx=fx, fy=fy)

    h, w = frame.shape[:2]
    center_x = w / 2.0

    for d in detections:
        if d.tag_id == int(target_id):
            tag_x = float(d.center[0])
            tag_y = float(d.center[1])
            offset_px = tag_x - center_x

            distance_m = 0.0
            try:
                tx, ty, tz = d.pose_t.flatten()
                distance_m = float(tz) * float(cal_factor)
            except Exception:
                distance_m = 0.0

            return {
                "id": int(d.tag_id),
                "offset_px": offset_px,
                "distance_m": distance_m,
                "cx": tag_x,
                "cy": tag_y
            }

    return None


def tag_detector(target_id):
    tag = get_tag(target_id)
    return 1 if tag is not None else 0


def tag_offset_measurement(target_id):
    tag = get_tag(target_id)
    if tag is None:
        return 0, 0
    return 1, tag["offset_px"]


def tag_measurement(target_id, tag_size_m=0.20, fx=510.0, fy=510.0):
    tag = get_tag(target_id, tag_size_m=tag_size_m, fx=fx, fy=fy)
    if tag is None:
        return 0, 0
    return 1, tag["distance_m"]


def show_frame(window_name="cam"):
    global last_frame
    if last_frame is not None:
        cv2.imshow(window_name, last_frame)