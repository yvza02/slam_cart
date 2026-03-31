import cv2
from pupil_apriltags import Detector

# =========================
# Detector setup
# =========================
detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.8,
    refine_edges=1
)

# =========================
# Globals
# =========================
cap = None
last_frame = None
last_gray = None
last_simple_detections = []
last_pose_detections = []

# made by spectech
# "powerfull than magic call it engineering"


# =========================
# Camera control
# =========================
def start_camera(cam_index=0, w=1280, h=720):
    global cap

    stop_camera()

    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)

    if not cap.isOpened():
        cap = None
        raise RuntimeError("Camera not opened")


def stop_camera():
    global cap
    if cap is not None:
        cap.release()
        cap = None


def update_frame():
    global cap, last_frame, last_gray

    if cap is None:
        return 0, None

    ok, frame = cap.read()
    if not ok or frame is None:
        return 0, None

    last_frame = frame
    last_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return 1, frame


# =========================
# Detection
# =========================
def detect_tags_simple():
    global last_gray, last_simple_detections

    if last_gray is None:
        last_simple_detections = []
        return []

    last_simple_detections = detector.detect(last_gray)
    return last_simple_detections


def detect_tags_pose(tag_size_m=0.20, fx=510.0, fy=510.0):
    global last_frame, last_gray, last_pose_detections

    if last_frame is None or last_gray is None:
        last_pose_detections = []
        return []

    h, w = last_frame.shape[:2]
    camera_params = (float(fx), float(fy), w / 2.0, h / 2.0)

    last_pose_detections = detector.detect(
        last_gray,
        estimate_tag_pose=True,
        camera_params=camera_params,
        tag_size=float(tag_size_m)
    )
    return last_pose_detections


# =========================
# Internal helpers
# =========================
def _find_simple_detection(target_id):
    detections = detect_tags_simple()
    for d in detections:
        if d.tag_id == int(target_id):
            return d
    return None


def _find_pose_detection(target_id, tag_size_m=0.20, fx=510.0, fy=510.0):
    detections = detect_tags_pose(tag_size_m=tag_size_m, fx=fx, fy=fy)
    for d in detections:
        if d.tag_id == int(target_id):
            return d
    return None


# =========================
# 1) Is tag found?
# =========================
def is_tag_found(target_id):
    ok, _ = update_frame()
    if not ok:
        return 0

    d = _find_simple_detection(target_id)
    return 1 if d is not None else 0


# =========================
# 2) What is offset?
# offset_px = tag_center_x - frame_center_x
# negative = tag is left
# positive = tag is right
# =========================
def get_tag_offset(target_id):
    ok, frame = update_frame()
    if not ok:
        return 0, 0

    d = _find_simple_detection(target_id)
    if d is None:
        return 0, 0

    h, w = frame.shape[:2]
    center_x = w / 2.0
    tag_x = float(d.center[0])
    offset_px = tag_x - center_x

    return 1, offset_px


# =========================
# 3) What is pose distance?
# Uses pose estimation
# =========================
def get_tag_pose_distance(target_id, tag_size_m=0.20, fx=510.0, fy=510.0, cal_factor=1.0):
    ok, _ = update_frame()
    if not ok:
        return 0, 0

    d = _find_pose_detection(target_id, tag_size_m=tag_size_m, fx=fx, fy=fy)
    if d is None:
        return 0, 0

    try:
        tx, ty, tz = d.pose_t.flatten()
        distance_m = float(tz) * float(cal_factor)
        return 1, distance_m
    except Exception:
        return 0, 0


# =========================
# Full info helper
# =========================
def get_tag_info(target_id, tag_size_m=0.20, fx=510.0, fy=510.0, cal_factor=1.0):
    """
    Returns:
        None
        or
        {
            "id": int,
            "found": 1,
            "offset_px": float,
            "distance_m": float or 0.0,
            "cx": float,
            "cy": float
        }
    """
    ok, frame = update_frame()
    if not ok:
        return None

    d = _find_simple_detection(target_id)
    if d is None:
        return None

    h, w = frame.shape[:2]
    center_x = w / 2.0

    cx = float(d.center[0])
    cy = float(d.center[1])
    offset_px = cx - center_x

    distance_m = 0.0
    pd = _find_pose_detection(target_id, tag_size_m=tag_size_m, fx=fx, fy=fy)
    if pd is not None:
        try:
            tx, ty, tz = pd.pose_t.flatten()
            distance_m = float(tz) * float(cal_factor)
        except Exception:
            distance_m = 0.0

    return {
        "id": int(target_id),
        "found": 1,
        "offset_px": offset_px,
        "distance_m": distance_m,
        "cx": cx,
        "cy": cy
    }


# =========================
# Drawing / display
# =========================
def draw_simple_detections():
    global last_frame, last_simple_detections

    if last_frame is None:
        return None

    frame = last_frame.copy()
    h, w = frame.shape[:2]
    center_x = w // 2

    cv2.line(frame, (center_x, 0), (center_x, h), (255, 0, 0), 1)

    for d in last_simple_detections:
        corners = d.corners.astype(int)

        for i in range(4):
            p1 = tuple(corners[i])
            p2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, p1, p2, (0, 255, 0), 2)

        cx, cy = int(d.center[0]), int(d.center[1])
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        offset_px = cx - center_x
        text = f"ID:{d.tag_id} OFF:{offset_px}px"

        cv2.putText(
            frame,
            text,
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

    return frame


def show_frame(window_name="cam", annotated=True):
    global last_frame

    if last_frame is None:
        return

    if annotated:
        frame = draw_simple_detections()
        if frame is not None:
            cv2.imshow(window_name, frame)
    else:
        cv2.imshow(window_name, last_frame)