import cv2
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=1.0,
    quad_sigma=0.8,
    refine_edges=1
)

cap = cv2.VideoCapture(0)

while True:
    ok, frame = cap.read()
    if not ok:
        print("no frame")
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    dets = detector.detect(gray)

    print("detections:", len(dets))

    for d in dets:
        print("ID:", d.tag_id)
        pts = d.corners.astype(int)
        for i in range(4):
            p1 = tuple(pts[i])
            p2 = tuple(pts[(i + 1) % 4])
            cv2.line(frame, p1, p2, (0, 255, 0), 2)

        cx, cy = int(d.center[0]), int(d.center[1])
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"ID {d.tag_id}", (cx + 10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    cv2.imshow("AprilTag Test", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()