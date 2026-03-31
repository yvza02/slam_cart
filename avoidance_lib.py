AprilTag Vision Library (Python)
================================

A lightweight and fast AprilTag detection library built on OpenCV and pupil_apriltags, designed for robotics applications such as navigation, docking, and tracking.

--------------------------------------------------

FEATURES

- Camera control (start/stop/update)
- Fast AprilTag detection
- Pixel offset calculation (left/right alignment)
- Distance estimation using pose
- Full tag information (ID, position, distance)
- Real-time visualization with annotations
- Ready for robotics integration

--------------------------------------------------

CORE FUNCTIONS

is_tag_found(id)
    Check if a tag exists in frame

get_tag_offset(id)
    Get horizontal offset (negative = left, positive = right)

get_tag_pose_distance(id)
    Estimate distance in meters using pose

get_tag_info(id)
    Get full structured tag information

show_frame()
    Display camera feed with annotations

--------------------------------------------------

INSTALLATION

pip install opencv-python pupil-apriltags

--------------------------------------------------

HOW IT WORKS

1. Start camera
2. Capture frame
3. Convert to grayscale
4. Detect AprilTags
5. Extract:
   - Tag ID
   - Center position
   - Offset from center
   - Distance (pose estimation)

--------------------------------------------------

BASIC USAGE

import your_library_name as tag
tag.start_camera(0)

if tag.is_tag_found(1):
    print("Tag detected")

found, offset = tag.get_tag_offset(1)
if found:
    print("Offset:", offset)

--------------------------------------------------

CREDITS

Created by Spec-Tech

"powerfull than magic call it engineering"
