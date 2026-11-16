import cv2
import numpy as np

# AprilTag 36h11 detection and pose with OpenCV's aruco module, no extra build.
# The pose comes from the four corners with solvePnP (IPPE_SQUARE), using the
# calibrated K and distortion, so the raw (distorted) image is fine.

EDGE_PX = 3.0  # tags this close to the image border are skipped


def make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    # OpenCV 4.6 on the rover has the old API, 4.7+ the new one
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    # With a narrow white paper margin, the default (0.05) makes OpenCV 4.6 drop the
    # tag as too close to the paper edge. A quad just inside the black border can
    # then come out as a second, smaller copy of the same id, so detect() keeps
    # the largest per id.
    params.minMarkerDistanceRate = 0.01
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        return detector.detectMarkers
    return lambda gray: cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def tag_points(size):
    h = size / 2.0
    # aruco corner order: top-left, top-right, bottom-right, bottom-left
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def detect(detect_fn, gray, size, K, D):
    """Return a list of (id, corners 4x2, forward, left) in meters, camera frame."""
    corners, ids, _ = detect_fn(gray)
    out = []
    if ids is None:
        return out
    h, w = gray.shape[:2]
    obj = tag_points(size)
    largest = {}
    for c, i in zip(corners, ids.flatten()):
        c = c.reshape(4, 2).astype(np.float64)
        area = cv2.contourArea(c.astype(np.float32))
        if int(i) not in largest or area > largest[int(i)][0]:
            largest[int(i)] = (area, c)
    for i, (_, c) in largest.items():
        if (c[:, 0].min() < EDGE_PX or c[:, 1].min() < EDGE_PX or
                c[:, 0].max() > w - 1 - EDGE_PX or c[:, 1].max() > h - 1 - EDGE_PX):
            continue
        ok, _, tvec = cv2.solvePnP(obj, c, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        x, _, z = tvec.flatten()  # x right, y down, z forward
        if z <= 0:
            continue
        out.append((int(i), c, float(z), float(-x)))
    return out
