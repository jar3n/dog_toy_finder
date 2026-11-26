#!/usr/bin/env python3
"""Solve and save stereo intrinsics, rectification, and baseline.

Run this on the host, using the host vision venv's OpenCV. The images must be
left/right JPEG pairs captured from the same camera endpoints used by
stereo_camera_node.py. The script resizes each image to the launch resolution
before calibrating, so the printed values describe the images ROS processes.

The JSON output keeps both camera matrices (K1 and K2) and the solved relative
rotation. stereo_camera_node.py can load it through its ``calibration_file``
parameter, so the two eyes no longer have to share one K or identity R.
"""

import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

CALIBRATION_KIND = "dog_toy_finder_stereo_calibration"


def parse_args():
    """Parse command-line arguments."""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--left", default="/tmp/dtf_calib/left",
                   help="directory of left JPEG captures")
    p.add_argument("--right", default="/tmp/dtf_calib/right",
                   help="directory of right JPEG captures")
    p.add_argument("--pattern", default="8x11",
                   help="inner corners: COLSxROWS (default: 8x11)")
    p.add_argument("--square", type=float, default=0.020,
                   help="square size in metres (default: 0.020)")
    p.add_argument("--width", type=int, default=320,
                   help="processed image width in pixels")
    p.add_argument("--height", type=int, default=240,
                   help="processed image height in pixels")
    p.add_argument("--output",
                   default="software/ros2_ws/src/dog_toy_finder_vision/config/stereo_calibration.json",
                   help="path for the full stereo calibration JSON")
    return p.parse_args()


def inner_corner_pattern(pattern):
    """Return (cols, rows) from a COLSxROWS string."""
    try:
        cols, rows = (int(v) for v in pattern.lower().split("x"))
    except ValueError:
        raise ValueError("--pattern must look like 8x11 (cols x rows)")
    if cols < 2 or rows < 2:
        raise ValueError("--pattern needs at least 2 inner corners per axis")
    return cols, rows


def grey(path, width, height):
    """Load a JPEG, resize it, and return a grayscale image."""
    image = cv2.imread(path)
    if image is None:
        return None
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def matrix(value):
    """Convert a NumPy matrix to a plain nested list."""
    return [[float(item) for item in row] for row in np.asarray(value)]


def vector(value):
    """Convert a NumPy vector to a flat list of floats."""
    return [float(item) for item in np.asarray(value).reshape(-1)]


def main():
    """Calibrate the stereo pair and save the full geometry."""
    a = parse_args()
    try:
        cols, rows = inner_corner_pattern(a.pattern)
    except ValueError as error:
        sys.exit(str(error))

    object_template = np.zeros((cols * rows, 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * a.square)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

    object_points, left_points, right_points = [], [], []
    left_paths = sorted(glob.glob(os.path.join(a.left, "*.jpg")))
    right_paths = sorted(glob.glob(os.path.join(a.right, "*.jpg")))
    if len(left_paths) != len(right_paths) or not left_paths:
        sys.exit("capture the same number of left and right JPEGs first")

    for left_path, right_path in zip(left_paths, right_paths):
        gl = grey(left_path, a.width, a.height)
        gr = grey(right_path, a.width, a.height)
        if gl is None or gr is None:
            continue
        found_l, corners_l = cv2.findChessboardCorners(gl, (cols, rows), None)
        found_r, corners_r = cv2.findChessboardCorners(gr, (cols, rows), None)
        if not (found_l and found_r):
            print("skip", os.path.basename(left_path),
                  os.path.basename(right_path))
            continue
        corners_l = cv2.cornerSubPix(gl, corners_l, (5, 5), (-1, -1), criteria)
        corners_r = cv2.cornerSubPix(gr, corners_r, (5, 5), (-1, -1), criteria)
        object_points.append(object_template)
        left_points.append(corners_l)
        right_points.append(corners_r)

    if len(object_points) < 8:
        sys.exit("need at least 8 valid pairs; recapture with the board "
                 "fully visible")

    size = (a.width, a.height)
    guess_k = np.array([[500.0, 0.0, size[0] / 2.0],
                        [0.0, 500.0, size[1] / 2.0],
                        [0.0, 0.0, 1.0]], dtype=np.float64)
    zero_d = np.zeros((5, 1), dtype=np.float64)
    # The node publishes zero distortion, so keep that constraint while still
    # allowing K1 and K2 to differ. Do not pass CALIB_FIX_INTRINSIC: that would
    # discard the independent right-eye solution we need here.
    fixed_zero_distortion = (
        cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
        cv2.CALIB_FIX_TANGENT_DIST)

    rms, k1, d1, k2, d2, rotation, translation, _, _ = cv2.stereoCalibrate(
        object_points, left_points, right_points,
        guess_k.copy(), zero_d.copy(), guess_k.copy(), zero_d.copy(), size,
        criteria=criteria, flags=fixed_zero_distortion)

    # Rectification must be computed here, where OpenCV is available. The ROS
    # node only publishes R1/R2/P1/P2; image_proc builds the maps from them.
    r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(
        k1, d1, k2, d2, size, rotation, translation,
        flags=0, alpha=-1, newImageSize=size)

    # The effective baseline for rectified disparity is the translation norm,
    # not just its pre-rectification X component.
    baseline = float(np.linalg.norm(np.asarray(translation, dtype=float)))
    if baseline <= 0.0:
        sys.exit("calibration returned a non-positive baseline")

    calibration = {
        "kind": CALIBRATION_KIND,
        "width": a.width,
        "height": a.height,
        "square_m": a.square,
        "pattern": "%dx%d" % (cols, rows),
        "valid_pairs": len(object_points),
        "reprojection_rms_px": float(rms),
        "baseline_m": baseline,
        "rotation": matrix(rotation),
        "translation": vector(translation),
        "left": {
            "k": matrix(k1),
            "d": vector(d1),
            "r": matrix(r1),
            "p": matrix(p1),
            "valid_roi": vector(roi1),
        },
        "right": {
            "k": matrix(k2),
            "d": vector(d2),
            "r": matrix(r2),
            "p": matrix(p2),
            "valid_roi": vector(roi2),
        },
        "q": matrix(q),
        "recommended_config": {
            "width": a.width,
            "height": a.height,
            "fx": float(p1[0, 0]),
            "fy": float(p1[1, 1]),
            "cx": float(p1[0, 2]),
            "cy": float(p1[1, 2]),
            "baseline_m": baseline,
        },
    }

    output = os.path.abspath(a.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w") as handle:
        json.dump(calibration, handle, indent=2, sort_keys=True)
        handle.write("\n")

    config = calibration["recommended_config"]
    print("valid pairs: %d, reprojection RMS: %.2f px" % (
        len(object_points), rms))
    print("K1 fx/fy/cx/cy: %.2f %.2f %.2f %.2f" % (
        k1[0, 0], k1[1, 1], k1[0, 2], k1[1, 2]))
    print("K2 fx/fy/cx/cy: %.2f %.2f %.2f %.2f" % (
        k2[0, 0], k2[1, 1], k2[0, 2], k2[1, 2]))
    print("rectified P1 fx/fy/cx/cy: %.2f %.2f %.2f %.2f" % (
        p1[0, 0], p1[1, 1], p1[0, 2], p1[1, 2]))
    print("rectified P2 fx/fy/cx/cy: %.2f %.2f %.2f %.2f" % (
        p2[0, 0], p2[1, 1], p2[0, 2], p2[1, 2]))
    print("rotation: %s" % " ".join("%.4f" % v for v in rotation.reshape(-1)))
    print("translation: %s" % " ".join("%.4f" % v for v in translation))
    print("--- copy into vision_detectors.yaml ---")
    print("fx: %.2f" % config["fx"])
    print("fy: %.2f" % config["fy"])
    print("cx: %.2f" % config["cx"])
    print("cy: %.2f" % config["cy"])
    print("baseline_m: %.4f" % config["baseline_m"])
    print("--- full calibration with K1/K2/R/R1/R2/P1/P2 ---")
    print(output)


if __name__ == "__main__":
    main()
