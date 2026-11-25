#!/usr/bin/env python3
"""Solve stereo intrinsics and baseline from checkerboard captures.

Run this on the host, using the host vision venv's OpenCV. The images must be
left/right JPEG pairs captured from the same camera endpoints used by
stereo_camera_node.py. The script resizes each image to the launch resolution
before calibrating, so the printed values describe the images ROS processes.
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np


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


def main():
    """Calibrate the stereo pair and print the geometry values."""
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
    # The node publishes zero distortion and one shared K for both eyes.
    fixed_zero_distortion = (
        cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
        cv2.CALIB_FIX_TANGENT_DIST)

    rms, k1, _d1, _k2, _d2, _rotation, translation, _, _ = cv2.stereoCalibrate(
        object_points, left_points, right_points,
        guess_k.copy(), zero_d.copy(), guess_k.copy(), zero_d.copy(), size,
        criteria=criteria, flags=fixed_zero_distortion)

    baseline = abs(float(translation[0, 0]))
    print("valid pairs: %d, reprojection RMS: %.2f px" % (
        len(object_points), rms))
    print("fx: %.2f" % k1[0, 0])
    print("fy: %.2f" % k1[1, 1])
    print("cx: %.2f" % k1[0, 2])
    print("cy: %.2f" % k1[1, 2])
    print("baseline_m: %.4f" % baseline)


if __name__ == "__main__":
    main()
