"""Deterministic rectified-stereo toy detector.

This is the classical path. It uses no learned model:

1. threshold the target colour in HSV (from ``vision_detectors.yaml``);
2. clean the mask and find connected components;
3. reject candidates by area, shape, fill ratio, and edge strength;
4. match small greyscale patches between the rectified left/right images;
5. convert the matched disparity to metres with the calibrated baseline/fx,
   and cross-check it against the toy's measured real-world width.

Only NumPy is required. OpenCV is deliberately avoided because the container's
ROS/Pillow/NumPy stack is already sufficient and avoids the numpy 1.x/2.x ABI
problem of the installed cv2 wheel.
"""

import math

import numpy as np

from . import vision_config


def message_to_rgb(msg):
    """Convert a sensor_msgs/Image rgb8/bgr8 message to a NumPy array."""
    expected = msg.width * msg.height * 3
    if len(msg.data) != expected:
        raise ValueError(
            "image has %d bytes, expected %d for %dx%d rgb8"
            % (len(msg.data), expected, msg.width, msg.height))
    array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, 3).copy()
    if msg.encoding == "bgr8":
        array = array[:, :, ::-1]
    elif msg.encoding not in ("rgb8", "rgb"):
        raise ValueError("classical detector needs rgb8/bgr8 images, got %s"
                         % msg.encoding)
    return array


def rgb_to_gray(rgb):
    """Rec. 601 luma, matching the greyscale image_proc produces."""
    image = np.asarray(rgb, dtype=np.float32)
    return (0.299 * image[:, :, 0] + 0.587 * image[:, :, 1]
            + 0.114 * image[:, :, 2]).astype(np.uint8)


def rgb_to_hsv(rgb):
    """Vectorised RGB -> HSV with H in 0..179 and S/V in 0..255."""
    image = np.asarray(rgb, dtype=np.float32) / 255.0
    red, green, blue = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    maximum = np.max(image, axis=2)
    minimum = np.min(image, axis=2)
    delta = maximum - minimum

    hue = np.zeros(maximum.shape, dtype=np.float32)
    active = delta > 0.0
    red_max = active & (maximum == red)
    green_max = active & (maximum == green) & ~red_max
    blue_max = active & ~red_max & ~green_max
    hue[red_max] = ((green[red_max] - blue[red_max])
                    / delta[red_max]) % 6.0
    hue[green_max] = ((blue[green_max] - red[green_max])
                      / delta[green_max]) + 2.0
    hue[blue_max] = ((red[blue_max] - green[blue_max])
                     / delta[blue_max]) + 4.0
    hue = (hue * 30.0).astype(np.uint16)

    saturation = np.where(maximum > 0.0, delta / np.maximum(maximum, 1e-6),
                          0.0) * 255.0
    return hue, saturation.astype(np.uint8), (maximum * 255.0).astype(np.uint8)


def color_mask(rgb, ranges):
    """Combine all configured HSV ranges into one boolean mask."""
    height, width = np.asarray(rgb).shape[:2]
    mask = np.zeros((height, width), dtype=bool)
    if not ranges:
        return mask
    hue, saturation, value = rgb_to_hsv(rgb)
    for lower, upper in ranges:
        region = ((hue >= lower[0]) & (hue <= upper[0])
                  & (saturation >= lower[1]) & (saturation <= upper[1])
                  & (value >= lower[2]) & (value <= upper[2]))
        mask |= region
    return mask


def _shift(mask, dy, dx):
    """Shift a boolean mask, filling the vacated edge with False."""
    height, width = mask.shape
    source_y0, source_y1 = max(0, -dy), height - max(0, dy)
    source_x0, source_x1 = max(0, -dx), width - max(0, dx)
    dest_y0, dest_y1 = max(0, dy), height - max(0, -dy)
    dest_x0, dest_x1 = max(0, dx), width - max(0, -dx)
    shifted = np.zeros_like(mask)
    shifted[dest_y0:dest_y1, dest_x0:dest_x1] = \
        mask[source_y0:source_y1, source_x0:source_x1]
    return shifted


def morphology(mask, kernel_size, open_iterations, close_iterations):
    """Small binary open/close with a square structuring element."""
    radius = max(0, (int(kernel_size) - 1) // 2)
    offsets = [(dy, dx) for dy in range(-radius, radius + 1)
               for dx in range(-radius, radius + 1)]

    def erode(source):
        result = np.ones_like(source)
        for dy, dx in offsets:
            result &= _shift(source, dy, dx)
        return result

    def dilate(source):
        result = np.zeros_like(source)
        for dy, dx in offsets:
            result |= _shift(source, dy, dx)
        return result

    result = mask.copy()
    for _ in range(max(0, int(open_iterations))):
        result = dilate(erode(result))
    for _ in range(max(0, int(close_iterations))):
        result = erode(dilate(result))
    return result


def connected_components(mask):
    """Return 8-connected components as arrays of (y, x) points."""
    height, width = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components = []
    ys, xs = np.nonzero(mask)
    for start_y, start_x in zip(ys, xs):
        if visited[start_y, start_x]:
            continue
        stack = [(int(start_y), int(start_x))]
        visited[start_y, start_x] = True
        points = []
        while stack:
            y, x = stack.pop()
            points.append((y, x))
            for ny in range(max(0, y - 1), min(height, y + 2)):
                for nx in range(max(0, x - 1), min(width, x + 2)):
                    if not visited[ny, nx] and mask[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
        points_array = np.asarray(points, dtype=np.int32)
        components.append(points_array)
    return components


def sobel_magnitude(gray):
    """Simple Sobel edge magnitude without OpenCV."""
    image = np.asarray(gray, dtype=np.float32)
    gx = np.zeros_like(image)
    gy = np.zeros_like(image)
    gx[:, 1:-1] = image[:, 2:] - image[:, :-2]
    gy[1:-1, :] = image[2:, :] - image[:-2, :]
    return np.hypot(gx, gy)


def component_features(points, edge):
    """Compute bbox, centroid, fill ratio and mean edge strength."""
    ys, xs = points[:, 0], points[:, 1]
    x0, x1 = int(xs.min()), int(xs.max())
    y0, y1 = int(ys.min()), int(ys.max())
    bbox_area = max(1, (x1 - x0 + 1) * (y1 - y0 + 1))
    return {
        "points": points,
        "area": int(len(points)),
        "x0": x0, "x1": x1, "y0": y0, "y1": y1,
        "width": x1 - x0 + 1,
        "height": y1 - y0 + 1,
        "center_x": float(xs.mean()),
        "center_y": float(ys.mean()),
        "fill": len(points) / float(bbox_area),
        "aspect": (x1 - x0 + 1) / float(y1 - y0 + 1),
        "edge": float(np.mean(edge[ys, xs])),
    }


def match_disparity(left_gray, right_gray, x, y, stereo_config):
    """Return (disparity, mean_abs_error) for one point, or None."""
    height, width = left_gray.shape
    radius = int(stereo_config.get("patch_radius", 7))
    minimum = int(stereo_config.get("min_disparity", 3))
    maximum = min(int(stereo_config.get("max_disparity", 96)), width - 1)
    max_error = float(stereo_config.get("max_mean_abs_error", 55.0))
    if (x - radius < 0 or x + radius >= width or y - radius < 0
            or y + radius >= height):
        return None

    left_patch = left_gray[y - radius:y + radius + 1,
                           x - radius:x + radius + 1].astype(np.float32)
    costs = {}
    best_disparity, best_error = None, float("inf")
    for disparity in range(maximum, minimum - 1, -1):
        right_x = x - disparity
        if right_x - radius < 0:
            continue
        right_patch = right_gray[y - radius:y + radius + 1,
                                 right_x - radius:right_x + radius + 1]
        error = float(np.mean(np.abs(left_patch - right_patch.astype(np.float32))))
        costs[disparity] = error
        if error < best_error:
            best_error, best_disparity = error, disparity
    if best_disparity is None or best_error > max_error:
        return None

    # Parabolic interpolation around the best integer disparity gives a stable
    # sub-pixel estimate without adding a dependency.
    refined = float(best_disparity)
    left_cost = costs.get(best_disparity - 1)
    right_cost = costs.get(best_disparity + 1)
    if left_cost is not None and right_cost is not None:
        denominator = left_cost - 2.0 * best_error + right_cost
        if denominator > 0.0:
            adjustment = 0.5 * (left_cost - right_cost) / denominator
            refined += max(-0.5, min(0.5, adjustment))
    return refined, best_error


def estimate_depth(features, points, left_gray, right_gray, config):
    """Combine rectified-stereo disparity and apparent-size depth."""
    camera = config.get("camera", {})
    target = config.get("target", {})
    classical = config.get("classical", {})
    stereo_config = classical.get("stereo", {}) or {}
    depth_config = classical.get("depth", {}) or {}

    step = max(1, int(stereo_config.get("sample_step", 6)))
    minimum_samples = int(stereo_config.get("min_valid_samples", 3))
    samples = points[::step]
    matches = []
    for x, y in ((int(point[1]), int(point[0])) for point in samples):
        match = match_disparity(left_gray, right_gray, x, y, stereo_config)
        if match is not None:
            matches.append(match)

    fx = float(camera.get("fx"))
    baseline = float(camera.get("baseline_m"))
    depth_stereo = None
    if len(matches) >= minimum_samples:
        disparity = float(np.median([match[0] for match in matches]))
        if disparity > 0.0:
            depth_stereo = fx * baseline / disparity

    depth_size = None
    known_width = float(target.get("known_width_m", 0.0))
    if known_width > 0.0 and features["width"] > 0:
        depth_size = fx * known_width / float(features["width"])

    method = str(depth_config.get("method", "fuse")).lower()
    stereo_weight = float(depth_config.get("stereo_weight", 0.7))
    size_weight = float(depth_config.get("size_weight", 0.3))
    if method == "stereo":
        depth = depth_stereo
        source = "stereo_disparity"
    elif method == "size":
        depth = depth_size
        source = "apparent_size"
    else:
        values, weights = [], []
        if depth_stereo is not None:
            values.append(depth_stereo)
            weights.append(stereo_weight)
        if depth_size is not None:
            values.append(depth_size)
            weights.append(size_weight)
        if not values:
            depth, source = None, "none"
        elif len(values) == 1:
            depth = values[0]
            source = "stereo_disparity" if depth_stereo is not None \
                else "apparent_size"
        else:
            depth = sum(value * weight for value, weight in zip(values, weights)) \
                / sum(weights)
            source = "fused_stereo_size"

    if depth is not None:
        depth = max(float(camera.get("depth_min_m")),
                    min(float(camera.get("depth_max_m")), depth))

    consistency = 0.5
    if depth_stereo is not None and depth_size is not None:
        ratio = max(depth_stereo, depth_size) / max(1e-6, min(depth_stereo, depth_size))
        consistency = 1.0 / (1.0 + abs(math.log(ratio)))

    return {
        "depth_m": depth,
        "source": source,
        "depth_stereo_m": depth_stereo,
        "depth_size_m": depth_size,
        "disparity_px": float(np.median([match[0] for match in matches]))
                        if matches else None,
        "valid_matches": len(matches),
        "sample_count": len(samples),
        "consistency": consistency,
    }


def detect_obstacle(edge, toy_features, config):
    """Conservative edge-density check in the centre of the lower image."""
    obstacle_config = config.get("classical", {}).get("obstacle", {}) or {}
    if not obstacle_config.get("enabled", True):
        return "none"
    height, width = edge.shape
    x0, x1 = obstacle_config.get("roi_x", [0.30, 0.70])
    y0, y1 = obstacle_config.get("roi_y", [0.45, 1.00])
    row0, row1 = int(y0 * height), min(height, int(y1 * height))
    col0, col1 = int(x0 * width), min(width, int(x1 * width))
    if row1 <= row0 or col1 <= col0:
        return "none"

    region = edge[row0:row1, col0:col1].copy()
    if toy_features is not None:
        margin = 3
        ty0 = max(row0, toy_features["y0"] - margin)
        ty1 = min(row1, toy_features["y1"] + margin + 1)
        tx0 = max(col0, toy_features["x0"] - margin)
        tx1 = min(col1, toy_features["x1"] + margin + 1)
        if ty1 > ty0 and tx1 > tx0:
            region[ty0 - row0:ty1 - row0, tx0 - col0:tx1 - col0] = 0.0

    threshold = float(obstacle_config.get("edge_threshold", 55.0))
    minimum_fraction = float(obstacle_config.get("min_edge_fraction", 0.06))
    fraction = float(np.mean(region > threshold)) if region.size else 0.0
    if fraction >= minimum_fraction:
        return str(obstacle_config.get("label", "obstacle"))
    return "none"


def _position(center_x, config, image_width=None):
    width = float(image_width or config["camera"]["width"])
    if center_x < float(config["target"]["position_split_left"]) * width:
        return "left"
    if center_x < float(config["target"]["position_split_right"]) * width:
        return "center"
    return "right"


def _size(bbox_width, config, image_width=None):
    width = float(image_width or config["camera"]["width"])
    fraction = bbox_width / width
    if fraction < float(config["target"]["apparent_width_small_fraction"]):
        return "small"
    if fraction > float(config["target"]["apparent_width_large_fraction"]):
        return "large"
    return "medium"


def detect(rgb_left, rgb_right, target, config):
    """Run the full deterministic detector on two rectified RGB images.

    Returns a JSON-serialisable dict in the same shape as an ML model answer,
    with extra fields describing the candidate box, confidence, and how the
    metric distance was derived.
    """
    left = np.asarray(rgb_left)
    right = np.asarray(rgb_right)
    if left.shape != right.shape:
        raise ValueError("rectified image shapes differ: %s vs %s"
                         % (left.shape, right.shape))

    classical = config.get("classical", {}) or {}
    image_height, image_width = left.shape[:2]
    gray_left = rgb_to_gray(left)
    gray_right = rgb_to_gray(right)
    edge = sobel_magnitude(gray_left)

    result = {
        "description": "Classical detector found no matching toy.",
        "target_visible": False,
        "position": "none",
        "size": "none",
        "distance_m": 0.0,
        "obstacle_ahead": "none",
        "path_clear": True,
        "method": "classical",
        "distance_source": "none",
        "confidence": 0.0,
        "bbox_px": [],
    }

    mask = color_mask(left, vision_config.color_ranges(config, target))
    mask = morphology(
        mask,
        classical.get("morph_kernel_size", 3),
        classical.get("morph_open_iterations", 1),
        classical.get("morph_close_iterations", 2))

    candidates = []
    for points in connected_components(mask):
        features = component_features(points, edge)
        if features["area"] < int(classical.get("min_component_area", 45)):
            continue
        if features["area"] > float(classical.get(
                "max_component_area_fraction", 0.35)) * image_height * image_width:
            continue
        if not (int(classical.get("min_bbox_width_px", 7)) <= features["width"]
                <= int(classical.get("max_bbox_width_px", 180))):
            continue
        if not (int(classical.get("min_bbox_height_px", 7)) <= features["height"]
                <= int(classical.get("max_bbox_height_px", 180))):
            continue
        if features["aspect"] > float(classical.get("max_aspect_ratio", 3.0)) \
                or 1.0 / max(features["aspect"], 1e-6) > float(
                    classical.get("max_aspect_ratio", 3.0)):
            continue
        if features["fill"] < float(classical.get("min_fill_ratio", 0.22)):
            continue
        if features["edge"] < float(classical.get("min_edge_strength", 6.0)):
            continue
        depth = estimate_depth(features, points, gray_left, gray_right, config)
        fill_score = min(1.0, features["fill"] / 0.6)
        size_score = min(1.0, features["width"] / (0.12 * image_width))
        edge_score = min(1.0, features["edge"] / 40.0)
        sample_score = (depth["valid_matches"] / max(1, depth["sample_count"]))
        confidence = (0.25 * fill_score + 0.15 * size_score
                      + 0.20 * edge_score + 0.25 * sample_score
                      + 0.15 * depth["consistency"])
        if confidence < float(classical.get("min_confidence", 0.30)):
            continue
        candidates.append((confidence, features, depth))

    best = max(candidates, key=lambda item: item[0]) if candidates else None
    if best is not None:
        confidence, features, depth = best
        result.update({
            "description": (
                "Classical colour/edge detector found a toy at "
                "(%.0f, %.0f) px, bbox %dx%d px, confidence %.2f."
                % (features["center_x"], features["center_y"],
                   features["width"], features["height"], confidence)),
            "target_visible": True,
            "position": _position(features["center_x"], config, image_width),
            "size": _size(features["width"], config, image_width),
            "distance_m": round(depth["depth_m"], 3)
                          if depth["depth_m"] is not None else 0.0,
            "confidence": round(confidence, 3),
            "distance_source": depth["source"],
            "bbox_px": [features["x0"], features["y0"],
                        features["x1"], features["y1"]],
            "disparity_px": round(depth["disparity_px"], 2)
                            if depth["disparity_px"] is not None else None,
            "stereo_matches": depth["valid_matches"],
        })

    obstacle = detect_obstacle(edge, best[1] if best else None, config)
    result["obstacle_ahead"] = obstacle
    result["path_clear"] = obstacle == "none"
    return result
