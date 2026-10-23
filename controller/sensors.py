"""Measured sensor projection. Missing samples remain invalid, never clear."""
import math
import numpy as np


def rotation(quaternion):
    x, y, z, w = quaternion
    q = np.asarray([x, y, z, w], dtype=float)
    length = np.linalg.norm(q)
    if not np.isfinite(q).all() or length < 1e-9:
        raise ValueError('Invalid sensor TF quaternion')
    x, y, z, w = q / length
    return np.array([[1 - 2 * (y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def depth_image_points(image, info, stride=4):
    """Project sparse, measured depth pixels in the camera optical frame."""
    if (image.width, image.height, image.header.frame_id) != (info.width, info.height, info.header.frame_id):
        raise ValueError('Depth image and calibration frame/size differ')
    encoding = {'16UC1': ('u2', .001), '32FC1': ('f4', 1.)}.get(image.encoding)
    if encoding is None or min(info.k[0], info.k[4]) <= 0:
        raise ValueError('Unsupported depth encoding or invalid intrinsics')
    dtype = np.dtype(('>' if image.is_bigendian else '<') + encoding[0])
    depth = np.ndarray((image.height, image.width), dtype=dtype, buffer=image.data,
                       strides=(image.step, dtype.itemsize))
    rows, cols = np.mgrid[0:image.height:stride, 0:image.width:stride]
    z = depth[::stride, ::stride].astype(np.float32) * encoding[1]
    valid = np.isfinite(z) & (z > 0)
    z = z[valid]
    x = (cols[valid] - info.k[2]) * z / info.k[0]
    y = (rows[valid] - info.k[5]) * z / info.k[4]
    return np.column_stack((x, y, z))


def lidar_features(ranges, angle_min, angle_increment, range_min, range_max, translation, quaternion, maximum):
    ranges = np.asarray(ranges, dtype=float)
    valid = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    points = np.column_stack((ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid]), np.zeros(valid.sum())))
    points = points @ rotation(quaternion).T + translation
    angles = np.arctan2(points[:, 1], points[:, 0])
    bins = np.floor((angles + math.pi) * 72 / (2 * math.pi) + .5).astype(int) % 72
    output, seen = np.full(72, maximum), np.zeros(72, dtype=bool)
    np.minimum.at(output, bins, np.linalg.norm(points[:, :2], axis=1))
    seen[bins] = True
    return np.clip(output, 0, maximum), seen


def depth_features(points, translation, quaternion, robot, diagnostics=None):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1) & (points[:, 2] > 0)]
    radial = np.linalg.norm(points, axis=1)
    good = (radial >= robot.camera_min_range) & (radial <= robot.camera_range)
    # A finite return beyond the navigation horizon is observed open space,
    # even though its distance is clipped to the policy's camera range.
    far = (radial > robot.camera_range) & (radial <= robot.camera_range * 2)
    too_close = radial < robot.camera_min_range
    base = points @ rotation(quaternion).T + translation
    azimuth = np.degrees(np.arctan2(base[:, 1], base[:, 0]))
    fov = robot.camera_hfov_deg
    bins = np.floor((azimuth + fov / 2) * 35 / fov + .5).astype(int)
    visible = (bins >= 0) & (bins < 36)
    output, valid = np.full(36, robot.camera_range), np.zeros(36, dtype=bool)
    valid[bins[visible & (good | far)]] = True
    valid[bins[visible & too_close]] = False
    obstacle = visible & (good | too_close) & (base[:, 2] > .03) & (base[:, 2] < robot.height + .05)
    horizontal = np.linalg.norm(base[:, :2], axis=1)
    np.minimum.at(output, bins[obstacle], horizontal[obstacle])
    # Actual measured ground/below-ground points, not missing depth pixels.
    floor = np.full(12, robot.camera_range)
    floor_valid = np.zeros(12, dtype=bool)
    floor_bins = np.floor((azimuth + fov / 2) * 11 / fov + .5).astype(int)
    ground = (floor_bins >= 0) & (floor_bins < 12) & good & (base[:, 2] <= .03)
    floor_valid[floor_bins[ground]] = True
    drops = ground & (base[:, 2] < -.05)
    np.minimum.at(floor, floor_bins[drops], horizontal[drops])
    if diagnostics is not None:
        diagnostics.update(closest_surface_m=float(radial.min()) if len(radial) else None,
            nearest_obstacle_m=float(horizontal[obstacle].min()) if obstacle.any() else None,
            obstacle_points=int(obstacle.sum()), measured_points=len(points),
            floor_points=int(ground.sum()), too_close_points=int((visible & too_close).sum()),
            depth_coverage=float(valid.mean()), floor_coverage=float(floor_valid.mean()))
    return (np.clip(output, 0, robot.camera_range), valid), (floor, floor_valid)
