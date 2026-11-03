#!/usr/bin/env python3
"""Fit a metric ground plane from a stationary floor checkerboard.

Known intrinsics are kept fixed. A provisional BEV makes corner detection
possible; detected points are mapped BACK to raw before fitting. The fitted
plane is not a new intrinsic calibration or an independently measured pose.
Detection bounds and independent lane-validation ROIs target stationary_camera_2.
Run after sourcing ROS; no ROS nodes or motion topics are created.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
import yaml

from camera_bev.geometry import BEVMapper, calibration_fingerprint, read_yaml


def transform(points, matrix):
    return cv2.perspectiveTransform(np.asarray(points, float).reshape(-1, 1, 2), matrix).reshape(-1, 2)


def detect(image, probe, pattern):
    view = probe.render(image)
    ok, corners = cv2.findChessboardCornersSB(
        cv2.cvtColor(view, cv2.COLOR_BGR2GRAY), pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        raise ValueError('Complete checkerboard not detected')
    corners = corners.reshape(pattern[1], pattern[0], 2)
    if corners[-1, :, 1].mean() < corners[0, :, 1].mean():
        corners = corners[::-1]
    if corners[:, -1, 0].mean() < corners[:, 0, 0].mean():
        corners = corners[:, ::-1]
    corners = corners.reshape(-1, 2)
    ground = np.c_[probe.x_max - (corners[:, 1] + .5) * probe.resolution,
                   probe.y_max - (corners[:, 0] + .5) * probe.resolution,
                   np.zeros(len(corners))]
    u, v, valid = probe.camera.project((ground - probe.center) @ probe.rotation.T, probe.radius)
    if not valid.all():
        raise ValueError('Checkerboard extends outside invertible lens domain')
    raw = np.c_[u, v]
    normalized, valid, _ = probe.camera.undistort_points(raw, probe.radius)
    if not valid.all():
        raise ValueError('Corner inverse projection failed')
    return raw, normalized


def stats(values):
    values = np.asarray(values, float).ravel()
    return dict(count=int(values.size), median=float(np.median(values)),
                mean=float(np.mean(values)), min=float(values.min()), max=float(values.max()),
                p95=float(np.percentile(values, 95)))


def lane_edges(image):
    """Independent visible inner-edge samples; excludes the paper occlusion.

    ROIs match this stationary capture. They are validation annotations, not
    a general lane detector or constraints used in the checkerboard fit.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = [cv2.inRange(hsv, (12, 55, 90), (40, 255, 255)),
             cv2.inRange(hsv, (0, 0, 180), (179, 65, 255))]
    points = []
    for row in list(range(125, 180, 3)) + list(range(355, 651, 5)):
        pair = []
        for mask, lo, hi, right in [(masks[0], 120, 600, True), (masks[1], 700, 1190, False)]:
            changes = np.flatnonzero(np.diff(np.r_[False, mask[row, lo:hi] > 0, False]))
            segments = [(lo + a, lo + b - 1) for a, b in changes.reshape(-1, 2) if 4 <= b - a <= 100]
            if not segments:
                break
            segment = max(segments) if right else min(segments)
            pair.append([segment[1] if right else segment[0], row])
        if len(pair) == 2:
            points.append(pair)
    return np.asarray(points, float)


def lane_widths(mapper, raw):
    points, valid, _ = mapper.raw_to_ground(raw.reshape(-1, 2))
    points = points.reshape(-1, 2, 3)
    valid = valid.reshape(-1, 2).all(axis=1)
    result = {}
    for selection, label in [(raw[:, 0, 1] < 180, 'far'), (raw[:, 0, 1] > 350, 'near')]:
        selected = points[selection & valid]
        # Interpolate the two boundaries at the SAME ground X. Then report
        # separation normal to their average direction (not raw image rows).
        left = selected[:, 0, :2]; right = selected[:, 1, :2]
        left = left[np.argsort(left[:, 0])]; right = right[np.argsort(right[:, 0])]
        x = np.linspace(max(left[0, 0], right[0, 0]), min(left[-1, 0], right[-1, 0]), 30)
        yl = np.interp(x, left[:, 0], left[:, 1]); yr = np.interp(x, right[:, 0], right[:, 1])
        slope = np.polyfit(x, .5 * (yl + yr), 1)[0]
        result[label] = stats((yl - yr) / np.sqrt(1 + slope * slope) * 1000)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--initial-settings', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--columns', type=int, default=10, help='Internal corner columns')
    parser.add_argument('--rows', type=int, default=7, help='Internal corner rows')
    parser.add_argument('--square-m', type=float, default=.020)
    parser.add_argument('--height-m', type=float, default=.135, help='Approximate origin reference only')
    parser.add_argument('--pitch-deg', type=float, default=32.5, help='Approximate origin reference only')
    args = parser.parse_args()
    if not (3 <= args.columns <= 30 and 3 <= args.rows <= 30 and 0 < args.square_m < 1
            and 0 < args.height_m < 2 and 5 < args.pitch_deg < 80):
        parser.error('Invalid checkerboard dimensions or origin reference')
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    calibration = read_yaml(args.calibration); initial = read_yaml(args.initial_settings)
    if initial.get('ground_projection'):
        raise ValueError('Initial settings must be a pose-only projection')
    reference = deepcopy(initial)
    reference['ground'].update(height_m=args.height_m, pitch_deg=args.pitch_deg,
                               roll_deg=0., yaw_deg=0., offset_x_m=0., offset_y_m=0., measured=False)
    probe_settings = deepcopy(reference)
    probe_settings['output'] = dict(x_min_m=.2, x_max_m=.7, y_min_m=-.2, y_max_m=.2, resolution_m=.0005)
    probe = BEVMapper(calibration, probe_settings)
    database = sorted(args.bag.glob('*.db3'))
    if len(database) != 1:
        raise ValueError('This tool expects a single SQLite3 bag segment')
    connection = sqlite3.connect(database[0].resolve().as_uri() + '?mode=ro', uri=True)
    rows = connection.execute("SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                              "WHERE t.name='/camera_raw_16_9' ORDER BY m.timestamp").fetchall()
    connection.close()
    if len(rows) < 17:
        raise ValueError('At least 17 frames required for repeatability checks')
    indices = np.linspace(0, len(rows) - 1, 17).round().astype(int)
    object_xy = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2) * args.square_m
    detections = []
    for index in indices:
        msg = deserialize_message(rows[index][0], CompressedImage)
        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), 1)
        raw, normalized = detect(image, probe, (args.columns, args.rows))
        detections.append(dict(index=int(index), raw=raw, normalized=normalized))
        print('Detected frame %d: %d corners' % (index + 1, len(raw)), flush=True)
    train = detections[::2]; validation = detections[1::2]
    observed = np.concatenate([d['normalized'] for d in train])
    targets = np.tile(object_xy, (len(train), 1))
    normalized_to_board, _ = cv2.findHomography(observed, targets, 0)
    # Orient the plane like the provisional camera ground axes, without
    # importing its incorrect metric scale. Reflection accounts for board
    # axes (right/toward-camera) versus ground axes (forward/left).
    approx, _, _ = probe.raw_to_ground(train[0]['raw'])
    u, _, vt = np.linalg.svd((object_xy - object_xy.mean(axis=0)).T @
                             (approx[:, :2] - approx[:, :2].mean(axis=0)))
    orientation = (u @ vt).T
    optical_intersection_board = transform([[0., 0.]], normalized_to_board)[0]
    anchor = np.array([args.height_m / np.tan(np.deg2rad(args.pitch_deg)), 0.])
    board_to_ground = np.eye(3)
    board_to_ground[:2, :2] = orientation
    board_to_ground[:2, 2] = anchor - orientation @ optical_intersection_board
    ground_to_normalized = np.linalg.inv(board_to_ground @ normalized_to_board)
    ground_to_normalized /= (ground_to_normalized @ np.r_[anchor, 1.])[2]
    profile = deepcopy(reference)
    profile['ground_projection'] = dict(type='normalized_homography',
        ground_to_normalized=ground_to_normalized.tolist(), reference_ground=deepcopy(profile['ground']),
        calibration_fingerprint=calibration_fingerprint(calibration),
        square_size_m=args.square_m, inner_corners=[args.columns, args.rows],
        origin='Approximate camera ground projection anchored by reported height/pitch; not base_link',
        scale_source='User measured checker square on a flat floor; lane width held out',
        source_bag=str(args.bag.resolve()))
    # Spatial validation: remove alternating inner corners from every train
    # frame, then predict them. Temporal validation is repeated observation
    # of the SAME stationary geometry, not an independent accuracy reference.
    keep = (np.indices((args.rows, args.columns)).sum(axis=0).ravel() % 2 == 0)
    partial, _ = cv2.findHomography(np.concatenate([d['normalized'][keep] for d in train]),
                                  np.tile(object_xy[keep], (len(train), 1)), 0)
    spatial = np.concatenate([np.linalg.norm(transform(d['normalized'][~keep], partial) - object_xy[~keep], axis=1)
                              for d in validation]) * 1000
    temporal = np.concatenate([np.linalg.norm(transform(d['normalized'], normalized_to_board) - object_xy, axis=1)
                               for d in validation]) * 1000
    new = BEVMapper(calibration, profile); old = BEVMapper(calibration, initial)
    cell_metrics = {}
    for label, mapper in [('before', old), ('after', new)]:
        along, across = [], []
        for d in validation:
            points, valid, _ = mapper.raw_to_ground(d['raw']); assert valid.all()
            grid = points.reshape(args.rows, args.columns, 3)
            along.extend(np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel() * 1000)
            across.extend(np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel() * 1000)
        cell_metrics[label] = dict(along_mm=stats(along), across_mm=stats(across))
    first = cv2.imdecode(np.frombuffer(deserialize_message(rows[0][0], CompressedImage).data, np.uint8), 1)
    edges = lane_edges(first)
    widths = {'before': lane_widths(old, edges), 'after': lane_widths(new, edges), 'reported_inner_width_mm':260.}
    report = dict(square_size_m=args.square_m, pattern_inner_corners=[args.columns, args.rows],
                  detected_frames=[d['index'] + 1 for d in detections], train_frames=[d['index'] + 1 for d in train],
                  validation_frames=[d['index'] + 1 for d in validation],
                  temporal_ground_error_mm=stats(temporal), spatial_holdout_ground_error_mm=stats(spatial),
                  cell_sizes=cell_metrics, independent_lane_inner_width=widths,
                  reference_ground=profile['ground'], origin_independently_measured=False,
                  notes=['Repeated frames share one board placement; they measure stability, not independent accuracy.',
                         'Lane width was not used to force the result to 260 mm.',
                         'Reference height/pitch anchor the origin; they are not a recovered physical pose.',
                         'Single raw remap includes the existing lens distortion model.'])
    if report['spatial_holdout_ground_error_mm']['p95'] > 2. or report['temporal_ground_error_mm']['p95'] > 2.:
        raise ValueError('Held-out checkerboard error exceeds 2 mm; do not apply this profile')
    (args.output / 'profile.yaml').write_text(yaml.safe_dump(profile, sort_keys=False))
    (args.output / 'calibration.json').write_text(json.dumps(report, indent=2))
    np.savez(args.output / 'correspondences.npz', object_xy=object_xy, normalized_to_board=normalized_to_board,
             raw=np.stack([d['raw'] for d in detections]), indices=indices)
    cv2.imwrite(str(args.output / 'before.png'), old.debug(old.render(first)))
    cv2.imwrite(str(args.output / 'after.png'), new.debug(new.render(first)))
    cv2.imwrite(str(args.output / 'comparison.png'), np.hstack([old.debug(old.render(first)), new.debug(new.render(first))]))
    marks = first.copy()
    for pt in detections[0]['raw']:
        cv2.circle(marks, tuple(np.round(pt).astype(int)), 2, (0, 0, 255), -1)
    cv2.imwrite(str(args.output / 'corners.png'), marks)
    edge_marks = first.copy()
    for point in edges.reshape(-1, 2):
        cv2.circle(edge_marks, tuple(point.astype(int)), 2, (0, 0, 255), -1)
    cv2.putText(edge_marks, 'Lane inner edges only; paper and checkerboard excluded',
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
    cv2.imwrite(str(args.output / 'lane_validation_edges.png'), edge_marks)
    np.save(args.output / 'lane_validation_edges.npy', edges)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
