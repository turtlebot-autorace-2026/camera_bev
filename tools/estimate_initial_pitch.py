#!/usr/bin/env python3
"""Data-specific initial straight-lane pitch estimate, NOT a lane detector.

Assumes flat ground, parallel paint, zero roll; manual image bands and HSV limits
are chosen for normal.bag's stationary first 10 seconds. No height is estimated.
Inspect the saved feature overlays before trusting the result on another bag.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

from camera_bev.geometry import BEVMapper, read_yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    m = BEVMapper(read_yaml(root / 'config/camera_1280x720.yaml'), read_yaml(root / 'config/initial_20261028.yaml'))
    targets = {1, 12, 24, 35, 46, 58, 69, 92, 115}
    con = sqlite3.connect((args.bag / 'normal_0.db3').resolve().as_uri() + '?mode=ro', uri=True)
    rows = con.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY timestamp LIMIT 115', ('/camera_raw_16_9',))
    results = []
    for index, (blob,) in enumerate(rows, 1):
        if index not in targets:
            continue
        msg = deserialize_message(blob, CompressedImage)
        im = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        result = dict(frame=index, assumptions=['flat ground', 'parallel lane paint', 'zero roll'], lanes={})
        lines = []
        for name, lower, upper, xmin, xmax, color in [
            ('yellow', (12, 50, 75), (40, 255, 255), 130, 640, (0, 255, 0)),
            ('white', (0, 0, 150), (179, 60, 255), 750, 1150, (0, 0, 255))]:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            points = []
            for y in range(220, 671, 5):
                xs = np.flatnonzero(mask[y, xmin:xmax]) + xmin
                segments = np.split(xs, np.where(np.diff(xs) > 1)[0] + 1)
                segments = [s for s in segments if 5 <= len(s) <= 70]
                if segments:
                    segment = segments[0] if name == 'white' else max(segments, key=len)
                    points.append((float(np.mean(segment)), float(y)))
            points = np.array(points)
            normalized, ok, residual = m.camera.undistort_points(points, m.radius)
            points, normalized = points[ok], normalized[ok]
            if len(points) < 20:
                raise ValueError('Too few reliable lane samples in the configured initial bands')
            vx, vy, x, y = cv2.fitLine(normalized.astype('float32'), cv2.DIST_HUBER, 0, .001, .001).ravel()
            line = np.array([-vy, vx, vy * x - vx * y])
            # Reflections/adjacent paint can satisfy the color band. Keep only
            # points within ~10 normalized-equivalent pixels of the robust line.
            inliers = np.abs(np.c_[normalized, np.ones(len(normalized))] @ line) < .012
            rejected_outliers = int(np.sum(~inliers))
            points, normalized = points[inliers], normalized[inliers]
            vx, vy, x, y = cv2.fitLine(normalized.astype('float32'), cv2.DIST_HUBER, 0, .001, .001).ravel()
            line = np.array([-vy, vx, vy * x - vx * y])
            lines.append(line)
            ground, ground_valid, _ = m.raw_to_ground(points)
            gx, gy = ground[ground_valid, 0], ground[ground_valid, 1]
            slope, intercept = np.polyfit(gx, gy, 1)
            result['lanes'][name] = dict(raw_points=points.tolist(), line_normalized=line.tolist(),
                                        rejected_color_outliers=rejected_outliers,
                                        inverse_residual_px_max=float(np.max(residual[ok])),
                                        bev_heading_deg=float(np.rad2deg(np.arctan(slope))),
                                        bev_line_rms_nominal_m=float(np.sqrt(np.mean((gy - slope * gx - intercept) ** 2))),
                                        ground_x_range_nominal_m=[float(min(gx)), float(max(gx))])
            for x, y in points:
                cv2.circle(im, (int(round(x)), int(round(y))), 3, color, -1)
        vp = np.cross(*lines)
        vp /= vp[2]
        pitch = np.arctan(-vp[1])
        result.update(vanishing_point_normalized=vp.tolist(), pitch_deg=float(np.rad2deg(pitch)),
                      apparent_lane_yaw_deg=float(np.rad2deg(np.arctan(vp[0] * np.cos(pitch)))))
        results.append(result)
        cv2.imwrite(str(args.output / 'pose_features_{:04d}.jpg'.format(index)), im)
    con.close()
    report = dict(note=__doc__, configured_pitch_deg=m.ground['pitch_deg'], height='not estimated',
                  pitch_deg=dict(median=float(np.median([r['pitch_deg'] for r in results])),
                                 min=float(min(r['pitch_deg'] for r in results)),
                                 max=float(max(r['pitch_deg'] for r in results))), samples=results)
    (args.output / 'initial_pose_estimate.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['pitch_deg']))


if __name__ == '__main__':
    main()
