#!/usr/bin/env python3
"""Read-only SQLite/CDR evaluation. No rclpy.init, ROS publishers or motion.

Source ROS setup and install_bev/local_setup.bash before running. All products
are written to --output, never into the source bag directory.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sqlite3
import time

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

from camera_bev.geometry import BEVMapper, read_yaml


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'config')
    args = parser.parse_args()
    if args.output.resolve() == args.bag.resolve() or args.bag.resolve() in args.output.resolve().parents:
        raise ValueError('Review output must be separate from the source bag')
    args.output.mkdir(parents=True, exist_ok=True)
    cv2.setNumThreads(1)
    t0 = time.perf_counter()
    mapper = BEVMapper(read_yaml(args.config / 'camera_1280x720.yaml'), read_yaml(args.config / 'estimated.yaml'))
    map_ms = (time.perf_counter() - t0) * 1000
    scale_label = 'BOARD SCALE / APPROX ORIGIN' if mapper.ground_homography is not None else 'ESTIMATED SCALE / POSE'
    old_k, old_roi = cv2.getOptimalNewCameraMatrix(mapper.camera.k, mapper.camera.d, mapper.camera.size, 0)
    alpha_one_k, alpha_one_roi = cv2.getOptimalNewCameraMatrix(mapper.camera.k, mapper.camera.d, mapper.camera.size, 1)
    z = mapper.camera_points[..., 2]
    safe_z = np.where(z > 1e-9, z, 1)
    old_u = (old_k[0, 0] * mapper.camera_points[..., 0] / safe_z + old_k[0, 2]).astype('float32')
    old_v = (old_k[1, 1] * mapper.camera_points[..., 1] / safe_z + old_k[1, 2]).astype('float32')
    old_support = ((mapper.valid_mask > 0) & (old_u >= 0) & (old_u < 1279) & (old_v >= 0) & (old_v < 719))
    old_maps = cv2.initUndistortRectifyMap(mapper.camera.k, mapper.camera.d, None, old_k, mapper.camera.size, cv2.CV_16SC2)
    cv2.imwrite(str(args.output / 'valid_mask.png'), mapper.valid_mask)
    cv2.imwrite(str(args.output / 'rect_valid_mask.png'), mapper.rect_valid_mask)
    support_comparison = np.zeros((mapper.height, mapper.width, 3), np.uint8)
    support_comparison[old_support] = (110, 110, 110)
    support_comparison[(mapper.valid_mask > 0) & ~old_support] = (40, 210, 70)
    cv2.imwrite(str(args.output / 'coverage_gain.png'), support_comparison)
    sample_ids = {1, 12, 24, 35, 46, 58, 69, 92, 115, 172, 350, 550, 727, 1000, 1300, 1600, 1900, 2200, 2500, 2755}
    products = []
    timings = []
    n = 0
    stamps = []
    bag_hash_before = {str(p): digest(p) for p in sorted(args.bag.glob('*')) if p.is_file()}
    video = cv2.VideoWriter(str(args.output / 'preview_source.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 12.61488, (1360, 400))
    if not video.isOpened():
        raise RuntimeError('Could not open review video writer')
    with (args.output / 'frames.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['frame', 'bag_seconds', 'header_sec', 'header_nanosec', 'decode_ms', 'bev_ms', 'rect_ms', 'debug_ms', 'jpeg_ms'])
        start_ns = None
        for db in sorted(args.bag.glob('*.db3')):
            con = sqlite3.connect(db.resolve().as_uri() + '?mode=ro', uri=True)
            con.execute('PRAGMA query_only=ON')
            if start_ns is None:
                start_ns = con.execute('SELECT min(timestamp) FROM messages').fetchone()[0]
            rows = con.execute('SELECT m.timestamp,m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY m.timestamp', ('/camera_raw_16_9',))
            for timestamp, blob in rows:
                n += 1
                msg = deserialize_message(blob, CompressedImage)
                now = time.perf_counter()
                im = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                decoded = time.perf_counter()
                bev = mapper.render(im)
                remapped = time.perf_counter()
                rect = mapper.rectify(im)
                rectified = time.perf_counter()
                debug = mapper.debug(bev)
                debugged = time.perf_counter()
                for pixels in (bev, rect, debug):
                    ok, _ = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    if not ok:
                        raise RuntimeError('JPEG encoding failed')
                encoded = time.perf_counter()
                times = np.diff([now, decoded, remapped, rectified, debugged, encoded]) * 1000
                timings.append(times.tolist())
                sec = (timestamp - start_ns) * 1e-9
                stamps.append((msg.header.stamp.sec, msg.header.stamp.nanosec))
                writer.writerow([n, sec, *stamps[-1], *times])
                canvas = np.full((400, 1360, 3), 22, np.uint8)
                canvas[40:, :640] = cv2.resize(im, (640, 360), interpolation=cv2.INTER_AREA)
                canvas[40:, 640:1000] = cv2.resize(bev, (360, 360), interpolation=cv2.INTER_AREA)
                canvas[40:, 1000:] = cv2.resize(debug, (360, 360), interpolation=cv2.INTER_AREA)
                for label, x in [('RAW | #{:04d} | {:.2f}s'.format(n, sec), 10), ('DIRECT BEV', 650), (scale_label, 1010)]:
                    cv2.putText(canvas, label, (x, 27), cv2.FONT_HERSHEY_SIMPLEX, .6, (235, 235, 235), 1)
                video.write(canvas)
                if n in sample_ids:
                    cv2.imwrite(str(args.output / 'sample_{:04d}.jpg'.format(n)), canvas)
                    old_rect = cv2.remap(im, *old_maps, interpolation=cv2.INTER_LINEAR)
                    old_bev = cv2.remap(old_rect, old_u, old_v, interpolation=cv2.INTER_LINEAR)
                    old_bev[~old_support] = 0
                    compare = np.full((540, 1500, 3), 22, np.uint8)
                    compare[40:, :500] = old_bev
                    compare[40:, 500:1000] = bev
                    compare[40:, 1000:] = debug
                    for label, x in [('alpha=0 intermediate -> BEV', 10), ('RAW -> BEV (single remap)', 510), (scale_label, 1010)]:
                        cv2.putText(compare, label, (x, 27), cv2.FONT_HERSHEY_SIMPLEX, .57, (235, 235, 235), 1)
                    cv2.imwrite(str(args.output / 'compare_{:04d}.jpg'.format(n)), compare)
                    products.append(dict(frame=n, bag_seconds=sec, sample='sample_{:04d}.jpg'.format(n), comparison='compare_{:04d}.jpg'.format(n)))
                    if n == 1:
                        cv2.imwrite(str(args.output / 'bev_first.png'), bev)
                        cv2.imwrite(str(args.output / 'bev_debug_first.png'), debug)
                        cv2.imwrite(str(args.output / 'rectified_wide_first.png'), rect)
                if n % 500 == 0:
                    print('Evaluated {} frames'.format(n), flush=True)
            con.close()
    video.release()
    bag_hash_after = {str(p): digest(p) for p in sorted(args.bag.glob('*')) if p.is_file()}
    assert bag_hash_before == bag_hash_after
    timing_array = np.array(timings)
    result = dict(frames=n, errors=0, map_build_ms=map_ms, projection=mapper.info(),
                  processing_ms={label: dict(mean=float(np.mean(values)), p50=float(np.percentile(values, 50)),
                                             p95=float(np.percentile(values, 95)), max=float(np.max(values)))
                                 for label, values in zip(['decode', 'bev', 'rectification', 'debug', 'jpeg_three', 'total'],
                                                          list(timing_array.T) + [timing_array.sum(axis=1)])},
                  original_hashes=bag_hash_after, original_unchanged=True,
                  alpha0_matrix=old_k.tolist(), alpha0_roi=list(old_roi),
                  alpha1_matrix=alpha_one_k.tolist(), alpha1_roi=list(alpha_one_roi),
                  direct_support_pixels=int(np.count_nonzero(mapper.valid_mask)),
                  alpha0_support_pixels=int(np.count_nonzero(old_support)),
                  recovered_ground_grid_pixels=int(np.count_nonzero((mapper.valid_mask > 0) & ~old_support)),
                  comparison_note='Same ground grid and central valid model domain; excludes later crop/resize losses. Pixel counts measure support, not lane accuracy.',
                  video_note='Constant 12.61488 fps review video; bag timestamps are shown, but original timing jitter is not reproduced.',
                  samples=products, runtime=dict(opencv=cv2.__version__, numpy=np.__version__))
    (args.output / 'evaluation.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('frames', 'errors', 'processing_ms', 'direct_support_pixels', 'alpha0_support_pixels', 'recovered_ground_grid_pixels')}, indent=2))


if __name__ == '__main__':
    main()
