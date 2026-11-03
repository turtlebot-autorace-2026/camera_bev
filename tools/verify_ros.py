#!/usr/bin/env python3
"""Full-speed, full-bag ROS validation in an isolated localhost domain.

Runs preview.py --once on domain 85 / bridge port 8766. No robot commands.
"""
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from camera_bev.geometry import BEVMapper, read_yaml


def stamp(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def main():
    root = Path(__file__).resolve().parents[1]
    output = Path(os.environ.get('BEV_CHECK_OUTPUT', str(Path.home() / 'bags/reviews/normal_bev_20261028/ros_check')))
    output.mkdir(parents=True, exist_ok=False)
    bag = Path(os.environ.get('BEV_CHECK_BAG', str(Path.home() / 'bags/normal')))
    assert os.environ.get('ROS_DOMAIN_ID') == '85'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    expected, payloads = [], []
    databases = sorted(bag.glob('*.db3'))
    if len(databases) != 1:
        raise ValueError('Verification expects one SQLite3 bag segment')
    con = sqlite3.connect(databases[0].resolve().as_uri() + '?mode=ro', uri=True)
    for (blob,) in con.execute('SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id WHERE t.name=? ORDER BY timestamp', ('/camera_raw_16_9',)):
        msg = deserialize_message(blob, CompressedImage)
        expected.append(stamp(msg))
        payloads.append(hashlib.sha256(bytes(msg.data)).hexdigest())
    con.close()
    if not expected:
        raise ValueError('No camera images in bag')
    sample_indices = set(np.linspace(0, len(expected) - 1, min(4, len(expected))).round().astype(int))
    expected_indices = {key: i for i, key in enumerate(expected)}
    cv2.setNumThreads(1)
    mapper = BEVMapper(read_yaml(root / 'config/camera_1280x720.yaml'), read_yaml(root / 'config/estimated.yaml'))
    rclpy.init()
    node = rclpy.create_node('bev_recording_validation')
    until = time.monotonic() + 1.0
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
    occupied = [name for name in node.get_node_names() if name != node.get_name()]
    if occupied:
        raise RuntimeError('Test domain is occupied: ' + repr(occupied))
    received = {}
    errors = []
    status = []
    sample_results = {}
    raw_samples = {}
    output_samples = {}
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    input_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = []

    def receive_raw(msg):
        key = stamp(msg)
        received['/camera_raw_16_9'].append(key)
        idx = expected_indices.get(key)
        if idx is None or hashlib.sha256(bytes(msg.data)).hexdigest() != payloads[idx]:
            errors.append('raw payload/stamp mismatch')
        if idx in sample_indices:
            raw_samples[key] = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)

    received['/camera_raw_16_9'] = []
    subscriptions.append(node.create_subscription(CompressedImage, '/camera_raw_16_9', receive_raw, input_qos))

    def image_callback(topic, msg):
        key = stamp(msg)
        received[topic].append(key)
        rect = 'rect' in topic
        mask = 'mask' in topic
        expected_shape = (720, 1280) if rect else (500, 500)
        if (msg.height, msg.width) != expected_shape or msg.encoding != ('mono8' if mask else 'bgr8'):
            errors.append(topic + ': wrong shape/encoding')
        if msg.header.frame_id != ('camera_optical_frame' if rect else 'camera_ground_estimated'):
            errors.append(topic + ': wrong frame')
        if msg.step != msg.width * (1 if mask else 3) or len(msg.data) != msg.height * msg.step:
            errors.append(topic + ': wrong stride/data length')
        if expected_indices.get(key) in sample_indices:
            output_samples[(key, topic)] = bytes(msg.data)

    for name in ('image', 'debug', 'image_rect', 'valid_mask', 'rect_valid_mask'):
        topic = '/bev/' + name
        received[topic] = []
        subscriptions.append(node.create_subscription(Image, topic, lambda msg, t=topic: image_callback(t, msg), qos))

    def compressed_callback(topic, msg):
        received[topic].append(stamp(msg))
        im = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if im is None or im.shape[:2] != ((720, 1280) if 'rect' in topic else (500, 500)):
            errors.append(topic + ': JPEG decode/shape error')

    for name in ('image', 'debug', 'image_rect'):
        topic = '/bev/' + name + '/compressed'
        received[topic] = []
        subscriptions.append(node.create_subscription(CompressedImage, topic, lambda msg, t=topic: compressed_callback(t, msg), qos))

    def camera_info(msg):
        received['/bev/camera_info_rect'].append(stamp(msg))
        if (not np.allclose(msg.k, mapper.rect_k.ravel()) or any(msg.d)
                or (msg.width, msg.height) != (1280, 720) or msg.header.frame_id != 'camera_optical_frame'):
            errors.append('rectified CameraInfo mismatch')

    received['/bev/camera_info_rect'] = []
    subscriptions.append(node.create_subscription(CameraInfo, '/bev/camera_info_rect', camera_info, qos))
    subscriptions.append(node.create_subscription(String, '/bev/status', lambda msg: status.append(json.loads(msg.data)), qos))
    command = ['python3', str(root / 'tools/preview.py'), '--once', '--domain', '85', '--port', '8766',
               '--bag', str(bag), '--output', str(output / 'processes')]
    supervisor = None
    graph_checks = []
    start = time.monotonic()
    failure = None
    try:
        supervisor = subprocess.Popen(command, start_new_session=True)
        last_check = start
        while supervisor.poll() is None:
            rclpy.spin_once(node, timeout_sec=.01)
            now = time.monotonic()
            if now - last_check > 3:
                graph_checks.append(dict(wall_s=now-start, cmd_vel_publishers=node.count_publishers('/cmd_vel'),
                                         odom_publishers=node.count_publishers('/odom')))
                print(json.dumps(dict(wall_s=round(now-start, 1), counts={k: len(v) for k, v in received.items()})), flush=True)
                last_check = now
            if now - start > 260:
                raise TimeoutError('Full bag verification exceeded 260 s')
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
        for key, raw in raw_samples.items():
            bev = mapper.render(raw)
            reference = {'/bev/image': bev, '/bev/debug': mapper.debug(bev), '/bev/image_rect': mapper.rectify(raw),
                         '/bev/valid_mask': mapper.valid_mask, '/bev/rect_valid_mask': mapper.rect_valid_mask}
            for topic, image in reference.items():
                matches = output_samples.get((key, topic)) == image.tobytes()
                sample_results[str(key) + topic] = matches
            cv2.imwrite(str(output / ('received_{:04d}.png'.format(expected_indices[key] + 1))), bev)
    except BaseException as error:
        failure = repr(error)
        raise
    finally:
        if supervisor is not None and supervisor.poll() is None:
            supervisor.send_signal(signal.SIGINT)
            try:
                supervisor.wait(timeout=25)
            except subprocess.TimeoutExpired:
                supervisor.terminate()
                supervisor.wait(timeout=10)
        summary = dict(expected=len(expected), topics={topic: dict(received=len(keys),
                           exact_stamp_sequence=keys == expected,
                           missing=sum((Counter(expected) - Counter(keys)).values()),
                           missing_frame_indices=[expected_indices[k] + 1 for k in expected if k not in set(keys)],
                           unexpected=sum((Counter(keys) - Counter(expected)).values())) for topic, keys in received.items()},
                       errors=errors, sample_pixels_equal_offline=sample_results,
                       graph_checks=graph_checks, last_status=status[-1] if status else None,
                       supervisor_exit=supervisor.returncode if supervisor else None,
                       runtime=os.environ.get('ROS_DISTRO'), wall_s=time.monotonic()-start, failure=failure)
        summary['success'] = (failure is None and not errors and supervisor.returncode == 0
                              and all(keys == expected for keys in received.values())
                              and len(sample_results) == 5 * len(sample_indices) and all(sample_results.values())
                              and bool(graph_checks) and all(g['cmd_vel_publishers'] == g['odom_publishers'] == 0 for g in graph_checks))
        (output / 'result.json').write_text(json.dumps(summary, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(summary, indent=2), flush=True)
    if not summary['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
