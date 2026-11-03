"""ROS message/parameter contract; run after sourcing ROS and the package overlay."""
from array import array
from pathlib import Path

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage

from camera_bev.node import BEVNode


@pytest.fixture(scope='module')
def node():
    rclpy.init(args=['--ros-args', '-p', 'settings_file:=' + str(
        Path(__file__).resolve().parents[1] / 'config/initial_20261028.yaml')])
    instance = BEVNode()
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_atomic_invalid_pose_keeps_previous_map(node):
    previous = node.mapper
    result = node.set_parameters_atomically([Parameter('ground.height_m', value=-1.0)])
    assert not result.successful
    assert node.mapper is previous
    assert node.get_parameter('ground.height_m').value == .12


def test_valid_parameter_rebuild_and_restore(node):
    previous = node.mapper
    result = node.set_parameters_atomically([Parameter('ground.height_m', value=.15)])
    assert result.successful and node.mapper is not previous
    assert node.mapper.center[2] == .15
    result = node.set_parameters_atomically([Parameter('ground.height_m', value=.12)])
    assert result.successful


def test_bad_jpeg_and_wrong_size_are_rejected(node):
    before = node.rejected
    msg = CompressedImage(data=array('B', b'not a jpeg'))
    node.on_image(msg)
    _, encoded = cv2.imencode('.jpg', np.zeros((480, 640, 3), np.uint8))
    msg.data = array('B', encoded.tobytes())
    node.on_image(msg)
    assert node.rejected == before + 2


def test_original_stamp_and_backward_bag_loop_accepted(node):
    _, encoded = cv2.imencode('.jpg', np.zeros((720, 1280, 3), np.uint8))
    msg = CompressedImage(data=array('B', encoded.tobytes()))
    before = node.processed
    for sec in (20, 21, 1, 2):
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = 12345
        node.on_image(msg)
        assert node.last_stamp == [sec, 12345]
    assert node.processed == before + 4


def test_checkerboard_profile_freezes_pose_but_allows_output_crop(node):
    # A fresh process below exercises actual declarations with the default profile.
    import subprocess
    import sys
    script = '''
import rclpy
from rclpy.parameter import Parameter
from camera_bev.node import BEVNode
rclpy.init(args=[])
n = BEVNode()
try:
    assert n.mapper.ground_homography is not None
    assert n.describe_parameter('ground.height_m').read_only
    previous = n.mapper
    result = n.set_parameters_atomically([Parameter('ground.height_m', value=.15)])
    assert not result.successful and n.mapper is previous
    assert n.get_parameter('ground.height_m').value == .135
    assert n.set_parameters_atomically([Parameter('output.x_max_m', value=.8)]).successful
    assert n.mapper.height == 400 and n.mapper.ground_homography is not None
finally:
    n.destroy_node()
    rclpy.shutdown()
'''
    subprocess.run([sys.executable, '-c', script], check=True, timeout=20)
