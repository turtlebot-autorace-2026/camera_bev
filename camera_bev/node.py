"""CompressedImage input -> calibrated BEV and independent diagnostic images."""
from copy import deepcopy
from array import array
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String

from .geometry import BEVMapper, read_yaml


class BEVNode(Node):
    def __init__(self):
        super().__init__('camera_bev')
        share = Path(get_package_share_directory('camera_bev'))
        readonly = ParameterDescriptor(read_only=True)

        def fixed(name, default):
            return self.declare_parameter(name, default, readonly).value

        calibration_file = fixed('calibration_file', str(share / 'config/camera_1280x720.yaml'))
        settings_file = fixed('settings_file', str(share / 'config/estimated.yaml'))
        self.input_topic = fixed('input_topic', '/camera_raw_16_9')
        self.optical_frame = fixed('optical_frame_id', 'camera_optical_frame')
        self.ground_frame = fixed('ground_frame_id', 'camera_ground_estimated')
        self.jpeg_quality = int(fixed('jpeg_quality', 85))
        output_depth = int(fixed('output_qos_depth', 5))
        if not 1 <= output_depth <= 50:
            raise ValueError('output_qos_depth must be in [1,50]')
        cv2.setNumThreads(int(fixed('opencv_threads', 1)))
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be in [1,100]')
        self.calibration = read_yaml(calibration_file)
        self.settings = read_yaml(settings_file)
        self.geometry_parameters = {}
        for group in ('ground', 'output', 'rectification'):
            for key, value in self.settings[group].items():
                name = group + '.' + key
                descriptor = (ParameterDescriptor(read_only=True,
                              description='Ground reference is fixed for the checkerboard projection; recalibrate to change it')
                              if group == 'ground' and self.settings.get('ground_projection') else ParameterDescriptor())
                self.settings[group][key] = self.declare_parameter(name, value, descriptor).value
                self.geometry_parameters[name] = (group, key)
        self.settings['max_normalized_radius'] = self.declare_parameter(
            'max_normalized_radius', self.settings['max_normalized_radius']).value
        self.mapper = BEVMapper(self.calibration, self.settings)
        qos = QoSProfile(depth=output_depth, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.image_pubs = {name: self.create_publisher(Image, '/bev/' + name, qos)
                           for name in ('image', 'image_rect', 'debug', 'valid_mask', 'rect_valid_mask')}
        self.compressed_pubs = {name: self.create_publisher(CompressedImage, '/bev/' + name + '/compressed', qos)
                                for name in ('image', 'image_rect', 'debug')}
        self.info_pub = self.create_publisher(CameraInfo, '/bev/camera_info_rect', qos)
        self.status_pub = self.create_publisher(String, '/bev/status', qos)
        sensor_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(CompressedImage, self.input_topic, self.on_image, sensor_qos)
        self.processed = 0
        self.rejected = 0
        self.last_duration_ms = 0.0
        self.last_error_log = float('-inf')
        self.last_stamp = None
        self.add_on_set_parameters_callback(self.on_parameters)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Direct raw -> BEV ready: ' + json.dumps(self.mapper.info()))

    def on_parameters(self, parameters):
        settings = deepcopy(self.settings)
        changed = False
        for parameter in parameters:
            if parameter.name in self.geometry_parameters:
                group, key = self.geometry_parameters[parameter.name]
                settings[group][key] = parameter.value
                changed = True
            elif parameter.name == 'max_normalized_radius':
                settings[parameter.name] = parameter.value
                changed = True
        if changed:
            try:
                mapper = BEVMapper(self.calibration, settings)
            except (ValueError, TypeError, OverflowError, cv2.error) as error:
                return SetParametersResult(successful=False, reason=str(error))
            self.settings, self.mapper = settings, mapper
            self.get_logger().info('Projection maps updated; measured=' + str(settings['ground']['measured']))
        return SetParametersResult(successful=True)

    def wanted(self, name):
        return (self.image_pubs[name].get_subscription_count() > 0
                or (name in self.compressed_pubs and self.compressed_pubs[name].get_subscription_count() > 0))

    def publish_image(self, name, pixels, stamp, frame_id):
        publisher = self.image_pubs[name]
        if publisher.get_subscription_count():
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.height, msg.width = pixels.shape[:2]
            msg.encoding = 'bgr8' if pixels.ndim == 3 else 'mono8'
            msg.is_bigendian = 0
            msg.step = pixels.shape[1] * (3 if pixels.ndim == 3 else 1)
            # Generated ROS setters fast-path array('B'); bytes otherwise triggers
            # a Python assertion loop over millions of pixels on Galactic/Humble.
            msg.data = array('B', np.ascontiguousarray(pixels).tobytes())
            publisher.publish(msg)
        compressed = self.compressed_pubs.get(name)
        if compressed is not None and compressed.get_subscription_count():
            ok, data = cv2.imencode('.jpg', pixels, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                raise ValueError('JPEG encoder failed')
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            msg.format = 'bgr8; jpeg compressed bgr8'
            msg.data = array('B', data.tobytes())
            compressed.publish(msg)

    def publish_camera_info(self, stamp, frame_id):
        if not self.info_pub.get_subscription_count():
            return
        msg = CameraInfo()
        msg.header.stamp, msg.header.frame_id = stamp, frame_id
        msg.width = self.mapper.rect_valid_mask.shape[1]
        msg.height = self.mapper.rect_valid_mask.shape[0]
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0] * 5
        msg.k = self.mapper.rect_k.ravel().tolist()
        msg.r = np.eye(3).ravel().tolist()
        msg.p = np.c_[self.mapper.rect_k, np.zeros(3)].ravel().tolist()
        self.info_pub.publish(msg)

    def on_image(self, msg):
        start = time.perf_counter()
        try:
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
            self.mapper.check_image(image)
            # One sampling step for BEV. The rectification branch is independent.
            if self.wanted('image') or self.wanted('debug'):
                bev = self.mapper.render(image)
                self.publish_image('image', bev, msg.header.stamp, self.ground_frame)
                if self.wanted('debug'):
                    self.publish_image('debug', self.mapper.debug(bev), msg.header.stamp, self.ground_frame)
            optical_frame = msg.header.frame_id or self.optical_frame
            if self.wanted('image_rect'):
                self.publish_image('image_rect', self.mapper.rectify(image), msg.header.stamp, optical_frame)
            self.publish_image('valid_mask', self.mapper.valid_mask, msg.header.stamp, self.ground_frame)
            self.publish_image('rect_valid_mask', self.mapper.rect_valid_mask, msg.header.stamp, optical_frame)
            self.publish_camera_info(msg.header.stamp, optical_frame)
            self.processed += 1
            self.last_stamp = [msg.header.stamp.sec, msg.header.stamp.nanosec]
        except (ValueError, cv2.error) as error:
            self.rejected += 1
            if time.monotonic() - self.last_error_log > 2:
                self.get_logger().error('Rejected image: ' + str(error))
                self.last_error_log = time.monotonic()
        self.last_duration_ms = (time.perf_counter() - start) * 1000

    def publish_status(self):
        data = self.mapper.info()
        data.update(processed=self.processed, rejected=self.rejected,
                    last_processing_ms=self.last_duration_ms, input_topic=self.input_topic,
                    last_input_stamp=self.last_stamp, ground_frame_id=self.ground_frame)
        self.status_pub.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BEVNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
