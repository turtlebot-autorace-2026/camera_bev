#!/usr/bin/env python3
"""Verify actual Foxglove WebSocket advertisements, subscriptions and CDR images.

Standard-library WebSocket client plus ROS deserialization. No messages are
published to ROS. Accepts both current SDK and legacy Foxglove subprotocols.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import time

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class WebSocket:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=3)
        self.buffer = bytearray()
        key = base64.b64encode(os.urandom(16)).decode()
        request = ('GET / HTTP/1.1\r\nHost: {}:{}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                   'Sec-WebSocket-Key: {}\r\nSec-WebSocket-Version: 13\r\n'
                   'Sec-WebSocket-Protocol: foxglove.websocket.v1, foxglove.sdk.v1\r\n\r\n').format(host, port, key)
        self.socket.sendall(request.encode())
        while b'\r\n\r\n' not in self.buffer:
            self.buffer.extend(self.socket.recv(4096))
        headers, rest = self.buffer.split(b'\r\n\r\n', 1)
        self.buffer = bytearray(rest)
        assert headers.startswith(b'HTTP/1.1 101 '), headers
        accept = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
        assert accept in headers
        self.headers = headers.decode()

    def read(self, size):
        while len(self.buffer) < size:
            data = self.socket.recv(max(4096, size - len(self.buffer)))
            if not data:
                raise EOFError('WebSocket closed')
            self.buffer.extend(data)
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def send(self, payload, opcode=1):
        mask = os.urandom(4)
        size = len(payload)
        prefix = bytes([0x80 | opcode])
        if size < 126:
            prefix += bytes([0x80 | size])
        elif size < 65536:
            prefix += bytes([0x80 | 126]) + struct.pack('>H', size)
        else:
            prefix += bytes([0x80 | 127]) + struct.pack('>Q', size)
        self.socket.sendall(prefix + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def receive(self):
        parts = []
        message_opcode = None
        while True:
            first, second = self.read(2)
            opcode, final = first & 15, bool(first & 128)
            size = second & 127
            if size == 126:
                size = struct.unpack('>H', self.read(2))[0]
            elif size == 127:
                size = struct.unpack('>Q', self.read(8))[0]
            mask = self.read(4) if second & 128 else None
            payload = self.read(size)
            if mask:
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
            if opcode == 9:
                self.send(payload, 10)
                continue
            if opcode == 8:
                raise EOFError('Server sent close')
            if opcode == 10:
                continue
            if opcode:
                message_opcode = opcode
            parts.append(payload)
            if final:
                return message_opcode, b''.join(parts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    client = WebSocket(args.host, args.port)
    desired = {'/camera_raw_16_9': CompressedImage, '/bev/image/compressed': CompressedImage,
               '/bev/debug/compressed': CompressedImage, '/bev/image_rect/compressed': CompressedImage,
               '/bev/valid_mask': Image, '/bev/status': String}
    channels = {}
    counts = {topic: 0 for topic in desired}
    subscriptions = {}
    stamp_sets = {topic: set() for topic in desired if topic != '/bev/status'}
    server_info = None
    statuses = []
    end = time.monotonic() + 15
    try:
        while time.monotonic() < end and min(counts.values()) < 3:
            opcode, payload = client.receive()
            if opcode == 1:
                msg = json.loads(payload)
                if msg.get('op') == 'serverInfo':
                    server_info = msg
                elif msg.get('op') == 'advertise':
                    additions = []
                    for channel in msg['channels']:
                        channels[channel['topic']] = {k: v for k, v in channel.items() if k != 'schema'}
                        topic = channel['topic']
                        if topic in desired and topic not in subscriptions.values():
                            sid = len(subscriptions) + 1
                            subscriptions[sid] = topic
                            additions.append(dict(id=sid, channelId=channel['id']))
                    if additions:
                        client.send(json.dumps(dict(op='subscribe', subscriptions=additions)).encode())
                elif msg.get('op') == 'status':
                    statuses.append(msg)
            elif opcode == 2 and payload[0] == 1:
                sid = struct.unpack_from('<I', payload, 1)[0]
                topic = subscriptions[sid]
                msg = deserialize_message(payload[13:], desired[topic])
                counts[topic] += 1
                if topic != '/bev/status':
                    stamp_sets[topic].add((msg.header.stamp.sec, msg.header.stamp.nanosec))
                    if isinstance(msg, CompressedImage):
                        image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                        assert image is not None
                        shape = (720, 1280) if 'rect' in topic or topic == '/camera_raw_16_9' else (500, 500)
                        assert image.shape[:2] == shape
                    else:
                        assert msg.encoding == 'mono8' and (msg.height, msg.width) == (500, 500)
                        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width)
                        assert set(np.unique(image)).issubset({0, 255})
                    if counts[topic] == 1:
                        cv2.imwrite(str(args.output / (topic.strip('/').replace('/', '_') + '.png')), image)
        common = set.intersection(*stamp_sets.values())
        capabilities = server_info.get('capabilities', []) if server_info else []
        result = dict(host=args.host, port=args.port, handshake=client.headers, server_info=server_info,
                      channels=channels, received=counts, common_image_stamps=len(common),
                      server_statuses=statuses,
                      no_command_channels=not any(t in channels for t in ('/cmd_vel', '/odom')),
                      no_command_capabilities=not any(c in capabilities for c in ('clientPublish', 'services')))
        result['success'] = (min(counts.values()) >= 3 and len(common) >= 2
                             and result['no_command_channels'] and result['no_command_capabilities'])
        (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
    finally:
        client.send(b'', 8)
        client.socket.close()
    if not result['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
