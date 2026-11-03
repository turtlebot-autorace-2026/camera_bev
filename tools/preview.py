#!/usr/bin/env python3
"""Own and supervise BEV + image-only bag replay + read-only Foxglove bridge.

Ctrl-C/TERM stops only this run's process groups. No shell command evaluation.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path.home() / 'bags/normal')
    parser.add_argument('--domain', type=int, default=84)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--no-bridge', action='store_true')
    parser.add_argument('--settings', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not (args.bag / 'metadata.yaml').is_file():
        parser.error('Bag must contain metadata.yaml')
    if not 1 <= args.domain <= 232 or not 1 <= args.port <= 65535 or args.rate <= 0:
        parser.error('Invalid domain, port or rate')
    output = args.output or Path.home() / 'bags/reviews' / ('bev_preview_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    if not args.no_bridge:
        with socket.socket() as check:
            check.bind(('0.0.0.0', args.port))
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1')
    # These are fixed sensor-only replay arguments. Do not add /cmd_vel.
    player = ['ros2', 'bag', 'play', str(args.bag), '--rate', str(args.rate),
              '--topics', '/camera_raw_16_9', '--clock', '--delay', '3.0']
    help_text = subprocess.check_output(['ros2', 'bag', 'play', '--help'], env=env, text=True)
    if '--disable-keyboard-controls' in help_text:
        player.append('--disable-keyboard-controls')
    if not args.once:
        player.append('--loop')
    processor = ['ros2', 'run', 'camera_bev', 'bev_node', '--ros-args', '-p', 'use_sim_time:=true']
    if args.settings:
        processor += ['-p', 'settings_file:=' + str(args.settings.resolve())]
    commands = [('processor', processor)]
    if not args.no_bridge:
        commands.append(('bridge', ['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge', '--ros-args',
                                   '-p', 'port:=' + str(args.port), '-p', 'address:=0.0.0.0',
                                   '-p', 'topic_whitelist:=["^/camera_raw_16_9$", "^/bev/.*$"]',
                                   '-p', 'capabilities:=["connectionGraph"]',
                                   '-p', 'num_threads:=2', '-p', 'send_buffer_limit:=20000000']))
    commands.append(('player', player))
    children = []
    files = []
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    manifest = dict(supervisor_pid=os.getpid(), ros_distro=env.get('ROS_DISTRO'), domain=args.domain,
                    localhost_only=True, bag=str(args.bag), image_only=True, port=args.port,
                    output=str(output), processes=[], status='starting')
    try:
        for name, command in commands:
            logfile = (output / (name + '.log')).open('w')
            files.append(logfile)
            process = subprocess.Popen(command, env=env, stdin=subprocess.DEVNULL, stdout=logfile,
                                       stderr=subprocess.STDOUT, start_new_session=True)
            children.append((name, process))
            manifest['processes'].append(dict(name=name, pid=process.pid, command=command))
            time.sleep(.5)
        manifest['status'] = 'running'
        (output / 'processes.json').write_text(json.dumps(manifest, indent=2))
        print('BEV preview: ws://<this-machine-ip>:{} | ROS domain {} | {}'.format(args.port, args.domain, output), flush=True)
        while not stopping:
            for name, process in children:
                code = process.poll()
                if code is not None:
                    if name == 'player' and args.once and code == 0:
                        stopping = True
                    else:
                        raise RuntimeError('{} exited with {}. See {}'.format(name, code, output / (name + '.log')))
            time.sleep(.2)
    finally:
        for _, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in reversed(children):
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for logfile in files:
            logfile.close()
        manifest['status'] = 'stopped'
        manifest['return_codes'] = {name: process.returncode for name, process in children}
        (output / 'processes.json').write_text(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()
