#!/usr/bin/env python3
"""Synthetic cross-process ROS large-payload deployment check; never runs algorithms."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def role(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField
    rclpy.init()
    node = Node('impact_portable_payload_' + args.role)
    payload = b'\x00\x00\x80\x3f' * 4 * 23040
    expected = hashlib.sha256(payload).hexdigest()
    output = args.output / (args.role + '.json')
    count, bad = 0, 0
    timeline = []
    if args.role == 'subscriber':
        def callback(message):
            nonlocal count, bad
            count += 1
            bad += hashlib.sha256(message.data).hexdigest() != expected
            timeline.append({'count': count, 'monotonic_ns': time.monotonic_ns(), 'stamp': message.header.stamp.sec})
        subscription = node.create_subscription(PointCloud2, '/impact/server/payload_probe', callback,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT))
        (args.output / 'subscriber-ready.json').write_text(json.dumps({'monotonic_ns': time.monotonic_ns(), 'pid': os.getpid()}))
        start = time.monotonic()
        while count < 10 and time.monotonic() - start < 40:
            rclpy.spin_once(node, timeout_sec=.05)
        discovered = node.count_publishers('/impact/server/payload_probe') > 0
    else:
        publisher = node.create_publisher(PointCloud2, '/impact/server/payload_probe',
            QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE))
        start = time.monotonic()
        while publisher.get_subscription_count() == 0 and time.monotonic() - start < 12:
            rclpy.spin_once(node, timeout_sec=.05)
        discovered = publisher.get_subscription_count() > 0
        for index in range(10):
            message = PointCloud2()
            message.header.frame_id = 'synthetic_fixture_only'
            message.header.stamp.sec = index + 1
            message.width, message.height, message.point_step = 23040, 1, 16
            message.row_step = len(payload)
            message.fields = [PointField(name=name, offset=4*i, datatype=7, count=1) for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            message.data = payload
            publisher.publish(message)
            count += 1
            timeline.append({'count': count, 'monotonic_ns': time.monotonic_ns()})
            end = time.monotonic() + .4
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
        end = time.monotonic() + 2
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
    output.write_text(json.dumps({'role': args.role, 'count': count, 'bad_payloads': bad,
        'payload_bytes': len(payload), 'payload_sha256': expected, 'peer_discovered': discovered,
        'timeline': timeline}, indent=2) + '\n')
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--role', choices=('publisher', 'subscriber'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.role:
        role(args)
        return
    args.output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ)
    for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH',
        'ROS_DISTRO', 'ROS_DISCOVERY_SERVER', 'CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE', 'LD_PRELOAD'):
        env.pop(key, None)
    env.update(ROS_DOMAIN_ID='86', ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
        ROS2CLI_DISABLE_DAEMON='1', PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', ROS_LOG_DIR=str(args.output / 'ros-logs'))
    command = ['bash', '-c', 'set -e; source /opt/ros/humble/setup.bash; exec "$@"', 'impact_transport']
    children, logs = {}, []
    try:
        for name in ('subscriber', 'publisher'):
            if name == 'publisher':
                deadline = time.monotonic() + 15
                while not (args.output / 'subscriber-ready.json').exists():
                    if children['subscriber'].poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('Subscriber did not become ready')
                    time.sleep(.05)
            handle = (args.output / (name + '.log')).open('x')
            logs.append(handle)
            children[name] = subprocess.Popen(command + [sys.executable, str(Path(__file__).resolve()), '--role', name, '--output', str(args.output)],
                env=env, cwd=args.output, stdout=handle, stderr=subprocess.STDOUT)
        for process in children.values():
            process.wait(timeout=50)
    finally:
        for process in children.values():
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=5)
        for handle in logs:
            handle.close()
    data = {name: json.loads((args.output / (name + '.json')).read_text()) for name in children}
    checks = {'both_processes_exit_zero': all(process.returncode == 0 for process in children.values()),
        'publisher_discovered_subscriber': data['publisher']['peer_discovered'],
        'ten_messages_received': data['publisher']['count'] == data['subscriber']['count'] == 10,
        'all_payloads_match': data['subscriber']['bad_payloads'] == 0 and data['subscriber']['count'] > 0}
    result = {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks, 'results': data,
        'environment': {key: env[key] for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY', 'RMW_IMPLEMENTATION')},
        'FASTRTPS_DEFAULT_PROFILES_FILE': None, 'scope': 'Synthetic deployment transport only; no Gazebo/AP/algorithm execution.'}
    (args.output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'status': result['status'], 'checks': checks, 'output': str(args.output)}), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
