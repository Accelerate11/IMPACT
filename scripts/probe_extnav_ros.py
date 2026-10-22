#!/usr/bin/env python3
"""Observe the disarmed ExternalNav communication chain; never command flight.

Run from source after sourcing ROS Humble. No package installation is required.
Only --source synthetic publishes: static zero-motion Odometry at 10 wall Hz,
stamped from a received /clock after connected+disarmed has been observed.
All output paths must be new. This diagnostic is not a localization experiment.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import struct
import sys
import time

import rclpy
from mavros_msgs.msg import Mavlink, State
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String


TOPICS = {
    'source_received': '/localization/odom',
    'adapter_received': '/uav1/mavros/odometry/out',
    'adapter_status': '/xq/p4/extnav/status',
    'mavlink_sink': '/uas1/mavlink_sink',
    'mavlink_source': '/uas1/mavlink_source',
    'fcu_state': '/uav1/mavros/state',
    'clock': '/clock',
}
KEPT_MAVLINK_IDS = (0, 2, 111, 331)


def json_safe(value):
    """Retain nonfinite observations as explicit strings, never invent numbers."""
    if isinstance(value, float) and not math.isfinite(value):
        return 'nan' if math.isnan(value) else 'inf' if value > 0 else '-inf'
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


def write_new_json(path: Path, value):
    with path.open('x', encoding='utf-8') as handle:
        json.dump(json_safe(value), handle, indent=2, allow_nan=False)
        handle.write('\n')


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def header_record(header):
    return {'stamp': {'sec': int(header.stamp.sec), 'nanosec': int(header.stamp.nanosec)},
            'frame_id': str(header.frame_id)}


def vector_record(vector):
    return {'x': float(vector.x), 'y': float(vector.y), 'z': float(vector.z)}


def odometry_record(message):
    pose = message.pose.pose
    return {
        'header': header_record(message.header), 'child_frame_id': message.child_frame_id,
        'pose': {'pose': {'position': vector_record(pose.position), 'orientation': {
            key: float(getattr(pose.orientation, key)) for key in ('x', 'y', 'z', 'w')}},
                 'covariance': list(message.pose.covariance)},
        'twist': {'twist': {'linear': vector_record(message.twist.twist.linear),
                            'angular': vector_record(message.twist.twist.angular)},
                  'covariance': list(message.twist.covariance)},
    }


def decode_mavlink_payload(msgid: int, payload: bytes):
    """Decode official MAVLink little-endian wire layout, including v2 trimming.

    Original payload64/len remain in each record. Padding only represents omitted
    zero wire bytes in MAVLink2; it is never substituted for missing measurements.
    Layout is verified against the installed generated common message headers.
    """
    if msgid == 331:
        if len(payload) < 1 or len(payload) > 233:
            raise ValueError('ODOMETRY payload length outside MAVLink2 bounds')
        data = payload.ljust(233, b'\0')
        return {
            'time_usec': struct.unpack_from('<Q', data, 0)[0],
            'x': struct.unpack_from('<f', data, 8)[0],
            'y': struct.unpack_from('<f', data, 12)[0],
            'z': struct.unpack_from('<f', data, 16)[0],
            'q': list(struct.unpack_from('<4f', data, 20)),
            'vx': struct.unpack_from('<f', data, 36)[0],
            'vy': struct.unpack_from('<f', data, 40)[0],
            'vz': struct.unpack_from('<f', data, 44)[0],
            'rollspeed': struct.unpack_from('<f', data, 48)[0],
            'pitchspeed': struct.unpack_from('<f', data, 52)[0],
            'yawspeed': struct.unpack_from('<f', data, 56)[0],
            'pose_covariance': list(struct.unpack_from('<21f', data, 60)),
            'velocity_covariance': list(struct.unpack_from('<21f', data, 144)),
            'frame_id': data[228], 'child_frame_id': data[229],
            'reset_counter': data[230], 'estimator_type': data[231],
            'quality': struct.unpack_from('<b', data, 232)[0],
        }
    if msgid == 111:
        data = payload.ljust(18, b'\0')
        return {'tc1': struct.unpack_from('<q', data, 0)[0],
                'ts1': struct.unpack_from('<q', data, 8)[0],
                'target_system': data[16], 'target_component': data[17]}
    if msgid == 2:
        data = payload.ljust(12, b'\0')
        return {'time_unix_usec': struct.unpack_from('<Q', data, 0)[0],
                'time_boot_ms': struct.unpack_from('<I', data, 8)[0]}
    if msgid == 0:
        data = payload.ljust(9, b'\0')
        return {'custom_mode': struct.unpack_from('<I', data, 0)[0],
                'type': data[4], 'autopilot': data[5], 'base_mode': data[6],
                'system_status': data[7], 'mavlink_version': data[8]}
    raise ValueError('Payload type is outside the diagnostic allowlist')


class ExternalNavProbe(Node):
    def __init__(self, output: Path, source: str, pause_after_s=0.0, pause_duration_s=0.0):
        super().__init__('impact_extnav_disarmed_probe')
        self.output, self.source_mode = output, source
        self.started_wall_ns, self.started_monotonic_ns = time.time_ns(), time.monotonic_ns()
        self.events = (output / 'events.jsonl').open('x', encoding='utf-8', buffering=1)
        self.counts = Counter({kind: 0 for kind in (*TOPICS, 'source_published')})
        self.mavlink_id_counts = Counter()
        self.decode_errors = Counter()
        self.synthetic_skips = Counter()
        self.stop_reason = ''
        self.armed_seen = False
        self.connected_seen = False
        self.disarmed_seen = False
        self.connected_disarmed_seen = False
        self.current_connected = False
        self.current_armed = None
        self.clock_seen = False
        self.latest_clock_ns = 0
        self.last_published_stamp_ns = 0
        self.synthetic_first_publish_ns = None
        self.pause_after_ns = int(pause_after_s*1e9)
        self.pause_duration_ns = int(pause_duration_s*1e9)
        self.pause_started = self.pause_ended = False
        self.clock_regressions = 0
        self.event_sequence = 0
        self.subscriptions_owned = []
        sensor = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned.append(self.create_subscription(
            Odometry, TOPICS['source_received'],
            lambda value: self._odometry('source_received', value), sensor))
        self.subscriptions_owned.append(self.create_subscription(
            Odometry, TOPICS['adapter_received'],
            lambda value: self._odometry('adapter_received', value), sensor))
        self.subscriptions_owned.append(self.create_subscription(
            String, TOPICS['adapter_status'], self._adapter_status, sensor))
        for kind in ('mavlink_sink', 'mavlink_source'):
            self.subscriptions_owned.append(self.create_subscription(
                Mavlink, TOPICS[kind], lambda value, key=kind: self._mavlink(key, value), sensor))
        self.subscriptions_owned.append(self.create_subscription(
            State, TOPICS['fcu_state'], self._state, sensor))
        self.subscriptions_owned.append(self.create_subscription(
            ClockMessage, TOPICS['clock'], self._on_clock, sensor))
        self.source_publisher = None
        self.synthetic_timer = None
        if source == 'synthetic':
            self.source_publisher = self.create_publisher(
                Odometry, TOPICS['source_received'],
                QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.synthetic_timer = self.create_timer(
                0.1, self._publish_synthetic, clock=self.steady_clock)

    def record(self, kind, message, **fields):
        self.event_sequence += 1
        self.counts[kind] += 1
        record = {
            'schema_version': 1, 'event_sequence': self.event_sequence,
            'kind': kind, 'topic': TOPICS.get(kind, TOPICS['source_received']),
            'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
            **fields, 'message': message,
        }
        self.events.write(json.dumps(json_safe(record), separators=(',', ':'), allow_nan=False) + '\n')

    def _odometry(self, kind, message):
        self.record(kind, odometry_record(message), stamp_ns=stamp_ns(message.header.stamp),
                    frame_id=message.header.frame_id, child_frame_id=message.child_frame_id)

    def _adapter_status(self, message):
        record = {'data': message.data}
        try:
            record['parsed'] = json.loads(message.data)
        except (ValueError, TypeError) as error:
            record['parse_error'] = str(error)
        self.record('adapter_status', record)

    def _mavlink(self, kind, message):
        msgid = int(message.msgid)
        self.mavlink_id_counts[f'{kind}:{msgid}'] += 1
        if msgid not in KEPT_MAVLINK_IDS:
            return
        fields = {name: int(getattr(message, name)) for name in (
            'framing_status', 'magic', 'len', 'incompat_flags', 'compat_flags',
            'seq', 'sysid', 'compid', 'msgid', 'checksum')}
        fields['payload64'] = [int(value) for value in message.payload64]
        fields['signature'] = [int(value) for value in message.signature]
        raw = {'header': header_record(message.header), **fields}
        try:
            packed = b''.join(struct.pack('<Q', value) for value in fields['payload64'])
            if len(packed) < fields['len']:
                raise ValueError('payload64 does not cover declared wire length')
            payload = packed[:fields['len']]
            raw['payload_hex'] = payload.hex()
            decoded = decode_mavlink_payload(msgid, payload)
            raw['decoded'] = decoded
            if msgid == 331:
                fields.update({key: decoded[key] for key in ('time_usec', 'frame_id', 'child_frame_id')})
        except (ValueError, struct.error, OverflowError) as error:
            self.decode_errors[str(msgid)] += 1
            raw['decode_error'] = str(error)
        self.record(kind, raw, **fields)

    def _state(self, message):
        self.connected_seen |= bool(message.connected)
        self.disarmed_seen |= not bool(message.armed)
        self.connected_disarmed_seen |= bool(message.connected) and not bool(message.armed)
        self.current_connected, self.current_armed = bool(message.connected), bool(message.armed)
        self.armed_seen |= bool(message.armed)
        self.record('fcu_state', {'header': header_record(message.header),
            'connected': bool(message.connected), 'armed': bool(message.armed),
            'guided': bool(message.guided), 'manual_input': bool(message.manual_input),
            'mode': message.mode, 'system_status': int(message.system_status)},
            connected=bool(message.connected), armed=bool(message.armed))
        if message.armed:
            self.stop_reason = 'armed_state_observed'

    def _on_clock(self, message):
        value = stamp_ns(message.clock)
        if message.clock.sec < 0 or message.clock.nanosec >= 1_000_000_000:
            self.stop_reason = 'invalid_clock_observed'
        if self.clock_seen and value < self.latest_clock_ns:
            self.clock_regressions += 1
            self.stop_reason = 'clock_regression_observed'
        self.clock_seen = True
        self.latest_clock_ns = value
        self.record('clock', {'clock': {'sec': int(message.clock.sec),
                                      'nanosec': int(message.clock.nanosec)}}, stamp_ns=value)

    def _publish_synthetic(self):
        if self.stop_reason or self.armed_seen:
            return
        if not self.current_connected or self.current_armed is not False:
            self.synthetic_skips['await_connected_disarmed'] += 1
            return
        if not self.clock_seen:
            self.synthetic_skips['await_clock'] += 1
            return
        stamp = self.latest_clock_ns
        if stamp <= self.last_published_stamp_ns:
            self.synthetic_skips['clock_not_advancing'] += 1
            return
        now_ns = time.monotonic_ns()
        if self.synthetic_first_publish_ns is None:
            self.synthetic_first_publish_ns = now_ns
        elapsed = now_ns - self.synthetic_first_publish_ns
        if self.pause_duration_ns and elapsed >= self.pause_after_ns:
            if not self.pause_started:
                self.pause_started = True
                self.record('source_pause_started', {}, pause_duration_ns=self.pause_duration_ns,
                            elapsed_since_first_source_ns=elapsed)
            if elapsed < self.pause_after_ns + self.pause_duration_ns:
                self.synthetic_skips['predeclared_source_pause'] += 1
                return
            if not self.pause_ended:
                self.pause_ended = True
                self.record('source_pause_ended', {}, elapsed_since_first_source_ns=elapsed)
        source = Odometry()
        source.header.stamp.sec, source.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        source.header.frame_id, source.child_frame_id = 'xq_lio_map', 'livox_imu'
        source.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            source.pose.covariance[index] = 0.0025
        for index in (21, 28, 35):
            source.pose.covariance[index] = 0.0012
        for index in (0, 7, 14, 21, 28, 35):
            source.twist.covariance[index] = 0.04
        # Zero position (including z), linear/angular motion are the message defaults.
        self.source_publisher.publish(source)
        self.last_published_stamp_ns = stamp
        self.record('source_published', odometry_record(source), stamp_ns=stamp,
                    frame_id=source.header.frame_id, child_frame_id=source.child_frame_id,
                    diagnostic_synthetic=True)

    def graph(self):
        result = {'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                  'nodes': sorted([{'name': name, 'namespace': namespace}
                      for name, namespace in self.get_node_names_and_namespaces()],
                      key=lambda value: (value['namespace'], value['name'])),
                  'topic_names_and_types': self.get_topic_names_and_types(), 'endpoints': {}}
        for topic in TOPICS.values():
            endpoints = {}
            for kind, getter in (('publishers', self.get_publishers_info_by_topic),
                                 ('subscribers', self.get_subscriptions_info_by_topic)):
                endpoints[kind] = []
                for item in getter(topic):
                    qos = item.qos_profile
                    endpoints[kind].append({
                        'node_name': item.node_name, 'node_namespace': item.node_namespace,
                        'topic_type': item.topic_type, 'endpoint_gid': list(item.endpoint_gid),
                        'qos': {'history': str(qos.history), 'depth': qos.depth,
                                'reliability': str(qos.reliability), 'durability': str(qos.durability),
                                'deadline_ns': qos.deadline.nanoseconds,
                                'lifespan_ns': qos.lifespan.nanoseconds,
                                'liveliness': str(qos.liveliness),
                                'liveliness_lease_duration_ns': qos.liveliness_lease_duration.nanoseconds},
                    })
            result['endpoints'][topic] = endpoints
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--source', choices=('synthetic', 'lio'), required=True)
    parser.add_argument('--pause-after-s', type=float, default=0.0)
    parser.add_argument('--pause-duration-s', type=float, default=0.0)
    args, ros_args = parser.parse_known_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if any(not math.isfinite(v) or v < 0 for v in (args.pause_after_s, args.pause_duration_s)):
        parser.error('Pause times must be finite and nonnegative')
    if args.pause_duration_s and (args.source != 'synthetic' or args.pause_after_s <= 0 or
                                 args.pause_after_s+args.pause_duration_s >= args.duration):
        parser.error('A source pause must be strictly inside a synthetic diagnostic')
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    write_new_json(output / 'metadata.json', {
        'schema_version': 1, 'classification': 'DISARMED_COMMUNICATION_DIAGNOSTIC_ONLY',
        'formal_localization_or_fusion_evidence': False, 'source_mode': args.source,
        'duration_wall_s': args.duration, 'synthetic_rate_wall_hz': 10.0,
        'source_pause_after_s': args.pause_after_s, 'source_pause_duration_s': args.pause_duration_s,
        'synthetic_waits_for_clock_and_connected_disarmed': True,
        'synthetic_pose_and_velocity_zero': True,
        'clock_domains': {'source_stamp': 'received_gazebo_clock',
                         'adapter_stamp': 'adapter_mavros_transport_wall_clock',
                         'observation_timestamps': 'observer_callback_wall_and_monotonic'},
        'observer_qos': {'reliability': 'BEST_EFFORT', 'depth': 1000},
        'topics': TOPICS, 'mavlink_ids_kept': list(KEPT_MAVLINK_IDS),
        'independent_observer_is_not_endpoint_receive_proof': True,
        'service_clients_created': False, 'flight_command_publishers_created': False,
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION'),
        'FASTDDS_DEFAULT_PROFILES_FILE': os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE'),
        'FASTRTPS_DEFAULT_PROFILES_FILE': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE'),
        'official_sources': [
            'https://github.com/mavlink/mavros/blob/2.14.0/mavros_extras/src/plugins/odom.cpp',
            'https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/mavros_uas.cpp',
            'https://github.com/mavlink/mavros/blob/2.14.0/mavros/src/lib/mavros_router.cpp',
            'https://mavlink.io/en/messages/common.html#ODOMETRY'],
    })
    received_signal = []
    def handle_signal(signum, frame):
        received_signal.append(signum)
    previous_handlers = {sig: signal.signal(sig, handle_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    probe = None
    completed = False
    error = ''
    graph_errors = []
    ros_initialized = False
    started_monotonic_ns = 0
    finished_monotonic_ns = 0
    try:
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        ros_initialized = True
        probe = ExternalNavProbe(output, args.source, args.pause_after_s, args.pause_duration_s)
        write_new_json(output / 'graph.before.json', probe.graph())
        started_monotonic_ns = time.monotonic_ns()
        deadline = started_monotonic_ns / 1e9 + args.duration
        while rclpy.ok() and not received_signal and not probe.stop_reason:
            if time.monotonic() >= deadline:
                completed = True
                break
            rclpy.spin_once(probe, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        finished_monotonic_ns = time.monotonic_ns()
    except (Exception, ExternalShutdownException) as exception:
        error = f'{type(exception).__name__}: {exception}'
    finally:
        if not finished_monotonic_ns:
            finished_monotonic_ns = time.monotonic_ns()
        if probe is not None:
            try:
                write_new_json(output / 'graph.after.json', probe.graph())
            except Exception as exception:
                graph_errors.append(f'{type(exception).__name__}: {exception}')
            probe.events.flush()
            probe.events.close()
        checks = {
            'duration_completed': completed, 'no_error': not error,
            'graph_snapshots_complete': not graph_errors and (output / 'graph.before.json').exists(),
            'connected_disarmed_seen': bool(probe and probe.connected_disarmed_seen),
            'armed_never_seen': bool(probe and not probe.armed_seen),
            'no_clock_regression': bool(probe and probe.clock_regressions == 0),
            'no_signal_interruption': not received_signal,
            'source_mode_contract': bool(probe and (
                probe.counts['source_published'] > 0 if args.source == 'synthetic'
                else probe.source_publisher is None)),
        }
        passed = all(checks.values())
        summary = {
            'schema_version': 1, 'status': 'PASS' if passed else 'FAIL',
            'pass_meaning': 'Observer completed safely; does not assert link delivery, FCU health, localization, or fusion success',
            'generated_at_utc': datetime.now(timezone.utc).isoformat(),
            'source_mode': args.source, 'duration_requested_wall_s': args.duration,
            'started_monotonic_ns': started_monotonic_ns,
            'finished_monotonic_ns': finished_monotonic_ns,
            'elapsed_wall_s': (finished_monotonic_ns - started_monotonic_ns) / 1e9 if started_monotonic_ns else 0,
            'counts': dict(probe.counts) if probe else {},
            'mavlink_id_counts': dict(probe.mavlink_id_counts) if probe else {},
            'decode_errors': dict(probe.decode_errors) if probe else {},
            'synthetic_skips': dict(probe.synthetic_skips) if probe else {},
            'armed_seen': bool(probe and probe.armed_seen),
            'connected_seen': bool(probe and probe.connected_seen),
            'disarmed_seen': bool(probe and probe.disarmed_seen),
            'connected_disarmed_seen': bool(probe and probe.connected_disarmed_seen),
            'stop_reason': probe.stop_reason if probe else 'node_not_created',
            'signals': received_signal, 'error': error, 'graph_errors': graph_errors,
            'checks': checks, 'output_directory': str(output),
        }
        write_new_json(output / 'summary.json', summary)
        if probe is not None:
            probe.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        print(json.dumps(summary, separators=(',', ':'), allow_nan=False), flush=True)
    return 0 if passed else 130 if received_signal else 2


if __name__ == '__main__':
    sys.exit(main())
