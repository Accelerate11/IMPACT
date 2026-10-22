"""Observe the R2 simulation data path; never publish navigation or control."""
from __future__ import annotations

from collections import Counter, deque
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .multisensor_contract import validate_contract_config, check_gyro_coverage


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ObservedStream:
    def __init__(self):
        self.count = self.duplicate_or_backward = 0
        self.first_ns = self.last_ns = None
        self.maximum_gap_s = 0.0
        self.ages = []

    def add(self, source_ns, now_ns):
        self.count += 1
        if self.first_ns is None:
            self.first_ns = source_ns
        if self.last_ns is not None:
            if source_ns <= self.last_ns:
                self.duplicate_or_backward += 1
            else:
                self.maximum_gap_s = max(self.maximum_gap_s, (source_ns - self.last_ns) * 1e-9)
        self.last_ns = source_ns
        if now_ns > 0:
            self.ages.append((now_ns - source_ns) * 1e-9)

    def snapshot(self):
        span = (self.last_ns - self.first_ns) * 1e-9 if self.count > 1 else 0.0
        return {
            'count': self.count, 'first_stamp_ns': self.first_ns, 'last_stamp_ns': self.last_ns,
            'rate_hz': (self.count - 1) / span if span > 0 else 0.0,
            'maximum_gap_s': self.maximum_gap_s,
            'duplicate_or_backward': self.duplicate_or_backward,
            'age_p99_s': float(np.percentile(self.ages, 99)) if self.ages else None,
            'age_min_s': min(self.ages) if self.ages else None,
        }


def quaternion_matrix(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.all(np.isfinite(values)) or norm <= 1e-12:
        raise ValueError('invalid transform quaternion')
    x, y, z, w = values / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def record_image_diagnostics(research, latest, key, pixels, phase, source_ns):
    """Freeze the first research frame; later landing pixels remain diagnostic."""
    statistics = {'std': float(pixels.std()), 'mean': float(pixels.mean()),
                  'distinct_values': int(len(np.unique(pixels))),
                  'phase': phase, 'acquisition_stamp_ns': source_ns}
    latest[key] = statistics
    if phase == 'RESEARCH' and key not in research:
        research[key] = statistics.copy()


class MultisensorMonitorNode(Node):
    def __init__(self):
        super().__init__('impact_multisensor_monitor')
        self.declare_parameter('config_file', '')
        self.declare_parameter('result_file', '')
        self.config = json.loads(Path(str(self.get_parameter('config_file').value)).read_text())
        self.validation = validate_contract_config(self.config)
        self.result_file = Path(str(self.get_parameter('result_file').value))
        self.result_file.parent.mkdir(parents=True, exist_ok=True)
        self.streams = {key: ObservedStream() for key in self.config['streams']}
        self.started = time.monotonic()
        self.errors = Counter()
        self.flow_reasons = Counter()
        self.flow_valid_count = self.flow_invalid_count = 0
        self.flow_gyro_verified_count = self.flow_gyro_bad_count = 0
        self.maximum_flow_norm_rad = 0.0
        self.maximum_tracks = 0
        self.gyro_stamps = deque(maxlen=6000)
        self.pending_flow = deque()
        self.source_examples = {}
        self.infos = {}
        self.image_stamps = {key: set() for key in ('front_camera', 'down_camera')}
        self.info_stamps = {key: set() for key in ('front_camera', 'down_camera')}
        self.pixel_stats = {}
        self.latest_pixel_stats = {}
        self.tf_checks = {}
        self.phase = 'WAIT_FCU'
        self.saved_images = set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        sensor = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, self.config['streams']['lidar']['topic'], self._lidar, reliable)
        self.create_subscription(Imu, self.config['streams']['imu']['topic'], self._imu, sensor)
        for key in ('front_camera', 'down_camera'):
            spec = self.config['streams'][key]
            self.create_subscription(Image, spec['topic'], lambda m, k=key: self._image(k, m), reliable)
            self.create_subscription(CameraInfo, spec['camera_info_topic'], lambda m, k=key: self._info(k, m), reliable)
        self.create_subscription(String, self.config['streams']['flow']['topic'], self._flow, reliable)
        self.create_subscription(String, '/impact/ap/mission_status', self._mission, latched)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.write_result, clock=self.steady)

    def _observe(self, key, source_ns, frame_id):
        now_ns = self.get_clock().now().nanoseconds
        self.streams[key].add(source_ns, now_ns)
        if source_ns <= 0:
            self.errors[key + '_zero_stamp'] += 1
        if frame_id != self.config['streams'][key]['frame_id']:
            self.errors[key + '_frame_mismatch'] += 1
        if now_ns > 0 and -0.03 <= (now_ns-source_ns)*1e-9 <= 0.15:
            self.source_examples.setdefault(key, {
                'stamp_ns': source_ns, 'frame_id': frame_id,
                'source_now_ns': now_ns, 'receipt_monotonic_s': time.monotonic(),
            })

    def _lidar(self, msg):
        self._observe('lidar', stamp_ns(msg.header.stamp), msg.header.frame_id)
        if msg.width * msg.height <= 0 or len(msg.data) != msg.height * msg.row_step:
            self.errors['lidar_layout'] += 1

    def _imu(self, msg):
        value = stamp_ns(msg.header.stamp)
        self._observe('imu', value, msg.header.frame_id)
        self.gyro_stamps.append(value * 1e-9)
        if not all(math.isfinite(v) for v in (
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)):
            self.errors['imu_nonfinite'] += 1

    def _image(self, key, msg):
        value = stamp_ns(msg.header.stamp)
        self._observe(key, value, msg.header.frame_id)
        self.image_stamps[key].add(value)
        spec = self.config['streams'][key]['intrinsics']
        if (msg.width != spec['width'] or msg.height != spec['height'] or msg.encoding != 'rgb8'
                or msg.step < msg.width * 3 or len(msg.data) != msg.step * msg.height):
            self.errors[key + '_layout'] += 1
            return
        pixels = np.ndarray((msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data,
                            strides=(msg.step, 3, 1))
        record_image_diagnostics(self.pixel_stats, self.latest_pixel_stats, key, pixels, self.phase, value)
        label = key + ('_research' if self.phase == 'RESEARCH' else '_initial')
        if label not in self.saved_images:
            cv2.imwrite(str(self.result_file.parent / (label + '.png')), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR))
            self.saved_images.add(label)
        if key in self.source_examples:
            self.source_examples[key].update({'width': msg.width, 'height': msg.height, 'encoding': msg.encoding})

    def _info(self, key, msg):
        self.info_stamps[key].add(stamp_ns(msg.header.stamp))
        expected = self.config['streams'][key]['intrinsics']
        observed = (msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        reference = tuple(expected[name] for name in ('width', 'height', 'fx', 'fy', 'cx', 'cy'))
        if not np.allclose(observed, reference, atol=1e-6, rtol=0):
            self.errors[key + '_intrinsics'] += 1
        if msg.header.frame_id != self.config['streams'][key]['frame_id']:
            self.errors[key + '_info_frame'] += 1
        self.infos[key] = {'observed': list(observed), 'reference': list(reference)}

    def _flow(self, msg):
        try:
            data = json.loads(msg.data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
            self._observe('flow', int(data['acquisition_end_ns']), data['frame_id'])
            self.flow_reasons[str(data.get('reason'))] += 1
            self.maximum_tracks = max(self.maximum_tracks, int(data['track_count']))
            if (data.get('ground_truth_used') is not False
                    or data.get('metric_velocity_valid') is not False
                    or data.get('metric_velocity_mps') is not None
                    or data.get('scale_source') != 'unavailable'
                    or set(data.get('dependencies', [])) != {'down_camera', 'livox_imu'}):
                self.errors['flow_provenance_or_unknown_scale'] += 1
            if data['valid'] is True:
                values = data['compensated_integrated_flow_rad']
                if len(values) != 2 or not all(math.isfinite(float(v)) for v in values):
                    raise ValueError('invalid angular flow')
                start, end = int(data['acquisition_start_ns']), int(data['acquisition_end_ns'])
                if (start >= end or data['source_image_stamps_ns'] != [start, end]
                        or abs(float(data['dt_s']) - (end-start)*1e-9) > 1e-8):
                    raise ValueError('integration timestamps inconsistent')
                self.flow_valid_count += 1
                self.maximum_flow_norm_rad = max(self.maximum_flow_norm_rad, math.hypot(*values))
                self.pending_flow.append(data)
                self.source_examples.setdefault('valid_flow', data)
            else:
                self.flow_invalid_count += 1
        except (ValueError, TypeError, KeyError, OverflowError):
            self.errors['flow_schema'] += 1

    def _mission(self, msg):
        try:
            self.phase = json.loads(msg.data)['phase']
        except (ValueError, KeyError, TypeError):
            self.errors['mission_status_parse'] += 1

    def _check_gyro_windows(self):
        while self.pending_flow:
            data = self.pending_flow[0]
            end = data['acquisition_end_ns'] * 1e-9
            if not self.gyro_stamps or self.gyro_stamps[-1] < end:
                break
            self.pending_flow.popleft()
            result = check_gyro_coverage(data['acquisition_start_ns']*1e-9, end,
                                         list(self.gyro_stamps), max_gap_s=0.02)
            # The independent monitor receives the same raw IMU; its buffered
            # interval must support every angular measurement labelled valid.
            passed = result.get('valid', result.get('status') in {'PASS', 'READY'})
            if passed:
                self.flow_gyro_verified_count += 1
                if 'gyro_window' not in self.source_examples:
                    self.source_examples['gyro_window'] = {
                        'start_s': data['acquisition_start_ns'] * 1e-9, 'end_s': end,
                        'stamps_s': [t for t in self.gyro_stamps
                                     if data['acquisition_start_ns']*1e-9-0.02 <= t <= end+0.02],
                    }
            else:
                self.flow_gyro_bad_count += 1

    def _check_transforms(self):
        for key, spec in self.config['streams'].items():
            if key == 'flow':
                continue
            try:
                transform = self.buffer.lookup_transform(self.config['body_frame'], spec['frame_id'], Time())
                expected = spec['body_from_sensor']
                actual = transform.transform
                valid = (np.allclose(quaternion_matrix(actual.rotation), expected['rotation'], atol=1e-6)
                         and np.allclose([actual.translation.x, actual.translation.y, actual.translation.z],
                                         expected['translation_m'], atol=1e-6))
                self.tf_checks[key] = {'valid': bool(valid), 'source': 'ROS static TF vs frozen simulation contract'}
            except (TransformException, ValueError) as exc:
                self.tf_checks[key] = {'valid': False, 'error': str(exc)[:200]}

    def write_result(self):
        self._check_gyro_windows()
        self._check_transforms()
        result = {
            'schema_version': 1, 'scope': 'R2A rendered cameras and angular optical flow shadow contract',
            'config_validation': self.validation, 'wall_elapsed_s': time.monotonic()-self.started,
            'mission_phase': self.phase, 'streams': {k: v.snapshot() for k, v in self.streams.items()},
            'errors': dict(self.errors), 'camera_info': self.infos, 'pixel_stats': self.pixel_stats,
            'latest_pixel_stats': self.latest_pixel_stats,
            'pixel_stats_sampling': 'First image per camera in RESEARCH, frozen before landing; latest pixels separately reported.',
            'camera_info_stamp_match': {
                key: {'image_count': len(self.image_stamps[key]), 'info_count': len(self.info_stamps[key]),
                      'matched': len(self.image_stamps[key] & self.info_stamps[key]),
                      'missing_info': len(self.image_stamps[key] - self.info_stamps[key])}
                for key in self.image_stamps},
            'flow': {'valid_count': self.flow_valid_count, 'invalid_count': self.flow_invalid_count,
                     'gyro_verified_count': self.flow_gyro_verified_count,
                     'gyro_bad_count': self.flow_gyro_bad_count, 'gyro_pending': len(self.pending_flow),
                     'maximum_compensated_norm_rad': self.maximum_flow_norm_rad,
                     'maximum_tracks': self.maximum_tracks, 'reasons': dict(self.flow_reasons)},
            'tf_checks': self.tf_checks, 'source_examples': self.source_examples,
            'hardware_calibration_validated': False, 'metric_flow_fusion_ready': False,
            'publishes_navigation_or_control': False, 'ground_truth_subscribed': False,
            'sampling_note': 'Source-stamp rates and callback-age percentiles; not hardware synchronization accuracy.',
        }
        temporary = self.result_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        temporary.replace(self.result_file)


def main(args=None):
    cv2.setNumThreads(1)
    rclpy.init(args=args)
    node = MultisensorMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
