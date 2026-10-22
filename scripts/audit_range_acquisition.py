#!/usr/bin/env python3
"""Independent read-only R2B3B1 acquisition/noise audit, never a ROS node.

All targeted ROS2 CDR payloads are deserialized using the installed ROS types,
not a bridge or estimator helper. Closed uncompressed SQLite chunks are opened
mode=ro&immutable=1. Every bag file is hashed before and after this audit.
The noise-free ray is exclusively an evaluation reference, never feature depth.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import statistics
import struct
import sys


TYPE_NAMES = {
    'range_noisy': 'sensor_msgs/msg/Range',
    'range_reference': 'sensor_msgs/msg/Range',
    'imu_covariance': 'sensor_msgs/msg/Imu',
    'imu_raw': 'sensor_msgs/msg/Imu',
    'tf_static': 'tf2_msgs/msg/TFMessage',
    'range_metadata': 'std_msgs/msg/String',
    'range_status': 'std_msgs/msg/String',
    'raw_scan_noisy': 'sensor_msgs/msg/LaserScan',
    'raw_scan_reference': 'sensor_msgs/msg/LaserScan',
    'camera_metadata': 'std_msgs/msg/String',
    'range_source': 'std_msgs/msg/String',
}

PENCIL_STEP_POLICY = ('finite_zero_or_nan_only_for_one_by_one_zero_span_pencil; '
                      'derived_ros_increment_zero; retain_original_fields_per_sample')

REFERENCE_GRAPHS = {
    'range_reference': 'topic-graph_impact_r2b3_eval_range_reference.txt',
    'raw_scan_reference': 'topic-graph_impact_r2b3_eval_reference_scan.txt',
    'range_source': 'topic-graph_impact_r2b3_native_range_source.txt',
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def snapshot(directory):
    """Reject live/aliased storage; retain all metadata and payload file hashes."""
    directory = Path(directory)
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError('bag must be an existing nonsymlink directory')
    result = {}
    for path in sorted(directory.rglob('*')):
        if path.is_symlink():
            raise ValueError('bag symlink is forbidden: ' + str(path))
        if path.is_file():
            if path.name.endswith(('-wal', '-shm', '-journal', '.active')):
                raise ValueError('closed bag required, SQLite sidecar/live artifact: ' + str(path))
            result[path.relative_to(directory).as_posix()] = {
                'bytes': path.stat().st_size, 'sha256': sha256(path)}
    if not result or 'metadata.yaml' not in result:
        raise ValueError('closed rosbag metadata.yaml required')
    return result


def loads(raw):
    def pairs(items):
        value = {}
        for key, item in items:
            if key in value:
                raise ValueError('duplicate JSON key: ' + key)
            value[key] = item
        return value
    value = json.loads(raw, object_pairs_hook=pairs,
                       parse_constant=lambda item: (_ for _ in ()).throw(ValueError(item)))
    def visit(item):
        if isinstance(item, float) and not math.isfinite(item):
            raise ValueError('nonfinite JSON number')
        if isinstance(item, dict):
            for child in item.values():
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)
    visit(value)
    return value


def number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def require_number(value, name, positive=False):
    if not number(value) or positive and value <= 0:
        raise ValueError('invalid numerical ' + name)
    return float(value)


def validate_protocol(protocol):
    if not isinstance(protocol, dict) or protocol.get('schema_version') != 1:
        raise ValueError('unsupported protocol schema')
    if protocol.get('phase') != 'R2B3B1_RANGE_AND_NOISE_ACQUISITION':
        raise ValueError('unexpected experiment phase')
    if type(protocol.get('protocol_revision')) is not int or protocol['protocol_revision'] != 2:
        raise ValueError('native source-ledger audit requires protocol revision 2')
    required = {
        'predeclared_before_first_formal_acquisition': True, 'actual_flow_state_updates': 0,
        'statistical_integrity_validated': False, 'hardware_readiness': 'NOT_READY'}
    for key, expected in required.items():
        if type(protocol.get(key)) is not type(expected) or protocol[key] != expected:
            raise ValueError('invalid scope field ' + key)
    topics = protocol['topics']
    values = []
    for key in TYPE_NAMES:
        value = topics[key]
        if not isinstance(value, str) or not value.startswith('/'):
            raise ValueError('invalid topic ' + key)
        values.append(value)
    for key in ('camera_images', 'camera_infos'):
        if not isinstance(topics[key], list) or len(topics[key]) != 2:
            raise ValueError('exactly two camera streams required')
        for value in topics[key]:
            if not isinstance(value, str) or not value.startswith('/'):
                raise ValueError('invalid camera topic')
            values.append(value)
    if len(values) != len(set(values)):
        raise ValueError('protocol topics must be distinct')
    for key in ('range', 'imu'):
        if not isinstance(protocol['frames'][key], str) or not protocol['frames'][key]:
            raise ValueError('invalid frame')
    r = protocol['range']
    for key in ('rate_hz', 'min_m', 'max_m', 'noise_std_m'):
        require_number(r[key], 'range.' + key, True)
    for key in ('field_of_view_rad', 'noise_mean_m'):
        require_number(r[key], 'range.' + key)
    if r['max_m'] <= r['min_m'] or r['field_of_view_rad'] != 0:
        raise ValueError('protocol requires a valid pencil range')
    if r.get('white_noise_independent') is not True or r.get('not_all_feature_depths') is not True:
        raise ValueError('range scope must explicitly preserve feature-depth limitation')
    if r.get('single_sample_angular_step_policy') != PENCIL_STEP_POLICY:
        raise ValueError('unexpected single-pencil derived angular-step policy')
    for key in ('rate_hz', 'gyro_std_rad_s', 'accel_std_m_s2'):
        require_number(protocol['imu'][key], 'imu.' + key, True)
    if protocol['imu'].get('orientation_unknown') is not True:
        raise ValueError('IMU attitude must be explicitly unavailable')
    m = protocol['minimums']
    for key in ('range_pairs', 'imu_pairs', 'camera_count'):
        if type(m[key]) is not int or m[key] < 2:
            raise ValueError('invalid minimum count ' + key)
    require_number(m['source_span_s'], 'source_span_s', True)
    for key in ('rate_fraction', 'pair_fraction'):
        if not number(m[key]) or not 0 < m[key] <= 1:
            raise ValueError('invalid minimum fraction')
    g = protocol['noise_gates']
    for key in ('mean_abs_m', 'std_ratio_min', 'std_ratio_max', 'lag1_abs',
                'coverage95_min', 'coverage95_max'):
        require_number(g[key], key, True)
    if not g['std_ratio_min'] < 1 < g['std_ratio_max']:
        raise ValueError('invalid standard deviation gate')
    if not 0 < g['coverage95_min'] < .95 < g['coverage95_max'] <= 1 or g['lag1_abs'] >= 1:
        raise ValueError('invalid noise statistical gate')
    required_metadata = {
        'acquisition_clock_domain': 'gazebo_simulation_acquisition',
        'reference_evaluation_only': True, 'ground_truth_used_in_algorithm': False,
        'feature_depth_from_single_beam': False, 'metric_flow_fusion_ready': False,
        'hardware_calibration_validated': False}
    for key, expected in required_metadata.items():
        value = protocol['metadata_required'].get(key)
        if type(value) is not type(expected) or value != expected:
            raise ValueError('invalid metadata scope ' + key)
    for transform in protocol['transforms'].values():
        if not all(isinstance(transform[k], str) and transform[k] for k in ('parent', 'child')):
            raise ValueError('invalid TF identifiers')
        finite_array(transform['translation_m'], 3)
        matrix = transform['rotation']
        if len(matrix) != 3:
            raise ValueError('invalid TF rotation shape')
        for row in matrix:
            finite_array(row, 3)
        # Orthogonality and right-handedness checked independently of ROS TF.
        for i in range(3):
            for j in range(3):
                if abs(sum(matrix[k][i] * matrix[k][j] for k in range(3)) - (i == j)) > 1e-10:
                    raise ValueError('invalid TF rotation matrix')
        determinant = sum(matrix[0][i] * (matrix[1][(i+1)%3] * matrix[2][(i+2)%3] -
                          matrix[1][(i+2)%3] * matrix[2][(i+1)%3]) for i in range(3))
        if abs(determinant - 1) > 1e-10:
            raise ValueError('TF rotation must be right handed')
    return protocol


def finite_array(values, size):
    values = list(values)
    if len(values) != size or not all(number(value) for value in values):
        raise ValueError('nonfinite or incorrectly sized numerical array')
    return tuple(float(value) for value in values)


def vector(message):
    return finite_array((message.x, message.y, message.z), 3)


def stamp_ns(header):
    sec, nsec = header.stamp.sec, header.stamp.nanosec
    if type(sec) is not int or type(nsec) is not int or sec < 0 or not 0 <= nsec < 1_000_000_000:
        raise ValueError('invalid acquisition timestamp')
    value = sec * 1_000_000_000 + nsec
    if value <= 0:
        raise ValueError('zero acquisition timestamp')
    return value


def close(actual, expected, tolerance=2e-7):
    return number(actual) and abs(actual - expected) <= tolerance * max(1., abs(expected))


def range_value(message, cfg, frame):
    stamp = stamp_ns(message.header)
    if message.header.frame_id != frame or message.radiation_type != 1:
        raise ValueError('Range frame or radiation_type differs from pencil infrared contract')
    for key, field in (('min_m', 'min_range'), ('max_m', 'max_range'),
                       ('field_of_view_rad', 'field_of_view')):
        if not close(getattr(message, field), cfg[key]):
            raise ValueError('Range configuration differs from protocol: ' + field)
    value = message.range
    if not number(value) or not cfg['min_m'] - 1e-7 <= value <= cfg['max_m'] + 1e-7:
        raise ValueError('invalid or outside-limit range observation')
    return stamp, float(value)


def float32(value):
    """Independent IEEE754 quantization; no bridge conversion helper imported."""
    if not number(value):
        raise ValueError('nonfinite source range cannot be normalized')
    return struct.unpack('<f', struct.pack('<f', value))[0]


def source_value(value, protocol):
    if not isinstance(value, dict) or type(value.get('schema_version')) is not int or value['schema_version'] != 1:
        raise ValueError('invalid native range-source schema')
    kind, stamp = value['stream'], value['stamp_ns']
    if kind not in ('noisy', 'reference') or type(stamp) is not int or stamp <= 0:
        raise ValueError('invalid native source stream/timestamp')
    native_frame = value['native_frame']
    sensor_name = 'down_range_'+kind
    if (not isinstance(native_frame, str) or
            not (native_frame == sensor_name or native_frame.endswith('::'+sensor_name))):
        raise ValueError('native source frame must identify the correct pencil sensor')
    for key in ('count', 'vertical_count'):
        if type(value[key]) is not int or value[key] != 1:
            raise ValueError('angular-step exception requires exactly one by one ray')
    for key in ('angle_min', 'angle_max', 'vertical_angle_min', 'vertical_angle_max'):
        if not number(value[key]) or value[key] != 0.:
            raise ValueError('angular-step exception requires finite exactly zero angular bounds')
    undefined = False
    for key in ('native_angle_step', 'native_vertical_angle_step'):
        if value[key] == 'nan' and isinstance(value[key], str):
            undefined = True
        elif not number(value[key]) or value[key] != 0.:
            raise ValueError('only zero or explicitly tagged nan derived angular spacing is allowed')
    expected_reason = 'single_sample_spacing_undefined' if undefined else 'already_zero'
    if value['normalization_reason'] != expected_reason:
        raise ValueError('derived angular-spacing normalization reason mismatch')
    if not number(value['derived_ros_increment']) or value['derived_ros_increment'] != 0.:
        raise ValueError('derived ROS pencil increment must equal zero')
    distance = value['native_range_m']
    if (not number(distance) or
            not protocol['range']['min_m'] <= distance <= protocol['range']['max_m']):
        raise ValueError('native range must remain finite and within bounds')
    return kind, stamp, {'native_range_m': float(distance), 'ros_float32_range_m': float32(distance),
                         'undefined_spacing_normalized': undefined}


def scan_value(message, cfg, frame):
    stamp = stamp_ns(message.header)
    if message.header.frame_id != frame:
        raise ValueError('native scan frame mismatch')
    for field in ('angle_min', 'angle_max', 'angle_increment', 'time_increment'):
        if not close(getattr(message, field), 0.):
            raise ValueError('native scan must be an instantaneous single ray')
    if not close(message.scan_time, 1. / cfg['rate_hz']):
        raise ValueError('native scan period mismatch')
    if not close(message.range_min, cfg['min_m']) or not close(message.range_max, cfg['max_m']):
        raise ValueError('native scan limits mismatch')
    values = finite_array(message.ranges, 1)
    if len(message.intensities) not in (0, 1) or not all(number(v) for v in message.intensities):
        raise ValueError('native scan intensity shape/value mismatch')
    if not cfg['min_m'] - 1e-7 <= values[0] <= cfg['max_m'] + 1e-7:
        raise ValueError('native scan observation outside limits')
    return stamp, values[0]


def imu_value(message, cfg, frame, calibrated):
    stamp = stamp_ns(message.header)
    if message.header.frame_id != frame:
        raise ValueError('IMU frame mismatch')
    omega, acceleration = vector(message.angular_velocity), vector(message.linear_acceleration)
    orientation = finite_array((message.orientation.x, message.orientation.y,
                                message.orientation.z, message.orientation.w), 4)
    covariances = [finite_array(getattr(message, name), 9) for name in (
        'orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance')]
    if calibrated:
        expected = [(-1.,) + (0.,) * 8]
        for sigma in (cfg['gyro_std_rad_s'], cfg['accel_std_m_s2']):
            expected.append(tuple(sigma * sigma if i in (0, 4, 8) else 0. for i in range(9)))
        for actual, wanted in zip(covariances, expected):
            if any(abs(a-b) > 1e-15 * max(1., abs(b)) for a, b in zip(actual, wanted)):
                raise ValueError('IMU covariance differs from per-sample SDF variance/unknown orientation')
    return stamp, (omega, acceleration, orientation), tuple(covariances)


def quaternion_rotation(q):
    x, y, z, w = finite_array((q.x, q.y, q.z, q.w), 4)
    length = math.sqrt(x*x + y*y + z*z + w*w)
    if abs(length - 1.) > 1e-8:
        raise ValueError('TF quaternion must be normalized')
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def camera_value(message, info):
    stamp = stamp_ns(message.header)
    if not message.header.frame_id or type(message.width) is not int or type(message.height) is not int:
        raise ValueError('invalid camera dimensions/frame')
    if message.width != 320 or message.height != 240:
        raise ValueError('camera dimensions differ from frozen R2 simulation calibration')
    if info:
        if message.distortion_model != 'plumb_bob' or len(message.d) != 5:
            raise ValueError('unsupported camera distortion model')
        finite_array(message.d, 5)
        k, r, p = finite_array(message.k, 9), finite_array(message.r, 9), finite_array(message.p, 12)
        expected_k = (277.128129211, 0., 159.5, 0., 277.128129211, 119.5, 0., 0., 1.)
        expected_p = (277.128129211, 0., 159.5, 0., 0., 277.128129211, 119.5, 0., 0., 0., 1., 0.)
        if any(not close(a, b) for a, b in zip(k, expected_k)) or any(not close(a, b) for a, b in zip(p, expected_p)):
            raise ValueError('camera K/P differs from explicit simulation calibration')
        if r != (1., 0., 0., 0., 1., 0., 0., 0., 1.) or any(message.d):
            raise ValueError('unexpected camera rectification/distortion')
    else:
        channels = {'rgb8': 3, 'bgr8': 3, 'mono8': 1}.get(message.encoding)
        if channels is None or message.step < channels * message.width or len(message.data) != message.height * message.step:
            raise ValueError('invalid image encoding/step/payload shape')
    return stamp


def metadata_value(value, protocol):
    """Check declared SDF per-sample variances; keep unknown nuisance terms unknown."""
    for key, expected in protocol['metadata_required'].items():
        if type(value.get(key)) is not type(expected) or value[key] != expected:
            raise ValueError('metadata disagrees with required scope: ' + key)
    for key, expected in (('world', protocol['world']), ('model_definition', protocol['model']),
                          ('simulation_only', True), ('algorithm_state_updates', 0),
                          ('flight_control_output', False)):
        if type(value.get(key)) is not type(expected) or value[key] != expected:
            raise ValueError('metadata experiment identity/scope mismatch: '+key)
    r, imu = value['range'], value['imu']
    numerical = ((r, 'range_noise_mean_m', protocol['range']['noise_mean_m']),
                 (r, 'range_noise_stddev_m', protocol['range']['noise_std_m']),
                 (r, 'range_noise_variance_m2', protocol['range']['noise_std_m']**2),
                 (r, 'min_range_m', protocol['range']['min_m']),
                 (r, 'max_range_m', protocol['range']['max_m']),
                 (r, 'update_rate_hz', protocol['range']['rate_hz']),
                 (imu, 'angular_velocity_variance_per_axis', protocol['imu']['gyro_std_rad_s']**2),
                 (imu, 'linear_acceleration_variance_per_axis', protocol['imu']['accel_std_m_s2']**2))
    for section, key, expected in numerical:
        if not number(section.get(key)) or abs(section[key]-expected) > 1e-15*max(1., abs(expected)):
            raise ValueError('metadata variance/model differs from protocol: '+key)
    expected_values = ((r, 'range_noise_model', 'gazebo_gaussian_per_sample_white'),
                       (r, 'reference_noise', 'none'),
                       (r, 'variance_in_ros_range_message', False),
                       (r, 'feature_depth_cross_covariance', 'unknown'),
                       (r, 'physical_bias_covariance', 'unknown'),
                       (r, 'single_sample_angular_step_policy', protocol['range']['single_sample_angular_step_policy']),
                       (imu, 'noise_parameter_semantics', 'per_sample_measurement_variance_not_noise_density'),
                       (imu, 'physical_bias_covariance', 'unknown'),
                       (imu, 'bias_feature_depth_cross_covariance', 'unknown'),
                       (imu, 'real_hardware_calibration', False))
    for section, key, expected in expected_values:
        if type(section.get(key)) is not type(expected) or section[key] != expected:
            raise ValueError('metadata unknown/shared/physical scope mismatch: '+key)
    transform = protocol['transforms']['range']
    if (r.get('ros_frame_id') != protocol['frames']['range'] or
            imu.get('ros_frame_id') != protocol['frames']['imu'] or
            imu.get('ros_topic') != protocol['topics']['imu_covariance']):
        raise ValueError('metadata sensor frame/topic mismatch')
    if finite_array(r['translation_body_m'], 3) != tuple(transform['translation_m']):
        raise ValueError('metadata range lever-arm mismatch')
    rotation = tuple(finite_array(row, 3) for row in r['R_body_range'])
    if rotation != tuple(tuple(row) for row in transform['rotation']):
        raise ValueError('metadata range boresight rotation mismatch')
    if finite_array(r['beam_direction_body'], 3) != tuple(row[0] for row in rotation):
        raise ValueError('metadata beam direction must be sensor +X rotated into body')
    return value


def reference_graph(text, expected_type):
    """Verify a saved ROS endpoint snapshot, not an all-time provenance proof."""
    overall = re.search(r'^Type:\s*(\S+)\s*$', text, re.MULTILINE)
    counts = {}
    for key in ('Publisher', 'Subscription'):
        found = re.findall(r'^'+key+r' count:\s*(\d+)\s*$', text, re.MULTILINE)
        if len(found) != 1:
            raise ValueError('missing/ambiguous ROS graph endpoint count')
        counts[key.upper()] = int(found[0])
    endpoints = []
    for block in re.split(r'(?=^Node name:)', text, flags=re.MULTILINE)[1:]:
        fields = {}
        for key in ('Node name', 'Node namespace', 'Topic type', 'Endpoint type'):
            matches = re.findall(r'^'+key+r':\s*(\S+)\s*$', block, re.MULTILINE)
            if len(matches) != 1:
                raise ValueError('malformed ROS graph endpoint '+key)
            fields[key] = matches[0]
        if fields['Topic type'] != expected_type or fields['Endpoint type'] not in counts:
            raise ValueError('unexpected graph endpoint type')
        endpoints.append(fields)
    if overall is None or overall[1] != expected_type:
        raise ValueError('unexpected reference topic type')
    for key, count in counts.items():
        if sum(item['Endpoint type'] == key for item in endpoints) != count:
            raise ValueError('graph endpoint count differs from declared count')
    publishers = [item for item in endpoints if item['Endpoint type'] == 'PUBLISHER']
    subscribers = [item for item in endpoints if item['Endpoint type'] == 'SUBSCRIPTION']
    if len(publishers) != 1 or publishers[0]['Node name'] != 'impact_r2_range_bridge' or publishers[0]['Node namespace'] != '/':
        raise ValueError('reference must have exactly the isolated bridge publisher')
    if not subscribers or any(item['Node name'] != 'rosbag2_recorder' or item['Node namespace'] != '/' for item in subscribers):
        raise ValueError('reference subscriber must be exclusively rosbag2_recorder')
    return {'status': 'PASS', 'endpoints': endpoints, 'scope': 'saved endpoint snapshot only; no all-time subscriber-history claim'}


def noise_statistics(errors, sigma, mean=0.):
    errors = finite_array(errors, len(errors))
    if len(errors) < 3 or sigma <= 0:
        raise ValueError('insufficient paired noise observations')
    centre, sample_std = statistics.mean(errors), statistics.stdev(errors)
    left, right = errors[:-1], errors[1:]
    lm, rm = statistics.mean(left), statistics.mean(right)
    numerator = sum((a-lm) * (b-rm) for a, b in zip(left, right))
    denominator = math.sqrt(sum((a-lm)**2 for a in left) * sum((b-rm)**2 for b in right))
    if denominator <= 0:
        raise ValueError('degenerate paired noise variance')
    covered = sum(abs(value-mean) <= 1.959963984540054 * sigma for value in errors)
    count = len(errors)
    # Wilson interval is diagnostic; only the predeclared empirical gate is tested.
    z = 1.959963984540054
    fraction = covered / count
    denominator_w = 1 + z*z/count
    centre_w = (fraction + z*z/(2*count))/denominator_w
    radius_w = z * math.sqrt(fraction*(1-fraction)/count + z*z/(4*count*count))/denominator_w
    return {'pair_count': count, 'mean_m': centre, 'sample_std_m': sample_std,
            'variance_m2': sample_std**2, 'declared_variance_m2': sigma**2,
            'std_ratio': sample_std/sigma, 'lag1_correlation': numerator/denominator,
            'covered95_count': covered, 'coverage95': fraction,
            'coverage95_wilson_interval': [centre_w-radius_w, centre_w+radius_w],
            'residual_semantics': 'same-acquisition-stamp noisy pencil range minus noise-free pencil reference',
            'whole_trajectory_risk_or_feature_depth_validated': False}


class RecordAudit:
    """Independent semantic checks; receives decoded records from read-only SQL."""
    def __init__(self, protocol):
        self.protocol = validate_protocol(protocol)
        self.roles = {value: key for key, value in protocol['topics'].items() if isinstance(value, str)}
        for key in ('camera_images', 'camera_infos'):
            for index, value in enumerate(protocol['topics'][key]):
                self.roles[value] = key + '_' + str(index)
        self.rows = {role: {} for role in self.roles.values()}
        self.streams = {role: {'count': 0, 'first_stamp_ns': None, 'last_stamp_ns': None,
                               'duplicate_or_backward': 0, 'maximum_gap_s': 0.}
                        for role in self.roles.values()}
        self.errors, self.metadata, self.status = [], [], []
        self.transforms = {}
        self.raw_imu_covariance_unknown = 0
        self.payload_counts = {}
        self.source_rows = {'noisy': {}, 'reference': {}}
        self.source_stamps = {'noisy': None, 'reference': None}

    def expected_types(self):
        return {name: (TYPE_NAMES[role] if role in TYPE_NAMES else
                       'sensor_msgs/msg/Image' if role.startswith('camera_images_') else
                       'sensor_msgs/msg/CameraInfo') for name, role in self.roles.items()}

    def observe(self, role, stamp, value=None):
        stream = self.streams[role]
        previous = stream['last_stamp_ns']
        if previous is not None:
            if stamp <= previous:
                stream['duplicate_or_backward'] += 1
                raise ValueError('non-increasing source timestamp for ' + role)
            stream['maximum_gap_s'] = max(stream['maximum_gap_s'], (stamp-previous)*1e-9)
        if stream['first_stamp_ns'] is None:
            stream['first_stamp_ns'] = stamp
        stream['last_stamp_ns'] = stamp
        stream['count'] += 1
        if value is not None:
            self.rows[role][stamp] = value

    def add(self, topic, message, bag_stamp_ns):
        if topic not in self.roles:
            return
        role = self.roles[topic]
        self.payload_counts[role] = self.payload_counts.get(role, 0) + 1
        try:
            if type(bag_stamp_ns) is not int or bag_stamp_ns <= 0:
                raise ValueError('invalid bag receipt timestamp')
            p = self.protocol
            if role in ('range_noisy', 'range_reference'):
                stamp, value = range_value(message, p['range'], p['frames']['range'])
                self.observe(role, stamp, value)
            elif role in ('raw_scan_noisy', 'raw_scan_reference'):
                stamp, value = scan_value(message, p['range'], p['frames']['range'])
                self.observe(role, stamp, value)
            elif role in ('imu_raw', 'imu_covariance'):
                stamp, value, cov = imu_value(message, p['imu'], p['frames']['imu'], role == 'imu_covariance')
                self.observe(role, stamp, value)
                if role == 'imu_raw' and all(x == 0. for values in cov[1:] for x in values):
                    self.raw_imu_covariance_unknown += 1
            elif role == 'tf_static':
                if not message.transforms:
                    raise ValueError('empty static transform message')
                for transform in message.transforms:
                    edge = (transform.header.frame_id, transform.child_frame_id)
                    if not all(isinstance(x, str) and x for x in edge) or edge[0] == edge[1]:
                        raise ValueError('invalid TF identifiers')
                    value = (vector(transform.transform.translation), quaternion_rotation(transform.transform.rotation))
                    if edge in self.transforms and self.transforms[edge] != value:
                        raise ValueError('conflicting static transform')
                    self.transforms[edge] = value
                self.streams[role]['count'] += 1
            elif role in ('range_metadata', 'range_status', 'camera_metadata', 'range_source'):
                value = loads(message.data)
                if not isinstance(value, dict):
                    raise ValueError('metadata/status must be a JSON object')
                if role == 'range_source':
                    kind, stamp, item = source_value(value, p)
                    previous = self.source_stamps[kind]
                    if previous is not None and stamp <= previous:
                        raise ValueError('native source ledger timestamp must increase per sensor stream')
                    self.source_stamps[kind] = stamp
                    self.source_rows[kind][stamp] = item
                elif role == 'range_metadata':
                    self.metadata.append(metadata_value(value, p))
                elif role == 'camera_metadata':
                    for key, expected in (('world', p['world']), ('model', p['model']),
                                          ('simulation_calibration', True), ('real_camera_calibration', False),
                                          ('ground_truth_used', False)):
                        if type(value.get(key)) is not type(expected) or value[key] != expected:
                            raise ValueError('camera metadata world/model/calibration scope mismatch: '+key)
                else:
                    self.status.append(value)
                self.streams[role]['count'] += 1
            else:
                stamp = camera_value(message, role.startswith('camera_infos_'))
                expected_frame = 'impact_front_optical' if role.endswith('_0') else 'impact_down_optical'
                if message.header.frame_id != expected_frame:
                    raise ValueError('camera frame differs from explicit R2 optical frame')
                # Native CameraInfo can be repeated at the same image epoch.
                if role.startswith('camera_infos_'):
                    self.rows[role][stamp] = message.header.frame_id
                    self.streams[role]['count'] += 1
                else:
                    self.observe(role, stamp, message.header.frame_id)
        except (ValueError, TypeError, AttributeError, KeyError, OverflowError) as exc:
            self.errors.append({'topic': topic, 'bag_timestamp_ns': bag_stamp_ns, 'error': str(exc)})

    def finish(self):
        p, checks = self.protocol, {}
        minimum = p['minimums']
        for role, stream in self.streams.items():
            if role in ('range_noisy', 'range_reference', 'raw_scan_noisy', 'raw_scan_reference', 'imu_raw', 'imu_covariance'):
                span = ((stream['last_stamp_ns']-stream['first_stamp_ns'])*1e-9
                        if stream['count'] > 1 else 0.)
                rate = (stream['count']-1)/span if span else 0.
                stream.update(source_span_s=span, rate_hz=rate)
                rate_target = p['imu']['rate_hz'] if role.startswith('imu_') else p['range']['rate_hz']
                checks[role+'_source_span'] = span >= minimum['source_span_s']
                checks[role+'_rate'] = rate >= rate_target * minimum['rate_fraction']
                checks[role+'_monotonic'] = stream['duplicate_or_backward'] == 0
            checks[role+'_present'] = stream['count'] > 0
        checks['all_target_payloads_semantically_valid'] = not self.errors
        for key, transform in p['transforms'].items():
            actual = self.transforms.get((transform['parent'], transform['child']))
            expected = (transform['translation_m'], transform['rotation'])
            checks['static_transform_'+key] = actual is not None and all(
                abs(a-b) <= 1e-8 for part_a, part_b in zip(actual, expected)
                for a, b in zip(part_a if part_a and number(part_a[0]) else [x for row in part_a for x in row],
                                part_b if part_b and number(part_b[0]) else [x for row in part_b for x in row]))
        noisy, reference = self.rows['range_noisy'], self.rows['range_reference']
        paired = sorted(set(noisy) & set(reference))
        checks['range_pair_count'] = len(paired) >= minimum['range_pairs']
        pair_fraction = len(paired)/max(len(noisy), len(reference), 1)
        checks['range_exact_acquisition_pair_fraction'] = pair_fraction >= minimum['pair_fraction']
        noise = None
        if len(paired) >= 3:
            try:
                noise = noise_statistics([noisy[t]-reference[t] for t in paired],
                                         p['range']['noise_std_m'], p['range']['noise_mean_m'])
                g = p['noise_gates']
                checks['noise_mean'] = abs(noise['mean_m']-p['range']['noise_mean_m']) <= g['mean_abs_m']
                checks['noise_sample_std'] = g['std_ratio_min'] <= noise['std_ratio'] <= g['std_ratio_max']
                checks['noise_lag1'] = abs(noise['lag1_correlation']) <= g['lag1_abs']
                checks['noise_coverage95'] = g['coverage95_min'] <= noise['coverage95'] <= g['coverage95_max']
            except ValueError as exc:
                self.errors.append({'topic': 'range_pairs', 'error': str(exc)})
        if noise is None:
            for key in ('noise_mean', 'noise_sample_std', 'noise_lag1', 'noise_coverage95'):
                checks[key] = False
        raw_scan_matches = {}
        source_matches = {}
        for kind in ('noisy', 'reference'):
            ranges, scans = self.rows['range_'+kind], self.rows['raw_scan_'+kind]
            common = set(ranges) & set(scans)
            differences = [abs(ranges[t]-scans[t]) for t in common]
            fraction = len(common)/max(len(ranges), len(scans), 1)
            # Both carry the identical float32 native range; no re-noising/clipping.
            equal = bool(common) and all(value == 0. for value in differences)
            checks['native_'+kind+'_exact_value_and_pairing'] = equal and fraction >= minimum['pair_fraction']
            raw_scan_matches[kind] = {'pair_count': len(common), 'pair_fraction': fraction,
                                      'maximum_value_difference_m': max(differences) if differences else None}
            sources = self.source_rows[kind]
            all_common = set(ranges) & set(scans) & set(sources)
            source_fraction = len(all_common)/max(len(ranges), len(scans), len(sources), 1)
            value_matches = bool(all_common) and all(
                ranges[t] == scans[t] == sources[t]['ros_float32_range_m'] for t in all_common)
            checks['source_'+kind+'_native_double_to_float32_values'] = value_matches
            checks['source_'+kind+'_exact_epoch_pair_fraction'] = source_fraction >= minimum['pair_fraction']
            source_matches[kind] = {'source_ledger_count': len(sources), 'matched_all_three_count': len(all_common),
                                    'pair_fraction': source_fraction,
                                    'undefined_angular_spacing_count': sum(item['undefined_spacing_normalized'] for item in sources.values()),
                                    'unmatched_source_count': len(set(sources)-all_common),
                                    'unmatched_range_count': len(set(ranges)-all_common),
                                    'unmatched_scan_count': len(set(scans)-all_common),
                                    'distance_or_timestamp_normalization_permitted': False}
        raw, annotated = self.rows['imu_raw'], self.rows['imu_covariance']
        imu_paired = set(raw) & set(annotated)
        imu_fraction = len(imu_paired)/max(len(raw), len(annotated), 1)
        checks['imu_pair_count'] = len(imu_paired) >= minimum['imu_pairs']
        checks['imu_exact_acquisition_pair_fraction'] = imu_fraction >= minimum['pair_fraction']
        checks['imu_original_measurements_unchanged'] = bool(imu_paired) and all(raw[t] == annotated[t] for t in imu_paired)
        camera_pairing = []
        for index in range(2):
            images, infos = self.rows['camera_images_'+str(index)], self.rows['camera_infos_'+str(index)]
            same = [stamp for stamp in set(images) & set(infos) if images[stamp] == infos[stamp]]
            fraction = len(same)/max(len(images), 1)
            checks['camera_'+str(index)+'_count'] = len(images) >= minimum['camera_count']
            checks['camera_'+str(index)+'_paired_calibration'] = fraction >= minimum['pair_fraction']
            camera_pairing.append({'image_count': len(images), 'info_unique_epoch_count': len(infos),
                                   'same_stamp_and_frame_count': len(same), 'pair_fraction': fraction})
        checks['all_target_payloads_semantically_valid'] = not self.errors
        return {'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
                'failed_checks': [key for key, value in checks.items() if not value], 'errors': self.errors,
                'streams': self.streams, 'payload_counts': self.payload_counts, 'noise': noise,
                'range_pair_fraction': pair_fraction, 'native_scan_matches': raw_scan_matches,
                'native_source_matches': source_matches,
                'imu_pairing': {'count': len(imu_paired), 'pair_fraction': imu_fraction,
                               'raw_samples_with_unknown_zero_covariance': self.raw_imu_covariance_unknown},
                'camera_pairing': camera_pairing, 'metadata_records': self.metadata,
                'last_bridge_status': self.status[-1] if self.status else None}


def read_database(path, records, deserialize, get_message):
    report = {'file': path.name, 'sqlite_open_mode': 'mode=ro&immutable=1', 'errors': []}
    connection = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    try:
        connection.execute('PRAGMA query_only=ON')
        for pragma in ('quick_check', 'integrity_check'):
            values = [row[0] for row in connection.execute('PRAGMA '+pragma)]
            report[pragma] = values
            if values != ['ok']:
                raise ValueError('SQLite '+pragma+' failed')
        topics = {}
        for ident, name, type_name, serialization in connection.execute(
                'SELECT id,name,type,serialization_format FROM topics'):
            if ident in topics:
                raise ValueError('duplicate SQLite topic id')
            if serialization != 'cdr':
                raise ValueError('non-CDR SQLite topic serialization')
            topics[ident] = (name, type_name)
        expected = records.expected_types()
        classes, count, byte_count, targets = {}, 0, 0, 0
        payload_hash = hashlib.sha256()
        for ident, topic_id, receipt, data in connection.execute(
                'SELECT id,topic_id,timestamp,data FROM messages NOT INDEXED ORDER BY id'):
            if topic_id not in topics or not isinstance(data, bytes) or type(receipt) is not int or receipt <= 0:
                raise ValueError('malformed SQLite message row')
            name, type_name = topics[topic_id]
            payload_hash.update(json.dumps([ident, topic_id, receipt, len(data)], separators=(',', ':')).encode()+b'\n')
            payload_hash.update(data)
            payload_hash.update(b'\n')
            count += 1
            byte_count += len(data)
            if name in expected:
                if type_name != expected[name]:
                    raise ValueError('target ROS type mismatch: '+name)
                if type_name not in classes:
                    classes[type_name] = get_message(type_name)
                # Official ROS decoder independently checks target CDR layout.
                message = deserialize(data, classes[type_name])
                records.add(name, message, receipt)
                targets += 1
        independent = connection.execute('SELECT COUNT(*) FROM messages NOT INDEXED').fetchone()[0]
        if independent != count:
            raise ValueError('independent payload count mismatch')
        report.update(status='PASS', message_count=count, targeted_decoded_messages=targets,
                      serialized_bytes=byte_count, ordered_row_payload_sha256=payload_hash.hexdigest())
    finally:
        connection.close()
    return report


def audit(run_dir, protocol_path, deserialize=None, get_message=None):
    run, protocol_path = Path(run_dir).resolve(), Path(protocol_path).resolve()
    bag = run/'rosbag'
    result = {'schema_version': 1, 'status': 'FAIL', 'phase': 'R2B3B1_RANGE_AND_NOISE_ACQUISITION',
              'generated_at_utc': datetime.now(timezone.utc).isoformat(), 'run_dir': str(run),
              'scope': 'Independent offline acquisition and same-ray white-noise checks; no estimator or control execution',
              'method': 'all target CDR decoded by official ROS types; original SQLite mode=ro&immutable=1; no bridge helpers',
              'script_sha256': sha256(Path(__file__)), 'checks': {}, 'errors': [], 'databases': [],
              'actual_flow_state_updates': 0, 'metric_flow_fusion_ready': False,
              'per_feature_depth_available': False, 'pixel_covariance_available': False,
              'statistical_integrity_validated': False, 'hardware_readiness': 'NOT_READY',
              'noise_free_reference_consumed_for_evaluation_only': True,
              'vehicle_ground_truth_subscribed': False,
              'reference_is_navigation_or_feature_depth_input': False}
    before = None
    try:
        protocol_raw = protocol_path.read_bytes()
        protocol_digest = hashlib.sha256(protocol_raw).hexdigest()
        protocol = validate_protocol(loads(protocol_raw))
        result['protocol_sha256'] = protocol_digest
        before = snapshot(bag)
        result['original_files_before'] = before
        databases = sorted(bag.glob('*.db3'), key=lambda path: [
            int(piece) if piece.isdigit() else piece for piece in re.split(r'(\d+)', path.name)])
        if not databases:
            raise ValueError('no uncompressed SQLite chunks')
        if deserialize is None or get_message is None:
            from rclpy.serialization import deserialize_message
            from rosidl_runtime_py.utilities import get_message as ros_get_message
            deserialize, get_message = deserialize_message, ros_get_message
        records = RecordAudit(protocol)
        for path in databases:
            result['databases'].append(read_database(path, records, deserialize, get_message))
        result.update(records.finish())
        result['reference_topic_graphs'] = {}
        for role, filename in REFERENCE_GRAPHS.items():
            path = run/filename
            report = reference_graph(path.read_text(), TYPE_NAMES[role])
            report.update(file=filename, sha256=sha256(path))
            result['reference_topic_graphs'][role] = report
            result['checks'][role+'_recording_only_subscribers_at_graph_snapshot'] = True
        result['checks']['protocol_unchanged_during_audit'] = sha256(protocol_path) == protocol_digest
    except Exception as exc:
        result['errors'].append({'error': type(exc).__name__+': '+str(exc)})
        result['checks']['all_input_files_readable'] = False
    finally:
        if before is not None:
            try:
                after = snapshot(bag)
                result['original_files_after'] = after
                result['checks']['original_bag_file_set_and_sha256_unchanged'] = before == after
            except Exception as exc:
                result['errors'].append({'error': 'post-audit snapshot: '+str(exc)})
                result['checks']['original_bag_file_set_and_sha256_unchanged'] = False
    result['failed_checks'] = [key for key, value in result['checks'].items() if not value]
    result['status'] = 'PASS' if result['checks'] and not result['errors'] and not result['failed_checks'] else 'FAIL'
    return result


def validate_output(output, run, protocol):
    output, run, protocol = Path(output).resolve(), Path(run).resolve(), Path(protocol).resolve()
    bag = run/'rosbag'
    if output == protocol or output == Path(__file__).resolve() or output == run or bag == output or bag in output.parents:
        raise ValueError('output may not overwrite protocol, source, run directory, or any bag path')
    # An existing artifact is an input unless it is this auditor's conventional result.
    if output.exists() and output != run/'range-acquisition-result.json':
        raise ValueError('refusing to overwrite an existing non-result artifact')
    if output.is_symlink():
        raise ValueError('output symlinks are forbidden')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        output = validate_output(args.output or args.run_dir/'range-acquisition-result.json', args.run_dir, args.protocol)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    result = audit(args.run_dir, args.protocol)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({'status': result['status'], 'checks': len(result['checks']),
                      'failed_checks': result['failed_checks'], 'output': str(output)}))
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
