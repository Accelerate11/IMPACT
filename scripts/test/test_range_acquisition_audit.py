"""Independent acquisition auditor tests; synthetic fixtures are not flight results."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import random
import sqlite3
from types import SimpleNamespace as NS

import pytest


SCRIPT = Path(__file__).resolve().parents[1]/'audit_range_acquisition.py'
SPEC = importlib.util.spec_from_file_location('range_acquisition_audit', SCRIPT)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)
PROTOCOL = SCRIPT.parent.parent/'config/research_range_acquisition_gate.json'


def protocol():
    return json.loads(PROTOCOL.read_text())


def header(stamp=1_000_000_000, frame='impact_down_range'):
    return NS(stamp=NS(sec=stamp//1_000_000_000, nanosec=stamp%1_000_000_000), frame_id=frame)


def vector(values=(0., 0., 0.)):
    return NS(**dict(zip(('x', 'y', 'z'), values)))


def quaternion(values=(0., 0., 0., 0.)):
    return NS(**dict(zip(('x', 'y', 'z', 'w'), values)))


def range_message(value=2., stamp=1_000_000_000):
    return NS(header=header(stamp), radiation_type=1, field_of_view=0., min_range=.05,
              max_range=10., range=value)


def scan_message(value=2., stamp=1_000_000_000):
    return NS(header=header(stamp), angle_min=0., angle_max=0., angle_increment=0.,
              time_increment=0., scan_time=.02, range_min=.05, range_max=10.,
              ranges=[value], intensities=[])


def imu_message(stamp=1_000_000_000, calibrated=True):
    value = NS(header=header(stamp, 'livox_imu'), orientation=quaternion(),
               angular_velocity=vector((.01, -.02, .03)), linear_acceleration=vector((0., .1, 9.8)),
               orientation_covariance=[0.]*9, angular_velocity_covariance=[0.]*9,
               linear_acceleration_covariance=[0.]*9)
    if calibrated:
        value.orientation_covariance[0] = -1.
        for index in (0, 4, 8):
            value.angular_velocity_covariance[index] = .0002**2
            value.linear_acceleration_covariance[index] = .002**2
    return value


def camera(stamp=1_000_000_000, info=False, front=False):
    value = NS(header=header(stamp, 'impact_front_optical' if front else 'impact_down_optical'), width=320, height=240)
    if info:
        value.distortion_model = 'plumb_bob'
        value.d = [0.]*5
        value.k = [277.128129211, 0., 159.5, 0., 277.128129211, 119.5, 0., 0., 1.]
        value.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        value.p = [277.128129211, 0., 159.5, 0., 0., 277.128129211, 119.5, 0., 0., 0., 1., 0.]
    else:
        value.encoding, value.step, value.data = 'rgb8', 960, bytes(960*240)
    return value


def transforms():
    return NS(transforms=[NS(header=NS(frame_id='xq_base_link'), child_frame_id='impact_down_range',
               transform=NS(translation=vector((0., 0., -.03)),
                            rotation=quaternion((0., math.sqrt(.5), 0., math.sqrt(.5)))) )])


def metadata():
    return {**protocol()['metadata_required'],
            'world': protocol()['world'], 'model_definition': protocol()['model'], 'simulation_only': True,
            'algorithm_state_updates': 0, 'flight_control_output': False,
            'range': {'range_noise_mean_m': 0., 'range_noise_stddev_m': .01, 'range_noise_variance_m2': .0001,
                      'min_range_m': .05, 'max_range_m': 10., 'update_rate_hz': 50.,
                      'range_noise_model': 'gazebo_gaussian_per_sample_white', 'reference_noise': 'none',
                      'variance_in_ros_range_message': False, 'feature_depth_cross_covariance': 'unknown',
                      'physical_bias_covariance': 'unknown', 'ros_frame_id': 'impact_down_range',
                      'translation_body_m': [0.,0.,-.03],
                      'R_body_range': [[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]], 'beam_direction_body': [0.,0.,-1.],
                      'single_sample_angular_step_policy': AUDIT.PENCIL_STEP_POLICY},
            'imu': {'angular_velocity_variance_per_axis': 4e-8, 'linear_acceleration_variance_per_axis': 4e-6,
                    'noise_parameter_semantics': 'per_sample_measurement_variance_not_noise_density',
                    'physical_bias_covariance': 'unknown', 'bias_feature_depth_cross_covariance': 'unknown',
                    'real_hardware_calibration': False, 'ros_frame_id': 'livox_imu', 'ros_topic': '/impact/r2b3/imu'}}


def source_record(value=2., stamp=1_000_000_000, kind='noisy', undefined=True):
    return {'schema_version': 1, 'stream': kind, 'stamp_ns': stamp,
            'native_frame': 'model::down_range_'+kind, 'count': 1, 'vertical_count': 1,
            'angle_min': 0., 'angle_max': 0., 'vertical_angle_min': 0., 'vertical_angle_max': 0.,
            'native_angle_step': 'nan' if undefined else 0., 'native_vertical_angle_step': 'nan' if undefined else 0.,
            'native_range_m': value, 'derived_ros_increment': 0.,
            'normalization_reason': 'single_sample_spacing_undefined' if undefined else 'already_zero'}


def fill_records():
    p = protocol()
    audit = AUDIT.RecordAudit(p)
    topics = p['topics']
    rng = random.Random(81601)
    # 1101 samples cover 22 seconds. A fixture only, not an additional formal trial.
    for i in range(1101):
        stamp = 1_000_000_000 + i*20_000_000
        native_value = 2.+rng.gauss(0., .01)
        value = AUDIT.float32(native_value)
        for role, message in (('range_noisy', range_message(value, stamp)),
                              ('range_reference', range_message(2., stamp)),
                              ('raw_scan_noisy', scan_message(value, stamp)),
                              ('raw_scan_reference', scan_message(2., stamp))):
            audit.add(topics[role], message, stamp+1)
        for kind, native in (('noisy', native_value), ('reference', 2.)):
            audit.add(topics['range_source'], NS(data=json.dumps(source_record(native, stamp, kind))), stamp+1)
    for i in range(4401):
        stamp = 1_000_000_000 + i*5_000_000
        for role, calibrated in (('imu_raw', False), ('imu_covariance', True)):
            audit.add(topics[role], imu_message(stamp, calibrated), stamp+1)
    for index in range(2):
        for i in range(101):
            stamp = 1_000_000_000+i*66_666_667
            audit.add(topics['camera_images'][index], camera(stamp, front=index == 0), stamp+1)
            audit.add(topics['camera_infos'][index], camera(stamp, True, front=index == 0), stamp+1)
    audit.add(topics['tf_static'], transforms(), 1_000_000_001)
    audit.add(topics['range_metadata'], NS(data=json.dumps(metadata())), 1_000_000_001)
    audit.add(topics['camera_metadata'], NS(data=json.dumps({
        'world': p['world'], 'model': p['model'], 'simulation_calibration': True,
        'real_camera_calibration': False, 'ground_truth_used': False})), 1_000_000_001)
    audit.add(topics['range_status'], NS(data='{"scope":"fixture_only"}'), 1_000_000_001)
    return audit


def test_complete_protocol_preserves_scope():
    AUDIT.validate_protocol(protocol())


@pytest.mark.parametrize('key,value', [('actual_flow_state_updates', True),
    ('hardware_readiness', 'READY'), ('statistical_integrity_validated', True),
    ('predeclared_before_first_formal_acquisition', False)])
def test_scope_cannot_be_widened(key, value):
    p = protocol(); p[key] = value
    with pytest.raises(ValueError):
        AUDIT.validate_protocol(p)


@pytest.mark.parametrize('text', ['{"a":1,"a":2}', '{"a":NaN}', '{"a":1e999}', '{"a":Infinity}'])
def test_ambiguous_nonfinite_json_rejected(text):
    with pytest.raises(ValueError):
        AUDIT.loads(text)


@pytest.mark.parametrize('field,value', [('range', float('nan')), ('range', float('inf')),
    ('range', .01), ('range', 11.), ('field_of_view', .2), ('radiation_type', 0)])
def test_range_invalid_values_rejected(field, value):
    message = range_message(); setattr(message, field, value)
    with pytest.raises(ValueError):
        AUDIT.range_value(message, protocol()['range'], 'impact_down_range')


@pytest.mark.parametrize('field,value', [('ranges', []), ('ranges', [1., 2.]),
    ('ranges', [float('nan')]), ('intensities', [1., 2.]), ('angle_max', .1),
    ('scan_time', .1), ('time_increment', .01)])
def test_native_single_ray_layout_is_explicit(field, value):
    message = scan_message(); setattr(message, field, value)
    with pytest.raises(ValueError):
        AUDIT.scan_value(message, protocol()['range'], 'impact_down_range')


@pytest.mark.parametrize('undefined', [False, True])
def test_source_ledger_preserves_double_range_and_limits_normalization_to_angular_step(undefined):
    value = source_record(2.000000000000001, undefined=undefined)
    kind, stamp, result = AUDIT.source_value(value, protocol())
    assert kind == 'noisy' and stamp == 1_000_000_000
    assert result['native_range_m'] == 2.000000000000001
    assert result['ros_float32_range_m'] == 2.
    assert result['undefined_spacing_normalized'] is undefined


@pytest.mark.parametrize('key,value', [('native_angle_step', float('nan')),
    ('native_angle_step', float('inf')), ('native_vertical_angle_step', '-inf'),
    ('native_angle_step', 'NaN'), ('native_angle_step', .01),
    ('count', 2), ('vertical_count', True), ('angle_max', .01),
    ('vertical_angle_min', float('nan')), ('native_range_m', float('nan')),
    ('native_range_m', float('inf')), ('native_range_m', 'nan'),
    ('native_range_m', .01), ('derived_ros_increment', 1.),
    ('normalization_reason', 'already_zero'), ('stamp_ns', 1.),
    ('native_frame', 'down_range_reference')])
def test_source_ledger_rejects_distance_substitution_and_nonpencil_nan_exception(key, value):
    record = source_record(); record[key] = value
    with pytest.raises(ValueError):
        AUDIT.source_value(record, protocol())


@pytest.mark.parametrize('mutate', [
    lambda m: setattr(m.header, 'frame_id', 'camera_optical'),
    lambda m: setattr(m.header.stamp, 'nanosec', 1_000_000_000),
    lambda m: setattr(m.header.stamp, 'sec', -1),
    lambda m: setattr(m.angular_velocity, 'x', float('nan')),
    lambda m: m.angular_velocity_covariance.__setitem__(0, 0.),
    lambda m: m.angular_velocity_covariance.__setitem__(1, 1e-5),
    lambda m: m.orientation_covariance.__setitem__(0, 0.),
    lambda m: setattr(m, 'linear_acceleration_covariance', [0.]*8),
])
def test_imu_unknown_or_malformed_covariance_not_silently_used(mutate):
    message = imu_message(); mutate(message)
    with pytest.raises(ValueError):
        AUDIT.imu_value(message, protocol()['imu'], 'livox_imu', True)


def test_imu_sample_variance_is_not_noise_density_and_raw_unknown_stays_unknown():
    stamp, value, cov = AUDIT.imu_value(imu_message(), protocol()['imu'], 'livox_imu', True)
    assert cov[0] == (-1.,)+(0.,)*8
    assert cov[1][4] == 4e-8 and cov[2][8] == 4e-6
    _, original, rawcov = AUDIT.imu_value(imu_message(calibrated=False), protocol()['imu'], 'livox_imu', False)
    assert value == original and not any(v for c in rawcov for v in c)


def test_hand_computed_noise_statistics():
    value = AUDIT.noise_statistics([-2., -1., 0., 1., 2.], 1.)
    assert value['mean_m'] == 0.
    assert value['sample_std_m'] == math.sqrt(2.5)
    assert value['coverage95'] == .6
    assert value['lag1_correlation'] == 1.
    assert value['whole_trajectory_risk_or_feature_depth_validated'] is False
    with pytest.raises(ValueError):
        AUDIT.noise_statistics([0., 0., 0.], 1.)


@pytest.mark.parametrize('section,key,value', [('range','range_noise_variance_m2',.01),
    ('range','feature_depth_cross_covariance',0.), ('imu','bias_feature_depth_cross_covariance',0.),
    ('imu','angular_velocity_variance_per_axis',0.), ('imu','noise_parameter_semantics','continuous_noise_density')])
def test_metadata_does_not_turn_unknown_nuisance_covariance_into_zero(section,key,value):
    data = metadata(); data[section][key] = value
    with pytest.raises(ValueError):
        AUDIT.metadata_value(data, protocol())


def graph_text(subscriber='rosbag2_recorder', publisher='impact_r2_range_bridge'):
    return ('Type: sensor_msgs/msg/Range\n\nPublisher count: 1\n\n'
            f'Node name: {publisher}\nNode namespace: /\nTopic type: sensor_msgs/msg/Range\nEndpoint type: PUBLISHER\n'
            'GID: dummy\nQoS profile:\n  Reliability: RELIABLE\n\nSubscription count: 1\n\n'
            f'Node name: {subscriber}\nNode namespace: /\nTopic type: sensor_msgs/msg/Range\nEndpoint type: SUBSCRIPTION\n'
            'GID: other\nQoS profile:\n  Reliability: RELIABLE\n')


def test_reference_graph_allows_only_recorder_at_snapshot():
    assert AUDIT.reference_graph(graph_text(), 'sensor_msgs/msg/Range')['status'] == 'PASS'
    for text in (graph_text(subscriber='livo'), graph_text(publisher='fake'),
                 graph_text().replace('Subscription count: 1','Subscription count: 2'),
                 graph_text().replace('Node namespace: /','Node namespace: /AP'),
                 graph_text().replace('Type: sensor_msgs/msg/Range','Type: std_msgs/msg/String',1)):
        with pytest.raises(ValueError):
            AUDIT.reference_graph(text, 'sensor_msgs/msg/Range')


def test_metadata_range_pose_does_not_confuse_pencil_axis_with_optical_axis():
    value = metadata(); value['range']['beam_direction_body'] = [0.,0.,1.]
    with pytest.raises(ValueError):
        AUDIT.metadata_value(value, protocol())


def test_complete_decoded_synthetic_fixture_passes_all_semantic_gates():
    result = fill_records().finish()
    assert result['status'] == 'PASS', result
    assert result['noise']['pair_count'] == 1101
    assert result['imu_pairing']['raw_samples_with_unknown_zero_covariance'] == 4401
    assert result['native_scan_matches']['noisy']['maximum_value_difference_m'] == 0.
    assert result['native_source_matches']['noisy']['undefined_angular_spacing_count'] == 1101


def test_source_ledger_checks_original_double_to_float32_not_just_native_scan_equality():
    audit = fill_records()
    first = next(iter(audit.rows['range_noisy']))
    audit.rows['range_noisy'][first] += 1e-5
    audit.rows['raw_scan_noisy'][first] += 1e-5
    result = audit.finish()
    assert result['checks']['native_noisy_exact_value_and_pairing']
    assert not result['checks']['source_noisy_native_double_to_float32_values']


def test_source_ledger_allows_interleaved_same_stamp_sensors_but_rejects_per_sensor_replay():
    audit = AUDIT.RecordAudit(protocol()); topic = protocol()['topics']['range_source']
    for kind in ('reference', 'noisy'):
        audit.add(topic, NS(data=json.dumps(source_record(kind=kind))), 1_000_000_001)
    assert not audit.errors and audit.streams['range_source']['count'] == 2
    audit.add(topic, NS(data=json.dumps(source_record(kind='noisy'))), 1_000_000_002)
    assert audit.errors


@pytest.mark.parametrize('tamper', ['clone_stamp', 'native_value', 'imu_value', 'tf_axis', 'metadata', 'source_replay'])
def test_semantic_tampering_fails_without_changing_thresholds(tamper):
    audit = fill_records()
    if tamper == 'clone_stamp':
        audit.rows['range_reference'] = {stamp+1: value for stamp, value in audit.rows['range_reference'].items()}
    elif tamper == 'native_value':
        first = next(iter(audit.rows['raw_scan_noisy'])); audit.rows['raw_scan_noisy'][first] += 1e-6
    elif tamper == 'imu_value':
        first = next(iter(audit.rows['imu_covariance'])); audit.rows['imu_covariance'][first] = ((0.,0.,0.),)*3
    elif tamper == 'tf_axis':
        audit.transforms[('xq_base_link','impact_down_range')] = ((0.,0.,-.03), ((1.,0.,0.),(0.,1.,0.),(0.,0.,1.)))
    elif tamper == 'metadata':
        bad = dict(protocol()['metadata_required']); bad['feature_depth_from_single_beam'] = True
        audit.add(protocol()['topics']['range_metadata'], NS(data=json.dumps(bad)), 2_000_000_000)
    else:
        audit.add(protocol()['topics']['range_noisy'], range_message(), 2_000_000_000)
    assert audit.finish()['status'] == 'FAIL'


def test_native_and_range_need_float32_identity_not_just_approximate_value():
    audit = fill_records(); stamp = next(iter(audit.rows['raw_scan_reference']))
    audit.rows['raw_scan_reference'][stamp] += 1e-12
    assert not audit.finish()['checks']['native_reference_exact_value_and_pairing']


@pytest.mark.parametrize('field,value', [('data', b'x'), ('step', 1), ('encoding', 'jpeg'), ('width', 640)])
def test_image_payload_shape_rejected(field, value):
    message = camera(); setattr(message, field, value)
    with pytest.raises(ValueError):
        AUDIT.camera_value(message, False)


def test_camera_calibration_and_tf_are_independent_of_bridge_helpers():
    message = camera(info=True); message.k[0] += 1.
    with pytest.raises(ValueError):
        AUDIT.camera_value(message, True)
    with pytest.raises(ValueError):
        AUDIT.quaternion_rotation(quaternion((0.,0.,0.,2.)))


def test_snapshot_rejects_live_wal_and_retains_original_hashes(tmp_path):
    bag = tmp_path/'rosbag'; bag.mkdir()
    (bag/'metadata.yaml').write_text('closed fixture')
    (bag/'recording_0.db3').write_bytes(b'fixture')
    before = AUDIT.snapshot(bag)
    assert AUDIT.snapshot(bag) == before
    (bag/'recording_0.db3-wal').write_bytes(b'live')
    with pytest.raises(ValueError):
        AUDIT.snapshot(bag)


def test_sqlite_reads_all_target_payloads_and_cannot_write(tmp_path):
    database = tmp_path/'fixture.db3'
    connection = sqlite3.connect(database)
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT);'
                            'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    connection.execute('INSERT INTO topics VALUES(1,?,?,?)', (protocol()['topics']['range_noisy'], 'sensor_msgs/msg/Range', 'cdr'))
    connection.execute('INSERT INTO messages VALUES(1,1,1000000001,?)', (b'fixture CDR decoder input',))
    connection.commit(); connection.close()
    before = AUDIT.sha256(database)
    records = AUDIT.RecordAudit(protocol())
    calls = []
    def decode(data, cls):
        calls.append((data, cls)); return range_message()
    report = AUDIT.read_database(database, records, decode, lambda t: t)
    assert report['status'] == 'PASS' and report['targeted_decoded_messages'] == 1
    assert len(calls) == 1 and AUDIT.sha256(database) == before
    assert not list(tmp_path.glob('*-wal')) and not list(tmp_path.glob('*-shm'))


def test_sqlite_target_type_mismatch_and_unknown_topic_rows_fail(tmp_path):
    database = tmp_path/'fixture.db3'; connection = sqlite3.connect(database)
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT);'
                            'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    connection.execute('INSERT INTO topics VALUES(1,?,?,?)', (protocol()['topics']['range_noisy'], 'std_msgs/msg/String', 'cdr'))
    connection.execute('INSERT INTO messages VALUES(1,1,1000000001,?)', (b'fixture',))
    connection.commit(); connection.close()
    with pytest.raises(ValueError):
        AUDIT.read_database(database, AUDIT.RecordAudit(protocol()), lambda d,c: range_message(), lambda t:t)


def test_official_ros_cdr_roundtrip_and_immutable_sqlite(tmp_path):
    serialization = pytest.importorskip('rclpy.serialization')
    utilities = pytest.importorskip('rosidl_runtime_py.utilities')
    # The official ROS serializer is independent of both the C++ bridge and this
    # evaluator. No ROS init, node, subscriptions, replay, or publications occur.
    messages = []
    for role, fixture in (('range_noisy', range_message()),
                          ('raw_scan_noisy', scan_message()),
                          ('imu_covariance', imu_message())):
        type_name = AUDIT.TYPE_NAMES[role]
        message = utilities.get_message(type_name)()
        message.header.stamp.sec = 1
        message.header.frame_id = fixture.header.frame_id
        if role == 'range_noisy':
            for field in ('radiation_type', 'field_of_view', 'min_range', 'max_range', 'range'):
                setattr(message, field, getattr(fixture, field))
        elif role == 'raw_scan_noisy':
            for field in ('angle_min', 'angle_max', 'angle_increment', 'time_increment',
                          'scan_time', 'range_min', 'range_max', 'ranges', 'intensities'):
                setattr(message, field, getattr(fixture, field))
        else:
            for field in ('orientation_covariance', 'angular_velocity_covariance', 'linear_acceleration_covariance'):
                setattr(message, field, getattr(fixture, field))
            message.orientation.w = 0.
            for field in ('angular_velocity', 'linear_acceleration'):
                for coordinate in ('x', 'y', 'z'):
                    setattr(getattr(message, field), coordinate, getattr(getattr(fixture, field), coordinate))
        messages.append((role, type_name, serialization.serialize_message(message)))
    database = tmp_path/'ros_serialized_fixture.db3'
    connection = sqlite3.connect(database)
    connection.executescript('CREATE TABLE topics(id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT);'
                            'CREATE TABLE messages(id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB);')
    for index, (role, type_name, payload) in enumerate(messages, start=1):
        connection.execute('INSERT INTO topics VALUES(?,?,?,?)', (index, protocol()['topics'][role], type_name, 'cdr'))
        connection.execute('INSERT INTO messages VALUES(?,?,?,?)', (index, index, 1_000_000_001, payload))
    connection.commit(); connection.close()
    before = AUDIT.sha256(database)
    records = AUDIT.RecordAudit(protocol())
    report = AUDIT.read_database(database, records, serialization.deserialize_message, utilities.get_message)
    assert report['status'] == 'PASS' and report['targeted_decoded_messages'] == 3
    assert not records.errors and records.rows['range_noisy'][1_000_000_000] == 2.
    assert records.rows['imu_covariance'][1_000_000_000][0] == (.01, -.02, .03)
    assert AUDIT.sha256(database) == before


def test_audit_failure_is_reported_and_never_modifies_inputs(tmp_path):
    bag = tmp_path/'rosbag'; bag.mkdir()
    (bag/'metadata.yaml').write_text('closed fixture')
    (bag/'fixture.db3').write_bytes(b'not SQLite')
    cfg = tmp_path/'protocol.json'; cfg.write_text(json.dumps(protocol()))
    before = AUDIT.snapshot(bag)
    result = AUDIT.audit(tmp_path, cfg, lambda d,c: None, lambda t:t)
    assert result['status'] == 'FAIL'
    assert result['checks']['original_bag_file_set_and_sha256_unchanged']
    assert AUDIT.snapshot(bag) == before
    assert not (tmp_path/'range-acquisition-result.json').exists()


def test_output_cannot_overwrite_raw_bag_protocol_or_existing_artifacts(tmp_path):
    bag = tmp_path/'rosbag'; bag.mkdir()
    cfg = tmp_path/'protocol.json'; cfg.write_text('{}')
    old = tmp_path/'old-result.json'; old.write_text('keep')
    for output in (bag, bag/'metadata.yaml', cfg, old, SCRIPT):
        with pytest.raises(ValueError):
            AUDIT.validate_output(output, tmp_path, cfg)
    assert AUDIT.validate_output(tmp_path/'range-acquisition-result.json', tmp_path, cfg) == tmp_path/'range-acquisition-result.json'
