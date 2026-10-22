#!/usr/bin/env python3
"""Immutable B1 five-sensor replay with original stamps and cumulative barriers.

Preparation exports truth/reference only into a separate evaluator directory.
The publisher loads only its sensor index and allowed sensor CDR rows. Barriers
describe publication counts; the mapper must wait for actual callback receipts.
"""
from __future__ import annotations

import argparse
import bisect
from collections import Counter
import hashlib
import itertools
import json
import math
from pathlib import Path
import sqlite3
import time


SENSORS = {
    '/livox/imu': ('imu', 'sensor_msgs/msg/Imu'),
    '/livox/lidar': ('lidar', 'sensor_msgs/msg/PointCloud2'),
    '/impact/r2/front/image_raw': ('front', 'sensor_msgs/msg/Image'),
    '/impact/r2/down/image_raw': ('down', 'sensor_msgs/msg/Image'),
    '/impact/r2b3/down/range': ('range', 'sensor_msgs/msg/Range'),
}
ROLES = ('imu', 'lidar', 'front', 'down', 'range')
PREFIX = '/impact/joint/input/'
BARRIER = PREFIX + 'barrier'
TRUTH = '/xq/eval/p4/ground_truth'
REFERENCE = '/impact/r2b3/eval/range_reference'
INFOS = {'/impact/r2/front/camera_info': 'front', '/impact/r2/down/camera_info': 'down'}
PRIORITY = {'imu': 0, 'lidar': 1, 'range': 2, 'down': 3, 'front': 4}
ORIGIN = 'B1_original_acquisition_five_sensor_joint_replay'
PROFILE = 'exact_seed_next_original_camera_v1'
MAX_PAIR_NS = 300_000_000


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1048576), b''):
            value.update(block)
    return value.hexdigest()


def write_json(path, value):
    with path.open('x', encoding='utf-8') as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False, allow_nan=False)
        handle.write('\n')


def snapshot(bag):
    if not bag.is_dir() or any(path.is_symlink() for path in (bag, *bag.parents)):
        raise ValueError('Real nonsymlink source bag required')
    result = {}
    for path in sorted(bag.iterdir()):
        if path.is_symlink() or not path.is_file():
            raise ValueError('Source family contains a nonregular entry')
        if path.name.endswith(('-wal', '-shm', '-journal')):
            raise ValueError('Closed source must have no SQLite sidecars; originals will not be changed')
        before = path.stat()
        result[path.name] = {'bytes': before.st_size, 'sha256': digest(path)}
        after = path.stat()
        if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
            raise ValueError('Source changed while hashing: ' + path.name)
    if not result or 'metadata.yaml' not in result or not any(name.endswith('.db3') for name in result):
        raise ValueError('Missing original bag metadata/databases')
    return result


def connect(path):
    connection = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    connection.execute('PRAGMA query_only=ON')
    return connection


def fetch_sensor_payload(connection, row):
    if SENSORS.get(row.get('topic')) != (row.get('sensor'), row.get('type')):
        raise ValueError('Payload query must identify an allowed sensor')
    actual = connection.execute(
        'select m.data from messages m join topics t on m.topic_id=t.id where m.id=? and t.name=? and t.type=?',
        (row['rowid'], row['topic'], row['type'])).fetchone()
    if actual is None:
        raise ValueError('Source row is not the declared allowed sensor')
    blob = actual[0]
    if len(blob)!=row['serialized_size_bytes'] or hashlib.sha256(blob).hexdigest()!=row['payload_sha256']:
        raise ValueError('Indexed sensor payload changed')
    return blob


def barrier_payload(stamp, counts):
    values = [stamp, *[counts[role] for role in ROLES]]
    if type(stamp) is not int or stamp<=0 or any(type(value) is not int or not 0<=value<2**64 for value in values):
        raise ValueError('Invalid UInt64 barrier timestamp/counts')
    return values


def safe_output(bag, output):
    if any(path.is_symlink() for path in (output,*output.parents)):
        raise ValueError('Output aliases forbidden')
    resolved=output.resolve()
    if resolved==bag or bag in resolved.parents or resolved in bag.parents:
        raise ValueError('Output must not overlap the immutable source bag')
    return resolved


def ns(message):
    result = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
    if result <= 0 or not message.header.frame_id:
        raise ValueError('Invalid original acquisition header')
    return result


def camera_record(message):
    result = {'width': message.width, 'height': message.height, 'k': list(message.k),
        'd': list(message.d), 'distortion_model': message.distortion_model,
        'r': list(message.r), 'p': list(message.p), 'frame_id': message.header.frame_id}
    expected = [277.128129211, 0., 159.5, 0., 277.128129211, 119.5, 0., 0., 1.]
    if ((message.width, message.height, message.distortion_model) != (320, 240, 'plumb_bob')
            or len(message.k) != 9 or any(not math.isfinite(a) or abs(a-b)>1e-8 for a,b in zip(message.k, expected))
            or any(not math.isfinite(v) or v != 0 for v in message.d)):
        raise ValueError('CameraInfo differs from frozen zero-distortion pixel contract')
    return result


def strict_stream(entries, role):
    values = [row['stamp_ns'] for row in entries if row['sensor'] == role]
    if not values or any(b <= a for a,b in zip(values, values[1:])):
        raise ValueError('Missing or non-increasing source stream: ' + role)
    return values


def pair_profile(entries):
    streams = {role: strict_stream(entries, role) for role in ROLES}
    front, down, ranges = streams['front'], streams['down'], streams['range']
    if front != down:
        raise ValueError('Original front/down acquisition sequences must match exactly')
    exact = sorted(set(front) & set(ranges))
    pairs, rejected, consumed = [], [], set()
    for stamp in exact:
        index = bisect.bisect_left(front, stamp)
        if index+1 >= len(front):
            rejected.append({'t0_ns': stamp, 'reason': 'no_original_next_camera'}); continue
        end = front[index+1]
        if end-stamp > MAX_PAIR_NS or stamp in consumed or end in consumed:
            rejected.append({'t0_ns': stamp, 't1_ns': end, 'reason': 'pair_too_long_or_reused'}); continue
        pairs.append({'pair_id': len(pairs)+1, 't0_ns': stamp, 't1_ns': end,
            'dt_ns': end-stamp, 't0_front_down_range_exact': True,
            't1_is_next_original_front_down': True, 't1_requires_new_range': False})
        consumed.update((stamp, end))
    if not pairs:
        raise ValueError('No valid exact-seed/next-camera pairs')
    nearest = []
    for stamp in down:
        position = bisect.bisect_left(ranges, stamp)
        values = ranges[max(0, position-1):min(len(ranges), position+1)]
        nearest.append(min(values, key=lambda v:abs(v-stamp))-stamp)
    return {'schema_version': 1, 'profile': PROFILE, 'maximum_pair_dt_ns': MAX_PAIR_NS,
        'front_down_same_original_stamp_sequence': True, 'exact_seed_opportunities': len(exact),
        'exact_seed_stamps_ns': exact, 'valid_nonoverlapping_pair_count': len(pairs), 'pairs': pairs,
        'rejected_seed_opportunities': rejected,
        'seed_interval_histogram_ns': dict(Counter(str(b-a) for a,b in zip(exact,exact[1:]))),
        'maximum_seed_interval_s': max((b-a for a,b in zip(exact,exact[1:])),default=0)*1e-9,
        'nearest_range_minus_camera_offset_histogram_ns_diagnostic_only': dict(Counter(str(v) for v in nearest)),
        'range_messages_total': len(ranges), 'range_samples_used_for_seed_initialization': len(pairs),
        'other_ranges_input_audit_only_not_fused_by_this_profile': len(ranges)-len(pairs),
        'no_nearest_matching_interpolation_restamping_or_pair_frame_reuse': True,
        'scope': 'Low-duty recorded-data joint-flow validation: exact ToF/front/down t0, then next original front/down t1. Not full 50Hz range utilization or synchronous 15Hz range.'}


def prepare(bag, output):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    helper_sha = digest(Path(__file__))
    bag = bag.resolve()
    output = safe_output(bag,output)
    if output.exists() or output.is_symlink():
        raise ValueError('Index output must be new')
    before = snapshot(bag)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'source-files.before.json', before)
    classes = {name:get_message(kind) for name,(_,kind) in SENSORS.items()}
    classes.update({name:get_message('sensor_msgs/msg/CameraInfo') for name in INFOS})
    evaluation_classes = {TRUTH:get_message('nav_msgs/msg/Odometry'), REFERENCE:get_message('sensor_msgs/msg/Range')}
    entries, infos, counts = [], {}, Counter()
    truth, reference = [], []
    error = None
    try:
        for file in sorted(bag.glob('*.db3')):
            database = connect(file)
            try:
                all_topics = {row[0]:(row[1],row[2]) for row in database.execute('select id,name,type from topics')}
                topics = {key:value for key,value in all_topics.items() if value[0] in classes}
                for rowid,topicid,storage,blob in database.execute(
                        'select id,topic_id,timestamp,data from messages where topic_id in ('+','.join('?' for _ in topics)+') order by timestamp,id',tuple(topics)):
                    name, kind = topics[topicid]
                    expected = SENSORS[name][1] if name in SENSORS else 'sensor_msgs/msg/CameraInfo'
                    if kind != expected:
                        raise ValueError('Original sensor type mismatch: '+name)
                    message = deserialize_message(blob,classes[name]); stamp = ns(message)
                    counts[name] += 1
                    if name in INFOS:
                        role = INFOS[name]; current = camera_record(message)
                        if role in infos and infos[role] != current:
                            raise ValueError('Changing original camera calibration')
                        infos[role] = current; continue
                    role = SENSORS[name][0]
                    if role in ('front','down') and (message.width,message.height,message.encoding,message.step,len(message.data),message.is_bigendian) != (320,240,'rgb8',960,230400,0):
                        raise ValueError('Unexpected original camera layout')
                    if role == 'lidar':
                        fields = {field.name for field in message.fields}
                        if not {'x','y','z','intensity'} <= fields or fields & {'time','t','timestamp','offset_time'}:
                            raise ValueError('Instantaneous LiDAR snapshot field mismatch')
                    entries.append({'file':str(file),'rowid':rowid,'topic':name,'sensor':role,'type':kind,
                        'stamp_ns':stamp,'frame_id':message.header.frame_id,'storage_timestamp_ns':storage,
                        'serialized_size_bytes':len(blob),'payload_sha256':hashlib.sha256(blob).hexdigest()})
                # Separate SQL selection and separate evaluator outputs. These
                # classes/rows never enter the sensor index or publisher.
                evaluator_topics = {key:value for key,value in all_topics.items() if value[0] in evaluation_classes}
                for rowid,topicid,blob in database.execute('select id,topic_id,data from messages where topic_id in ('+','.join('?' for _ in evaluator_topics)+') order by timestamp,id',tuple(evaluator_topics)):
                    name, kind = evaluator_topics[topicid]
                    if kind != ('nav_msgs/msg/Odometry' if name==TRUTH else 'sensor_msgs/msg/Range'):
                        raise ValueError('Evaluator original type mismatch')
                    message = deserialize_message(blob,evaluation_classes[name]); stamp = ns(message)
                    if name == TRUTH:
                        p,q = message.pose.pose.position,message.pose.pose.orientation
                        truth.append({'stamp_ns':stamp,'stamp_s':stamp*1e-9,'position':[p.x,p.y,p.z],
                            'quaternion_xyzw':[q.x,q.y,q.z,q.w],'frame_id':message.header.frame_id,'child_frame_id':message.child_frame_id})
                    else:
                        reference.append({'stamp_ns':stamp,'range_m':float(message.range),'frame_id':message.header.frame_id})
            finally:
                database.close()
        entries.sort(key=lambda row:(row['stamp_ns'],PRIORITY[row['sensor']]))
        profile = pair_profile(entries)
        if set(infos) != {'front','down'} or not truth or not reference:
            raise ValueError('Missing independent camera models or evaluator data')
        definitions = {role:{'source_topic':topic,'type':kind,'published_topic':PREFIX+role} for topic,(role,kind) in SENSORS.items()}
        source = {'schema_version':1,'input_origin':ORIGIN,'bag':str(bag),'camera_keep_every_n':1,
            'source_counts':dict(counts),'published_sensor_counts':dict(Counter(row['sensor'] for row in entries)),
            'sensor_definitions':definitions,'camera_info':infos['front'],'down_camera_info':infos['down'],
            'source_files':before,'raw_files':[{'file':str(bag/name),'size_bytes':record['bytes'],'sha256':record['sha256']} for name,record in before.items() if name.endswith('.db3')],
            'entries':entries,'pair_profile':profile,
            'clock_policy':'Derived monotone simulation /clock from unmodified original acquisition headers, never storage or host wall stamps',
            'barrier_policy':{'topic':BARRIER,'type':'std_msgs/msg/UInt64MultiArray','fields':['stamp_ns',*ROLES],
                'counts':'cumulative actual publication calls in original streams; mapper waits for actual callback counts',
                'emission':'after every complete front-stamp bucket, plus final bucket if different',
                'all_six_input_topics_reliable':True,'cross_topic_arrival_order_assumed':False},
            'evaluation_payloads_in_sensor_index':False,'old_odometry_TF_commands_or_AP_inputs':False}
        write_json(output/'sensor-index.json',source)
        stamps = {role:[row['stamp_ns'] for row in entries if row['sensor']==role] for role in ROLES}
        write_json(output/'source-stamps.json',{'stamps_ns':stamps,'front_camera_stamps_s':[v*1e-9 for v in stamps['front']],
            'down_camera_stamps_s':[v*1e-9 for v in stamps['down']],'imu_stamps_s':[v*1e-9 for v in stamps['imu']],
            'lidar_stamps_s':[v*1e-9 for v in stamps['lidar']],'range_stamps_s':[v*1e-9 for v in stamps['range']]})
        write_json(output/'pair-profile.json',profile)
        evaluator = output/'evaluator'; evaluator.mkdir()
        for name,values in [('truth.jsonl',truth),('range-reference.jsonl',reference)]:
            with (evaluator/name).open('x') as handle:
                for value in sorted(values,key=lambda row:row['stamp_ns']):
                    handle.write(json.dumps(value,separators=(',',':'),allow_nan=False)+'\n')
    except Exception as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        after = snapshot(bag)
        write_json(output/'source-files.after.json',after)
    result = {'status':'PASS' if error is None and before==after else 'FAIL','error':error,
        'original_source_files_set_bytes_sha256_unchanged':before==after,'sqlite_open_mode':'mode=ro&immutable=1',
        'truth_reference_exported_only_for_separate_evaluator':True,'camera_decimation':False,
        'sensor_index_sha256':digest(output/'sensor-index.json') if (output/'sensor-index.json').is_file() else None,
        'scope':'Prepare original five-sensor CDR references only; no ROS node or state update.'}
    result['helper_sha256_before'] = helper_sha
    result['helper_sha256_after'] = digest(Path(__file__))
    if result['helper_sha256_after']!=helper_sha:
        result['status']='FAIL';result['error']='Preparation helper source changed during execution'
    if error is None:
        result.update({'published_sensor_counts':source['published_sensor_counts'],
            'exact_seed_opportunities':profile['exact_seed_opportunities'],'valid_pairs':profile['valid_nonoverlapping_pair_count'],
            'maximum_seed_interval_s':profile['maximum_seed_interval_s'],
            'evaluator_exports':{p.name:{'bytes':p.stat().st_size,'sha256':digest(p)} for p in evaluator.iterdir()}})
    write_json(output/'prepare-result.json',result)
    return result


def validate_index(source):
    if source.get('input_origin') != ORIGIN or source.get('camera_keep_every_n') != 1:
        raise ValueError('Unexpected joint source index origin/decimation')
    bag = Path(source['bag']).resolve()
    raw = {row['file'] for row in source['raw_files']}
    expected_definitions = {role:{'source_topic':topic,'type':kind,'published_topic':PREFIX+role} for topic,(role,kind) in SENSORS.items()}
    if source.get('sensor_definitions') != expected_definitions or not raw:
        raise ValueError('Sensor index whitelist differs')
    if raw != {str(bag/name) for name in source['source_files'] if name.endswith('.db3')}:
        raise ValueError('Raw database set differs from complete source-family identity')
    for row in source['raw_files']:
        if Path(row['file']).parent != bag or source['source_files'][Path(row['file']).name] != {'bytes':row['size_bytes'],'sha256':row['sha256']}:
            raise ValueError('Raw database is not bound to its original family')
    previous = None
    for row in source['entries']:
        role = row.get('sensor'); definition = expected_definitions.get(role)
        if (definition is None or row.get('file') not in raw or type(row.get('rowid')) is not int or row['rowid']<=0
                or row.get('topic') != definition['source_topic'] or row.get('type') != definition['type']
                or type(row.get('stamp_ns')) is not int or row['stamp_ns']<=0):
            raise ValueError('Invalid/foreign sensor index row')
        key = (row['stamp_ns'],PRIORITY[role])
        if previous is not None and key<=previous:
            raise ValueError('Source index is not in strict acquisition order')
        previous = key
    if source['published_sensor_counts'] != dict(Counter(row['sensor'] for row in source['entries'])):
        raise ValueError('Source counts differ from exact indexed rows')
    if source.get('pair_profile') != pair_profile(source['entries']):
        raise ValueError('Source pair profile differs from original exact keys')
    return bag


def publish(index_path, output, rate, ready_timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import UInt64MultiArray
    if not math.isfinite(rate) or not 0<rate<=1 or not math.isfinite(ready_timeout) or ready_timeout<=0:
        raise ValueError('Invalid replay rate or readiness timeout')
    helper_sha = digest(Path(__file__))
    index_bytes = index_path.read_bytes()
    source = json.loads(index_bytes); index_sha = hashlib.sha256(index_bytes).hexdigest()
    bag = validate_index(source); output = safe_output(bag,output); before = snapshot(bag)
    if before != source['source_files']:
        raise ValueError('Original source database family changed')
    output.mkdir(parents=True,exist_ok=True)
    owned = ['parameter-ready.json','input-graph.json','input-graph.after.json','publication-ledger.jsonl',
             'publisher-result.json','source-files.before.json','source-files.after.json']
    if any((output/name).exists() or (output/name).is_symlink() for name in owned):
        raise ValueError('Publisher outputs must not already exist')
    write_json(output/'source-files.before.json',before)
    rclpy.init(); node = Node('parameter_blackboard')
    info = source['camera_info']
    camera = {'cam_model':'Pinhole','cam_width':320,'cam_height':240,'scale':1.0,
        'cam_fx':float(info['k'][0]),'cam_fy':float(info['k'][4]),'cam_cx':float(info['k'][2]),'cam_cy':float(info['k'][5]),
        'cam_d0':0.0,'cam_d1':0.0,'cam_d2':0.0,'cam_d3':0.0}
    for key,value in camera.items():
        node.declare_parameter(key,value)
    qos = QoSProfile(depth=1000,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    publishers = {role:node.create_publisher(get_message(row['type']),PREFIX+role,qos) for role,row in source['sensor_definitions'].items()}
    barrier = node.create_publisher(UInt64MultiArray,BARRIER,qos)
    clock = node.create_publisher(Clock,'/clock',100)
    write_json(output/'parameter-ready.json',{'status':'READY','camera':camera,'source_index_sha256':index_sha})
    databases, counts = {}, Counter({role:0 for role in ROLES})
    maximum_lag, barrier_count, error = 0.0, 0, None
    last_barrier_stamp = None
    def graph_snapshot():
        topics = node.get_topic_names_and_types()
        subscriptions = node.get_subscriber_names_and_types_by_node('impact_livo_replay','/')
        allowed = {PREFIX+role for role in ROLES}|{BARRIER,'/clock','/parameter_events'}
        if any(name not in allowed for name,_ in subscriptions):
            raise ValueError('Mapper subscribes outside exact sensor/barrier/clock/parameter whitelist')
        endpoints = {}
        for role,typename in [*((role,source['sensor_definitions'][role]['type']) for role in ROLES),('barrier','std_msgs/msg/UInt64MultiArray')]:
            topic = PREFIX+role
            values = {}
            for label,getter in [('publishers',node.get_publishers_info_by_topic),('subscribers',node.get_subscriptions_info_by_topic)]:
                values[label] = [{'node_name':v.node_name,'node_namespace':v.node_namespace,'topic_type':v.topic_type,
                    'qos_reliability':v.qos_profile.reliability.name,'qos_durability':v.qos_profile.durability.name,
                    'qos_depth':v.qos_profile.depth} for v in getter(topic)]
            if ([v['node_name'] for v in values['publishers']] != ['parameter_blackboard'] or
                    [v['node_name'] for v in values['subscribers']] != ['impact_livo_replay'] or
                    any(v['node_namespace']!='/' or v['topic_type']!=typename or v['qos_reliability']!='RELIABLE'
                        or v['qos_durability']!='VOLATILE' for rows in values.values() for v in rows)):
                raise ValueError('Unexpected exact owner/type/QoS for '+topic)
            endpoints[role] = values
        if any('mavros' in name.split('/') or name in {'/tf','/tf_static',TRUTH,REFERENCE,'/localization/odom'} for name,_ in topics):
            raise ValueError('Shadow graph contains AP/TF/truth/reference/old-state topic')
        return {'topics':topics,'estimator_subscriptions':subscriptions,'input_endpoints':endpoints,
                'scope':'Discovery identity only, not proof of callback receipt or cross-topic ordering'}
    try:
        deadline = time.monotonic()+ready_timeout
        graph_error = None
        while True:
            if all(pub.get_subscription_count()>=1 for pub in [*publishers.values(),barrier]):
                try:
                    graph = graph_snapshot();break
                except (ValueError,RuntimeError) as exc:
                    graph_error = str(exc)
            if time.monotonic()>=deadline:
                raise TimeoutError('Mapper exact input graph did not become ready: '+str(graph_error))
            rclpy.spin_once(node,timeout_sec=.05)
        write_json(output/'input-graph.json',graph)
        classes = {role:get_message(row['type']) for role,row in source['sensor_definitions'].items()}
        start, first_stamp = time.monotonic(),source['entries'][0]['stamp_ns']
        with (output/'publication-ledger.jsonl').open('x',buffering=1) as ledger:
            for stamp,iterator in itertools.groupby(source['entries'],key=lambda row:row['stamp_ns']):
                bucket = list(iterator)
                target = start+(stamp-first_stamp)*1e-9/rate
                while True:
                    remaining = target-time.monotonic()
                    if remaining<=0:
                        break
                    rclpy.spin_once(node,timeout_sec=min(.01,remaining))
                maximum_lag = max(maximum_lag,time.monotonic()-target)
                for row in bucket:
                    file = row['file']
                    if file not in databases:
                        databases[file] = connect(Path(file))
                    # Topic/type predicates prevent loading a truth/reference
                    # blob even when a corrupt index points at an evaluator row.
                    blob = fetch_sensor_payload(databases[file],row)
                    message = deserialize_message(blob,classes[row['sensor']])
                    if ns(message)!=stamp or message.header.frame_id!=row['frame_id']:
                        raise ValueError('Original acquisition header changed')
                    clock.publish(Clock(clock=Time(nanoseconds=stamp).to_msg()))
                    publishers[row['sensor']].publish(bytes(blob));counts[row['sensor']]+=1
                    ledger.write(json.dumps({'event':'sensor_publication','source':row,
                        'sensor_publication_sequence':counts[row['sensor']],
                        'publication_call_monotonic_ns':time.monotonic_ns(),'original_cdr_published_unchanged':True},separators=(',',':'))+'\n')
                    rclpy.spin_once(node,timeout_sec=0)
                if any(row['sensor']=='front' for row in bucket):
                    data = barrier_payload(stamp,counts)
                    barrier.publish(UInt64MultiArray(data=data));barrier_count+=1;last_barrier_stamp=stamp
                    ledger.write(json.dumps({'event':'barrier_publication','data':data,'publication_call_monotonic_ns':time.monotonic_ns(),
                        'receipt_counts_must_be_verified_by_mapper':True},separators=(',',':'))+'\n')
            final_stamp = source['entries'][-1]['stamp_ns']
            if last_barrier_stamp != final_stamp:
                data = barrier_payload(final_stamp,counts)
                barrier.publish(UInt64MultiArray(data=data));barrier_count+=1
                ledger.write(json.dumps({'event':'barrier_publication','data':data,'terminal_barrier':True,
                    'publication_call_monotonic_ns':time.monotonic_ns()},separators=(',',':'))+'\n')
        end = time.monotonic()+2
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
        write_json(output/'input-graph.after.json',graph_snapshot())
    except (Exception,KeyboardInterrupt) as exc:
        error = type(exc).__name__+': '+str(exc)
    finally:
        for connection in databases.values():connection.close()
        node.destroy_node();rclpy.try_shutdown()
        after = snapshot(bag);write_json(output/'source-files.after.json',after)
    result = {'status':'PASS' if error is None and dict(counts)==source['published_sensor_counts'] and before==after and digest(index_path)==index_sha and digest(Path(__file__))==helper_sha else 'FAIL',
        'error':error,'published_counts':dict(counts),'barrier_count':barrier_count,
        'maximum_wall_schedule_lag_s':maximum_lag,'replay_rate':rate,'source_index_sha256':index_sha,
        'source_index_sha256_after':digest(index_path),'original_source_family_set_bytes_sha256_unchanged':before==after,
        'helper_sha256_before':helper_sha,'helper_sha256_after':digest(Path(__file__)),
        'original_acquisition_headers_and_serialized_payloads_preserved':True,
        'truth_reference_oldstate_TF_AP_payloads_loaded_or_published':False,
        'barrier_schema':['stamp_ns',*ROLES],'callback_receipt_or_processing_completion_proven_by_publisher':False,
        'profile':PROFILE,'hardware_timing_claim':False,
        'scope':'Original five-sensor publication calls plus cumulative barriers; mapper ledger must independently prove callback receipt, epoch completion and actual state updates.'}
    write_json(output/'publisher-result.json',result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=('prepare','publish'))
    parser.add_argument('--bag',type=Path)
    parser.add_argument('--index',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=.5)
    parser.add_argument('--ready-timeout',type=float,default=90.)
    args = parser.parse_args(argv)
    try:
        if args.mode=='prepare':
            if args.bag is None:raise ValueError('--bag required')
            result = prepare(args.bag,args.output)
        else:
            if args.index is None:raise ValueError('--index required')
            result = publish(args.index,args.output,args.rate,args.ready_timeout)
        print(json.dumps(result,allow_nan=False))
        return 0 if result['status']=='PASS' else 1
    except (OSError,ValueError,TypeError,KeyError,RuntimeError) as exc:
        print(json.dumps({'status':'FAIL','error':type(exc).__name__+': '+str(exc)}))
        return 1


if __name__=='__main__':
    raise SystemExit(main())
