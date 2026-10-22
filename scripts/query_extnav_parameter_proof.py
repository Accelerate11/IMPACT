#!/usr/bin/env python3
"""Read the MAVROS2 FCU parameter cache; no parameter writes or flight clients."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--timeout-s', type=float, default=45.)
    args = parser.parse_args(argv)
    if args.output.exists() or not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        raise SystemExit('A fresh output and a finite positive deadline are required')
    config = json.loads(args.config.read_text())['live_readiness']
    required = config['required_parameters']
    proof = {'schema_version': 1, 'method': 'mavros_ros2_GetParameters',
             'target_system_id': config['expected_system_id'],
             'target_component_id': config['expected_component_id'],
             'target_identity_scope': 'Configured target only. GetParameters responses do not carry MAVLink source identity; wire PARAM_VALUE and SYS_STATUS identities are checked independently after closure.',
             'service': '/uav1/mavros/param/get_parameters',
             'config_sha256': hashlib.sha256(args.config.read_bytes()).hexdigest(),
             'parameters': {}, 'responses': [], 'status': 'FAIL',
             'read_only': True, 'parameter_writes': 0, 'flight_commands': 0}
    rclpy.init()
    node = rclpy.create_node('impact_extnav_readonly_parameter_proof')
    client = node.create_client(GetParameters, proof['service'])
    deadline = time.monotonic()+args.timeout_s
    try:
        while time.monotonic() < deadline:
            if not client.wait_for_service(timeout_sec=.25):
                continue
            request = GetParameters.Request()
            request.names = list(required)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=min(3., deadline-time.monotonic()))
            if not future.done():
                continue
            response = future.result()
            raw = {'wall_time_ns': time.time_ns(), 'monotonic_ns': time.monotonic_ns(),
                   'requested_names': list(request.names), 'values': []}
            rows = {}
            for index, value in enumerate(response.values):
                name = request.names[index] if index < len(request.names) else 'EXTRA_RESPONSE_INDEX_'+str(index)
                record = {'name': name, 'ros_type': int(value.type),
                          'integer_value': int(value.integer_value),
                          'double_value': float(value.double_value),
                          'success': value.type in (ParameterType.PARAMETER_INTEGER, ParameterType.PARAMETER_DOUBLE)}
                raw['values'].append(record)
                rows[name] = {k: v for k, v in record.items() if k != 'name'}
            proof['responses'].append(raw)
            proof['parameters'] = rows
            if len(response.values) != len(request.names):
                continue
            if any(not math.isfinite(row['double_value']) for row in rows.values()):
                raise ValueError('A raw parameter response contains a nonfinite double field')
            if not all(v['success'] for v in rows.values()):
                rclpy.spin_once(node, timeout_sec=.2)
                continue
            for name, row in rows.items():
                actual = row['integer_value'] if row['ros_type'] == ParameterType.PARAMETER_INTEGER else row['double_value']
                if not math.isfinite(actual) or actual != required[name]:
                    raise ValueError(f'FCU cache {name}={actual!r}, expected {required[name]!r}')
            proof['status'] = 'PASS'
            break
        if proof['status'] != 'PASS':
            proof['error'] = 'Required FCU parameter values were not all available by the deadline'
    except Exception as exc:
        proof['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with args.output.open('x') as handle:
            json.dump(json_safe(proof), handle, indent=2, allow_nan=False)
            handle.write('\n')
    print(json.dumps({'status': proof['status'], 'error': proof.get('error'), 'output': str(args.output)}))
    return 0 if proof['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
