"""Real loopback TCP proxy tests; synthetic frames only, never a simulator/bag."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'probe_mavlink_tcp.py'
SPEC = importlib.util.spec_from_file_location('probe_mavlink_tcp', SCRIPT)
PROBE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PROBE)
SITE = Path(PROBE.DEFAULT_SITE)


@pytest.fixture(scope='module')
def dialect():
    if not (SITE / 'pymavlink').is_dir():
        pytest.skip('This WSL-specific diagnostic requires the explicit installed pymavlink site')
    module, metadata = PROBE.load_dialect(SITE)
    assert metadata['temporary_sys_path_restored']
    assert metadata['numpy_loaded_after'] is None or str(SITE) not in metadata['numpy_loaded_after']
    return module


def frames(module, armed=False):
    encoder = module.MAVLink(None, srcSystem=42, srcComponent=191)
    messages = [
        module.MAVLink_odometry_message(
            123456789, 20, 12, 1., 2., -3., [1., 0., 0., 0.],
            .1, .2, -.3, 0., 0., 0., [0.] * 21, [0.] * 21, 2, 2, 87),
        module.MAVLink_command_long_message(1, 1, 511, 0, 331., 20000., 0., 0., 0., 0., 0.),
        module.MAVLink_command_int_message(1, 1, 0, 512, 0, 0, 30., 0., 0., 0., 0, 0, 0.),
        module.MAVLink_attitude_message(71234, 0., .1, .2, 0., 0., 0.),
        module.MAVLink_system_time_message(1700000000000000, 71235),
    ]
    outgoing = bytearray()
    for index, message in enumerate(messages):
        encoder.seq = index
        outgoing.extend(message.pack(encoder))
    encoder.seq = 19
    reply = module.MAVLink_heartbeat_message(2, 3, 128 if armed else 0, 4, 3, 3).pack(encoder)
    return bytes(outgoing), bytes(reply)


def read_all(peer):
    result = bytearray()
    while True:
        part = peer.recv(65536)
        if not part:
            return bytes(result)
        result.extend(part)


def server(handler, buffer_bytes=0):
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if buffer_bytes:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffer_bytes)
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    listener.settimeout(10.)
    state = {'port': listener.getsockname()[1]}

    def run():
        try:
            peer, _ = listener.accept()
            with peer:
                peer.settimeout(10.)
                handler(peer, state)
        except Exception as exc:
            state['error'] = exc
        finally:
            listener.close()

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return state, thread


def start_proxy(output, target_port, extra=()):
    environment = dict(os.environ, PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1')
    process = subprocess.Popen(
        [sys.executable, str(SCRIPT), '--output-dir', str(output), '--listen-port', '0',
         '--target-port', str(target_port), '--connect-timeout-s', '2',
         '--accept-timeout-s', '3', '--drain-timeout-s', '2',
         '--pymavlink-site', str(SITE), *extra],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=environment)
    return process


def ready(process):
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        readable, _, _ = select.select([process.stdout], [], [], .05)
        if readable:
            line = process.stdout.readline()
            if not line:
                break
            value = json.loads(line)
            if value.get('kind') == 'ready':
                return value
    if process.poll() is None:
        process.kill()
    stdout, stderr = process.communicate(timeout=3)
    pytest.fail('Proxy did not become ready: ' + stdout + stderr)


def finish(process, output, expected_code=0):
    try:
        stdout, stderr = process.communicate(timeout=12)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate(timeout=3)
        pytest.fail('Proxy did not exit')
    assert process.returncode == expected_code, stdout + stderr
    summary = json.loads((output / 'summary.json').read_text())
    events = [json.loads(row) for row in (output / 'traffic.jsonl').read_text().splitlines()]
    assert summary['exit_code'] == expected_code
    assert summary['traffic_jsonl_sha256'] == hashlib.sha256((output / 'traffic.jsonl').read_bytes()).hexdigest()
    assert summary['autonomous_mavlink_messages'] == 0
    assert all(type(row['monotonic_ns']) is int and type(row['wall_time_ns']) is int for row in events)
    return summary, events


def join_server(thread, state):
    thread.join(12)
    assert not thread.is_alive()
    if 'error' in state:
        raise state['error']


@pytest.mark.parametrize('guard', [False, True])
def test_fragmented_frames_and_half_close_are_bidirectionally_exact(tmp_path, dialect, guard):
    outbound, inbound = frames(dialect)

    def handler(peer, state):
        state['received'] = read_all(peer)
        for byte in inbound:
            peer.sendall(bytes([byte]))
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed'] if guard else [])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        # Deliberately split headers, payloads, checksums and multiple messages.
        widths = [1, 2, 7, 3, 11]
        offset, index = 0, 0
        while offset < len(outbound):
            chunk = outbound[offset:offset+widths[index % len(widths)]]
            client.sendall(chunk)
            offset += len(chunk)
            index += 1
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == inbound
    summary, events = finish(process, output)
    join_server(thread, state)
    assert state['received'] == outbound
    for direction, data in [('to_fcu', outbound), ('from_fcu', inbound)]:
        flow = summary['flows'][direction]
        assert flow['received_bytes'] == flow['sent_bytes'] == len(data)
        assert flow['received_sha256'] == flow['sent_sha256'] == hashlib.sha256(data).hexdigest()
        assert flow['forwarded_stream_identical']
    assert summary['all_mavlink_frames_decoded']
    assert summary['termination_reason'] == 'both_peer_eof'
    rows = [row for row in events if row['kind'] == 'frame']
    odom = next(row for row in rows if row['type'] == 'ODOMETRY')
    assert (odom['msgid'], odom['sysid'], odom['compid'], odom['seq']) == (331, 42, 191, 0)
    assert (odom['time_usec'], odom['frame_id'], odom['child_frame_id'], odom['quality']) == (123456789, 20, 12, 87)
    assert odom['fields']['x'] == 1. and odom['fields']['q'] == [1., 0., 0., 0.]
    assert next(row for row in rows if row['type'] == 'ATTITUDE')['time_boot_ms'] == 71234
    assert next(row for row in rows if row['type'] == 'SYSTEM_TIME')['time_boot_ms'] == 71235
    assert next(row for row in rows if row['type'] == 'COMMAND_LONG')['command'] == 511
    assert next(row for row in rows if row['type'] == 'COMMAND_INT')['param1'] == 30.
    assert next(row for row in rows if row['type'] == 'HEARTBEAT')['armed'] is False


def test_bounded_backpressure_preserves_full_stream(tmp_path, dialect):
    outbound, inbound = frames(dialect)
    outbound *= 240
    inbound *= 100

    def handler(peer, state):
        time.sleep(.15)
        state['received'] = read_all(peer)
        peer.sendall(inbound)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler, buffer_bytes=4096)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--max-buffer-bytes', '1024',
                                                 '--decode-batch-bytes', '257', '--socket-buffer-bytes', '4096'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=8) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == inbound
    summary, _ = finish(process, output)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['forwarding_integrity_pass'] and summary['all_mavlink_frames_decoded']
    assert all(row['maximum_forward_pending_bytes'] <= 1024 and row['maximum_decode_pending_bytes'] <= 1024
               for row in summary['flows'].values())
    assert summary['flows']['to_fcu']['message_counts']['ODOMETRY'] == 240


def test_sigint_drains_observed_bytes_and_exits_cleanly(tmp_path, dialect):
    outbound, _ = frames(dialect)
    reached = threading.Event()

    def handler(peer, state):
        received = bytearray()
        while True:
            data = peer.recv(65536)
            if not data:
                break
            received.extend(data)
            if len(received) >= len(outbound):
                reached.set()
        state['received'] = bytes(received)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        assert reached.wait(4)
        process.send_signal(signal.SIGINT)
        assert read_all(client) == b''
    summary, _ = finish(process, output)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['termination_reason'] == 'signal_SIGINT'
    assert summary['forwarding_integrity_pass']


def test_fragmented_arm_suffix_is_never_forwarded(tmp_path, dialect):
    encoder = dialect.MAVLink(None, srcSystem=42, srcComponent=191)
    arm = bytes(dialect.MAVLink_command_long_message(1, 1, 400, 0, 1., 0., 0., 0., 0., 0., 0.).pack(encoder))
    prefix_arrived = threading.Event()

    def handler(peer, state):
        result = bytearray()
        while True:
            part = peer.recv(65536)
            if not part:
                break
            result.extend(part)
            if len(result) == len(arm)-1:
                prefix_arrived.set()
        state['received'] = bytes(result)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(arm[:-1])
        assert prefix_arrived.wait(4)
        client.sendall(arm[-1:])
        assert read_all(client) == b''
    summary, events = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == arm[:-1]
    parser = dialect.MAVLink(None)
    parser.robust_parsing = True
    assert not parser.parse_buffer(state['received'])
    assert summary['guard_blocking_occurred'] and not summary['forwarding_integrity_pass']
    assert summary['flows']['to_fcu']['guard_blocked_bytes'] == 1
    assert any(row['kind'] == 'guard_blocked' for row in events)


@pytest.mark.parametrize('kind', ['command_int_arm', 'param_set', 'set_mode'])
def test_guard_rejects_complete_control_or_parameter_writes(tmp_path, dialect, kind):
    encoder = dialect.MAVLink(None, srcSystem=42, srcComponent=191)
    messages = {
        'command_int_arm': dialect.MAVLink_command_int_message(1, 1, 0, 400, 0, 0, 1., 0., 0., 0., 0, 0, 0.),
        'param_set': dialect.MAVLink_param_set_message(1, 1, b'TEST_PARAM', 1., 9),
        'set_mode': dialect.MAVLink_set_mode_message(1, 128, 4),
    }
    payload = bytes(messages[kind].pack(encoder))

    def handler(peer, state):
        state['received'] = read_all(peer)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(payload)
        assert read_all(client) == b''
    summary, _ = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == b''
    assert summary['guard_blocking_occurred']


def test_guard_rejects_armed_fcu_heartbeat(tmp_path, dialect):
    _, reply = frames(dialect, armed=True)

    def handler(peer, state):
        peer.sendall(reply)
        state['received'] = read_all(peer)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        assert read_all(client) == b''
    summary, _ = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == b''
    assert summary['flows']['from_fcu']['guard_blocked_bytes'] == len(reply)


@pytest.mark.parametrize('damage', ['truncated_frame', 'bad_crc'])
@pytest.mark.parametrize('allow_startup', [False, True])
def test_incomplete_or_corrupt_decode_fails_even_when_wire_transport_matches(tmp_path, dialect, damage, allow_startup):
    outbound, inbound = frames(dialect)
    if damage == 'truncated_frame':
        inbound = inbound[:-1]
    else:
        inbound = inbound[:-1] + bytes([inbound[-1] ^ 1])

    def handler(peer, state):
        state['received'] = read_all(peer)
        peer.sendall(inbound)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--allow-fcu-startup-ascii'] if allow_startup else [])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == inbound
    summary, _ = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['status'] == 'FAIL'
    assert summary['forwarding_integrity_pass']
    assert not summary['all_mavlink_frames_decoded']
    if damage == 'truncated_frame':
        assert summary['flows']['from_fcu']['parser_partial_bytes'] > 0
    else:
        assert summary['flows']['from_fcu']['message_counts'].get('BAD_DATA', 0) > 0


def test_explicit_fcu_startup_ascii_is_classified_without_editing_bytes(tmp_path, dialect):
    outbound, inbound = frames(dialect)
    prefix = b'Init ArduCopter\r\nFree RAM: 123\nBooting\tfirmware erasing\r\n0 0\nAP Logger\nAP IRLock\r\n'

    def handler(peer, state):
        peer.sendall(prefix[:13])
        peer.sendall(prefix[13:])
        state['received'] = read_all(peer)
        peer.sendall(inbound)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed', '--allow-fcu-startup-ascii'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == prefix + inbound
    summary, events = finish(process, output)
    join_server(thread, state)
    assert state['received'] == outbound
    flow = summary['flows']['from_fcu']
    assert summary['all_mavlink_frames_decoded'] and summary['all_observed_bytes_classified']
    assert flow['startup_ascii_bytes'] == len(prefix)
    assert flow['startup_ascii_sha256'] == hashlib.sha256(prefix).hexdigest()
    assert flow['first_valid_mavlink_frame_seen']
    assert flow['unclassified_bad_data_bytes'] == 0
    classified = [row for row in events if row['kind'] == 'startup_ascii']
    assert bytes.fromhex(''.join(row['raw_hex'] for row in classified)) == prefix
    assert sum(len(row['byte_classes']) for row in classified) == len(prefix)
    assert all(len(row['byte_classes']) == row['bytes'] for row in classified)
    assert all(row['direction'] == 'from_fcu' for row in classified)
    assert summary['flows']['to_fcu']['startup_ascii_bytes'] == 0


@pytest.mark.parametrize('bad_prefix', [False, True])
def test_ascii_after_first_valid_or_nonascii_prefix_still_fails(tmp_path, dialect, bad_prefix):
    outbound, inbound = frames(dialect)
    returned = (b'Boot\x01error\r\n' + inbound) if bad_prefix else (inbound + b'Later ASCII\r\n')

    def handler(peer, state):
        state['received'] = read_all(peer)
        peer.sendall(returned)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed', '--allow-fcu-startup-ascii'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == returned
    summary, _ = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['forwarding_integrity_pass']
    assert not summary['all_mavlink_frames_decoded'] and not summary['all_observed_bytes_classified']
    assert summary['flows']['from_fcu']['unclassified_bad_data_bytes'] > 0
    if not bad_prefix:
        assert summary['flows']['from_fcu']['startup_ascii_bytes'] == 0


def test_startup_failure_never_listens_and_missing_decoder_is_explicit(tmp_path):
    output = tmp_path / 'proxy'
    process = start_proxy(output, 5760, ['--pymavlink-site', str(tmp_path / 'absent')])
    summary, events = finish(process, output, expected_code=1)
    assert not any(row['kind'] == 'ready' for row in events)
    assert summary['client_address'] is None and not summary['flows']
    assert summary['errors']


def test_target_deadline_and_output_exclusivity(tmp_path, dialect):
    reserve = socket.socket()
    reserve.bind(('127.0.0.1', 0))
    port = reserve.getsockname()[1]
    reserve.close()
    output = tmp_path / 'proxy'
    process = start_proxy(output, port, ['--connect-timeout-s', '.15'])
    summary, events = finish(process, output, expected_code=1)
    assert summary['target_connection_attempts'] > 0
    assert not any(row['kind'] == 'ready' for row in events)
    before = {p.name: p.read_bytes() for p in output.iterdir()}
    repeated = start_proxy(output, port)
    repeated.communicate(timeout=4)
    assert repeated.returncode == 1
    assert before == {p.name: p.read_bytes() for p in output.iterdir()}


def test_loader_restores_path_and_rejects_relative_site(dialect):
    before = list(sys.path)
    _, metadata = PROBE.load_dialect(SITE)
    assert sys.path == before and metadata['temporary_sys_path_restored']
    with pytest.raises(ValueError, match='absolute'):
        PROBE.load_dialect(Path('relative/site'))


class MemoryLog:
    def __init__(self):
        self.events = []

    def record(self, kind, **values):
        self.events.append(PROBE.json_safe({'kind': kind, **values}))


def irlock_fixture(dialect, *, direction='from_fcu', allow=True, first_valid=True):
    # No sleeps or system-clock changes: the classifier gets this explicit clock.
    now = [1_000_000_000]
    log = MemoryLog()
    flow = PROBE.Flow(direction, None, None, dialect,
                      allow_sitl_irlock_bootstrap_line=allow,
                      monotonic_ns=lambda: now[0])
    _, heartbeat = frames(dialect)
    if first_valid:
        feed_flow(flow, log, heartbeat)
    return flow, log, now, heartbeat


def feed_flow(flow, log, data):
    flow.enqueue(data, log)
    flow.decode(log, max(1, len(data)))


@pytest.mark.parametrize('split', range(1, len(PROBE.SITL_IRLOCK_BOOTSTRAP_LINE)))
def test_exact_irlock_line_all_two_fragment_boundaries(dialect, split):
    flow, log, now, _ = irlock_fixture(dialect)
    now[0] += 500_000_000
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    feed_flow(flow, log, line[:split])
    assert flow.irlock_bootstrap_count == 0
    feed_flow(flow, log, line[split:])
    result = flow.result()
    assert result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_line_count'] == 1
    assert result['sitl_irlock_bootstrap_bytes'] == len(line)
    assert result['sitl_irlock_bootstrap_sha256'] == hashlib.sha256(line).hexdigest()
    assert result['startup_ascii_bytes'] == 0  # The original option was not enabled.
    assert result['unclassified_bad_data_bytes'] == result['unclassified_bad_data_messages'] == 0
    assembly = [row for row in log.events if row['kind'] == 'sitl_irlock_bootstrap_assembly']
    assert len(assembly) == 1 and assembly[0]['accepted'] is True
    assert assembly[0]['raw_hex'] == line.hex()
    bad = [row for row in log.events if row['kind'] == 'frame' and row['type'] == 'BAD_DATA']
    assert len(bad) == 2
    assert [v['wire_sha256'] for v in bad] == [v['raw_sha256'] for v in assembly[0]['fragments']]
    assert all(row['classified_startup_ascii'] is False for row in bad)
    assert [row['stream_offset'] for row in bad] == [v['stream_offset'] for v in assembly[0]['fragments']]


def test_exact_irlock_line_byte_fragments_are_reassembled_without_editing(dialect):
    flow, log, now, _ = irlock_fixture(dialect)
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    for byte in line:
        now[0] += 1
        feed_flow(flow, log, bytes([byte]))
    result = flow.result()
    assert result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_fragment_count'] == len(line)
    bad = [row for row in log.events if row['kind'] == 'frame' and row['type'] == 'BAD_DATA']
    assert bytes.fromhex(''.join(row['fields']['data']['hex'] for row in bad)) == line


@pytest.mark.parametrize('elapsed_ns,allowed', [(-1, False), (10_000_000_000, True), (10_000_000_001, False)])
def test_irlock_ten_second_window_is_inclusive_and_uses_controlled_monotonic(dialect, elapsed_ns, allowed):
    flow, log, now, _ = irlock_fixture(dialect)
    now[0] += elapsed_ns
    feed_flow(flow, log, PROBE.SITL_IRLOCK_BOOTSTRAP_LINE)
    result = flow.result()
    assert result['all_observed_bytes_classified'] is allowed
    assert result['sitl_irlock_bootstrap_line_count'] == int(allowed)
    assembly = next(row for row in log.events if row['kind'] == 'sitl_irlock_bootstrap_assembly')
    assert assembly['after_first_valid_ns'] == elapsed_ns
    if not allowed:
        assert assembly['reason'] == 'outside_first_valid_10s_window'


def test_irlock_fragment_completion_after_deadline_never_classifies_partial_line(dialect):
    flow, log, now, _ = irlock_fixture(dialect)
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    now[0] += 9_000_000_000
    feed_flow(flow, log, line[:7])
    now[0] += 1_000_000_001
    feed_flow(flow, log, line[7:])
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0
    assert result['unclassified_bad_data_bytes'] == len(line)


def test_irlock_truncated_candidate_remains_failure_and_gets_final_assembly_event(dialect):
    flow, log, now, _ = irlock_fixture(dialect)
    feed_flow(flow, log, PROBE.SITL_IRLOCK_BOOTSTRAP_LINE[:-1])
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0
    assembly = next(row for row in log.events if row['kind'] == 'sitl_irlock_bootstrap_assembly')
    assert not assembly['accepted'] and assembly['reason'] == 'truncated_at_observation_end'


def test_irlock_duplicate_exact_line_fails_after_single_allowed_assembly(dialect):
    flow, log, now, _ = irlock_fixture(dialect)
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    feed_flow(flow, log, line)
    now[0] += 100
    feed_flow(flow, log, line)
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_line_count'] == 1
    assert result['unclassified_bad_data_bytes'] == len(line)
    assemblies = [row for row in log.events if row['kind'] == 'sitl_irlock_bootstrap_assembly']
    assert [row['accepted'] for row in assemblies] == [True, False]
    assert assemblies[1]['reason'] == 'successful_assembly_limit_already_used'


@pytest.mark.parametrize('unknown', [
    b'AP_IRLock_SITL::other()\n', b'AP_IRLock_SITL::init()\r\n',
    b'AP_IRLock_SITL::init()\nextra', b'other ASCII\n',
    b'unknown\nAP_IRLock_SITL::init()\n', b'AP_IRLock_SITL::init()\x01\n',
])
def test_irlock_option_does_not_whitelist_other_ascii_or_nonascii(dialect, unknown):
    flow, log, now, _ = irlock_fixture(dialect)
    feed_flow(flow, log, unknown)
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0
    assert result['unclassified_bad_data_bytes'] == len(unknown)


@pytest.mark.parametrize('direction,allow,first_valid', [
    ('to_fcu', True, True), ('from_fcu', False, True), ('from_fcu', True, False),
])
def test_irlock_option_is_explicit_from_fcu_and_after_first_valid_only(dialect, direction, allow, first_valid):
    flow, log, now, _ = irlock_fixture(dialect, direction=direction, allow=allow, first_valid=first_valid)
    feed_flow(flow, log, PROBE.SITL_IRLOCK_BOOTSTRAP_LINE)
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0


def test_irlock_fragments_cannot_span_an_intervening_valid_mavlink_frame(dialect):
    flow, log, now, heartbeat = irlock_fixture(dialect)
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    feed_flow(flow, log, line[:8])
    feed_flow(flow, log, heartbeat)
    feed_flow(flow, log, line[8:])
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0
    assert any(row['kind'] == 'sitl_irlock_bootstrap_assembly'
               and row['reason'] == 'interrupted_by_valid_mavlink_frame' for row in log.events)


def test_crc_bad_frame_containing_exact_ascii_never_enters_bootstrap_whitelist(dialect):
    flow, log, now, _ = irlock_fixture(dialect)
    encoder = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    message = dialect.MAVLink_statustext_message(6, PROBE.SITL_IRLOCK_BOOTSTRAP_LINE)
    raw = bytes(message.pack(encoder))
    assert PROBE.SITL_IRLOCK_BOOTSTRAP_LINE in raw
    corrupt = raw[:-1] + bytes([raw[-1] ^ 1])
    feed_flow(flow, log, corrupt)
    result = flow.result()
    assert not result['all_observed_bytes_classified']
    assert result['sitl_irlock_bootstrap_bytes'] == 0
    bad = [row for row in log.events if row['kind'] == 'frame' and row['type'] == 'BAD_DATA']
    assert any(row['fields']['reason'] != 'Bad prefix' for row in bad)
    assert not any(row['kind'] == 'sitl_irlock_bootstrap_assembly' and row['accepted'] for row in log.events)


def test_exact_irlock_bootstrap_option_keeps_real_tcp_stream_hashes_identical(tmp_path, dialect):
    outbound, heartbeat = frames(dialect)
    line = PROBE.SITL_IRLOCK_BOOTSTRAP_LINE
    returned = heartbeat + line + heartbeat

    def handler(peer, state):
        state['received'] = read_all(peer)
        peer.sendall(heartbeat)
        for byte in line:
            peer.sendall(bytes([byte]))
        peer.sendall(heartbeat)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'], ['--guard-unarmed', '--allow-sitl-irlock-bootstrap-line'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == returned
    summary, events = finish(process, output)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['flows']['from_fcu']['received_sha256'] == hashlib.sha256(returned).hexdigest()
    assert summary['flows']['from_fcu']['sent_sha256'] == hashlib.sha256(returned).hexdigest()
    assert summary['allow_sitl_irlock_bootstrap_line']
    assert summary['allow_fcu_startup_ascii'] is False
    assert summary['flows']['from_fcu']['sitl_irlock_bootstrap_line_count'] == 1
    assert summary['flows']['from_fcu']['startup_ascii_bytes'] == 0
    assert any(row['kind'] == 'sitl_irlock_bootstrap_assembly' and row['accepted'] for row in events)


@pytest.mark.parametrize('damage', ['truncated_frame', 'bad_crc'])
@pytest.mark.parametrize('old_option', [False, True])
def test_irlock_option_does_not_allow_crc_errors_or_incomplete_mavlink(tmp_path, dialect, damage, old_option):
    outbound, heartbeat = frames(dialect)
    damaged = heartbeat[:-1] if damage == 'truncated_frame' else heartbeat[:-1] + bytes([heartbeat[-1] ^ 1])
    # A preceding complete allowed line cannot waive corruption following it.
    returned = heartbeat + PROBE.SITL_IRLOCK_BOOTSTRAP_LINE + damaged

    def handler(peer, state):
        state['received'] = read_all(peer)
        peer.sendall(returned)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    flags = ['--allow-sitl-irlock-bootstrap-line'] + (['--allow-fcu-startup-ascii'] if old_option else [])
    process = start_proxy(output, state['port'], flags)
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == returned
    summary, events = finish(process, output, expected_code=1)
    join_server(thread, state)
    assert state['received'] == outbound
    assert summary['forwarding_integrity_pass'] and not summary['all_observed_bytes_classified']
    assert summary['flows']['from_fcu']['sitl_irlock_bootstrap_line_count'] == 1


def test_param_value_and_sys_status_fields_bind_original_fcu_wire(tmp_path, dialect):
    outbound, heartbeat = frames(dialect)
    encoder = dialect.MAVLink(None, srcSystem=1, srcComponent=1)
    param = dialect.MAVLink_param_value_message(b'PLND_ENABLED', 0., 9, 1454, 350)
    status = dialect.MAVLink_sys_status_message(
        onboard_control_sensors_present=1 << 18, onboard_control_sensors_enabled=(1 << 18) | 1,
        onboard_control_sensors_health=1 << 18, load=123, voltage_battery=12000,
        current_battery=-1, battery_remaining=92, drop_rate_comm=0, errors_comm=0,
        errors_count1=1, errors_count2=2, errors_count3=3, errors_count4=4)
    returned = bytes(param.pack(encoder)) + bytes(status.pack(encoder))

    def handler(peer, state):
        state['received'] = read_all(peer)
        peer.sendall(returned)
        peer.shutdown(socket.SHUT_WR)

    state, thread = server(handler)
    output = tmp_path / 'proxy'
    process = start_proxy(output, state['port'])
    endpoint = ready(process)
    with socket.create_connection((endpoint['host'], endpoint['port']), timeout=5) as client:
        client.sendall(outbound)
        client.shutdown(socket.SHUT_WR)
        assert read_all(client) == returned
    summary, events = finish(process, output)
    join_server(thread, state)
    rows = [row for row in events if row['kind'] == 'frame']
    param_row = next(row for row in rows if row['type'] == 'PARAM_VALUE')
    status_row = next(row for row in rows if row['type'] == 'SYS_STATUS')
    assert param_row['sysid'] == status_row['sysid'] == 1
    assert param_row['compid'] == status_row['compid'] == 1
    assert param_row['fields']['param_id'] == 'PLND_ENABLED'
    assert param_row['fields']['param_value'] == 0.
    assert param_row['fields']['param_index'] == 350
    assert set(status.to_dict()) == set(status_row['fields'])
    for field in ('onboard_control_sensors_present', 'onboard_control_sensors_enabled',
                  'onboard_control_sensors_health'):
        assert status_row[field] == status_row['fields'][field] == getattr(status, field)
    assert status_row['wire_sha256'] == hashlib.sha256(bytes(status.pack(encoder))).hexdigest()
    assert summary['forwarding_integrity_pass']
