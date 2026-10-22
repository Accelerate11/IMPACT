#!/usr/bin/env python3
"""Single-connection, byte-preserving diagnostic MAVLink TCP proxy.

No MAVLink frame is generated, re-encoded, acknowledged or edited here. Wire
bytes are forwarded before bounded offline decoding work in the same select
loop. Decoder/backpressure queues are bounded; half closes propagate only after
all queued bytes in that direction have been sent. This tool is a diagnostic
interposer and is not evidence that its added timing overhead is negligible.
"""
from __future__ import annotations

import argparse
from collections import Counter, deque
import errno
import hashlib
import importlib
import ipaddress
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import sys
import time


DEFAULT_SITE = '/home/accelerate/.local/lib/python3.10/site-packages'
DIRECTIONS = ('to_fcu', 'from_fcu')
READ_ONLY_COMMANDS = frozenset({410, 511, 512, 519, 520})
SITL_IRLOCK_BOOTSTRAP_LINE = b'AP_IRLock_SITL::init()\n'
SITL_IRLOCK_BOOTSTRAP_WINDOW_NS = 10_000_000_000
BLOCKED_TYPES = frozenset({
    'RC_CHANNELS_OVERRIDE', 'MANUAL_CONTROL', 'ACTUATOR_CONTROL_TARGET',
    'HIL_ACTUATOR_CONTROLS', 'HIL_CONTROLS', 'HIL_RC_INPUTS_RAW',
    'PARAM_SET', 'PARAM_EXT_SET', 'MISSION_SET_CURRENT', 'MISSION_ITEM',
    'MISSION_ITEM_INT', 'MISSION_WRITE_PARTIAL_LIST', 'MISSION_CLEAR_ALL', 'MISSION_COUNT',
})


class GuardViolation(RuntimeError):
    pass


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else 'Infinity' if value > 0 else '-Infinity'
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return {'hex': bytes(value).hex()}
    return value


def load_dialect(site_path):
    """Temporarily expose only the explicitly selected user site for pymavlink.

    The generated dialect does not need NumPy. Restore sys.path immediately so
    later imports cannot accidentally pick up the user's incompatible NumPy.
    """
    site = Path(site_path)
    if not site.is_absolute():
        raise ValueError('--pymavlink-site must be absolute')
    site = site.resolve(strict=True)
    if not (site / 'pymavlink/__init__.py').is_file():
        raise ValueError('pymavlink is absent from the explicit site: ' + str(site))
    previous = list(sys.path)
    numpy_before = getattr(sys.modules.get('numpy'), '__file__', None)
    try:
        sys.path.insert(0, str(site))
        package = importlib.import_module('pymavlink')
        dialect = importlib.import_module('pymavlink.dialects.v20.ardupilotmega')
    finally:
        sys.path[:] = previous
    for module in (package, dialect):
        if site not in Path(module.__file__).resolve().parents:
            raise ValueError('pymavlink resolved outside its explicit site: ' + module.__file__)
    numpy_after = getattr(sys.modules.get('numpy'), '__file__', None)
    if numpy_after and site in Path(numpy_after).resolve().parents:
        raise ValueError('User-site NumPy must not be imported by this diagnostic')
    return dialect, {'explicit_user_site': str(site), 'package_path': package.__file__,
                     'dialect_path': dialect.__file__, 'version': getattr(package, '__version__', None),
                     'temporary_sys_path_restored': sys.path == previous,
                     'numpy_loaded_before': numpy_before, 'numpy_loaded_after': numpy_after}


class EventLog:
    def __init__(self, path):
        self.path = path
        self.stream = path.open('x', encoding='utf-8', buffering=65536)
        self.events = 0

    def record(self, event, **values):
        row = {'kind': event, 'monotonic_ns': time.monotonic_ns(), 'wall_time_ns': time.time_ns(), **values}
        self.stream.write(json.dumps(json_safe(row), allow_nan=False, separators=(',', ':')) + '\n')
        self.events += 1

    def close(self):
        self.stream.close()


class Flow:
    def __init__(self, name, source, sink, dialect, allow_fcu_startup_ascii=False,
                 allow_sitl_irlock_bootstrap_line=False, monotonic_ns=None):
        self.name, self.source, self.sink = name, source, sink
        self.forward = deque()
        self.decode_queue = deque()
        self.pending_bytes = self.decode_bytes = 0
        self.received_bytes = self.sent_bytes = 0
        self.received_sha = hashlib.sha256()
        self.sent_sha = hashlib.sha256()
        self.maximum_forward_pending = self.maximum_decode_pending = 0
        self.source_eof = self.shutdown_sent = False
        self.parser = dialect.MAVLink(None)
        self.parser.robust_parsing = True
        self.counts = Counter()
        self.bad_data_bytes = self.decoded_wire_bytes = 0
        self.decode_errors = []
        self.max_decode_batch_ns = 0
        self.guard_blocked_bytes = self.decoder_skipped_bytes = 0
        self.guard_violations = []
        self.allow_startup_ascii = allow_fcu_startup_ascii and name == 'from_fcu'
        self.first_valid_frame_seen = False
        self.startup_ascii_bytes = self.unclassified_bad_data_bytes = 0
        self.unclassified_bad_data_messages = 0
        self.startup_ascii_sha = hashlib.sha256()
        self.allow_irlock_bootstrap = allow_sitl_irlock_bootstrap_line and name == 'from_fcu'
        self.monotonic_ns = monotonic_ns if monotonic_ns is not None else time.monotonic_ns
        self.first_valid_frame_monotonic_ns = None
        self.irlock_bootstrap_bytes = self.irlock_bootstrap_count = 0
        self.irlock_bootstrap_fragment_count = self.irlock_rejected_assemblies = 0
        self.irlock_bootstrap_sha = hashlib.sha256()
        self.irlock_pending = []
        self.last_decode_log = None

    def _finish_irlock_assembly(self, log, accepted, reason):
        if not self.irlock_pending:
            return
        fragments, self.irlock_pending = self.irlock_pending, []
        raw = b''.join(bytes.fromhex(part['raw_hex']) for part in fragments)
        if accepted:
            self.irlock_bootstrap_count += 1
            self.irlock_bootstrap_bytes += len(raw)
            self.irlock_bootstrap_fragment_count += len(fragments)
            self.irlock_bootstrap_sha.update(raw)
            self.unclassified_bad_data_bytes -= len(raw)
            self.unclassified_bad_data_messages -= len(fragments)
        else:
            self.irlock_rejected_assemblies += 1
        completed = fragments[-1]['decoder_observed_monotonic_ns']
        log.record('sitl_irlock_bootstrap_assembly', direction=self.name,
                   accepted=accepted, reason=reason,
                   classification='explicit_exact_sitl_irlock_bootstrap_line' if accepted else 'unclassified_bad_data',
                   bytes=len(raw), raw_hex=raw.hex(), raw_sha256=hashlib.sha256(raw).hexdigest(),
                   fragments=fragments, first_valid_mavlink_monotonic_ns=self.first_valid_frame_monotonic_ns,
                   completed_decoder_monotonic_ns=completed,
                   after_first_valid_ns=(completed-self.first_valid_frame_monotonic_ns
                                         if self.first_valid_frame_monotonic_ns is not None else None),
                   maximum_after_first_valid_ns=SITL_IRLOCK_BOOTSTRAP_WINDOW_NS,
                   successful_assembly_limit=1, original_bad_data_frames_preserved=True)

    def _observe_irlock_bad_prefix(self, raw, reason, offset, observed_ns, log):
        if not self.allow_irlock_bootstrap:
            return
        if reason != 'Bad prefix' or not raw:
            self._finish_irlock_assembly(log, False, 'interrupted_by_non_bad_prefix')
            return
        # Only a contiguous exact prefix can start an assembly. Unrelated bytes
        # remain unclassified; never search through or discard an unknown prefix.
        if not self.irlock_pending and not SITL_IRLOCK_BOOTSTRAP_LINE.startswith(raw[:1]):
            return
        if self.irlock_pending:
            previous = self.irlock_pending[-1]
            if previous['stream_offset']+previous['bytes'] != offset:
                self._finish_irlock_assembly(log, False, 'noncontiguous_stream_offsets')
        self.irlock_pending.append({
            'stream_offset': offset, 'bytes': len(raw), 'raw_hex': raw.hex(),
            'raw_sha256': hashlib.sha256(raw).hexdigest(),
            'decoder_observed_monotonic_ns': observed_ns,
        })
        if self.first_valid_frame_monotonic_ns is None:
            self._finish_irlock_assembly(log, False, 'no_preceding_valid_fcu_frame')
            return
        if not 0 <= observed_ns-self.first_valid_frame_monotonic_ns <= SITL_IRLOCK_BOOTSTRAP_WINDOW_NS:
            self._finish_irlock_assembly(log, False, 'outside_first_valid_10s_window')
            return
        if self.irlock_bootstrap_count:
            self._finish_irlock_assembly(log, False, 'successful_assembly_limit_already_used')
            return
        assembled = b''.join(bytes.fromhex(part['raw_hex']) for part in self.irlock_pending)
        if not SITL_IRLOCK_BOOTSTRAP_LINE.startswith(assembled):
            self._finish_irlock_assembly(log, False, 'content_not_exact_bootstrap_line')
        elif assembled == SITL_IRLOCK_BOOTSTRAP_LINE:
            self._finish_irlock_assembly(log, True, 'exact_line_after_first_valid_within_10s')

    def enqueue(self, data, log, guard=False):
        offset = self.received_bytes
        self.received_bytes += len(data)
        self.received_sha.update(data)
        log.record('stream_read', direction=self.name, bytes=len(data), stream_offset=offset,
                   chunk_sha256=hashlib.sha256(data).hexdigest())
        if guard:
            started = time.monotonic_ns()
            try:
                # The optional unarmed guard intentionally checks this complete
                # read before forwarding any of it. Previously forwarded partial
                # prefixes cannot form a complete dangerous frame without the
                # blocked suffix. This differs explicitly from transparent mode.
                self.decode_part(data, log, guard=True)
            except Exception as exc:
                self.guard_blocked_bytes += len(data)
                error = type(exc).__name__ + ': ' + str(exc)
                self.guard_violations.append(error)
                log.record('guard_blocked', direction=self.name, bytes=len(data), reason=error,
                           chunk_sha256=hashlib.sha256(data).hexdigest(), complete_stream_transparency=False)
                raise
            finally:
                self.max_decode_batch_ns = max(self.max_decode_batch_ns, time.monotonic_ns()-started)
        elif self.decode_errors:
            self.decoder_skipped_bytes += len(data)
        else:
            self.decode_queue.append(memoryview(data))
            self.decode_bytes += len(data)
            self.maximum_decode_pending = max(self.maximum_decode_pending, self.decode_bytes)
        self.forward.append(memoryview(data))
        self.pending_bytes += len(data)
        self.maximum_forward_pending = max(self.maximum_forward_pending, self.pending_bytes)

    def flush(self, log, budget=65536):
        while self.forward and budget > 0:
            view = self.forward[0][:budget]
            try:
                count = self.sink.send(view)
            except (BlockingIOError, InterruptedError):
                return
            if count <= 0:
                raise ConnectionError('A TCP send made no progress')
            sent = view[:count]
            offset = self.sent_bytes
            self.sent_sha.update(sent)
            self.sent_bytes += count
            self.pending_bytes -= count
            budget -= count
            original = self.forward.popleft()
            if count < len(original):
                self.forward.appendleft(original[count:])
            log.record('stream_write', direction=self.name, bytes=count, stream_offset=offset,
                       chunk_sha256=hashlib.sha256(sent).hexdigest())

    def decode_part(self, data, log, guard=False):
        self.last_decode_log = log
        for message in self.parser.parse_buffer(data) or []:
            observed_ns = self.monotonic_ns()
            kind = message.get_type()
            self.counts[kind] += 1
            raw = bytes(message.get_msgbuf())
            stream_offset = self.decoded_wire_bytes + self.bad_data_bytes
            startup_ascii = False
            if kind == 'BAD_DATA':
                self.bad_data_bytes += len(raw)
                startup_ascii = (self.allow_startup_ascii and not self.first_valid_frame_seen
                                 and getattr(message, 'reason', None) == 'Bad prefix' and bool(raw)
                                 and all(32 <= byte <= 126 or byte in (9, 10, 13) for byte in raw))
                if startup_ascii:
                    self.startup_ascii_bytes += len(raw)
                    self.startup_ascii_sha.update(raw)
                    classes = [('printable_ascii' if 32 <= byte <= 126 else
                                'tab' if byte == 9 else 'lf' if byte == 10 else 'cr') for byte in raw]
                    log.record('startup_ascii', direction=self.name, bytes=len(raw), stream_offset=stream_offset,
                               raw_hex=raw.hex(), raw_sha256=hashlib.sha256(raw).hexdigest(), byte_classes=classes,
                               classification='explicit_allowed_printable_bad_prefix_before_first_valid_mavlink_frame')
                else:
                    self.unclassified_bad_data_bytes += len(raw)
                    self.unclassified_bad_data_messages += 1
            else:
                self.decoded_wire_bytes += len(raw)
                if not self.first_valid_frame_seen:
                    self.first_valid_frame_monotonic_ns = observed_ns
                self.first_valid_frame_seen = True
            values = {'direction': self.name, 'type': kind,
                      'msgid': message.get_msgId(), 'sysid': message.get_srcSystem(),
                      'compid': message.get_srcComponent(), 'seq': message.get_seq(),
                      'wire_bytes': len(raw), 'wire_sha256': hashlib.sha256(raw).hexdigest(),
                      'stream_offset': stream_offset, 'decoder_observed_monotonic_ns': observed_ns}
            if kind == 'BAD_DATA':
                values['classified_startup_ascii'] = startup_ascii
            for field in ('time_usec', 'time_boot_ms'):
                if hasattr(message, field):
                    values[field] = getattr(message, field)
            if kind in {'ODOMETRY', 'HEARTBEAT', 'COMMAND_LONG', 'COMMAND_INT', 'BAD_DATA',
                        'PARAM_VALUE', 'SYS_STATUS'}:
                values['fields'] = message.to_dict()
            if kind == 'SYS_STATUS':
                for field in ('onboard_control_sensors_present', 'onboard_control_sensors_enabled',
                              'onboard_control_sensors_health'):
                    values[field] = getattr(message, field)
            if kind == 'ODOMETRY':
                values.update(frame_id=message.frame_id, child_frame_id=message.child_frame_id,
                              quality=message.quality)
            if kind == 'HEARTBEAT':
                values.update(armed=bool(message.base_mode & 128), base_mode=message.base_mode)
            if kind in {'COMMAND_LONG', 'COMMAND_INT'}:
                values.update(command=message.command, param1=message.param1)
            log.record('frame', **values)
            if kind == 'BAD_DATA' and not startup_ascii:
                self._observe_irlock_bad_prefix(raw, getattr(message, 'reason', None),
                                               stream_offset, observed_ns, log)
            elif kind != 'BAD_DATA':
                self._finish_irlock_assembly(log, False, 'interrupted_by_valid_mavlink_frame')
            if guard:
                if self.name == 'to_fcu' and (kind == 'BAD_DATA' or kind in BLOCKED_TYPES or
                        kind.startswith('SET_') or kind in {'COMMAND_LONG', 'COMMAND_INT'} and
                        message.command not in READ_ONLY_COMMANDS):
                    raise GuardViolation('Forbidden to_fcu ' + kind +
                                         (' command=' + str(message.command) if hasattr(message, 'command') else ''))
                if self.name == 'from_fcu' and kind == 'HEARTBEAT' and bool(message.base_mode & 128):
                    raise GuardViolation('FCU HEARTBEAT armed flag is set')

    def decode(self, log, budget):
        if self.decode_errors:
            return
        started = time.monotonic_ns()
        consumed = 0
        try:
            while self.decode_queue and consumed < budget:
                original = self.decode_queue.popleft()
                part = original[:budget-consumed]
                if len(part) < len(original):
                    self.decode_queue.appendleft(original[len(part):])
                self.decode_bytes -= len(part)
                consumed += len(part)
                self.decode_part(bytes(part), log)
        except Exception as exc:
            # Keep forwarding transparently; an incomplete decoder is visible
            # in the final FAIL and is never represented as zero wire messages.
            error = type(exc).__name__ + ': ' + str(exc)
            self.decode_errors.append(error)
            log.record('decoder_error', direction=self.name, error=error)
        self.max_decode_batch_ns = max(self.max_decode_batch_ns, time.monotonic_ns()-started)

    def result(self):
        if self.irlock_pending:
            self._finish_irlock_assembly(self.last_decode_log, False, 'truncated_at_observation_end')
        parser_pending = self.parser.buf_len()
        classified = (self.decode_bytes == 0 and parser_pending == 0 and not self.decode_errors
                      and not self.decoder_skipped_bytes and not self.unclassified_bad_data_messages
                      and self.decoded_wire_bytes + self.startup_ascii_bytes
                          + self.irlock_bootstrap_bytes == self.received_bytes)
        return {'received_bytes': self.received_bytes, 'sent_bytes': self.sent_bytes,
                'received_sha256': self.received_sha.hexdigest(), 'sent_sha256': self.sent_sha.hexdigest(),
                'forwarded_stream_identical': self.pending_bytes == 0 and self.received_bytes == self.sent_bytes
                    and self.received_sha.digest() == self.sent_sha.digest(),
                'unsent_bytes': self.pending_bytes, 'maximum_forward_pending_bytes': self.maximum_forward_pending,
                'maximum_decode_pending_bytes': self.maximum_decode_pending,
                'decoder_queued_bytes': self.decode_bytes, 'parser_partial_bytes': parser_pending,
                'message_counts': dict(self.counts), 'bad_data_bytes': self.bad_data_bytes,
                'bad_data_bytes_includes_classified_startup_ascii': True,
                'bad_data_bytes_includes_classified_irlock_bootstrap': True,
                'unclassified_bad_data_bytes': self.unclassified_bad_data_bytes,
                'unclassified_bad_data_messages': self.unclassified_bad_data_messages,
                'startup_ascii_allowed': self.allow_startup_ascii,
                'startup_ascii_bytes': self.startup_ascii_bytes,
                'startup_ascii_sha256': self.startup_ascii_sha.hexdigest(),
                'sitl_irlock_bootstrap_allowed': self.allow_irlock_bootstrap,
                'sitl_irlock_bootstrap_line_count': self.irlock_bootstrap_count,
                'sitl_irlock_bootstrap_bytes': self.irlock_bootstrap_bytes,
                'sitl_irlock_bootstrap_fragment_count': self.irlock_bootstrap_fragment_count,
                'sitl_irlock_bootstrap_sha256': self.irlock_bootstrap_sha.hexdigest(),
                'sitl_irlock_rejected_assemblies': self.irlock_rejected_assemblies,
                'sitl_irlock_bootstrap_maximum_after_first_valid_ns': SITL_IRLOCK_BOOTSTRAP_WINDOW_NS,
                'sitl_irlock_bootstrap_eligibility': 'from_fcu_bad_prefix_exact_line_after_first_valid_within_10s_once',
                'first_valid_mavlink_frame_seen': self.first_valid_frame_seen,
                'first_valid_mavlink_monotonic_ns': self.first_valid_frame_monotonic_ns,
                'all_observed_bytes_classified': classified,
                'decoded_wire_bytes': self.decoded_wire_bytes, 'decode_errors': self.decode_errors,
                'decoder_skipped_bytes': self.decoder_skipped_bytes,
                'guard_blocked_bytes': self.guard_blocked_bytes, 'guard_violations': self.guard_violations,
                'maximum_decode_batch_ns': self.max_decode_batch_ns,
                'source_eof_observed': self.source_eof, 'write_half_close_sent': self.shutdown_sent,
                'mavlink_decode_complete': classified and not self.guard_violations}


class Proxy:
    def __init__(self, args, log, dialect):
        self.args, self.log, self.dialect = args, log, dialect
        self.flows = []
        self.sockets = []
        self.requested_signal = None
        self.termination = 'not_started'
        self.target_attempts = 0
        self.client_address = None

    def stop(self, signum, _frame):
        self.requested_signal = signum

    def make_socket(self):
        peer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        peer.setblocking(False)
        peer.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        if self.args.socket_buffer_bytes:
            peer.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.args.socket_buffer_bytes)
            peer.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.args.socket_buffer_bytes)
        self.sockets.append(peer)
        return peer

    def target(self):
        deadline = time.monotonic() + self.args.connect_timeout_s
        while time.monotonic() < deadline and self.requested_signal is None:
            self.target_attempts += 1
            peer = self.make_socket()
            code = peer.connect_ex((self.args.target_host, self.args.target_port))
            while code in (errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY) and time.monotonic() < deadline:
                if self.requested_signal is not None:
                    break
                _, writable, exceptional = select.select([], [peer], [peer], .05)
                if writable or exceptional:
                    code = peer.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
                    break
            if code == 0 and self.requested_signal is None:
                self.log.record('target_connected', host=self.args.target_host, port=self.args.target_port,
                                attempts=self.target_attempts)
                return peer
            peer.close()
            self.sockets.remove(peer)
            select.select([], [], [], min(.05, max(0., deadline-time.monotonic())))
        raise TimeoutError('Target connection deadline or startup signal reached before listening')

    def accept(self):
        listener = self.make_socket()
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((self.args.listen_host, self.args.listen_port))
        listener.listen(1)
        address = listener.getsockname()
        self.log.record('ready', host=address[0], port=address[1], target_connected=True)
        print(json.dumps({'kind': 'ready', 'event': 'listening', 'host': address[0], 'port': address[1],
                          'target_connected': True}), flush=True)
        deadline = time.monotonic() + self.args.accept_timeout_s
        try:
            while time.monotonic() < deadline and self.requested_signal is None:
                readable, _, _ = select.select([listener], [], [], .05)
                if not readable:
                    continue
                client, remote = listener.accept()
                self.sockets.append(client)
                client.setblocking(False)
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                if self.args.socket_buffer_bytes:
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.args.socket_buffer_bytes)
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.args.socket_buffer_bytes)
                self.client_address = remote
                self.log.record('client_connected', address=list(remote), single_connection=True)
                return client
            raise TimeoutError('Single client accept deadline or startup signal reached')
        finally:
            listener.close()
            self.sockets.remove(listener)

    def run(self):
        target = self.target()
        client = self.accept()
        self.flows = [Flow(DIRECTIONS[0], client, target, self.dialect,
                           self.args.allow_fcu_startup_ascii, self.args.allow_sitl_irlock_bootstrap_line),
                      Flow(DIRECTIONS[1], target, client, self.dialect,
                           self.args.allow_fcu_startup_ascii, self.args.allow_sitl_irlock_bootstrap_line)]
        started = time.monotonic()
        drain_deadline = None
        self.termination = 'running'
        while True:
            runtime_expired = self.args.max_runtime_s > 0 and time.monotonic()-started >= self.args.max_runtime_s
            if drain_deadline is None and (self.requested_signal is not None or runtime_expired):
                self.termination = ('signal_' + signal.Signals(self.requested_signal).name
                                    if self.requested_signal is not None else 'max_runtime')
                drain_deadline = time.monotonic() + self.args.drain_timeout_s
                self.log.record('draining', reason=self.termination, unread_kernel_bytes_are_unknown=True)
            if drain_deadline is not None and time.monotonic() > drain_deadline:
                raise TimeoutError('Drain deadline reached with pending forwarding/decoder work')
            for flow in self.flows:
                if (flow.source_eof or drain_deadline is not None) and not flow.pending_bytes and not flow.shutdown_sent:
                    try:
                        flow.sink.shutdown(socket.SHUT_WR)
                    except OSError as exc:
                        if exc.errno not in (errno.ENOTCONN, errno.EPIPE):
                            raise
                    flow.shutdown_sent = True
                    self.log.record('write_half_close', direction=flow.name)
            done = all((flow.source_eof or drain_deadline is not None) and flow.pending_bytes == 0
                       and (flow.decode_bytes == 0 or flow.decode_errors) for flow in self.flows)
            if done:
                if drain_deadline is None:
                    self.termination = 'both_peer_eof'
                return
            read_flows = [flow for flow in self.flows if not flow.source_eof and drain_deadline is None
                          and flow.pending_bytes < self.args.max_buffer_bytes
                          and (flow.decode_errors or flow.decode_bytes < self.args.max_buffer_bytes)]
            write_flows = [flow for flow in self.flows if flow.pending_bytes]
            timeout = 0. if any(flow.decode_bytes and not flow.decode_errors for flow in self.flows) else .05
            readable, _, _ = select.select([f.source for f in read_flows], [f.sink for f in write_flows], [], timeout)
            for flow in read_flows:
                if flow.source not in readable:
                    continue
                capacity = self.args.max_buffer_bytes-flow.pending_bytes
                if not flow.decode_errors:
                    capacity = min(capacity, self.args.max_buffer_bytes-flow.decode_bytes)
                try:
                    data = flow.source.recv(min(65536, capacity))
                except (BlockingIOError, InterruptedError):
                    continue
                if not data:
                    flow.source_eof = True
                    self.log.record('source_eof', direction=flow.name)
                else:
                    flow.enqueue(data, self.log, guard=self.args.guard_unarmed)
            # Flush both existing AND newly received bytes before any decoding.
            # A nonwritable sink only raises EWOULDBLOCK and remains queued.
            for flow in self.flows:
                flow.flush(self.log)
            for flow in self.flows:
                flow.decode(self.log, self.args.decode_batch_bytes)

    def close(self):
        for peer in self.sockets:
            peer.close()


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--output-dir', type=Path, required=True)
    result.add_argument('--listen-host', default='127.0.0.1')
    result.add_argument('--listen-port', type=int, default=5761)
    result.add_argument('--target-host', default='127.0.0.1')
    result.add_argument('--target-port', type=int, default=5760)
    result.add_argument('--connect-timeout-s', type=float, default=45.)
    result.add_argument('--accept-timeout-s', type=float, default=45.)
    result.add_argument('--drain-timeout-s', type=float, default=5.)
    result.add_argument('--max-runtime-s', type=float, default=0.)
    result.add_argument('--max-buffer-bytes', type=int, default=4194304)
    result.add_argument('--decode-batch-bytes', type=int, default=4096)
    result.add_argument('--socket-buffer-bytes', type=int, default=0,
                        help='Optional OS socket buffer request for controlled backpressure tests; zero keeps OS defaults')
    result.add_argument('--pymavlink-site', default=DEFAULT_SITE)
    result.add_argument('--guard-unarmed', action='store_true',
                        help='Check each read before forwarding; block flight/actuator/parameter writes, non-read-only commands and armed FCU heartbeats')
    result.add_argument('--allow-fcu-startup-ascii', action='store_true',
                        help='Classify printable ASCII/Tab/CR/LF Bad-prefix bytes only from FCU before its first valid MAVLink frame; preserve every original byte; later bad data/CRC errors still fail')
    result.add_argument('--allow-sitl-irlock-bootstrap-line', action='store_true',
                        help='Additionally classify exactly one from-FCU Bad-prefix AP_IRLock_SITL::init() LF line after its first valid frame and within 10 monotonic seconds; retain original fragments/hashes; truncated/other/CRC bytes still fail')
    return result


def main(argv=None):
    cli = parser()
    args = cli.parse_args(argv)
    for name in ('listen_host', 'target_host'):
        try:
            address = ipaddress.IPv4Address(getattr(args, name))
        except ipaddress.AddressValueError:
            cli.error(name + ' must be an IPv4 loopback literal')
        if not address.is_loopback:
            cli.error(name + ' must be loopback; unrelated network clients are forbidden')
    if not 0 <= args.listen_port <= 65535 or not 1 <= args.target_port <= 65535:
        cli.error('Invalid port')
    if args.listen_port == args.target_port and args.listen_host == args.target_host:
        cli.error('Listen and target endpoints must differ')
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.connect_timeout_s, args.accept_timeout_s, args.drain_timeout_s)):
        cli.error('Startup/drain deadlines must be finite and positive')
    if not math.isfinite(args.max_runtime_s) or args.max_runtime_s < 0:
        cli.error('Runtime limit must be finite and nonnegative')
    if min(args.max_buffer_bytes, args.decode_batch_bytes) < 1 or args.socket_buffer_bytes < 0:
        cli.error('Invalid queue, decoder batch or socket buffer size')
    try:
        args.output_dir.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print(json.dumps({'status': 'FAIL', 'error': 'Output must be a new directory: ' + str(exc)}), file=sys.stderr)
        return 1
    log = EventLog(args.output_dir / 'traffic.jsonl')
    proxy = None
    metadata, errors = {}, []
    previous_handlers = {}
    started = time.monotonic_ns()
    try:
        dialect, metadata = load_dialect(args.pymavlink_site)
        proxy = Proxy(args, log, dialect)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, proxy.stop)
        log.record('startup', config={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
                   decoder=metadata, autonomous_mavlink_messages=0)
        proxy.run()
    except Exception as exc:
        errors.append(type(exc).__name__ + ': ' + str(exc))
        if proxy:
            proxy.termination = 'guard_blocked' if isinstance(exc, GuardViolation) else 'error'
        log.record('error', error=errors[-1])
    finally:
        if proxy:
            proxy.close()
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)
    flows = {flow.name: flow.result() for flow in proxy.flows} if proxy else {}
    transport_pass = len(flows) == 2 and all(row['forwarded_stream_identical'] for row in flows.values())
    decode_complete = len(flows) == 2 and all(row['mavlink_decode_complete'] for row in flows.values())
    code = 0 if transport_pass and decode_complete and not errors else 1
    result = {'schema_version': 1, 'status': 'PASS' if code == 0 else 'FAIL', 'exit_code': code,
              'scope': 'Transparent diagnostic byte transport and MAVLink observation only; not estimator, clock or flight validation',
              'termination_reason': proxy.termination if proxy else 'startup_failure',
              'signal_received': proxy.requested_signal if proxy else None,
              'elapsed_monotonic_ns': time.monotonic_ns()-started,
              'flows': flows, 'forwarding_integrity_pass': transport_pass,
              'all_mavlink_frames_decoded': decode_complete,
              'all_observed_bytes_classified': len(flows) == 2 and all(row['all_observed_bytes_classified'] for row in flows.values()),
              'allow_fcu_startup_ascii': args.allow_fcu_startup_ascii,
              'allow_sitl_irlock_bootstrap_line': args.allow_sitl_irlock_bootstrap_line,
              'decoding_scope': 'all_mavlink_frames_decoded excludes explicitly classified pre-first-frame FCU printable startup ASCII only with its original option, and one exact AP_IRLock_SITL::init() LF Bad-prefix line within 10 monotonic seconds after the first valid FCU frame only with its separate explicit option; all other BAD_DATA/non-ASCII/CRC/truncation remains failure',
              'autonomous_mavlink_messages': 0, 'wire_bytes_reencoded': False,
              'guard_unarmed': args.guard_unarmed,
              'guard_read_only_commands': sorted(READ_ONLY_COMMANDS) if args.guard_unarmed else None,
              'guard_blocking_occurred': any(row['guard_violations'] for row in flows.values()),
              'target_connection_attempts': proxy.target_attempts if proxy else 0,
              'client_address': list(proxy.client_address) if proxy and proxy.client_address else None,
              'pymavlink': metadata, 'errors': errors,
              'timing_overhead_assumed_zero': False,
              'stop_boundary': 'On a stop request, already-read bytes are drained; unread kernel/peer bytes are outside the observed stream.'}
    log.record('finished', status=result['status'], exit_code=code, termination_reason=result['termination_reason'])
    log.close()
    result['traffic_jsonl_sha256'] = hashlib.sha256((args.output_dir / 'traffic.jsonl').read_bytes()).hexdigest()
    result['traffic_events'] = log.events
    with (args.output_dir / 'summary.json').open('x', encoding='utf-8') as stream:
        json.dump(json_safe(result), stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'event': 'finished', 'status': result['status'], 'exit_code': code,
                      'summary': str(args.output_dir / 'summary.json')}), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
