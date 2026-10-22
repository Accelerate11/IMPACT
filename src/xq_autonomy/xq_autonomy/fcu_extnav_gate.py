"""Current local/FCU receipt gate, valid both disarmed and during flight.

This checks sampled SYS_STATUS backend receipt health, never per-frame ACK,
measurement quality, EKF fusion, or an independent permission to fly.
"""
from __future__ import annotations

from copy import deepcopy
import struct


class CurrentExtnavReadiness:
    MAX_AGE_NS = 350_000_000
    HEALTH_DWELL_NS = 5_000_000_000

    def __init__(self):
        self.local = None
        self.local_received_ns = None
        self.sys_received_ns = None
        self.sys_healthy = False
        self.sys_masks = None
        self.healthy_start_ns = None
        self.healthy_last_ns = None
        self.clock_fault_latched = False
        self.last_evaluated_ns = None
        self.wrong_identity_count = 0

    def observe_local(self, status, received_ns):
        if type(status) is not dict:
            self.local = None
            self.local_received_ns = received_ns
            return
        self.local = deepcopy(status)
        self.local_received_ns = received_ns
        self.clock_fault_latched |= status.get('clock_fault_latched') is True

    def observe_sys_status(self, *, system_id, component_id, message_id,
                           framing_status, magic, payload_length, payload64,
                           received_ns):
        if message_id != 1:
            return
        if type(system_id) is not int or type(component_id) is not int or (system_id, component_id) != (1, 1):
            self.wrong_identity_count += 1
            return
        previous = self.sys_received_ns
        self.sys_received_ns = received_ns
        healthy = False
        self.sys_masks = None
        try:
            if type(received_ns) is not int or received_ns <= 0 or (previous is not None and received_ns <= previous):
                self.clock_fault_latched = True
                raise ValueError('Invalid/nonadvancing SYS_STATUS receive monotonic clock')
            if type(framing_status) is not int or framing_status != 1 or type(magic) is not int or magic not in (253, 254):
                raise ValueError('SYS_STATUS frame is invalid')
            # AP 4.7.1's pinned MAVLink SYS_STATUS has 31 base bytes and
            # 12 extension bytes. MAVLink2 can trim trailing zeros; the legacy
            # present/enabled/health masks remain the first 12 payload bytes.
            if type(payload_length) is not int or not 1 <= payload_length <= 43 or (magic == 254 and payload_length != 31):
                raise ValueError('SYS_STATUS length is invalid')
            if not isinstance(payload64, (list, tuple)) or not 0 < len(payload64) <= 33 or len(payload64)*8 < payload_length:
                raise ValueError('SYS_STATUS payload64 is incomplete')
            if any(type(value) is not int or not 0 <= value < 2**64 for value in payload64):
                raise ValueError('SYS_STATUS payload64 values are invalid')
            raw = b''.join(struct.pack('<Q', value) for value in payload64)[:payload_length]
            masks = struct.unpack('<III', raw.ljust(12, b'\x00')[:12])
            self.sys_masks = dict(zip(('present', 'enabled', 'health'), masks))
            healthy = all(mask & 128 for mask in masks)
        except (ValueError, TypeError, struct.error):
            pass
        self.sys_healthy = healthy
        if not healthy:
            self.healthy_start_ns = self.healthy_last_ns = None
        else:
            if self.healthy_start_ns is None or previous is None or received_ns-previous > self.MAX_AGE_NS:
                self.healthy_start_ns = received_ns
            self.healthy_last_ns = received_ns

    def snapshot(self, now_ns, *, connected, state_received_ns, parameters_valid):
        reasons = []
        if type(now_ns) is not int or now_ns <= 0 or (self.last_evaluated_ns is not None and now_ns < self.last_evaluated_ns):
            self.clock_fault_latched = True
        self.last_evaluated_ns = now_ns
        def fresh(value, limit=self.MAX_AGE_NS):
            return type(value) is int and type(now_ns) is int and 0 <= now_ns-value <= limit
        if self.clock_fault_latched:
            reasons.append('clock_fault_latched')
        if not parameters_valid:
            reasons.append('FCU_parameters_not_verified')
        if not connected or not fresh(state_received_ns, 2_000_000_000):
            reasons.append('FCU_state_missing_stale_or_disconnected')
        if not fresh(self.local_received_ns):
            reasons.append('local_status_missing_or_stale')
        local = self.local or {}
        for field in ('healthy', 'local_source_ready', 'local_clock_ready', 'exact_mavros_startup_ready'):
            if local.get(field) is not True:
                reasons.append('local_'+field+'_false')
        if local.get('exact_mavros_startup_gate_enabled') is not True:
            reasons.append('exact_startup_gate_not_enabled')
        if local.get('acquisition_clock_check_enabled') is not True:
            reasons.append('independent_acquisition_clock_check_not_enabled')
        if not fresh(local.get('last_accepted_source_monotonic_ns')):
            reasons.append('advancing_source_missing_or_stale')
        if not fresh(self.sys_received_ns):
            reasons.append('FCU_SYS_STATUS_missing_or_stale')
            self.healthy_start_ns = self.healthy_last_ns = None
        if not self.sys_healthy:
            reasons.append('FCU_vision_position_bits_false')
        dwell = (0 if self.healthy_start_ns is None or self.healthy_last_ns is None else
                 self.healthy_last_ns-self.healthy_start_ns)
        if dwell < self.HEALTH_DWELL_NS:
            reasons.append('FCU_receipt_health_warming_up')
        return {'schema_version': 1, 'ready': not reasons, 'reasons': reasons,
                'clock_fault_latched': self.clock_fault_latched,
                'local_received_monotonic_ns': self.local_received_ns,
                'FCU_SYS_STATUS_received_monotonic_ns': self.sys_received_ns,
                'FCU_healthy_sample_dwell_ns': dwell, 'FCU_masks': deepcopy(self.sys_masks),
                'wrong_identity_count': self.wrong_identity_count,
                'armed_state_is_not_itself_a_fault': True,
                'scope': 'Current local validity and sampled FCU backend receipt only; no per-frame ACK, quality or EKF fusion proof'}
