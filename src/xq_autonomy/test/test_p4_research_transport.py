"""Opt-in transport/source/graph integration with actual ROS message types."""
import json
from types import SimpleNamespace

from rosgraph_msgs.msg import Clock
from rclpy.time import Time

from xq_autonomy.extnav_transport_clock import ExtnavTransportClock
from xq_autonomy.extnav_startup_gate import ExtnavStartupGate
from test_p4_external_nav import _adapter, _pose


def mapped_adapter(monkeypatch):
    node, now, output, status = _adapter(monkeypatch)
    node.transport_clock_mode = 'epoch_anchor_plus_monotonic'
    node.transport_mapper = ExtnavTransportClock()
    node.require_acquisition_clock = True
    node.last_acquisition_clock_ns = 0
    node.last_acquisition_clock_monotonic_ns = 0
    node.acquisition_clock_fault = False
    node.ledger, node.ledger_publisher = [], None
    node.ledger_file = SimpleNamespace(write=node.ledger.append)
    return node, now, output, status


def feed(node, stamp):
    node._acquisition_clock_cb(Clock(clock=Time(nanoseconds=stamp).to_msg()))
    node._odom_cb(_pose(stamp))


def test_mapping_does_not_forward_a_wall_clock_backstep_or_hide_it(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    feed(node, 1_000_000_000)
    now[0] += .1
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=1_789_999_999_700_000_000))
    feed(node, 1_100_000_000)
    assert len(output) == 2
    stamps = [message.header.stamp.sec*10**9+message.header.stamp.nanosec for message in output]
    assert stamps[1]-stamps[0] == 100_000_000
    record = json.loads(node.ledger[-1])
    assert record['raw_acquisition_stamp_ns'] == 1_100_000_000
    assert record['sampling']['raw_wall_anomaly'] is True
    assert record['sampling']['raw_wall_not_increasing'] is True
    assert record['decision'] == 'forwarded'
    assert record['sampling']['mapped_ns'] == stamps[1]
    node._status_cb()
    status = json.loads(statuses[-1].data)
    assert status['transport_clock_mapping']['ever_raw_wall_anomaly'] is True
    assert status['clock_fault_latched'] is False
    assert status['clock_guard']['transport_clock_domain'] == 'host_epoch_anchor_plus_monotonic'


def test_mapping_cannot_make_duplicate_or_old_acquisition_fresh(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    feed(node, 1_000_000_000)
    accepted = node.last_source_wall
    now[0] += .1
    feed(node, 1_000_000_000)
    assert len(output) == 1
    assert node.last_source_wall == accepted
    assert node.clock_guard.snapshot(int(now[0]*1e9))['fault_latched'] is True
    assert json.loads(node.ledger[-1])['decision'] == 'rejected'


def test_old_advancing_queue_is_rejected_against_independent_clock(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    node._acquisition_clock_cb(Clock(clock=Time(nanoseconds=2_000_000_000).to_msg()))
    node._odom_cb(_pose(1_000_000_000))
    assert output == []
    assert node.last_rejection_reason == 'source_acquisition_stamp_stale'
    assert node.last_source_wall == 0
    assert json.loads(node.ledger[-1])['sampling']['mapped_ns'] > 0


def test_repeated_clock_callbacks_do_not_refresh_frozen_acquisition_clock(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    feed(node, 1_000_000_000)
    now[0] += .4
    node._acquisition_clock_cb(Clock(clock=Time(nanoseconds=1_000_000_000).to_msg()))
    node._odom_cb(_pose(1_100_000_000))
    assert len(output) == 1
    assert node.last_rejection_reason == 'acquisition_clock_missing_or_stale'


def test_acquisition_clock_regression_latches_even_when_mapping_advances(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    feed(node, 1_000_000_000)
    now[0] += .1
    node._acquisition_clock_cb(Clock(clock=Time(nanoseconds=900_000_000).to_msg()))
    feed(node, 1_100_000_000)
    assert len(output) == 1
    node._status_cb()
    assert json.loads(statuses[-1].data)['clock_fault_latched'] is True


def graph_owner():
    return {'node_name':'odometry','node_namespace':'/uav1/mavros','topic_type':'nav_msgs/msg/Odometry',
            'endpoint_gid':[1,2,3], 'qos':{'reliability':'RELIABLE','durability':'VOLATILE'}}


class ImmediateGraph:
    def __init__(self):
        self.endpoints = [graph_owner()]
        self.stuck = False
    def poll(self, now):
        return None if self.stuck else {'endpoints':self.endpoints,'query_within_budget':True,
            'query_finished_monotonic_ns':now,'query_started_monotonic_ns':now,'query_duration_ns':0,'query_error':None}
    def snapshot(self, now):
        return {'pending_exceeds_budget':self.stuck}


def test_exact_graph_warmup_skips_without_queueing_or_replaying_old_frames(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    node.startup_gate_enabled = True
    node.startup_gate = ExtnavStartupGate()
    node.graph_query = ImmediateGraph()
    node.last_graph_query_result = None
    node.startup_skipped_count = 0
    for index in range(12):
        now[0] = 100+index*.1
        feed(node, 1_000_000_000+index*100_000_000)
    assert node.startup_skipped_count == 10
    assert len(output) == 2
    assert node.last_source_stamp_ns == 2_100_000_000
    assert [json.loads(line)['decision'] for line in node.ledger[:10]] == ['startup_skipped']*10
    assert json.loads(node.ledger[10])['raw_acquisition_stamp_ns'] == 2_000_000_000
    node.graph_query.endpoints = []
    now[0] += .1
    feed(node, 2_200_000_000)
    assert len(output) == 2
    node._status_cb()
    assert json.loads(statuses[-1].data)['healthy'] is False


def test_unrelated_subscriber_and_stuck_query_never_open_gate(monkeypatch):
    node, now, output, statuses = mapped_adapter(monkeypatch)
    node.startup_gate_enabled = True
    node.startup_gate = ExtnavStartupGate()
    node.graph_query = ImmediateGraph()
    node.graph_query.endpoints[0]['node_name'] = 'unrelated_diagnostic'
    node.last_graph_query_result = None
    node.startup_skipped_count = 0
    for index in range(20):
        now[0] = 100+index*.1
        feed(node, 1_000_000_000+index*100_000_000)
    assert output == []
    assert node.publisher.get_subscription_count() == 1
    node.graph_query.stuck = True
    now[0] += .4
    feed(node, 3_300_000_000)
    assert output == []
