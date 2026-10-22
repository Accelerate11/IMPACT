"""ROS-message hook tests with fake time/publishers/processes only.

No rclpy.init(), ROS nodes, subprocesses, AP/Gazebo, SQLite, simulator pause
service, or sim freeze helper is invoked. The predeclared *source-publication*
pause deliberately leaves the simulated acquisition clock advancing. These
tests therefore do not compete with the separate sim freeze helper.

Reproduce in WSL after sourcing /opt/ros/humble/setup.bash:
  PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 python3 -m pytest \
    scripts/test/test_extnav_guard_probe_hooks.py -v -p no:cacheprovider
"""
from collections import Counter
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy', reason='Source ROS Humble to exercise real ROS message types')
from mavros_msgs.msg import State
from rosgraph_msgs.msg import Clock as ClockMessage


SCRIPTS = Path(__file__).resolve().parents[1]
NS = 1_000_000_000


def load_script(name, filename):
    previous = list(sys.path)
    try:
        sys.path.insert(0, str(SCRIPTS))
        spec = importlib.util.spec_from_file_location(name, SCRIPTS/filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path[:] = previous


PROBE = load_script('extnav_guard_probe_hooks_ros', 'probe_extnav_ros.py')
RUNNER = load_script('extnav_guard_probe_hooks_runner', 'run_extnav_guard_probe.py')


class FakeClock:
    def __init__(self):
        self.now_ns = 100 * NS
        self.wall_offset_ns = 1_790_000_000 * NS
        self.sleeps = []

    def monotonic_ns(self):
        return self.now_ns

    def monotonic(self):
        return self.now_ns/NS

    def time_ns(self):
        return self.wall_offset_ns + self.now_ns

    def sleep(self, seconds):
        self.sleeps.append(seconds)
        self.now_ns += round(seconds*NS)


@pytest.fixture
def synthetic(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr(PROBE, 'time', clock)
    # Bypass Node.__init__: this is a callback harness, not a live ROS node.
    probe = object.__new__(PROBE.ExternalNavProbe)
    probe.source_mode = 'synthetic'
    probe.events = io.StringIO()
    probe.counts, probe.synthetic_skips = Counter(), Counter()
    probe.event_sequence = 0
    probe.stop_reason = ''
    probe.armed_seen = probe.connected_seen = probe.disarmed_seen = False
    probe.connected_disarmed_seen = False
    probe.current_connected, probe.current_armed = False, None
    probe.clock_seen = False
    probe.latest_clock_ns = probe.last_published_stamp_ns = 0
    probe.synthetic_first_publish_ns = None
    probe.pause_after_ns, probe.pause_duration_ns = 20*NS, 2*NS
    probe.pause_started = probe.pause_ended = False
    probe.clock_regressions = 0
    published = []
    probe.source_publisher = SimpleNamespace(publish=published.append)
    return probe, clock, published


def set_state(probe, *, connected=True, armed=False):
    message = State()
    message.connected, message.armed = connected, armed
    probe._state(message)


def source_clock(probe, value):
    message = ClockMessage()
    message.clock.sec, message.clock.nanosec = divmod(value, NS)
    probe._on_clock(message)


def rows(probe, kind=None):
    values = [json.loads(line) for line in probe.events.getvalue().splitlines()]
    return [v for v in values if kind is None or v['kind'] == kind]


def message_stamp(message):
    return message.header.stamp.sec*NS + message.header.stamp.nanosec


def start_source(probe):
    set_state(probe)
    source_clock(probe, 7*NS)
    probe._publish_synthetic()


def test_source_pause_starts_at_20s_and_resumes_at_22s_with_latest_stamp(synthetic):
    probe, clock, published = synthetic
    start_source(probe)
    anchor = clock.now_ns
    assert probe.synthetic_first_publish_ns == anchor
    assert len(published) == 1
    clock.now_ns = anchor + 20*NS - 1
    source_clock(probe, 31*NS)
    probe._publish_synthetic()
    assert len(published) == 2
    before = probe.last_published_stamp_ns
    # Acquisition /clock continues advancing while only source publication stops.
    for index in range(20):
        clock.now_ns = anchor + 20*NS + index*100_000_000
        source_clock(probe, 32*NS + index*100_000_000)
        probe._publish_synthetic()
        assert len(published) == 2
        assert probe.last_published_stamp_ns == before
    assert probe.synthetic_skips['predeclared_source_pause'] == 20
    assert len(rows(probe, 'source_pause_started')) == 1
    assert rows(probe, 'source_pause_started')[0]['elapsed_since_first_source_ns'] == 20*NS
    assert not rows(probe, 'source_pause_ended')
    # No held pause-frame queue is drained: only the current clock is published.
    clock.now_ns = anchor + 22*NS
    latest = 90*NS + 123
    source_clock(probe, latest)
    probe._publish_synthetic()
    assert len(published) == 3
    assert message_stamp(published[-1]) == probe.last_published_stamp_ns == latest
    assert len(rows(probe, 'source_published')) == 3
    assert len(rows(probe, 'source_pause_ended')) == 1
    assert rows(probe, 'source_pause_ended')[0]['elapsed_since_first_source_ns'] == 22*NS
    assert all(not (anchor+20*NS <= v['monotonic_ns'] < anchor+22*NS)
               for v in rows(probe, 'source_published'))


def test_pause_events_are_single_and_wall_step_cannot_advance_pause(synthetic):
    probe, clock, published = synthetic
    start_source(probe)
    anchor = clock.now_ns
    clock.wall_offset_ns += 500*NS
    clock.now_ns = anchor + 19*NS
    source_clock(probe, 8*NS)
    probe._publish_synthetic()
    assert not probe.pause_started
    clock.now_ns = anchor + 20*NS
    source_clock(probe, 9*NS)
    probe._publish_synthetic()
    clock.now_ns = anchor + 22*NS
    source_clock(probe, 10*NS)
    probe._publish_synthetic()
    clock.now_ns = anchor + 23*NS
    source_clock(probe, 11*NS)
    probe._publish_synthetic()
    assert len(rows(probe, 'source_pause_started')) == len(rows(probe, 'source_pause_ended')) == 1
    assert len(published) == 4
    assert all(v['diagnostic_synthetic'] for v in rows(probe, 'source_published'))


def test_disarmed_and_clock_gates_precede_first_source_and_pause_anchor(synthetic):
    probe, clock, published = synthetic
    source_clock(probe, 7*NS)
    probe._publish_synthetic()
    assert not published and probe.synthetic_first_publish_ns is None
    assert probe.synthetic_skips['await_connected_disarmed'] == 1
    # Waiting does not consume the first publication's 20s pause offset.
    clock.now_ns += 30*NS
    set_state(probe)
    probe._publish_synthetic()
    assert len(published) == 1
    assert probe.synthetic_first_publish_ns == clock.now_ns
    assert not probe.pause_started


def test_clock_is_required_after_connected_disarmed_and_frozen_stamp_is_not_replayed(synthetic):
    probe, clock, published = synthetic
    set_state(probe)
    probe._publish_synthetic()
    assert not published and probe.synthetic_first_publish_ns is None
    assert probe.synthetic_skips['await_clock'] == 1
    source_clock(probe, 7*NS)
    probe._publish_synthetic()
    probe._publish_synthetic()
    assert len(published) == 1
    assert probe.synthetic_skips['clock_not_advancing'] == 1


@pytest.mark.parametrize('cause', ['disconnected', 'armed', 'previously_armed', 'stop_reason'])
def test_pause_resume_does_not_bypass_safety_gates(synthetic, cause):
    probe, clock, published = synthetic
    start_source(probe)
    clock.now_ns += 20*NS
    source_clock(probe, 8*NS)
    probe._publish_synthetic()
    assert probe.pause_started and len(published) == 1
    clock.now_ns += 2*NS
    source_clock(probe, 9*NS)
    if cause == 'disconnected':
        set_state(probe, connected=False)
    elif cause == 'armed':
        set_state(probe, armed=True)
    elif cause == 'previously_armed':
        probe.armed_seen = True
    else:
        probe.stop_reason = 'diagnostic_stop'
    probe._publish_synthetic()
    assert len(published) == 1
    assert not probe.pause_ended
    assert not rows(probe, 'source_pause_ended')
    if cause == 'disconnected':
        set_state(probe)
        source_clock(probe, 10*NS)
        probe._publish_synthetic()
        assert message_stamp(published[-1]) == 10*NS
    elif cause == 'armed':
        assert probe.stop_reason == 'armed_state_observed'
        set_state(probe, armed=False)
        probe._publish_synthetic()
        assert len(published) == 1  # Prior armed observation is terminal.


def test_pause_resume_keeps_zero_motion_frames_and_covariance(synthetic):
    probe, clock, published = synthetic
    start_source(probe)
    clock.now_ns += 20*NS
    source_clock(probe, 8*NS)
    probe._publish_synthetic()
    clock.now_ns += 2*NS
    source_clock(probe, 9*NS)
    probe._publish_synthetic()
    for message in published:
        assert (message.header.frame_id, message.child_frame_id) == ('xq_lio_map', 'livox_imu')
        assert (message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z) == (0., 0., 0.)
        assert (message.pose.pose.orientation.w, message.pose.pose.orientation.x,
                message.pose.pose.orientation.y, message.pose.pose.orientation.z) == (1., 0., 0., 0.)
        assert (message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.linear.z) == (0., 0., 0.)
        assert [message.pose.covariance[i] for i in (0, 7, 14)] == [0.0025]*3
        assert [message.pose.covariance[i] for i in (21, 28, 35)] == [0.0012]*3
        assert [message.twist.covariance[i] for i in (0, 7, 14, 21, 28, 35)] == [0.04]*6


class FakeProcess:
    def __init__(self, clock, finish_after_ns=None, code=0):
        self.clock = clock
        self.finish_ns = None if finish_after_ns is None else clock.now_ns + finish_after_ns
        self.code, self.returncode = code, None
        self.poll_count = 0

    def poll(self):
        self.poll_count += 1
        if self.finish_ns is not None and self.clock.now_ns >= self.finish_ns:
            self.returncode = self.code
        return self.returncode


def tail_fixture(monkeypatch, *, finish_after_ns=300_000_000, code=0):
    clock = FakeClock()
    monkeypatch.setattr(RUNNER, 'time', clock)
    diagnostic = object.__new__(RUNNER.GuardDiagnostic)
    diagnostic.stage = 'observer_finished'
    diagnostic.readiness = FakeProcess(clock, finish_after_ns, code)
    diagnostic.groups = [
        ('observer', FakeProcess(clock, 0, 0)), ('readiness', diagnostic.readiness),
        *[(name, FakeProcess(clock)) for name in ('gazebo', 'sitl', 'wire', 'mavros', 'adapter')],
    ]
    return diagnostic, clock


@pytest.mark.parametrize('finish_after_ns', [0, 300_000_000])
def test_tail_observation_completes_normally_with_only_finished_observer_exempt(monkeypatch, finish_after_ns):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=finish_after_ns)
    diagnostic.after_observer()
    assert diagnostic.stage == 'readiness_tail_observation'
    assert diagnostic.readiness.returncode == 0
    assert clock.now_ns == 100*NS + finish_after_ns
    assert all(p.returncode is None for label, p in diagnostic.groups if label not in {'observer', 'readiness'})


@pytest.mark.parametrize('name', ['gazebo', 'sitl', 'wire', 'mavros', 'adapter'])
@pytest.mark.parametrize('code', [0, 7])
def test_tail_observation_rejects_any_other_owned_early_exit(monkeypatch, name, code):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=2*NS)
    diagnostic.groups = [(label, FakeProcess(clock, 100_000_000, code) if label == name else process)
                         for label, process in diagnostic.groups]
    with pytest.raises(RuntimeError, match='Owned '+name+' exited during readiness tail'):
        diagnostic.after_observer()
    assert clock.now_ns < 102*NS


def test_tail_checks_owned_processes_again_when_monitor_finishes_between_polls(monkeypatch):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=200_000_000)
    diagnostic.groups.append(('range_bridge', FakeProcess(clock, 190_000_000, 0)))
    with pytest.raises(RuntimeError, match='Owned range_bridge exited during readiness tail'):
        diagnostic.after_observer()


def test_tail_rejects_already_exited_owned_process_even_if_monitor_already_completed(monkeypatch):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=0)
    diagnostic.groups.append(('range_bridge', FakeProcess(clock, 0, 9)))
    with pytest.raises(RuntimeError, match='Owned range_bridge exited during readiness tail'):
        diagnostic.after_observer()


@pytest.mark.parametrize('code', [1, 2, -15])
@pytest.mark.parametrize('finish_after_ns', [0, 200_000_000])
def test_tail_rejects_monitor_nonzero_without_waiting_for_deadline(monkeypatch, code, finish_after_ns):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=finish_after_ns, code=code)
    with pytest.raises(RuntimeError, match='read-only readiness monitor did not finish safely'):
        diagnostic.after_observer()
    assert clock.now_ns == 100*NS + finish_after_ns
    assert diagnostic.readiness.returncode == code


def test_tail_times_out_at_30_fake_seconds_without_real_sleep(monkeypatch):
    diagnostic, clock = tail_fixture(monkeypatch, finish_after_ns=None)
    with pytest.raises(RuntimeError, match='read-only readiness monitor did not finish safely'):
        diagnostic.after_observer()
    assert clock.now_ns == 130*NS
    assert len(clock.sleeps) == 300
    assert diagnostic.readiness.returncode is None
