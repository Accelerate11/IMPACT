"""Atomic command provenance at the shared P11/P12/P13 publishing boundary."""

import json
from types import SimpleNamespace

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time

from xq_autonomy.p11_flight_controller_node import _ProvenanceCommandPublisher


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _boundary(*, generation_ns=7_123_456_789, odometry=None):
    clock = SimpleNamespace(now=lambda: Time(nanoseconds=generation_ns))
    node = SimpleNamespace(_odom=odometry, get_clock=lambda: clock)
    legacy, stamped, provenance = (_CapturePublisher() for _ in range(3))
    publisher = _ProvenanceCommandPublisher(node, legacy, stamped, provenance)
    return publisher, node, legacy, stamped, provenance


def _odometry(stamp_ns):
    message = Odometry()
    message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    return message


def _command():
    message = Twist()
    message.linear.x, message.linear.y, message.linear.z = 0.20, -0.15, 0.08
    message.angular.x, message.angular.y, message.angular.z = 0.01, -0.02, 0.03
    return message


def test_atomic_command_retains_both_times_frame_and_all_velocity_components():
    generation_ns, odometry_ns = 7_123_456_789, 6_987_654_321
    publisher, _, legacy, stamped, provenance = _boundary(
        generation_ns=generation_ns, odometry=_odometry(odometry_ns)
    )
    command = _command()

    publisher.publish(command)

    assert legacy.messages == [command]
    output = stamped.messages[0]
    record = json.loads(provenance.messages[0].data)
    assert output.twist == command
    assert output.header.frame_id == record["frame_id"] == "xq_lio_map"
    assert Time.from_msg(output.header.stamp).nanoseconds == generation_ns
    assert record["generation_stamp_ns"] == generation_ns
    assert record["odometry_stamp_ns"] == odometry_ns
    assert record["sequence_id"] == 1
    assert record["linear"] == {"x": 0.20, "y": -0.15, "z": 0.08}
    assert record["angular"] == {"x": 0.01, "y": -0.02, "z": 0.03}


def test_missing_odometry_is_explicit_and_next_command_uses_latest_source_time():
    publisher, node, _, _, provenance = _boundary()
    publisher.publish(Twist())
    node._odom = _odometry(5_000_000_001)
    publisher.publish(_command())
    records = [json.loads(message.data) for message in provenance.messages]

    assert [record["odometry_stamp_ns"] for record in records] == [0, 5_000_000_001]
    assert records[0]["linear"] == {"x": 0.0, "y": 0.0, "z": 0.0}
    assert records[0]["angular"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_repeated_ros_time_keeps_actual_stamp_and_distinct_sequence_ids():
    publisher, _, _, stamped, provenance = _boundary()
    publisher.publish(_command())
    publisher.publish(Twist())
    records = [json.loads(message.data) for message in provenance.messages]

    assert [record["sequence_id"] for record in records] == [1, 2]
    assert records[0]["generation_stamp_ns"] == records[1]["generation_stamp_ns"]
    assert stamped.messages[0].header.stamp == stamped.messages[1].header.stamp
    assert records[0]["linear"] != records[1]["linear"]


def test_audit_snapshot_cannot_change_when_callers_reuse_command_object():
    publisher, _, _, stamped, provenance = _boundary()
    command = _command()
    publisher.publish(command)
    command.linear.x = 9.0

    assert stamped.messages[0].twist.linear.x == 0.20
    assert json.loads(provenance.messages[0].data)["linear"]["x"] == 0.20
