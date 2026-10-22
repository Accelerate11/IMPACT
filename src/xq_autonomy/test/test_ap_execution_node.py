"""Exercise the execution wrapper with a real, isolated rclpy context."""

import math
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.clock import ClockType
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor

from xq_autonomy.ap_execution import APVelocityExecutor
from xq_autonomy.ap_execution_node import APExecutionNode, yaw_of


def test_heading_normalizes_a_scaled_valid_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=2.0 * math.sin(0.4), w=2.0 * math.cos(0.4))
    assert yaw_of(q) == pytest.approx(0.8)


@pytest.mark.parametrize("q", [
    SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
    SimpleNamespace(x=math.nan, y=0.0, z=0.0, w=1.0),
])
def test_invalid_heading_is_rejected_before_status_serialization(q):
    with pytest.raises(ValueError):
        yaw_of(q)


def test_execution_node_constructs_spins_and_preserves_ros_executor():
    context = Context()
    node = None
    executor = None
    # Use a private context and domain, leaving the default context and any
    # simultaneously running flight stack untouched.
    rclpy.init(args=[], context=context, domain_id=211)
    try:
        node = APExecutionNode(context=context)
        assert isinstance(node.velocity_executor, APVelocityExecutor)
        assert node.executor is None

        timers = list(node.timers)
        assert len(timers) == 1
        assert timers[0].timer_period_ns == 50_000_000
        assert timers[0].clock.clock_type == ClockType.STEADY_TIME
        assert timers[0].callback == node._tick

        executor = SingleThreadedExecutor(context=context)
        assert executor.add_node(node)
        assert node.executor is executor
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)
            if node.last_status_wall > 0.0:
                break
        assert node.last_status_wall > 0.0
        assert node.published == 0
        assert node.last_reason == "disabled"
        assert node.executor is executor

        node.executor = None
        assert node.executor is None
        assert executor.get_nodes() == []
        node.destroy_node()
        node = None
    finally:
        if executor is not None:
            if node is not None:
                node.executor = None
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
