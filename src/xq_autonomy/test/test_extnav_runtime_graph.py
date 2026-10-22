import threading
import time

from xq_autonomy.extnav_runtime_graph import BoundedGraphQuery


def test_stuck_query_does_not_block_executor_or_spawn_more_threads():
    entered, release = threading.Event(), threading.Event()
    def query():
        entered.set()
        release.wait(2)
        return []
    worker = BoundedGraphQuery(query)
    try:
        start = time.monotonic()
        assert worker.poll(time.monotonic_ns()) is None
        assert time.monotonic()-start < .1
        assert entered.wait(.5)
        for _ in range(20):
            assert worker.poll(time.monotonic_ns()) is None
        assert worker.started_count == 1
    finally:
        worker.close()
        release.set()


def test_completed_slow_query_is_classified_as_outside_budget():
    done = threading.Event()
    def query():
        time.sleep(.03)
        done.set()
        return [{'node_name':'odometry'}]
    worker = BoundedGraphQuery(query, maximum_duration_ns=1_000_000)
    worker.poll(time.monotonic_ns())
    assert done.wait(.5)
    worker.close()
    result = None
    deadline = time.monotonic()+.5
    while result is None and time.monotonic() < deadline:
        result = worker.poll(time.monotonic_ns())
    assert result['query_within_budget'] is False
    assert result['query_duration_ns'] > 1_000_000
