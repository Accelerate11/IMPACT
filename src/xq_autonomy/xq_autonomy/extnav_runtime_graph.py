"""Bound a local ROS graph query without blocking source/status callbacks.

Only one daemon worker can be outstanding. A slow or stuck RMW graph lookup
cannot create a new query, refresh evidence, or hold the executor callback.
"""
from __future__ import annotations

import threading
import time


class BoundedGraphQuery:
    def __init__(self, query, *, maximum_duration_ns=50_000_000):
        self.query = query
        self.maximum_duration_ns = maximum_duration_ns
        self._lock = threading.Lock()
        self._pending = False
        self._started_ns = None
        self._result = None
        self._closed = False
        self.started_count = 0

    def _work(self):
        before = time.monotonic_ns()
        endpoints, error = [], None
        try:
            endpoints = self.query()
        except Exception as exc:
            error = f'{type(exc).__name__}: {exc}'
        after = time.monotonic_ns()
        with self._lock:
            self._result = {
                'query_started_monotonic_ns': before,
                'query_finished_monotonic_ns': after,
                'query_duration_ns': after-before,
                'query_error': error,
                'query_within_budget': error is None and 0 <= after-before <= self.maximum_duration_ns,
                'endpoints': endpoints,
            }

    def poll(self, now_monotonic_ns):
        with self._lock:
            result, self._result = self._result, None
            if result is not None:
                self._pending = False
            if not self._pending and not self._closed:
                self._pending = True
                self._started_ns = now_monotonic_ns
                self.started_count += 1
                threading.Thread(target=self._work, daemon=True,
                                 name='impact_extnav_graph_query').start()
        return result

    def snapshot(self, now_monotonic_ns):
        with self._lock:
            age = None if self._started_ns is None else now_monotonic_ns-self._started_ns
            return {'query_pending': self._pending, 'pending_age_ns': age,
                    'pending_exceeds_budget': bool(self._pending and age is not None and age > self.maximum_duration_ns),
                    'maximum_query_duration_ns': self.maximum_duration_ns,
                    'queries_started': self.started_count, 'closed': self._closed,
                    'at_most_one_outstanding_query': True,
                    'executor_waits_for_query': False}

    def close(self):
        with self._lock:
            self._closed = True
