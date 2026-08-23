from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

from .types import Packet


@dataclass
class NetworkStats:
    sent: int = 0
    dropped: int = 0
    delivered: int = 0
    stale_rejected: int = 0

    @property
    def drop_rate(self) -> float:
        return self.dropped / self.sent if self.sent else 0.0


@dataclass
class FaultWindowStats:
    sent: int = 0
    dropped: int = 0
    delivered: int = 0

    @property
    def drop_rate(self) -> float:
        return self.dropped / self.sent if self.sent else 0.0


class PacketLossRelay:
    """Project-scoped deterministic network impairment model.

    It never changes a host interface and therefore cannot affect other WSL
    simulations.  Delay, jitter and probabilistic loss are reproducible by seed.
    """

    def __init__(
        self,
        drop_rate: float = 0.20,
        delay_s: float = 0.03,
        jitter_s: float = 0.01,
        seed: int = 20260820,
        max_age_s: float = 1.0,
    ) -> None:
        if not 0.0 <= drop_rate <= 1.0:
            raise ValueError("drop_rate must be in [0, 1]")
        self.drop_rate = float(drop_rate)
        self.delay_s = max(0.0, float(delay_s))
        self.jitter_s = max(0.0, float(jitter_s))
        self.max_age_s = max(0.0, float(max_age_s))
        self._rng = random.Random(seed)
        self._queue: List[Tuple[float, int, Packet]] = []
        self.stats = NetworkStats()
        self._latest_seq = -1

    def set_drop_rate(self, drop_rate: float) -> None:
        if not 0.0 <= drop_rate <= 1.0:
            raise ValueError("drop_rate must be in [0, 1]")
        self.drop_rate = float(drop_rate)

    def reseed(self, seed: int) -> None:
        """Start a deterministic impairment window without clearing counters."""
        self._rng.seed(int(seed))

    def send(self, packet: Packet, now_s: float) -> bool:
        self.stats.sent += 1
        # A healthy link must not consume loss RNG samples.  Consequently the
        # same seed produces the same fault-window loss pattern regardless of
        # how long the simulation ran before the fault was injected.
        if self.drop_rate > 0.0 and self._rng.random() < self.drop_rate:
            self.stats.dropped += 1
            return False
        jitter = (
            self._rng.uniform(-self.jitter_s, self.jitter_s)
            if self.jitter_s > 0.0
            else 0.0
        )
        delivery = now_s + max(0.0, self.delay_s + jitter)
        heapq.heappush(self._queue, (delivery, packet.seq, packet))
        return True

    def deliver(self, now_s: float) -> List[Packet]:
        delivered: List[Packet] = []
        while self._queue and self._queue[0][0] <= now_s:
            _, _, packet = heapq.heappop(self._queue)
            if now_s - packet.stamp_s > self.max_age_s or packet.seq <= self._latest_seq:
                self.stats.stale_rejected += 1
                continue
            self._latest_seq = packet.seq
            self.stats.delivered += 1
            delivered.append(packet)
        return delivered


class GroundLinkHeartbeatRelay:
    """Pure-Python state machine for the project-scoped ground link.

    Only ``ground_link/drop`` events alter the relay.  Healthy operation has
    exactly zero configured loss, while an active event uses its severity as
    the drop probability.  No host network interface is modified.
    """

    _FAULT_MARKER = "_xq_ground_link_fault_window"

    def __init__(
        self,
        seed: int = 20260820,
        delay_s: float = 0.03,
        jitter_s: float = 0.0,
        max_age_s: float = 1.0,
    ) -> None:
        self.seed = int(seed)
        self.relay = PacketLossRelay(
            drop_rate=0.0,
            delay_s=delay_s,
            jitter_s=jitter_s,
            seed=self.seed,
            max_age_s=max_age_s,
        )
        self.fault_window = FaultWindowStats()
        self._active = False
        self._fault_id = ""
        self._last_fault_id = ""
        self._fault_drop_rate = 0.0
        self._fault_expires_at_s = -math.inf

    @property
    def active(self) -> bool:
        return self._active

    @property
    def fault_id(self) -> str:
        return self._fault_id if self._active else self._last_fault_id

    @property
    def current_drop_rate(self) -> float:
        return self._fault_drop_rate if self._active else 0.0

    def update(self, now_s: float) -> None:
        if self._active and float(now_s) >= self._fault_expires_at_s:
            self._deactivate()

    def handle_fault(
        self,
        *,
        target_module: str,
        action: str,
        fault_id: str,
        severity: float,
        duration_s: float,
        now_s: float,
        seed: Optional[int] = None,
    ) -> bool:
        """Apply a relevant event and return whether it was consumed."""
        target = str(target_module).strip().lower()
        operation = str(action).strip().lower()
        if target != "ground_link":
            return False
        if operation in ("clear", "recover"):
            self._deactivate()
            return True
        if operation != "drop":
            return False
        rate = float(severity)
        if not 0.0 <= rate <= 1.0:
            raise ValueError("ground-link drop severity must be in [0, 1]")
        duration = max(0.0, float(duration_s))
        self._active = True
        self._fault_id = str(fault_id) or "ground_link_drop"
        self._last_fault_id = self._fault_id
        self._fault_drop_rate = rate
        self._fault_expires_at_s = (
            float(now_s) + duration if duration > 0.0 else math.inf
        )
        self.relay.reseed(self.seed if seed is None else int(seed))
        self.relay.set_drop_rate(rate)
        return True

    def send(self, packet: Packet, now_s: float) -> bool:
        self.update(now_s)
        payload: Dict[str, Any] = dict(packet.payload)
        payload[self._FAULT_MARKER] = self._active
        wrapped = Packet(seq=packet.seq, stamp_s=packet.stamp_s, payload=payload)
        if self._active:
            self.fault_window.sent += 1
        accepted = self.relay.send(wrapped, now_s)
        if self._active and not accepted:
            self.fault_window.dropped += 1
        return accepted

    def deliver(self, now_s: float) -> List[Packet]:
        self.update(now_s)
        output: List[Packet] = []
        for packet in self.relay.deliver(now_s):
            payload = dict(packet.payload)
            from_fault_window = bool(payload.pop(self._FAULT_MARKER, False))
            if from_fault_window:
                self.fault_window.delivered += 1
            output.append(Packet(packet.seq, packet.stamp_s, payload))
        return output

    def stats_dict(self, now_s: float) -> Dict[str, Any]:
        self.update(now_s)
        return {
            "sent": self.relay.stats.sent,
            "dropped": self.relay.stats.dropped,
            "delivered": self.relay.stats.delivered,
            "stale_rejected": self.relay.stats.stale_rejected,
            "fault_window_sent": self.fault_window.sent,
            "fault_window_dropped": self.fault_window.dropped,
            "fault_window_delivered": self.fault_window.delivered,
            "current_drop_rate": self.current_drop_rate,
            "observed_drop_rate": self.fault_window.drop_rate,
            "overall_observed_drop_rate": self.relay.stats.drop_rate,
            "active": self._active,
            "fault_id": self.fault_id,
        }

    def _deactivate(self) -> None:
        self._active = False
        self._fault_id = ""
        self._fault_drop_rate = 0.0
        self._fault_expires_at_s = -math.inf
        self.relay.set_drop_rate(0.0)
