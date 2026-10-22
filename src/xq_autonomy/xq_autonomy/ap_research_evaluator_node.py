"""Evaluation-only truth alignment and progress for the ArduPilot research run."""

from __future__ import annotations

import bisect
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass(frozen=True)
class PoseSample:
    stamp_s: float
    position: np.ndarray
    rotation: np.ndarray


def rotation_from_xyzw(quaternion: np.ndarray) -> np.ndarray:
    """Return body-to-parent rotation, rejecting invalid quaternions."""
    values = np.asarray(quaternion, dtype=float).reshape(4)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(values).all() or not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = values / norm
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def pose_sample(message: Odometry) -> PoseSample:
    stamp = float(message.header.stamp.sec) + 1.0e-9 * float(message.header.stamp.nanosec)
    point = message.pose.pose.position
    position = np.asarray((point.x, point.y, point.z), dtype=float)
    quaternion = message.pose.pose.orientation
    if not math.isfinite(stamp) or not np.isfinite(position).all():
        raise ValueError("nonfinite odometry pose or timestamp")
    return PoseSample(
        stamp,
        position,
        rotation_from_xyzw(
            np.asarray((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        ),
    )


class ResearchEvaluation:
    """Bounded nearest-stamp matcher with a frozen first-pose SE(3) alignment."""

    def __init__(self, maximum_pair_skew_s: float = 0.03, cache_size: int = 2000):
        if not 0.0 < maximum_pair_skew_s <= 0.03 or cache_size < 2:
            raise ValueError("matching requires 0 < skew <= 0.03 seconds and cache >= 2")
        self.maximum_pair_skew_s = maximum_pair_skew_s
        self.cache_size = cache_size
        self.truth: list[PoseSample] = []
        self.pending_odometry: list[PoseSample] = []
        self.phase = "WAITING"
        self.terminal_seen = False
        self.terminal_phase: str | None = None
        self.research_seen = False
        self.research_exit_phase: str | None = None
        self.pending_research_end = False
        self.research_start: dict[str, object] | None = None
        self.research_end: dict[str, object] | None = None
        self.last_pair: dict[str, object] | None = None
        self.rotation: np.ndarray | None = None
        self.translation: np.ndarray | None = None
        self.matched_samples = 0
        self.research_matched_samples = 0
        self.sum_squared_error = 0.0
        self.maximum_error = 0.0
        self.maximum_pair_skew_seen_s = 0.0
        self.last_matched_odom_stamp_s = -math.inf
        self.unmatched_odometry_samples = 0
        self.duplicate_or_old_odometry_samples = 0
        self.invalid_samples = 0

    @staticmethod
    def _insert(cache: list[PoseSample], sample: PoseSample) -> bool:
        index = bisect.bisect_left([entry.stamp_s for entry in cache], sample.stamp_s)
        if index < len(cache) and cache[index].stamp_s == sample.stamp_s:
            return False
        cache.insert(index, sample)
        return True

    def add_truth(self, sample: PoseSample) -> None:
        self._insert(self.truth, sample)
        self._match()
        if len(self.truth) > self.cache_size:
            del self.truth[: len(self.truth) - self.cache_size]

    def add_odometry(self, sample: PoseSample) -> None:
        if sample.stamp_s <= self.last_matched_odom_stamp_s:
            self.duplicate_or_old_odometry_samples += 1
            return
        if not self._insert(self.pending_odometry, sample):
            self.duplicate_or_old_odometry_samples += 1
            return
        self._match()
        if len(self.pending_odometry) > self.cache_size:
            excess = len(self.pending_odometry) - self.cache_size
            self.unmatched_odometry_samples += excess
            del self.pending_odometry[:excess]

    def _match(self) -> None:
        if not self.truth:
            return
        truth_stamps = [entry.stamp_s for entry in self.truth]
        consumed = 0
        for odometry in self.pending_odometry:
            # Wait for the first truth sample at/after the LIO stamp so both
            # nearest neighbours are available; never interpolate or match twice.
            if odometry.stamp_s > truth_stamps[-1]:
                break
            index = bisect.bisect_left(truth_stamps, odometry.stamp_s)
            neighbours = self.truth[max(0, index - 1) : min(len(self.truth), index + 1)]
            truth = min(neighbours, key=lambda entry: abs(entry.stamp_s - odometry.stamp_s))
            skew = abs(truth.stamp_s - odometry.stamp_s)
            consumed += 1
            if skew > self.maximum_pair_skew_s + 1.0e-12:
                self.unmatched_odometry_samples += 1
                continue
            self._record_pair(odometry, truth, skew)
        del self.pending_odometry[:consumed]

    def _record_pair(self, odometry: PoseSample, truth: PoseSample, skew: float) -> None:
        if self.rotation is None:
            self.rotation = odometry.rotation @ truth.rotation.T
            self.translation = odometry.position - self.rotation @ truth.position
        assert self.translation is not None
        aligned_truth = self.rotation @ truth.position + self.translation
        error = float(np.linalg.norm(odometry.position - aligned_truth))
        self.matched_samples += 1
        self.sum_squared_error += error * error
        self.maximum_error = max(self.maximum_error, error)
        self.maximum_pair_skew_seen_s = max(self.maximum_pair_skew_seen_s, skew)
        self.last_matched_odom_stamp_s = odometry.stamp_s
        self.last_pair = {
            "matched_sample_index": self.matched_samples,
            "received_phase": self.phase,
            "lio_stamp_s": odometry.stamp_s,
            "truth_stamp_s": truth.stamp_s,
            "pair_skew_s": skew,
            "lio_xyz_m": odometry.position.tolist(),
            "aligned_truth_xyz_m": aligned_truth.tolist(),
            "position_error_m": error,
        }
        if self.phase == "RESEARCH" and self.research_exit_phase is None:
            if self.research_start is None:
                self.research_start = dict(self.last_pair)
            self.research_matched_samples += 1
        if self.pending_research_end and self.research_end is None:
            self.research_end = dict(self.last_pair)
            self.pending_research_end = False

    def set_phase(self, phase: str) -> None:
        previous = self.phase
        self.phase = phase
        if phase == "RESEARCH" and not self.research_seen:
            self.research_seen = True
            if self.last_pair is not None:
                self.research_start = dict(self.last_pair)
                self.research_matched_samples = 1
        if previous == "RESEARCH" and phase != "RESEARCH" and self.research_exit_phase is None:
            self.research_exit_phase = phase
            self.pending_research_end = self.research_start is not None
        if phase in {"DONE", "FAILED"}:
            self.terminal_seen = True
            self.terminal_phase = phase

    def snapshot(self) -> dict[str, object]:
        truth_delta = None
        lio_delta = None
        if self.research_start is not None and self.research_end is not None:
            truth_delta = (
                np.asarray(self.research_end["aligned_truth_xyz_m"])
                - np.asarray(self.research_start["aligned_truth_xyz_m"])
            ).tolist()
            lio_delta = (
                np.asarray(self.research_end["lio_xyz_m"])
                - np.asarray(self.research_start["lio_xyz_m"])
            ).tolist()
        samples_valid = self.matched_samples >= 20 and self.research_matched_samples >= 20
        complete = bool(
            self.terminal_phase == "DONE"
            and self.research_start is not None
            and self.research_end is not None
            and samples_valid
        )
        return {
            "schema_version": 1,
            "status": "COMPLETE" if complete else "READY",
            "mission_phase": self.phase,
            "mission_terminal_seen": self.terminal_seen,
            "mission_terminal_phase": self.terminal_phase,
            "evaluation_only": True,
            "ground_truth_subscribed": True,
            "ground_truth_used_for_control": False,
            "publishes_commands": False,
            "gate_decision": "external_runner_only",
            "matched_samples": self.matched_samples,
            "research_matched_samples": self.research_matched_samples,
            "minimum_valid_samples": 20,
            "samples_valid": samples_valid,
            "ate_rms_m": (
                math.sqrt(self.sum_squared_error / self.matched_samples)
                if self.matched_samples else None
            ),
            "ate_max_m": self.maximum_error if self.matched_samples else None,
            "maximum_pair_skew_s": self.maximum_pair_skew_s,
            "maximum_observed_pair_skew_s": self.maximum_pair_skew_seen_s,
            "unmatched_odometry_samples": self.unmatched_odometry_samples,
            "pending_odometry_samples": len(self.pending_odometry),
            "duplicate_or_old_odometry_samples": self.duplicate_or_old_odometry_samples,
            "invalid_samples": self.invalid_samples,
            "alignment_rotation_truth_to_lio": (
                self.rotation.tolist() if self.rotation is not None else None
            ),
            "alignment_translation_truth_to_lio_m": (
                self.translation.tolist() if self.translation is not None else None
            ),
            "research_start": self.research_start,
            "research_end": self.research_end,
            "research_exit_phase": self.research_exit_phase,
            "research_end_pending": self.pending_research_end,
            "research_truth_delta_xyz_m": truth_delta,
            "research_lio_delta_xyz_m": lio_delta,
            "research_truth_progress_m": truth_delta[0] if truth_delta is not None else None,
            "research_lio_progress_m": lio_delta[0] if lio_delta is not None else None,
            "last_pair": self.last_pair,
            "sampling_note": (
                "Nearest simulation-stamp pairs; no interpolation. Research starts at the "
                "latest matched sample when RESEARCH arrives (or its first subsequent pair) "
                "and ends at the first new pair after leaving RESEARCH. Phase boundaries "
                "are classified by callback receipt, so sampling and transport delay apply; "
                "later landing does not replace research endpoints. ATE covers all pairs."
            ),
        }


class APResearchEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("impact_ap_research_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("ground_truth_topic", "/xq/eval/p4/ground_truth")
        self.declare_parameter("maximum_pair_skew_s", 0.03)
        destination = str(self.get_parameter("result_file").value).strip()
        if not destination:
            raise ValueError("result_file is required for the research evaluator")
        self.result_file = Path(destination).expanduser()
        self.ground_truth_topic = str(self.get_parameter("ground_truth_topic").value)
        self.evaluation = ResearchEvaluation(
            float(self.get_parameter("maximum_pair_skew_s").value)
        )
        self._started_wall_s = time.monotonic()
        self._phase_received_wall_elapsed_s: float | None = None
        self._invalid_mission_messages = 0
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, "/localization/odom", self._odom_cb, reliable)
        self.create_subscription(
            Odometry, self.ground_truth_topic, self._truth_cb, reliable
        )
        self.create_subscription(
            String, "/impact/ap/mission_status", self._mission_cb, latched
        )
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.write_result, clock=self._wall_clock)
        self.write_result()
        self.get_logger().info(
            "ArduPilot research evaluator ready; truth is evaluation-only; no publishers"
        )

    def _odom_cb(self, message: Odometry) -> None:
        end_was_pending = self.evaluation.pending_research_end
        try:
            self.evaluation.add_odometry(pose_sample(message))
        except ValueError:
            self.evaluation.invalid_samples += 1
        if (
            self.evaluation.terminal_seen
            and end_was_pending
            and not self.evaluation.pending_research_end
        ):
            self.write_result()

    def _truth_cb(self, message: Odometry) -> None:
        end_was_pending = self.evaluation.pending_research_end
        try:
            self.evaluation.add_truth(pose_sample(message))
        except ValueError:
            self.evaluation.invalid_samples += 1
        if (
            self.evaluation.terminal_seen
            and end_was_pending
            and not self.evaluation.pending_research_end
        ):
            self.write_result()

    def _mission_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            payload = message.data
        phase = payload.get("phase") if isinstance(payload, dict) else payload
        if not isinstance(phase, str) or not phase.strip():
            self._invalid_mission_messages += 1
            return
        phase = phase.strip().upper()
        if phase != self.evaluation.phase:
            self._phase_received_wall_elapsed_s = time.monotonic() - self._started_wall_s
        self.evaluation.set_phase(phase)
        self.write_result()

    def write_result(self) -> None:
        result = self.evaluation.snapshot()
        result.update({
            "ground_truth_topic": self.ground_truth_topic,
            "wall_elapsed_s": time.monotonic() - self._started_wall_s,
            "phase_received_wall_elapsed_s": self._phase_received_wall_elapsed_s,
            "invalid_mission_messages": self._invalid_mission_messages,
        })
        temporary: str | None = None
        try:
            self.result_file.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self.result_file.parent,
                prefix=f".{self.result_file.name}.",
                suffix=".tmp",
                delete=False,
            ) as stream:
                temporary = stream.name
                json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
            os.replace(temporary, self.result_file)
            temporary = None
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Cannot write research evaluation: {error}")
        finally:
            if temporary is not None:
                try:
                    Path(temporary).unlink(missing_ok=True)
                except OSError:
                    pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = APResearchEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_result()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
