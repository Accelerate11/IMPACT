"""Evaluation-only 0.05 m map of the P5 structured-room flight slice."""

from __future__ import annotations

import hashlib
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class P5EvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__("xq_p5_evaluator")
        self.declare_parameter("result_file", "")
        self.declare_parameter("evaluation_resolution_m", 0.05)
        self.declare_parameter("vehicle_radius_m", 0.35)
        self.declare_parameter("minimum_airborne_duration_s", 10.0)
        self.resolution = float(self.get_parameter("evaluation_resolution_m").value)
        if abs(self.resolution - 0.05) > 1e-9:
            raise ValueError("P5 evaluation map must be 0.05 m")
        self.origin_x = -10.0
        self.origin_y = -8.0
        self.size_x = int(round(20.0 / self.resolution))
        self.size_y = int(round(16.0 / self.resolution))
        self.occupied = self._make_map()
        self.inflated = self._inflate(self.occupied)
        self.map_sha256 = hashlib.sha256(self.occupied.tobytes()).hexdigest()
        self.armed = False
        self.airborne_started: float | None = None
        self.airborne_duration = 0.0
        self.samples = 0
        self.collisions = 0
        self.out_of_bounds = 0
        self.path_length = 0.0
        self.previous: np.ndarray | None = None
        self.minimum_clearance = math.inf
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Odometry, "/xq/eval/p5/ground_truth", self._truth_cb, qos)
        self.create_subscription(State, "/uav1/mavros/state", self._state_cb, qos)
        self.create_timer(1.0, self._write)
        self.get_logger().info("P5 evaluator: 0.05 m map; ground truth is evaluation-only")

    def _index_mesh(self):
        x = self.origin_x + (np.arange(self.size_x) + 0.5) * self.resolution
        y = self.origin_y + (np.arange(self.size_y) + 0.5) * self.resolution
        return np.meshgrid(x, y, indexing="ij")

    @staticmethod
    def _oriented_box(x, y, cx, cy, sx, sy, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = c * (x - cx) + s * (y - cy)
        local_y = -s * (x - cx) + c * (y - cy)
        return (np.abs(local_x) <= sx / 2.0) & (np.abs(local_y) <= sy / 2.0)

    def _make_map(self) -> np.ndarray:
        x, y = self._index_mesh()
        # Exact horizontal footprints from xq_p5_structured_room.sdf for
        # geometry intersecting the nominal 2.0 m flight slice. Low desks,
        # cabinet 02 and the crate are intentionally absent from this slice;
        # cabinet 01 reaches 2.0 m and is included.
        occupied = (np.abs(x) >= 9.9) | (np.abs(y) >= 7.9)
        occupied |= self._oriented_box(x, y, -3.0, 0.0, 0.18, 4.4, 0.0)
        occupied |= self._oriented_box(x, y, 3.0, 0.0, 0.18, 4.4, 0.0)
        occupied |= self._oriented_box(x, y, 0.0, -2.2, 6.18, 0.18, 0.0)
        occupied |= self._oriented_box(x, y, 0.0, 2.2, 6.18, 0.18, 0.0)
        occupied |= self._oriented_box(x, y, -6.7, 3.9, 4.8, 0.16, 0.0)
        occupied |= self._oriented_box(x, y, 6.8, 4.7, 0.16, 5.2, 0.0)
        occupied |= self._oriented_box(x, y, -5.8, -4.8, 0.16, 3.8, 0.0)
        occupied |= self._oriented_box(x, y, 6.6, -4.5, 5.0, 0.16, 0.0)
        occupied |= (x + 7.6) ** 2 + (y - 1.3) ** 2 <= 0.24**2
        occupied |= (x - 7.4) ** 2 + (y + 0.8) ** 2 <= 0.30**2
        occupied |= (x + 4.6) ** 2 + (y - 6.3) ** 2 <= 0.20**2
        occupied |= self._oriented_box(x, y, -8.8, -1.7, 0.7, 1.3, 0.0)
        return occupied

    def _inflate(self, occupied: np.ndarray) -> np.ndarray:
        radius = int(math.ceil(float(self.get_parameter("vehicle_radius_m").value) / self.resolution))
        output = occupied.copy()
        indices = np.argwhere(occupied)
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                shifted = indices + np.array((dx, dy))
                valid = (
                    (shifted[:, 0] >= 0)
                & (shifted[:, 0] < self.size_x)
                & (shifted[:, 1] >= 0)
                & (shifted[:, 1] < self.size_y)
                )
                output[shifted[valid, 0], shifted[valid, 1]] = True
        return output

    def _state_cb(self, message: State) -> None:
        self.armed = bool(message.armed)

    def _truth_cb(self, message: Odometry) -> None:
        p = message.pose.pose.position
        xyz = np.array((p.x, p.y, p.z), dtype=np.float64)
        if not np.isfinite(xyz).all() or not self.armed or xyz[2] < 0.5:
            return
        now = time.monotonic()
        if self.airborne_started is None:
            self.airborne_started = now
        self.airborne_duration = now - self.airborne_started
        self.samples += 1
        if self.previous is not None:
            self.path_length += float(np.linalg.norm(xyz - self.previous))
        self.previous = xyz
        ix = int(math.floor((xyz[0] - self.origin_x) / self.resolution))
        iy = int(math.floor((xyz[1] - self.origin_y) / self.resolution))
        if not (0 <= ix < self.size_x and 0 <= iy < self.size_y):
            self.out_of_bounds += 1
            self.collisions += 1
            return
        if self.inflated[ix, iy]:
            self.collisions += 1
        obstacle_indices = np.argwhere(self.occupied)
        if len(obstacle_indices):
            delta = (obstacle_indices - np.array((ix, iy))) * self.resolution
            clearance = float(np.sqrt(np.min(np.sum(delta * delta, axis=1)))) - float(
                self.get_parameter("vehicle_radius_m").value
            )
            self.minimum_clearance = min(self.minimum_clearance, clearance)

    def _snapshot(self) -> dict[str, object]:
        duration_ok = self.airborne_duration >= float(
            self.get_parameter("minimum_airborne_duration_s").value
        )
        checks = {
            "evaluation_resolution_0_05_m": abs(self.resolution - 0.05) < 1e-9,
            "airborne_duration": duration_ok,
            "samples_present": self.samples >= 50,
            "no_collision": self.collisions == 0,
            "inside_evaluation_map": self.out_of_bounds == 0,
        }
        status = "FAIL" if self.collisions else ("PASS" if all(checks.values()) else "IN_PROGRESS")
        return {
            "schema_version": 1,
            "gate": "P5_EVALUATION_MAP_COLLISION",
            "status": status,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "ground_truth_consumer": "xq_p5_evaluator_only",
            "world": "xq_p5_structured_room",
            "evaluation_map": {
                "resolution_m": self.resolution,
                "width": self.size_x,
                "height": self.size_y,
                "occupied_cells": int(np.count_nonzero(self.occupied)),
                "sha256": self.map_sha256,
            },
            "metrics": {
                "airborne_duration_s": self.airborne_duration,
                "samples": self.samples,
                "path_length_m": self.path_length,
                "collision_samples": self.collisions,
                "minimum_clearance_m": self.minimum_clearance if math.isfinite(self.minimum_clearance) else None,
            },
            "checks": checks,
        }

    def _write(self) -> None:
        path = Path(str(self.get_parameter("result_file").value))
        if str(path) in ("", "."):
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(self._snapshot(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = P5EvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
