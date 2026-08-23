#!/usr/bin/env python3
"""Render the P4 truth/LIO trajectory and altitude profile from Gate evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    evaluation = json.loads(
        (args.result_dir / "localization-evaluation.json").read_text(encoding="utf-8")
    )
    checkpoints = evaluation["metrics"]["error_checkpoints"]
    elapsed = [point["elapsed_s"] for point in checkpoints]
    truth = [point["truth_xyz_m"] for point in checkpoints]
    lio = [point["lio_xyz_m"] for point in checkpoints]

    figure, (trajectory_axis, altitude_axis) = plt.subplots(
        1, 2, figsize=(12.8, 5.6), constrained_layout=True
    )
    trajectory_axis.plot(
        [point[0] for point in truth],
        [point[1] for point in truth],
        color="#1769aa",
        linewidth=2.3,
        label="Gazebo truth (evaluation only)",
    )
    trajectory_axis.plot(
        [point[0] for point in lio],
        [point[1] for point in lio],
        color="#ef6c00",
        linewidth=1.4,
        linestyle="--",
        label="FAST-LIO",
    )
    trajectory_axis.set_title("GPS-off ExternalNav rectangle")
    trajectory_axis.set_xlabel("local x / m")
    trajectory_axis.set_ylabel("local y / m")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, alpha=0.25)
    trajectory_axis.legend(fontsize=8)

    altitude_axis.plot(
        elapsed,
        [point[2] for point in truth],
        color="#1769aa",
        linewidth=2.3,
        label="Gazebo truth",
    )
    altitude_axis.plot(
        elapsed,
        [point[2] for point in lio],
        color="#ef6c00",
        linewidth=1.4,
        linestyle="--",
        label="FAST-LIO",
    )
    altitude_axis.set_title("Takeoff, hover and landing")
    altitude_axis.set_xlabel("evaluation time / s")
    altitude_axis.set_ylabel("z / m")
    altitude_axis.grid(True, alpha=0.25)
    altitude_axis.legend(fontsize=8)

    metrics = evaluation["metrics"]
    figure.suptitle(
        "IMPACT Gate P4 PASS  |  ATE RMS "
        f"{metrics['ate_rms_m']:.4f} m  |  GPS disabled  |  LIO → MAVROS → EKF3",
        fontsize=13,
    )
    output = args.output or args.result_dir / "p4-trajectory.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    print(output)


if __name__ == "__main__":
    main()
