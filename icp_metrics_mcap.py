#!/usr/bin/env python3
"""Generate ICP localization metrics from one or more ROS 2 MCAP bags.

The SLAM node publishes one ``diagnostic_msgs/DiagnosticArray`` message on
``/slam/icp_diagnostics`` for every processed lidar scan. This script turns
those samples into:

* ``<bag>_icp_samples.csv`` with one row per processed scan;
* ``icp_summary.csv`` with one row per supplied bag; and
* ``<bag>_icp_distributions.png`` when matplotlib is installed.

Run this from a shell where ROS 2 is sourced and rosbag2's MCAP storage plugin
is installed.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional


DEFAULT_TOPIC = "/slam/icp_diagnostics"
DIAGNOSTIC_NAME = "ros_pathfinder/icp_scan_match"
REJECTION_STATUSES = (
    "no_icp_result",
    "not_converged",
    "high_rmse",
    "low_inlier_ratio",
    "odom_disagreement",
)


@dataclass
class ICPSample:
    stamp_s: float
    status: str
    icp_attempted: bool
    icp_accepted: bool
    current_point_count: Optional[int] = None
    match_count: Optional[int] = None
    inlier_ratio: Optional[float] = None
    rmse_m: Optional[float] = None
    iterations: Optional[int] = None
    converged: Optional[bool] = None
    icp_processing_time_ms: Optional[float] = None
    correction_translation_m: Optional[float] = None
    correction_rotation_rad: Optional[float] = None
    pose_step_translation_m: Optional[float] = None
    pose_step_rotation_rad: Optional[float] = None
    accepted_pose_jump_translation_m: Optional[float] = None
    accepted_pose_jump_rotation_rad: Optional[float] = None


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    data = sorted(values)
    if len(data) == 1:
        return data[0]
    rank = (pct / 100.0) * (len(data) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    fraction = rank - lower
    return data[lower] * (1.0 - fraction) + data[upper] * fraction


def optional_float(values: dict[str, str], key: str) -> Optional[float]:
    text = values.get(key, "").strip()
    if not text:
        return None
    value = float(text)
    return value if math.isfinite(value) else None


def optional_int(values: dict[str, str], key: str) -> Optional[int]:
    text = values.get(key, "").strip()
    return int(text) if text else None


def optional_bool(values: dict[str, str], key: str) -> Optional[bool]:
    text = values.get(key, "").strip().lower()
    if not text:
        return None
    if text == "true":
        return True
    if text == "false":
        return False
    raise ValueError(f"invalid boolean value for {key}: {text!r}")


def sample_from_values(
    stamp_s: float,
    message: str,
    values: dict[str, str],
) -> ICPSample:
    schema_version = values.get("schema_version", "")
    if schema_version and schema_version != "1":
        raise ValueError(
            f"unsupported ICP diagnostic schema version {schema_version!r}"
        )

    status = values.get("status", message).strip()
    if not status:
        raise ValueError("ICP diagnostic has no status")

    attempted = optional_bool(values, "icp_attempted")
    accepted = optional_bool(values, "icp_accepted")
    if attempted is None:
        attempted = status not in {"initialized", "stationary"}
    if accepted is None:
        accepted = status == "icp_accepted"

    correction_translation_m = optional_float(
        values,
        "correction_translation_m",
    )
    correction_rotation_rad = optional_float(
        values,
        "correction_rotation_rad",
    )
    jump_translation_m = optional_float(
        values,
        "accepted_pose_jump_translation_m",
    )
    jump_rotation_rad = optional_float(
        values,
        "accepted_pose_jump_rotation_rad",
    )
    if accepted and jump_translation_m is None:
        jump_translation_m = correction_translation_m
    if accepted and jump_rotation_rad is None:
        jump_rotation_rad = correction_rotation_rad

    return ICPSample(
        stamp_s=stamp_s,
        status=status,
        icp_attempted=attempted,
        icp_accepted=accepted,
        current_point_count=optional_int(values, "current_point_count"),
        match_count=optional_int(values, "match_count"),
        inlier_ratio=optional_float(values, "inlier_ratio"),
        rmse_m=optional_float(values, "rmse_m"),
        iterations=optional_int(values, "iterations"),
        converged=optional_bool(values, "converged"),
        icp_processing_time_ms=optional_float(
            values,
            "icp_processing_time_ms",
        ),
        correction_translation_m=correction_translation_m,
        correction_rotation_rad=correction_rotation_rad,
        pose_step_translation_m=optional_float(
            values,
            "pose_step_translation_m",
        ),
        pose_step_rotation_rad=optional_float(
            values,
            "pose_step_rotation_rad",
        ),
        accepted_pose_jump_translation_m=jump_translation_m,
        accepted_pose_jump_rotation_rad=jump_rotation_rad,
    )


def read_mcap(path: Path, topic: str) -> list[ICPSample]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages were not found. Run this from a sourced "
            "ROS 2 environment."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(path),
        storage_id="mcap",
    )
    reader.open(storage_options, rosbag2_py.ConverterOptions("", ""))

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    if topic not in topic_types:
        raise RuntimeError(f"{path.name}: missing required topic {topic}")
    message_type = get_message(topic_types[topic])

    samples: list[ICPSample] = []
    while reader.has_next():
        current_topic, raw, bag_time_ns = reader.read_next()
        if current_topic != topic:
            continue

        msg = deserialize_message(raw, message_type)
        stamp_s = stamp_to_sec(msg.header.stamp) or bag_time_ns * 1e-9
        for diagnostic in msg.status:
            if diagnostic.name != DIAGNOSTIC_NAME:
                continue
            values = {item.key: item.value for item in diagnostic.values}
            samples.append(
                sample_from_values(
                    stamp_s=stamp_s,
                    message=diagnostic.message,
                    values=values,
                )
            )

    samples.sort(key=lambda sample: sample.stamp_s)
    if not samples:
        raise RuntimeError(
            f"{path.name}: {topic} contains no {DIAGNOSTIC_NAME} samples"
        )
    return samples


def values_for(samples: list[ICPSample], field: str) -> list[float]:
    values = []
    for sample in samples:
        value = getattr(sample, field)
        if value is not None and math.isfinite(float(value)):
            values.append(float(value))
    return values


def percentage(count: int, total: int) -> float:
    return 100.0 * count / total if total else math.nan


def distribution_summary(
    row: dict[str, object],
    prefix: str,
    values: list[float],
    percentiles: tuple[float, ...],
) -> None:
    row[f"{prefix}_sample_count"] = len(values)
    for pct in percentiles:
        label = f"p{int(pct)}"
        row[f"{prefix}_{label}"] = percentile(values, pct)
    row[f"{prefix}_max"] = max(values) if values else math.nan


def summarize(samples: list[ICPSample]) -> dict[str, object]:
    statuses = Counter(sample.status for sample in samples)
    attempts = [sample for sample in samples if sample.icp_attempted]
    accepted = [sample for sample in attempts if sample.icp_accepted]
    results = [sample for sample in attempts if sample.converged is not None]
    converged_count = sum(sample.converged is True for sample in attempts)

    row: dict[str, object] = {
        "processed_scans": len(samples),
        "initialized_scans": statuses["initialized"],
        "stationary_scans": statuses["stationary"],
        "icp_attempts": len(attempts),
        "icp_accepted": len(accepted),
        "icp_rejected": len(attempts) - len(accepted),
        "acceptance_rate_pct": percentage(len(accepted), len(attempts)),
        "icp_results": len(results),
        "converged_results": converged_count,
        "convergence_rate_of_attempts_pct": percentage(
            converged_count,
            len(attempts),
        ),
        "convergence_rate_of_results_pct": percentage(
            converged_count,
            len(results),
        ),
    }

    known_rejections = 0
    for status in REJECTION_STATUSES:
        count = statuses[status]
        known_rejections += count
        row[f"reject_{status}_count"] = count
        row[f"reject_{status}_pct_of_attempts"] = percentage(
            count,
            len(attempts),
        )
    other_rejections = len(attempts) - len(accepted) - known_rejections
    row["reject_other_count"] = other_rejections
    row["reject_other_pct_of_attempts"] = percentage(
        other_rejections,
        len(attempts),
    )

    distribution_summary(
        row,
        "inlier_ratio",
        values_for(attempts, "inlier_ratio"),
        (5.0, 50.0, 95.0),
    )
    distribution_summary(
        row,
        "rmse_m",
        values_for(attempts, "rmse_m"),
        (50.0, 95.0),
    )
    distribution_summary(
        row,
        "correction_translation_m",
        values_for(attempts, "correction_translation_m"),
        (50.0, 95.0),
    )
    distribution_summary(
        row,
        "correction_rotation_rad",
        values_for(attempts, "correction_rotation_rad"),
        (50.0, 95.0),
    )
    distribution_summary(
        row,
        "iterations",
        values_for(attempts, "iterations"),
        (50.0, 95.0),
    )
    distribution_summary(
        row,
        "icp_processing_time_ms",
        values_for(attempts, "icp_processing_time_ms"),
        (50.0, 95.0),
    )

    accepted_translation_jumps = values_for(
        accepted,
        "accepted_pose_jump_translation_m",
    )
    accepted_rotation_jumps = values_for(
        accepted,
        "accepted_pose_jump_rotation_rad",
    )
    row["max_accepted_pose_jump_translation_m"] = (
        max(accepted_translation_jumps)
        if accepted_translation_jumps else math.nan
    )
    row["max_accepted_pose_jump_rotation_rad"] = (
        max(accepted_rotation_jumps)
        if accepted_rotation_jumps else math.nan
    )
    row["max_accepted_pose_jump_rotation_deg"] = (
        math.degrees(max(accepted_rotation_jumps))
        if accepted_rotation_jumps else math.nan
    )
    return row


def write_detail_csv(path: Path, samples: list[ICPSample]) -> Path:
    output = path.with_name(path.stem + "_icp_samples.csv")
    fields = ["elapsed_s", *asdict(samples[0]).keys()]
    start_stamp = samples[0].stamp_s
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            row = asdict(sample)
            row["elapsed_s"] = sample.stamp_s - start_stamp
            writer.writerow(row)
    return output


def histogram(ax, values: list[float], title: str, xlabel: str) -> None:
    if not values:
        ax.text(0.5, 0.5, "No samples", ha="center", va="center")
    else:
        bins = min(30, max(1, int(math.sqrt(len(values)))))
        ax.hist(values, bins=bins, edgecolor="black", alpha=0.75)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.2)


def maybe_plot(path: Path, samples: list[ICPSample]) -> Optional[Path]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    attempts = [sample for sample in samples if sample.icp_attempted]
    accepted = [sample for sample in attempts if sample.icp_accepted]
    status_counts = Counter(sample.status for sample in attempts)

    figure, axes = plt.subplots(2, 4, figsize=(16, 8))
    status_names = [
        "icp_accepted",
        *REJECTION_STATUSES,
    ]
    status_names = [name for name in status_names if status_counts[name]]
    if status_names:
        axes[0, 0].bar(
            status_names,
            [status_counts[name] for name in status_names],
        )
        axes[0, 0].tick_params(axis="x", rotation=40)
    else:
        axes[0, 0].text(
            0.5,
            0.5,
            "No ICP attempts",
            ha="center",
            va="center",
        )
    axes[0, 0].set_title("ICP outcomes")
    axes[0, 0].set_ylabel("Count")

    histogram(
        axes[0, 1],
        values_for(attempts, "inlier_ratio"),
        "Inlier ratio",
        "Ratio",
    )
    histogram(
        axes[0, 2],
        values_for(attempts, "rmse_m"),
        "ICP RMSE",
        "RMSE [m]",
    )
    histogram(
        axes[0, 3],
        values_for(attempts, "iterations"),
        "ICP iterations",
        "Iterations",
    )
    histogram(
        axes[1, 0],
        values_for(attempts, "correction_translation_m"),
        "Translation correction",
        "Magnitude [m]",
    )
    histogram(
        axes[1, 1],
        [
            math.degrees(value)
            for value in values_for(attempts, "correction_rotation_rad")
        ],
        "Rotation correction",
        "Magnitude [deg]",
    )
    histogram(
        axes[1, 2],
        values_for(attempts, "icp_processing_time_ms"),
        "ICP processing time",
        "Time [ms]",
    )

    accepted_times = [sample.stamp_s for sample in accepted]
    accepted_jumps = values_for(
        accepted,
        "accepted_pose_jump_translation_m",
    )
    if accepted_times and len(accepted_times) == len(accepted_jumps):
        start_stamp = samples[0].stamp_s
        axes[1, 3].plot(
            [stamp - start_stamp for stamp in accepted_times],
            accepted_jumps,
            marker=".",
            linewidth=1,
        )
    else:
        axes[1, 3].text(
            0.5,
            0.5,
            "No accepted matches",
            ha="center",
            va="center",
        )
    axes[1, 3].set_title("Accepted pose correction")
    axes[1, 3].set_xlabel("Elapsed time [s]")
    axes[1, 3].set_ylabel("Translation [m]")
    axes[1, 3].grid(True, alpha=0.2)

    figure.suptitle(f"ICP localization diagnostics: {path.stem}")
    figure.tight_layout()
    output = path.with_name(path.stem + "_icp_distributions.png")
    figure.savefig(output, dpi=180)
    plt.close(figure)
    return output


def print_summary(metrics: dict[str, object]) -> None:
    def number(key: str) -> float:
        return float(metrics[key])

    print(f"  processed scans:       {metrics['processed_scans']}")
    print(f"  ICP attempts:          {metrics['icp_attempts']}")
    print(f"  accepted:              {metrics['icp_accepted']}")
    print(
        "  acceptance rate:       "
        f"{number('acceptance_rate_pct'):.2f}%"
    )
    for status in REJECTION_STATUSES:
        print(
            f"  {status + ':':23}"
            f"{metrics[f'reject_{status}_count']} "
            f"({number(f'reject_{status}_pct_of_attempts'):.2f}%)"
        )
    print(
        "  inlier ratio p5/med:   "
        f"{number('inlier_ratio_p5'):.3f} / "
        f"{number('inlier_ratio_p50'):.3f}"
    )
    print(
        "  RMSE med/p95/max:      "
        f"{number('rmse_m_p50'):.4f} / "
        f"{number('rmse_m_p95'):.4f} / "
        f"{number('rmse_m_max'):.4f} m"
    )
    print(
        "  runtime med/p95:       "
        f"{number('icp_processing_time_ms_p50'):.2f} / "
        f"{number('icp_processing_time_ms_p95'):.2f} ms"
    )
    print(
        "  convergence/attempts:  "
        f"{number('convergence_rate_of_attempts_pct'):.2f}%"
    )
    print(
        "  max accepted jump:     "
        f"{number('max_accepted_pose_jump_translation_m'):.4f} m, "
        f"{number('max_accepted_pose_jump_rotation_deg'):.2f} deg"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate ICP metrics from ROS 2 MCAP bags."
    )
    parser.add_argument(
        "mcaps",
        nargs="+",
        type=Path,
        help="One or more .mcap files",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"Diagnostic topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Do not generate per-bag PNG distribution plots.",
    )
    args = parser.parse_args()

    rows: list[dict[str, object]] = []
    for supplied_path in args.mcaps:
        bag = supplied_path.expanduser().resolve()
        if not bag.exists():
            print(f"SKIP: {bag} does not exist")
            continue

        print(f"\nReading {bag.name} ...")
        try:
            samples = read_mcap(bag, args.topic)
        except (RuntimeError, ValueError) as error:
            print(f"SKIP: {error}")
            continue

        metrics = summarize(samples)
        rows.append({"bag": bag.name, **metrics})
        detail_csv = write_detail_csv(bag, samples)
        print_summary(metrics)
        print(f"  detail CSV:            {detail_csv}")
        if not args.no_plots:
            plot = maybe_plot(bag, samples)
            if plot is not None:
                print(f"  distribution plot:     {plot}")

    if not rows:
        raise SystemExit("No MCAPs were successfully evaluated")

    output_dir = args.mcaps[0].expanduser().resolve().parent
    output = output_dir / "icp_summary.csv"
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nSummary CSV: {output}")


if __name__ == "__main__":
    main()


# ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: { position: {x: 3.0, y: -2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0} }}" 