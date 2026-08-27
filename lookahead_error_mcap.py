#!/usr/bin/env python3
"""Evaluate ROS 2 path-following error from one or more MCAP bags.

The script compares /odom against the /path that was active at each odometry
sample. If the frames differ, it applies a recorded direct 2-D TF between the
/path frame and /odom frame (for example map -> odom).

Metrics:
  - signed cross-track error (CTE)
  - absolute tracking error = abs(CTE)
  - RMSE CTE
  - mean absolute CTE (MAE)
  - 95th-percentile absolute CTE
  - maximum absolute CTE

Outputs:
  - <bag>_tracking_error.csv for each MCAP
  - path_tracking_summary.csv across all supplied MCAPs
  - optional PNG plots when matplotlib is installed

Example:
  python mcap_path_tracking_error.py lookahead0p10_2_0.mcap
  python mcap_path_tracking_error.py lookahead*.mcap

Run this from a shell where ROS 2 is sourced and rosbag2's MCAP storage plugin
is installed.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python packages were not found. Run this from a sourced ROS 2 "
        "environment (for example your RoboStack/pixi shell).\n"
        f"Original import error: {exc}"
    )


@dataclass
class PathSample:
    stamp: float
    frame: str
    points: list[tuple[float, float]]


@dataclass
class OdomSample:
    stamp: float
    frame: str
    x: float
    y: float


@dataclass
class Transform2D:
    stamp: float
    parent: str
    child: str
    x: float
    y: float
    yaw: float
    is_static: bool = False


@dataclass
class ErrorSample:
    stamp: float
    x_map: float
    y_map: float
    signed_cte: float
    tracking_error: float


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def yaw_from_quaternion(q) -> float:
    # ROS quaternion ordering: x, y, z, w
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def transform_point(x: float, y: float, tf: Transform2D) -> tuple[float, float]:
    """Apply parent <- child transform to a point expressed in child."""
    c = math.cos(tf.yaw)
    s = math.sin(tf.yaw)
    return (
        tf.x + c * x - s * y,
        tf.y + s * x + c * y,
    )


def invert_transform(tf: Transform2D) -> Transform2D:
    """Return child <- parent for a parent <- child transform."""
    c = math.cos(tf.yaw)
    s = math.sin(tf.yaw)
    # inverse translation = -R^T t
    ix = -(c * tf.x + s * tf.y)
    iy = -(-s * tf.x + c * tf.y)
    return Transform2D(
        stamp=tf.stamp,
        parent=tf.child,
        child=tf.parent,
        x=ix,
        y=iy,
        yaw=-tf.yaw,
        is_static=tf.is_static,
    )


def signed_cte_to_polyline(
    px: float, py: float, points: list[tuple[float, float]]
) -> float:
    """Signed distance from P to the nearest segment of a 2-D polyline.

    Positive/negative sign indicates which side of the nearest directed path
    segment the robot lies on. The magnitude is the tracking error.
    """
    if len(points) < 2:
        raise ValueError("Path must contain at least two poses")

    best_distance = math.inf
    best_signed = math.nan

    for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
        abx = bx - ax
        aby = by - ay
        length_sq = abx * abx + aby * aby
        if length_sq < 1e-12:
            continue

        apx = px - ax
        apy = py - ay
        t = (apx * abx + apy * aby) / length_sq
        t = max(0.0, min(1.0, t))

        qx = ax + t * abx
        qy = ay + t * aby
        ox = px - qx
        oy = py - qy
        distance = math.hypot(ox, oy)

        if distance < best_distance:
            cross_z = abx * oy - aby * ox
            sign = 1.0 if cross_z >= 0.0 else -1.0
            best_distance = distance
            best_signed = sign * distance

    if not math.isfinite(best_signed):
        raise ValueError("Path contains no non-zero-length segments")

    return best_signed


def percentile(values: list[float], pct: float) -> float:
    if not values:
        return math.nan
    data = sorted(values)
    if len(data) == 1:
        return data[0]
    rank = (pct / 100.0) * (len(data) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return data[lo]
    f = rank - lo
    return data[lo] * (1.0 - f) + data[hi] * f


def parse_lookahead(path: Path) -> Optional[float]:
    """Extract values such as 0.10 from 'lookahead0p10_2_0.mcap'."""
    m = re.search(r"lookahead[_-]?([0-9]+(?:p[0-9]+|\.[0-9]+)?)", path.stem, re.I)
    if not m:
        return None
    return float(m.group(1).replace("p", "."))


def read_mcap(path: Path):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    required = {"/path", "/odom"}
    missing = required - set(topic_types)
    if missing:
        raise RuntimeError(f"{path.name}: missing required topic(s): {sorted(missing)}")

    wanted = {"/path", "/odom", "/tf", "/tf_static"}
    msg_types = {
        topic: get_message(type_name)
        for topic, type_name in topic_types.items()
        if topic in wanted
    }

    paths: list[PathSample] = []
    odoms: list[OdomSample] = []
    transforms: list[Transform2D] = []

    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        if topic not in msg_types:
            continue

        msg = deserialize_message(raw, msg_types[topic])
        bag_time = bag_time_ns * 1e-9

        if topic == "/path":
            stamp = stamp_to_sec(msg.header.stamp) or bag_time
            points = [
                (float(p.pose.position.x), float(p.pose.position.y))
                for p in msg.poses
            ]
            if len(points) >= 2:
                paths.append(PathSample(stamp, msg.header.frame_id, points))

        elif topic == "/odom":
            stamp = stamp_to_sec(msg.header.stamp) or bag_time
            odoms.append(
                OdomSample(
                    stamp=stamp,
                    frame=msg.header.frame_id,
                    x=float(msg.pose.pose.position.x),
                    y=float(msg.pose.pose.position.y),
                )
            )

        elif topic in ("/tf", "/tf_static"):
            is_static = topic == "/tf_static"
            for tr in msg.transforms:
                stamp = stamp_to_sec(tr.header.stamp) or bag_time
                transforms.append(
                    Transform2D(
                        stamp=stamp,
                        parent=tr.header.frame_id,
                        child=tr.child_frame_id,
                        x=float(tr.transform.translation.x),
                        y=float(tr.transform.translation.y),
                        yaw=yaw_from_quaternion(tr.transform.rotation),
                        is_static=is_static,
                    )
                )

    paths.sort(key=lambda x: x.stamp)
    odoms.sort(key=lambda x: x.stamp)
    transforms.sort(key=lambda x: x.stamp)
    return paths, odoms, transforms


def build_direct_tf_lookup(
    transforms: list[Transform2D], target_frame: str, source_frame: str
):
    """Build a timestamp lookup for target <- source.

    This intentionally supports a direct TF edge (or its inverse), which covers
    the common map <- odom case without reimplementing the full TF2 graph.
    """
    direct = [
        t for t in transforms if t.parent == target_frame and t.child == source_frame
    ]
    inverse = [
        invert_transform(t)
        for t in transforms
        if t.parent == source_frame and t.child == target_frame
    ]
    candidates = direct + inverse

    static = [t for t in candidates if t.is_static]
    dynamic = sorted((t for t in candidates if not t.is_static), key=lambda x: x.stamp)
    times = [t.stamp for t in dynamic]

    if not static and not dynamic:
        raise RuntimeError(
            f"No direct TF found for {target_frame} <- {source_frame}. "
            "The script currently supports a direct TF edge or its inverse."
        )

    def lookup(stamp: float) -> Transform2D:
        if dynamic:
            idx = bisect.bisect_right(times, stamp) - 1
            if idx >= 0:
                return dynamic[idx]
        if static:
            return static[-1]
        raise RuntimeError(
            f"No {target_frame} <- {source_frame} transform available at t={stamp:.3f}"
        )

    return lookup


def evaluate(
    path_samples: list[PathSample],
    odom_samples: list[OdomSample],
    transforms: list[Transform2D],
    reference: str,
    trim_start: float,
    trim_end: float,
) -> tuple[list[ErrorSample], str, str]:
    if not path_samples:
        raise RuntimeError("No usable /path messages with at least two poses")
    if not odom_samples:
        raise RuntimeError("No /odom messages")

    path_frame = path_samples[0].frame
    odom_frame = odom_samples[0].frame

    if any(p.frame != path_frame for p in path_samples):
        raise RuntimeError("/path frame_id changes during the bag; unsupported")
    if any(o.frame != odom_frame for o in odom_samples):
        raise RuntimeError("/odom frame_id changes during the bag; unsupported")

    path_times = [p.stamp for p in path_samples]

    if path_frame == odom_frame:
        transform_odom = lambda x, y, stamp: (x, y)
    else:
        lookup_tf = build_direct_tf_lookup(transforms, path_frame, odom_frame)

        def transform_odom(x, y, stamp):
            return transform_point(x, y, lookup_tf(stamp))

    # Only evaluate while there is a defined reference path.
    start_time = path_samples[0].stamp + max(0.0, trim_start)
    end_time = odom_samples[-1].stamp - max(0.0, trim_end)

    errors: list[ErrorSample] = []
    for odom in odom_samples:
        if odom.stamp < start_time or odom.stamp > end_time:
            continue

        if reference == "first":
            path = path_samples[0]
        else:
            idx = bisect.bisect_right(path_times, odom.stamp) - 1
            if idx < 0:
                continue
            path = path_samples[idx]

        x_ref, y_ref = transform_odom(odom.x, odom.y, odom.stamp)
        cte = signed_cte_to_polyline(x_ref, y_ref, path.points)
        errors.append(
            ErrorSample(
                stamp=odom.stamp,
                x_map=x_ref,
                y_map=y_ref,
                signed_cte=cte,
                tracking_error=abs(cte),
            )
        )

    if not errors:
        raise RuntimeError("No samples remained after aligning path/odom and trimming")

    return errors, path_frame, odom_frame


def summarize(errors: list[ErrorSample]) -> dict[str, float]:
    signed = [e.signed_cte for e in errors]
    absolute = [e.tracking_error for e in errors]
    return {
        "samples": len(errors),
        "rmse_cte_m": math.sqrt(sum(e * e for e in signed) / len(signed)),
        "mae_cte_m": sum(absolute) / len(absolute),
        "p95_abs_cte_m": percentile(absolute, 95.0),
        "max_abs_cte_m": max(absolute),
        "mean_signed_cte_m": sum(signed) / len(signed),
    }


def write_detail_csv(path: Path, errors: list[ErrorSample]) -> Path:
    out = path.with_name(path.stem + "_tracking_error.csv")
    t0 = errors[0].stamp
    with out.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "elapsed_s",
                "stamp_s",
                "x_in_path_frame_m",
                "y_in_path_frame_m",
                "signed_cte_m",
                "tracking_error_m",
            ]
        )
        for e in errors:
            writer.writerow(
                [
                    f"{e.stamp - t0:.9f}",
                    f"{e.stamp:.9f}",
                    f"{e.x_map:.9f}",
                    f"{e.y_map:.9f}",
                    f"{e.signed_cte:.9f}",
                    f"{e.tracking_error:.9f}",
                ]
            )
    return out


def maybe_plot_run(path: Path, errors: list[ErrorSample]) -> Optional[Path]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    t0 = errors[0].stamp
    times = [e.stamp - t0 for e in errors]
    cte = [e.signed_cte for e in errors]

    out = path.with_name(path.stem + "_cte.png")
    plt.figure(figsize=(8, 4.5))
    plt.plot(times, cte)
    plt.axhline(0.0, linewidth=1)
    plt.xlabel("Elapsed time [s]")
    plt.ylabel("Signed cross-track error [m]")
    plt.title(f"Cross-track error: {path.stem}")
    plt.grid(True, alpha=0.25)
    plt.tight_layout()
    plt.savefig(out, dpi=180)
    plt.close()
    return out


def maybe_plot_summary(rows: list[dict], output_dir: Path) -> Optional[Path]:
    usable = [r for r in rows if r.get("lookahead_m") is not None]
    if len(usable) < 2:
        return None

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return None

    usable.sort(key=lambda r: r["lookahead_m"])
    x = [r["lookahead_m"] for r in usable]
    rmse = [r["rmse_cte_m"] for r in usable]
    p95 = [r["p95_abs_cte_m"] for r in usable]

    out = output_dir / "lookahead_cte_summary.png"
    plt.figure(figsize=(7, 4.5))
    plt.plot(x, rmse, marker="o", label="RMSE CTE")
    plt.plot(x, p95, marker="o", label="95th percentile |CTE|")
    plt.xlabel("Lookahead distance [m]")
    plt.ylabel("Tracking error [m]")
    plt.title("Path tracking error vs. lookahead distance")
    plt.grid(True, alpha=0.25)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out, dpi=180)
    plt.close()
    return out


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute path tracking / cross-track error from ROS 2 MCAP bags."
    )
    parser.add_argument("mcaps", nargs="+", type=Path, help="One or more .mcap files")
    parser.add_argument(
        "--reference",
        choices=("active", "first"),
        default="active",
        help=(
            "Path used for CTE: 'active' uses the latest /path at each /odom "
            "timestamp (default); 'first' holds the first recorded /path fixed."
        ),
    )
    parser.add_argument(
        "--lookahead",
        type=float,
        default=None,
        help="Override lookahead value for a single MCAP. Otherwise parsed from filename.",
    )
    parser.add_argument(
        "--trim-start",
        type=float,
        default=0.0,
        help="Seconds to exclude after the first usable /path message.",
    )
    parser.add_argument(
        "--trim-end",
        type=float,
        default=0.0,
        help="Seconds to exclude from the end of the run.",
    )
    parser.add_argument(
        "--no-plots", action="store_true", help="Do not generate PNG plots."
    )
    args = parser.parse_args()

    if args.lookahead is not None and len(args.mcaps) != 1:
        parser.error("--lookahead can only be used when evaluating one MCAP")

    rows: list[dict] = []

    for bag in args.mcaps:
        bag = bag.expanduser().resolve()
        if not bag.exists():
            print(f"SKIP: {bag} does not exist")
            continue

        print(f"\nReading {bag.name} ...")
        paths, odoms, transforms = read_mcap(bag)
        errors, path_frame, odom_frame = evaluate(
            paths,
            odoms,
            transforms,
            reference=args.reference,
            trim_start=args.trim_start,
            trim_end=args.trim_end,
        )
        metrics = summarize(errors)
        lookahead = args.lookahead if args.lookahead is not None else parse_lookahead(bag)
        detail_csv = write_detail_csv(bag, errors)

        row = {
            "bag": bag.name,
            "lookahead_m": lookahead,
            "path_frame": path_frame,
            "odom_frame": odom_frame,
            **metrics,
        }
        rows.append(row)

        print(f"  frames:       /path={path_frame}, /odom={odom_frame}")
        print(f"  samples:      {metrics['samples']}")
        if lookahead is not None:
            print(f"  lookahead:    {lookahead:.3f} m")
        print(f"  RMSE CTE:     {metrics['rmse_cte_m']:.4f} m")
        print(f"  MAE |CTE|:    {metrics['mae_cte_m']:.4f} m")
        print(f"  P95 |CTE|:    {metrics['p95_abs_cte_m']:.4f} m")
        print(f"  Max |CTE|:    {metrics['max_abs_cte_m']:.4f} m")
        print(f"  detail CSV:   {detail_csv}")

        if not args.no_plots:
            plot = maybe_plot_run(bag, errors)
            if plot:
                print(f"  CTE plot:     {plot}")

    if not rows:
        raise SystemExit("No MCAPs were successfully evaluated")

    output_dir = args.mcaps[0].expanduser().resolve().parent
    summary_path = output_dir / "path_tracking_summary.csv"
    fields = [
        "bag",
        "lookahead_m",
        "path_frame",
        "odom_frame",
        "samples",
        "rmse_cte_m",
        "mae_cte_m",
        "p95_abs_cte_m",
        "max_abs_cte_m",
        "mean_signed_cte_m",
    ]
    with summary_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nSummary CSV: {summary_path}")
    if not args.no_plots:
        summary_plot = maybe_plot_summary(rows, output_dir)
        if summary_plot:
            print(f"Summary plot: {summary_plot}")


if __name__ == "__main__":
    main()