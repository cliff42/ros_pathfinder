#!/usr/bin/env python3
import argparse
import math
import re
import statistics
from pathlib import Path


OLD_ICP_RE = re.compile(
    r"v=(?P<v>[-0-9.]+), w=(?P<w>[-0-9.]+), "
    r"icp_dx=(?P<icp_dx>[-0-9.]+), icp_dy=(?P<icp_dy>[-0-9.]+), "
    r"icp_dtheta=(?P<icp_dtheta>[-0-9.]+), "
    r"odom=\((?P<odom_x>[-0-9.]+), (?P<odom_y>[-0-9.]+), (?P<odom_theta>[-0-9.]+)\), "
    r"mu=\((?P<mu_x>[-0-9.]+), (?P<mu_y>[-0-9.]+), (?P<mu_theta>[-0-9.]+)\)"
)

NEW_ICP_RE = re.compile(
    r"v=(?P<v>[-0-9.]+), w=(?P<w>[-0-9.]+), "
    r"odom_dx=(?P<odom_dx>[-0-9.]+), odom_dy=(?P<odom_dy>[-0-9.]+), "
    r"odom_dtheta=(?P<odom_dtheta>[-0-9.]+), "
    r"icp_dx=(?P<icp_dx>[-0-9.]+), icp_dy=(?P<icp_dy>[-0-9.]+), "
    r"icp_dtheta=(?P<icp_dtheta>[-0-9.]+), "
    r"icp_matches=(?P<matches>[0-9]+), icp_rmse=(?P<rmse>[-0-9.]+), "
    r"icp_iters=(?P<iters>[0-9]+), trans_err=(?P<trans_err>[-0-9.]+), "
    r"rot_err=(?P<rot_err>[-0-9.]+), "
    r"odom=\((?P<odom_x>[-0-9.]+), (?P<odom_y>[-0-9.]+), (?P<odom_theta>[-0-9.]+)\), "
    r"mu=\((?P<mu_x>[-0-9.]+), (?P<mu_y>[-0-9.]+), (?P<mu_theta>[-0-9.]+)\)"
)

FOLLOW_RE = re.compile(
    r"follow target=(?P<target>[0-9]+)/(?P<last>[0-9]+), "
    r"pose=\((?P<x>[-0-9.]+), (?P<y>[-0-9.]+), yaw=(?P<yaw>[-0-9.]+)\), "
    r".*heading_error=(?P<heading_error>[-0-9.]+), "
    r"v=(?P<v>[-0-9.]+), w=(?P<w>[-0-9.]+)"
)

CMD_LINEAR_RE = re.compile(r"linear:\s*\n\s*x: (?P<x>[-0-9.eE]+)", re.MULTILINE)
CMD_ANGULAR_RE = re.compile(r"angular:\s*\n\s*x: [-0-9.eE]+\s*\n\s*y: [-0-9.eE]+\s*\n\s*z: (?P<z>[-0-9.eE]+)", re.MULTILINE)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Summarize ros_pathfinder ICP, path follower, and cmd_vel logs."
    )
    parser.add_argument("stack_log_dir", type=Path)
    parser.add_argument("--cmd-vel", type=Path, default=None)
    return parser.parse_args()


def as_float_dict(match):
    row = {}
    for key, value in match.groupdict().items():
        if value is None:
            continue
        row[key] = int(value) if key in {"matches", "iters", "target", "last"} else float(value)
    return row


def parse_icp(path):
    rows = []
    rejects = {}
    applies = 0
    debug_only = 0
    stationary = 0
    mode = []
    if not path.exists():
        return rows, rejects, applies, debug_only, stationary, mode

    for line in path.read_text(errors="ignore").splitlines():
        if "slam pose estimator mode" in line:
            mode.append(line.strip())
        match = NEW_ICP_RE.search(line) or OLD_ICP_RE.search(line)
        if match:
            rows.append(as_float_dict(match))
        if "ICP_REJECT" in line:
            reason_match = re.search(r"reason=([a-z_]+)", line)
            reason = reason_match.group(1) if reason_match else "unknown"
            rejects[reason] = rejects.get(reason, 0) + 1
        if "ICP_APPLY" in line:
            applies += 1
        if "ICP_DEBUG_ONLY" in line:
            debug_only += 1
        if "STATIONARY: rejecting ICP" in line:
            stationary += 1
    return rows, rejects, applies, debug_only, stationary, mode


def parse_follow(path):
    if not path.exists():
        return []
    return [as_float_dict(match) for match in FOLLOW_RE.finditer(path.read_text(errors="ignore"))]


def parse_cmd_vel(path):
    if path is None or not path.exists():
        return []
    text = path.read_text(errors="ignore")
    linear = [float(match.group("x")) for match in CMD_LINEAR_RE.finditer(text)]
    angular = [float(match.group("z")) for match in CMD_ANGULAR_RE.finditer(text)]
    return list(zip(linear, angular))


def stat_line(name, values):
    if not values:
        return f"{name}: n=0"
    return (
        f"{name}: n={len(values)} min={min(values):.5f} "
        f"median={statistics.median(values):.5f} max={max(values):.5f}"
    )


def print_icp_summary(rows, rejects, applies, debug_only, stationary, mode):
    print("ICP")
    for entry in mode:
        print(f"  {entry}")
    print(f"  samples={len(rows)} applies={applies} debug_only={debug_only} stationary_rejects={stationary}")
    if rejects:
        reject_text = ", ".join(f"{key}={value}" for key, value in sorted(rejects.items()))
        print(f"  rejects: {reject_text}")
    if not rows:
        return

    moving = [row for row in rows if abs(row["v"]) >= 0.005 or abs(row["w"]) >= 0.005]
    sample = moving or rows
    for key in ["v", "w", "icp_dx", "icp_dy", "icp_dtheta", "odom_x", "odom_y", "mu_x", "mu_y"]:
        print("  " + stat_line(key, [row[key] for row in sample if key in row]))
    if "trans_err" in sample[0]:
        for key in ["odom_dx", "odom_dy", "odom_dtheta", "trans_err", "rot_err", "rmse"]:
            print("  " + stat_line(key, [row[key] for row in sample if key in row]))
    first = sample[0]
    last = sample[-1]
    print(
        "  first pose: "
        f"odom=({first['odom_x']:.3f}, {first['odom_y']:.3f}, {first['odom_theta']:.3f}) "
        f"mu=({first['mu_x']:.3f}, {first['mu_y']:.3f}, {first['mu_theta']:.3f})"
    )
    print(
        "  last pose:  "
        f"odom=({last['odom_x']:.3f}, {last['odom_y']:.3f}, {last['odom_theta']:.3f}) "
        f"mu=({last['mu_x']:.3f}, {last['mu_y']:.3f}, {last['mu_theta']:.3f})"
    )
    print(
        "  final odom-vs-mu distance: "
        f"{math.hypot(last['odom_x'] - last['mu_x'], last['odom_y'] - last['mu_y']):.3f} m"
    )


def print_follow_summary(rows):
    print("Path follower")
    print(f"  samples={len(rows)}")
    if not rows:
        return
    for key in ["heading_error", "v", "w", "x", "y", "yaw"]:
        print("  " + stat_line(key, [row[key] for row in rows]))
    last = rows[-1]
    print(
        f"  last target={last['target']:.0f}/{last['last']:.0f} "
        f"pose=({last['x']:.3f}, {last['y']:.3f}, yaw={last['yaw']:.3f})"
    )


def print_cmd_vel_summary(rows):
    if not rows:
        return
    print("cmd_vel")
    print("  " + stat_line("linear.x", [row[0] for row in rows]))
    print("  " + stat_line("angular.z", [row[1] for row in rows]))


def main():
    args = parse_args()
    log_dir = args.stack_log_dir
    icp = parse_icp(log_dir / "slam_pose_estimator.log")
    follow = parse_follow(log_dir / "path_follower.log")
    cmd_vel = parse_cmd_vel(args.cmd_vel)

    print(f"log_dir={log_dir}")
    print_icp_summary(*icp)
    print_follow_summary(follow)
    print_cmd_vel_summary(cmd_vel)


if __name__ == "__main__":
    main()
