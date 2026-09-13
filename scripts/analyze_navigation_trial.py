#!/usr/bin/env python3
"""Offline metrics for a navigation trial recorded by record_navigation_trial.sh."""

from __future__ import annotations

import argparse
import bisect
import csv
import glob
import json
import math
from pathlib import Path
import sqlite3
import xml.etree.ElementTree as ET  # noqa: F401 - catches broken Python env early

import cv2
import numpy as np
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Bag:
    def __init__(self, path: Path):
        self.databases = sorted(path.glob("*.db3"))
        if not self.databases:
            raise RuntimeError(f"no db3 files in {path}")
        self.start = min(
            sqlite3.connect(db).execute("SELECT min(timestamp) FROM messages").fetchone()[0]
            for db in self.databases
        )

    def rows(self, topic: str):
        rows = []
        for database in self.databases:
            connection = sqlite3.connect(database)
            match = connection.execute(
                "SELECT id FROM topics WHERE name=?", (topic,)
            ).fetchone()
            if match:
                rows.extend(
                    connection.execute(
                        "SELECT timestamp,data FROM messages WHERE topic_id=? "
                        "ORDER BY timestamp",
                        (match[0],),
                    ).fetchall()
                )
            connection.close()
        rows.sort(key=lambda item: item[0])
        return rows

    def seconds(self, timestamp: int) -> float:
        return (timestamp - self.start) / 1.0e9


def nearest_index(times: np.ndarray, value: float) -> int:
    index = int(np.searchsorted(times, value))
    if index <= 0:
        return 0
    if index >= len(times):
        return len(times) - 1
    return index if times[index] - value < value - times[index - 1] else index - 1


def percentile(values, q: float):
    return float(np.percentile(values, q)) if len(values) else None


def windows(times: np.ndarray, mask: np.ndarray, minimum: float, gap: float = 0.35):
    hits = times[mask]
    if len(hits) == 0:
        return []
    groups = []
    start = previous = float(hits[0])
    for current in hits[1:]:
        current = float(current)
        if current - previous > gap:
            if previous - start >= minimum:
                groups.append((start, previous))
            start = current
        previous = current
    if previous - start >= minimum:
        groups.append((start, previous))
    return groups


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trial", type=Path)
    args = parser.parse_args()
    trial = args.trial.resolve()
    bag = Bag(trial / "bag")
    output = trial / "analysis"
    output.mkdir(exist_ok=True)

    waypoint_file = next((trial / "snapshot").glob("*_waypoints.yaml"))
    waypoints = {
        entry["name"]: entry
        for entry in yaml.safe_load(waypoint_file.read_text())["waypoints"]
    }

    status_events = []
    for timestamp, data in bag.rows("/waypoint_patrol/status"):
        message = deserialize_message(data, String)
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            continue
        if state.get("message", "").startswith("navigating to "):
            status_events.append((bag.seconds(timestamp), state))

    legs = []
    cycle = 0
    for index, (start, state) in enumerate(status_events):
        target = state["current_waypoint_name"]
        if target == "A":
            cycle += 1
        end = status_events[index + 1][0] if index + 1 < len(status_events) else None
        if end is None:
            continue
        legs.append(
            {
                "cycle": cycle,
                "target": target,
                "start": start,
                "end": end,
                "duration": end - start,
                "reverse_selected": "reverse approach" in state.get("message", ""),
            }
        )

    amcl = []
    for timestamp, data in bag.rows("/amcl_pose"):
        message = deserialize_message(data, PoseWithCovarianceStamped)
        covariance = message.pose.covariance
        amcl.append(
            (
                bag.seconds(timestamp),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw(message.pose.pose.orientation),
                math.sqrt(max(0.0, covariance[0] + covariance[7])),
                math.sqrt(max(0.0, covariance[35])),
            )
        )
    amcl = np.asarray(amcl, dtype=float)

    plans = []
    for timestamp, data in bag.rows("/plan"):
        message = deserialize_message(data, NavPath)
        if message.poses:
            points = np.asarray(
                [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses],
                dtype=np.float32,
            )
            plans.append((bag.seconds(timestamp), points))
    plan_times = np.asarray([entry[0] for entry in plans])

    # Cross-track error is evaluated at 5 Hz against the most recent global
    # plan. Vertex distance is within one map cell of segment distance because
    # Smac emits paths at costmap resolution.
    cross_track = []
    for sample in amcl[::2]:
        plan_index = int(np.searchsorted(plan_times, sample[0], side="right")) - 1
        if plan_index < 0 or sample[0] - plans[plan_index][0] > 3.0:
            continue
        delta = plans[plan_index][1] - sample[1:3]
        distance = float(np.sqrt(np.min(np.sum(delta * delta, axis=1))))
        cross_track.append((sample[0], distance))
    cross_track = np.asarray(cross_track, dtype=float)

    def read_twists(topic: str):
        values = []
        for timestamp, data in bag.rows(topic):
            message = deserialize_message(data, Twist)
            values.append(
                (
                    bag.seconds(timestamp),
                    message.linear.x,
                    message.linear.y,
                    message.angular.z,
                )
            )
        return np.asarray(values, dtype=float)

    nav = read_twists("/cmd_vel_nav")
    shaped = read_twists("/cmd_vel_shaped")
    safe = read_twists("/cmd_vel_safe")

    odometry = []
    last_kept = -1.0
    for timestamp, data in bag.rows("/odometry/a2"):
        current = bag.seconds(timestamp)
        if current - last_kept < 0.095:
            continue
        last_kept = current
        message = deserialize_message(data, Odometry)
        odometry.append(
            (
                current,
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.angular.z,
            )
        )
    odometry = np.asarray(odometry, dtype=float)

    # Map->odom changes are the corrections AMCL applies to raw odometry.
    corrections = []
    for timestamp, data in bag.rows("/tf"):
        message = deserialize_message(data, TFMessage)
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                corrections.append(
                    (
                        bag.seconds(timestamp),
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        yaw(transform.transform.rotation),
                    )
                )
    corrections = np.asarray(corrections, dtype=float)

    # Match collision-monitor input and output.
    clamp_samples = []
    if len(shaped) and len(safe):
        shaped_times = shaped[:, 0]
        for item in safe:
            index = nearest_index(shaped_times, item[0])
            if abs(shaped[index, 0] - item[0]) > 0.12:
                continue
            source = math.hypot(shaped[index, 1], shaped[index, 2]) + 0.35 * abs(shaped[index, 3])
            result = math.hypot(item[1], item[2]) + 0.35 * abs(item[3])
            clamp_samples.append((item[0], source, result))
    clamp_samples = np.asarray(clamp_samples, dtype=float)

    # Infer unmarked manual motion conservatively: substantial measured motion
    # while the final ROS command is near zero, or motion opposite to it.
    manual_mask = np.zeros(len(odometry), dtype=bool)
    stuck_mask = np.zeros(len(odometry), dtype=bool)
    if len(safe) and len(odometry):
        safe_times = safe[:, 0]
        active_start = status_events[0][0] if status_events else 0.0
        for index, item in enumerate(odometry):
            if item[0] < active_start:
                continue
            command_index = nearest_index(safe_times, item[0])
            command = safe[command_index]
            if abs(command[0] - item[0]) > 0.25:
                command_vector = np.zeros(3)
            else:
                command_vector = command[1:4]
            actual_vector = item[1:4]
            command_motion = math.hypot(*command_vector[:2]) + 0.35 * abs(command_vector[2])
            actual_motion = math.hypot(*actual_vector[:2]) + 0.35 * abs(actual_vector[2])
            opposite = (
                np.linalg.norm(command_vector[:2]) > 0.05
                and np.linalg.norm(actual_vector[:2]) > 0.05
                and float(np.dot(command_vector[:2], actual_vector[:2])) < -0.001
            )
            manual_mask[index] = (
                (actual_motion > 0.07 and command_motion < 0.025)
                or opposite
                or (abs(actual_vector[2]) > 0.18 and abs(command_vector[2]) < 0.035)
            )
            stuck_mask[index] = command_motion > 0.065 and actual_motion < 0.018

    manual_windows = windows(odometry[:, 0], manual_mask, minimum=0.45)
    stuck_windows = windows(odometry[:, 0], stuck_mask, minimum=1.5)
    clamp_windows = []
    if len(clamp_samples):
        clamp_mask = (clamp_samples[:, 1] > 0.04) & (
            clamp_samples[:, 2] < 0.65 * clamp_samples[:, 1]
        )
        clamp_windows = windows(clamp_samples[:, 0], clamp_mask, minimum=0.4)

    # Static-map scan alignment, sampled at 1 Hz. /scan is in base_link.
    map_yaml = next(
        path
        for path in (trial / "snapshot").glob("*.yaml")
        if not path.name.endswith("_waypoints.yaml")
        and path.name not in {"nav2_params.yaml", "collision_monitor.yaml", "patrol.yaml", "velocity_deadband_adapter.yaml"}
    )
    metadata = yaml.safe_load(map_yaml.read_text())
    image = cv2.imread(str(map_yaml.parent / metadata["image"]), cv2.IMREAD_GRAYSCALE)
    resolution = float(metadata["resolution"])
    origin_x, origin_y, _ = metadata["origin"]
    distance_image = cv2.distanceTransform((image > 89).astype(np.uint8), cv2.DIST_L2, 5) * resolution
    alignment = []
    last_scan = -1.0
    amcl_times = amcl[:, 0]
    for timestamp, data in bag.rows("/scan"):
        current = bag.seconds(timestamp)
        if current - last_scan < 0.95:
            continue
        last_scan = current
        pose_index = nearest_index(amcl_times, current)
        if abs(amcl_times[pose_index] - current) > 0.25:
            continue
        message = deserialize_message(data, LaserScan)
        ranges = np.asarray(message.ranges, dtype=float)
        indices = np.arange(len(ranges))[::4]
        ranges = ranges[::4]
        maximum = min(float(message.range_max) - 0.05, 8.0)
        valid = np.isfinite(ranges) & (ranges >= max(0.3, message.range_min)) & (ranges <= maximum)
        if int(valid.sum()) < 10:
            continue
        angles = message.angle_min + indices[valid] * message.angle_increment
        local_x = ranges[valid] * np.cos(angles)
        local_y = ranges[valid] * np.sin(angles)
        pose = amcl[pose_index]
        cosine, sine = math.cos(pose[3]), math.sin(pose[3])
        world_x = pose[1] + cosine * local_x - sine * local_y
        world_y = pose[2] + sine * local_x + cosine * local_y
        columns = np.floor((world_x - origin_x) / resolution).astype(int)
        rows = image.shape[0] - 1 - np.floor((world_y - origin_y) / resolution).astype(int)
        inside = (
            (columns >= 0) & (columns < image.shape[1])
            & (rows >= 0) & (rows < image.shape[0])
        )
        if int(inside.sum()) < 10:
            continue
        distances = distance_image[rows[inside], columns[inside]]
        alignment.append(
            (
                current,
                float(np.mean(distances <= 0.15)),
                float(np.percentile(distances, 20)),
                float(np.median(distances)),
            )
        )
    alignment = np.asarray(alignment, dtype=float)

    def select(array: np.ndarray, start: float, end: float):
        return array[(array[:, 0] >= start) & (array[:, 0] < end)] if len(array) else array

    for leg in legs:
        start, end = leg["start"], leg["end"]
        poses = select(amcl, start, end)
        errors = select(cross_track, start, end)
        commands = select(safe, start, end)
        reverse_commands = select(nav, start, end)
        clamps = select(clamp_samples, start, end)
        scans = select(alignment, start, end)
        target = waypoints[leg["target"]]
        if len(poses):
            final = poses[-1]
            leg["arrival_position_error"] = math.hypot(final[1] - target["x"], final[2] - target["y"])
            target_yaw = 2.0 * math.atan2(target["qz"], target["qw"])
            leg["arrival_yaw_error_deg"] = abs(math.degrees(angle_delta(final[3], target_yaw)))
            leg["position_cov_std_mean"] = float(np.mean(poses[:, 4]))
            leg["yaw_cov_std_deg_mean"] = float(np.degrees(np.mean(poses[:, 5])))
        leg["cross_track_median"] = percentile(errors[:, 1], 50) if len(errors) else None
        leg["cross_track_p95"] = percentile(errors[:, 1], 95) if len(errors) else None
        leg["cross_track_max"] = float(np.max(errors[:, 1])) if len(errors) else None
        leg["negative_nav_vx_fraction"] = (
            float(np.mean(reverse_commands[:, 1] < -0.01)) if len(reverse_commands) else None
        )
        leg["mean_safe_speed"] = (
            float(np.mean(np.hypot(commands[:, 1], commands[:, 2]))) if len(commands) else None
        )
        leg["collision_clamp_fraction"] = (
            float(np.mean((clamps[:, 1] > 0.04) & (clamps[:, 2] < 0.65 * clamps[:, 1])))
            if len(clamps) else None
        )
        leg["scan_alignment_fraction_15cm"] = float(np.mean(scans[:, 1])) if len(scans) else None
        leg["scan_alignment_q20_m"] = float(np.mean(scans[:, 2])) if len(scans) else None

    fieldnames = list(legs[0].keys()) if legs else []
    with (output / "leg_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(legs)

    # Per-cycle aggregates.
    cycles = []
    for number in sorted({leg["cycle"] for leg in legs}):
        members = [leg for leg in legs if leg["cycle"] == number]
        start, end = members[0]["start"], members[-1]["end"]
        errors = select(cross_track, start, end)
        scans = select(alignment, start, end)
        covariance = select(amcl, start, end)
        correction = select(corrections, start, end)
        row = {
            "cycle": number,
            "start": start,
            "end": end,
            "duration": end - start,
            "legs": len(members),
            "cross_track_median": percentile(errors[:, 1], 50) if len(errors) else None,
            "cross_track_p95": percentile(errors[:, 1], 95) if len(errors) else None,
            "cross_track_max": float(np.max(errors[:, 1])) if len(errors) else None,
            "scan_alignment_fraction_15cm": float(np.mean(scans[:, 1])) if len(scans) else None,
            "scan_alignment_q20_m": float(np.mean(scans[:, 2])) if len(scans) else None,
            "position_cov_std_mean": float(np.mean(covariance[:, 4])) if len(covariance) else None,
            "yaw_cov_std_deg_mean": float(np.degrees(np.mean(covariance[:, 5]))) if len(covariance) else None,
        }
        if len(correction) >= 2:
            row["map_odom_net_translation"] = math.hypot(
                correction[-1, 1] - correction[0, 1], correction[-1, 2] - correction[0, 2]
            )
            row["map_odom_net_yaw_deg"] = abs(
                math.degrees(angle_delta(correction[-1, 3], correction[0, 3]))
            )
        cycles.append(row)
    with (output / "cycle_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(cycles[0].keys()))
        writer.writeheader()
        writer.writerows(cycles)

    def label_at(value: float):
        for leg in legs:
            if leg["start"] <= value < leg["end"]:
                return f"cycle {leg['cycle']} to {leg['target']}"
        return "outside patrol"

    summary = {
        "duration_seconds": float((max(amcl[:, 0]) if len(amcl) else 0.0)),
        "completed_cycles": max(0, cycle - 1),
        "plan_count": len(plans),
        "manual_intervention_markers": 0,
        "inferred_manual_windows": [
            {"start": a, "end": b, "duration": b - a, "context": label_at(a)}
            for a, b in manual_windows
        ],
        "stuck_windows": [
            {"start": a, "end": b, "duration": b - a, "context": label_at(a)}
            for a, b in stuck_windows
        ],
        "collision_clamp_windows": [
            {"start": a, "end": b, "duration": b - a, "context": label_at(a)}
            for a, b in clamp_windows
        ],
        "cycles": cycles,
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    # Draw AMCL trajectories by cycle over the exact map snapshot.
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    colors = [(230, 80, 30), (40, 170, 40), (30, 80, 230), (190, 60, 190), (30, 180, 220)]
    for number in sorted({leg["cycle"] for leg in legs}):
        members = [leg for leg in legs if leg["cycle"] == number]
        samples = select(amcl, members[0]["start"], members[-1]["end"])
        pixels = []
        for sample in samples[::2]:
            column = int((sample[1] - origin_x) / resolution)
            row = image.shape[0] - 1 - int((sample[2] - origin_y) / resolution)
            if 0 <= column < image.shape[1] and 0 <= row < image.shape[0]:
                pixels.append((column, row))
        if len(pixels) >= 2:
            cv2.polylines(canvas, [np.asarray(pixels)], False, colors[(number - 1) % len(colors)], 2)
    for name, point in waypoints.items():
        column = int((point["x"] - origin_x) / resolution)
        row = image.shape[0] - 1 - int((point["y"] - origin_y) / resolution)
        cv2.circle(canvas, (column, row), 6, (0, 0, 255), -1)
        cv2.putText(canvas, name, (column + 7, row - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    cv2.imwrite(str(output / "trajectory_by_cycle.png"), canvas)

    print(json.dumps(summary, indent=2))
    print(f"analysis written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
