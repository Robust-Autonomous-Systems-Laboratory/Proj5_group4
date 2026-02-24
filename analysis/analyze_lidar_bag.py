#!/usr/bin/env python3
"""
Analyze a ROS 2 MCAP bag containing sensor_msgs/msg/LaserScan data and compute
basic calibration statistics around a target angle.

Example:
    python3 analysis/analyze_lidar_bag.py \
      --bag data/lidar_calibration_/lidar_calibration_05m \
      --true-distance 0.5 \
      --target-angle 0.0 \
      --angle-window 0.1 \
      --out-figure analysis/figures/hist_05m.png \
      --out-yaml results/stats_05m.yaml
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml
import matplotlib.pyplot as plt

# ROS 2 imports (Jazzy)
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze LiDAR calibration bag.")
    parser.add_argument(
        "--bag",
        required=True,
        help="Path to ros2 bag directory (the folder containing metadata.yaml / *.mcap).",
    )
    parser.add_argument(
        "--topic",
        default="/scan",
        help="LaserScan topic to read (default: /scan).",
    )
    parser.add_argument(
        "--true-distance",
        type=float,
        required=True,
        help="Ground-truth target distance in meters (e.g., 0.5, 1.0, 2.0).",
    )
    parser.add_argument(
        "--target-angle",
        type=float,
        default=0.0,
        help="Target angle in radians (default: 0.0). Usually front beam.",
    )
    parser.add_argument(
        "--angle-window",
        type=float,
        default=0.1,
        help="Half-window around target angle in radians (default: 0.1).",
    )
    parser.add_argument(
        "--out-figure",
        required=True,
        help="Path to save histogram PNG.",
    )
    parser.add_argument(
        "--out-yaml",
        required=True,
        help="Path to save summary YAML.",
    )
    parser.add_argument(
        "--bins",
        type=int,
        default=30,
        help="Histogram bins (default: 30).",
    )
    return parser.parse_args()


def ensure_parent_dir(path_str: str) -> None:
    Path(path_str).parent.mkdir(parents=True, exist_ok=True)


def open_reader(bag_dir: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_dir, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def get_topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    topic_types = reader.get_all_topics_and_types()
    return {t.name: t.type for t in topic_types}


def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a-b in [-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def extract_window_ranges_from_scan(
    scan_msg: Any,
    target_angle: float,
    angle_window: float,
) -> Tuple[List[float], Optional[float]]:
    """
    Returns:
      - all valid ranges within [target_angle-angle_window, target_angle+angle_window]
      - representative value for this scan (median of valid window ranges), or None
    """
    valid_values: List[float] = []

    angle = float(scan_msg.angle_min)
    angle_increment = float(scan_msg.angle_increment)

    if angle_increment == 0.0:
        return [], None

    rmin = float(scan_msg.range_min)
    rmax = float(scan_msg.range_max)

    for r in scan_msg.ranges:
        rr = float(r)

        # Check angular inclusion first
        if abs(angle_diff(angle, target_angle)) <= angle_window:
            # Valid finite range and sensor bounds
            if math.isfinite(rr) and (rmin <= rr <= rmax):
                valid_values.append(rr)

        angle += angle_increment

    if not valid_values:
        return [], None

    rep = float(np.median(np.array(valid_values, dtype=np.float64)))
    return valid_values, rep


def summarize(
    all_window_values: List[float],
    per_scan_values: List[float],
    true_distance: float,
) -> Dict[str, Any]:
    if len(all_window_values) == 0:
        raise RuntimeError("No valid LiDAR ranges found in the requested angular window.")

    arr_all = np.array(all_window_values, dtype=np.float64)
    arr_scan = np.array(per_scan_values, dtype=np.float64) if per_scan_values else np.array([], dtype=np.float64)

    # Main stats based on representative per-scan values if available; otherwise fallback to all samples
    base = arr_scan if arr_scan.size > 0 else arr_all

    mean_val = float(np.mean(base))
    median_val = float(np.median(base))
    std_val = float(np.std(base, ddof=1)) if base.size > 1 else 0.0
    min_val = float(np.min(base))
    max_val = float(np.max(base))

    abs_errors = np.abs(base - true_distance)
    mae = float(np.mean(abs_errors))
    rmse = float(np.sqrt(np.mean((base - true_distance) ** 2)))
    bias = float(mean_val - true_distance)

    q1, q3 = (float(np.percentile(base, 25)), float(np.percentile(base, 75))) if base.size > 0 else (None, None)

    return {
        "true_distance_m": float(true_distance),
        "num_window_samples_total": int(arr_all.size),
        "num_scans_used": int(arr_scan.size),
        "statistics_basis": "per_scan_median_window_ranges" if arr_scan.size > 0 else "all_window_ranges",
        "mean_m": mean_val,
        "median_m": median_val,
        "std_m": std_val,
        "min_m": min_val,
        "max_m": max_val,
        "q1_m": q1,
        "q3_m": q3,
        "bias_m": bias,
        "mae_m": mae,
        "rmse_m": rmse,
    }


def save_histogram(
    values: List[float],
    true_distance: float,
    out_path: str,
    bins: int = 30,
) -> None:
    ensure_parent_dir(out_path)
    arr = np.array(values, dtype=np.float64)

    plt.figure(figsize=(8, 5))
    plt.hist(arr, bins=bins, edgecolor="black")
    plt.axvline(true_distance, linestyle="--", linewidth=2, label=f"True distance = {true_distance:.3f} m")
    plt.xlabel("Measured distance (m)")
    plt.ylabel("Count")
    plt.title("LiDAR Range Distribution (Selected Angular Window)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def save_yaml(data: Dict[str, Any], out_path: str) -> None:
    ensure_parent_dir(out_path)
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def analyze_bag(
    bag_dir: str,
    topic: str,
    true_distance: float,
    target_angle: float,
    angle_window: float,
) -> Dict[str, Any]:
    if not os.path.isdir(bag_dir):
        raise FileNotFoundError(f"Bag directory not found: {bag_dir}")

    reader = open_reader(bag_dir)
    topic_type_map = get_topic_type_map(reader)

    if topic not in topic_type_map:
        available = ", ".join(sorted(topic_type_map.keys()))
        raise RuntimeError(
            f"Topic '{topic}' not found in bag.\nAvailable topics: {available}"
        )

    msg_type_str = topic_type_map[topic]
    if msg_type_str != "sensor_msgs/msg/LaserScan":
        raise RuntimeError(
            f"Topic '{topic}' is '{msg_type_str}', expected 'sensor_msgs/msg/LaserScan'."
        )

    msg_cls = get_message(msg_type_str)

    all_window_values: List[float] = []
    per_scan_representatives: List[float] = []

    total_msgs = 0
    used_msgs = 0
    first_stamp_ns: Optional[int] = None
    last_stamp_ns: Optional[int] = None

    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()

        if topic_name != topic:
            continue

        total_msgs += 1
        if first_stamp_ns is None:
            first_stamp_ns = int(timestamp_ns)
        last_stamp_ns = int(timestamp_ns)

        scan_msg = deserialize_message(serialized_data, msg_cls)
        window_vals, rep = extract_window_ranges_from_scan(
            scan_msg=scan_msg,
            target_angle=target_angle,
            angle_window=angle_window,
        )

        if window_vals:
            all_window_values.extend(window_vals)
            used_msgs += 1
        if rep is not None:
            per_scan_representatives.append(rep)

    if total_msgs == 0:
        raise RuntimeError(f"No messages read from topic '{topic}'.")

    summary = summarize(all_window_values, per_scan_representatives, true_distance=true_distance)

    summary.update(
        {
            "bag_path": bag_dir,
            "topic": topic,
            "topic_type": "sensor_msgs/msg/LaserScan",
            "target_angle_rad": float(target_angle),
            "angle_window_rad": float(angle_window),
            "total_scan_messages": int(total_msgs),
            "scan_messages_with_valid_window_data": int(used_msgs),
            "first_timestamp_ns": int(first_stamp_ns) if first_stamp_ns is not None else None,
            "last_timestamp_ns": int(last_stamp_ns) if last_stamp_ns is not None else None,
            "duration_s_from_selected_topic": (
                float((last_stamp_ns - first_stamp_ns) / 1e9)
                if first_stamp_ns is not None and last_stamp_ns is not None
                else None
            ),
        }
    )

    return summary


def main() -> None:
    args = parse_args()

    summary = analyze_bag(
        bag_dir=args.bag,
        topic=args.topic,
        true_distance=args.true_distance,
        target_angle=args.target_angle,
        angle_window=args.angle_window,
    )

    # Histogram uses per-scan representative values if present, else fallback to all samples
    # Recompute chosen list from summary basis not stored as values, so read YAML-friendly stats only.
    # To avoid re-reading bag, use the bag again minimally OR simply histogram all window samples by rerun.
    # Better: rerun once and collect both arrays here for figure consistency.
    # Simpler/robust approach: just read again for plotting.
    reader = open_reader(args.bag)
    topic_type_map = get_topic_type_map(reader)
    msg_cls = get_message(topic_type_map[args.topic])

    all_window_values: List[float] = []
    per_scan_representatives: List[float] = []

    while reader.has_next():
        topic_name, serialized_data, _ = reader.read_next()
        if topic_name != args.topic:
            continue
        scan_msg = deserialize_message(serialized_data, msg_cls)
        window_vals, rep = extract_window_ranges_from_scan(
            scan_msg=scan_msg,
            target_angle=args.target_angle,
            angle_window=args.angle_window,
        )
        if window_vals:
            all_window_values.extend(window_vals)
        if rep is not None:
            per_scan_representatives.append(rep)

    plot_values = per_scan_representatives if per_scan_representatives else all_window_values

    save_histogram(
        values=plot_values,
        true_distance=args.true_distance,
        out_path=args.out_figure,
        bins=args.bins,
    )
    save_yaml(summary, args.out_yaml)

    print("Analysis complete.")
    print(f"  Bag:        {args.bag}")
    print(f"  Topic:      {args.topic}")
    print(f"  Figure:     {args.out_figure}")
    print(f"  YAML:       {args.out_yaml}")
    print(f"  Mean (m):   {summary['mean_m']:.6f}")
    print(f"  Bias (m):   {summary['bias_m']:.6f}")
    print(f"  RMSE (m):   {summary['rmse_m']:.6f}")
    print(f"  Scans used: {summary['num_scans_used']}")


if __name__ == "__main__":
    main()
