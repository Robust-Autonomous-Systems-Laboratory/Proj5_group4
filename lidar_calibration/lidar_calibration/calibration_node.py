#!/usr/bin/env python3
import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
import yaml

from rclpy.qos import QoSProfile, ReliabilityPolicy
import rclpy.parameter
from rclpy.parameter_event_handler import ParameterEventHandler 

@dataclass
class RunningStats:
    """Welford online mean/variance."""
    n: int = 0
    mean: float = 0.0
    M2: float = 0.0

    def update(self, x: float) -> None:
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        self.M2 += delta * (x - self.mean)

    @property
    def variance(self) -> float:
        return self.M2 / (self.n - 1) if self.n > 1 else 0.0

    @property
    def std(self) -> float:
        return math.sqrt(self.variance)

    def as_dict(self):
        return {"n": self.n, "mean": self.mean, "std": self.std, "variance": self.variance}


class LidarCalibrationNode(Node):
    def __init__(self):
        super().__init__("lidar_calibration_node")

        # Required parameters
        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("target_angle", 0.0)
        self.declare_parameter("angle_window", 0.1)

        # Nice-to-have parameters
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("outlier_sigma_thresh", 3.0)
        self.declare_parameter("results_path", "results/runtime_calibration.yaml")
        self.declare_parameter("use_median", False)  # average vs median inside window

        #Allow for paramters to be updated on fly
        self.handler = ParameterEventHandler(self)
        self.event_callback_handle = self.handler.add_parameter_event_callback(callback=self.event_callback,)


        self.target_distance = float(self.get_parameter("target_distance").value)
        self.target_angle = float(self.get_parameter("target_angle").value)
        self.angle_window = float(self.get_parameter("angle_window").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.outlier_sigma_thresh = float(self.get_parameter("outlier_sigma_thresh").value)
        self.results_path = str(self.get_parameter("results_path").value)
        self.use_median = bool(self.get_parameter("use_median").value)

        # Publishers
        self.pub_error = self.create_publisher(Float64, "/calibration/range_error", 10)
        self.pub_stats = self.create_publisher(String, "/calibration/statistics", 10)

        # Subscriber
        subscriber_qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", self.scan_callback, subscriber_qos_profile)

        # Running stats
        self.range_stats = RunningStats()  # stats of z_meas
        self.error_stats = RunningStats()  # stats of (z_meas - target_distance)

        # Counters for robustness + beam model discussion
        self.total_scans = 0
        self.valid_scans = 0
        self.invalid_scans = 0
        self.valid_beams_total = 0
        self.invalid_beams_total = 0
        self.max_range_count = 0
        self.outlier_count = 0

        self.last_scan_meta = None
        self.start_time = time.time()

        # Periodic publisher/logging
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self.publish_stats)

        self.get_logger().info(
            f"Started lidar_calibration_node | target_distance={self.target_distance:.3f} m, "
            f"target_angle={self.target_angle:.3f} rad, angle_window={self.angle_window:.3f} rad, "
            f"use_median={self.use_median}"
        )

    def angle_to_index(self, angle: float, angle_min: float, angle_increment: float) -> int:
        return int(round((angle - angle_min) / angle_increment))

    def get_window_indices(self, msg: LaserScan) -> List[int]:
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return []

        # center index for target angle
        center = self.angle_to_index(self.target_angle, msg.angle_min, msg.angle_increment)

        # half window in indices (window is ±angle_window)
        half = int(round(self.angle_window / msg.angle_increment))
        if half < 0:
            half = 0

        # wrap indices safely
        idxs = []
        for k in range(center - half, center + half + 1):
            idxs.append(k % n)
        return idxs
    
    #Update parameters when new ones are set 
    def event_callback(self, parameter_event):
        self.target_distance = float(self.get_parameter("target_distance").value)
        self.target_angle = float(self.get_parameter("target_angle").value)
        self.angle_window = float(self.get_parameter("angle_window").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.outlier_sigma_thresh = float(self.get_parameter("outlier_sigma_thresh").value)
        self.results_path = str(self.get_parameter("results_path").value)
        self.use_median = bool(self.get_parameter("use_median").value)






    def scan_callback(self, msg: LaserScan) -> None:
        self.total_scans += 1
        self.last_scan_meta = {
            "angle_min": float(msg.angle_min),
            "angle_max": float(msg.angle_max),
            "angle_increment": float(msg.angle_increment),
            "range_min": float(msg.range_min),
            "range_max": float(msg.range_max),
            "n_ranges": len(msg.ranges),
        }

        idxs = self.get_window_indices(msg)
        if not idxs:
            self.invalid_scans += 1
            return

        vals = []
        for i in idxs:
            z = float(msg.ranges[i])

            if not math.isfinite(z):
                self.invalid_beams_total += 1
                continue

            # Track max-range behavior (p_max hint)
            if abs(z - float(msg.range_max)) < 1e-3:
                self.max_range_count += 1

            if z < float(msg.range_min) or z > float(msg.range_max):
                self.invalid_beams_total += 1
                continue

            vals.append(z)
            self.valid_beams_total += 1

        if not vals:
            self.invalid_scans += 1
            return

        # One measurement per scan (average or median of window)
        z_meas = float(sorted(vals)[len(vals) // 2]) if self.use_median else float(sum(vals) / len(vals))
        err = z_meas - self.target_distance

        # Outlier detection on error (after warm-up)
        if self.error_stats.n > 30 and self.error_stats.std > 0.0:
            if abs(err - self.error_stats.mean) > self.outlier_sigma_thresh * self.error_stats.std:
                self.outlier_count += 1

        self.range_stats.update(z_meas)
        self.error_stats.update(err)
        self.valid_scans += 1

        # Publish instantaneous error
        m = Float64()
        m.data = err
        self.pub_error.publish(m)

    def publish_stats(self) -> None:
        if self.valid_scans == 0:
            return

        payload = {
            "target_distance": self.target_distance,
            "target_angle": self.target_angle,
            "angle_window": self.angle_window,
            "total_scans": self.total_scans,
            "valid_scans": self.valid_scans,
            "invalid_scans": self.invalid_scans,
            "valid_beams_total": self.valid_beams_total,
            "invalid_beams_total": self.invalid_beams_total,
            "max_range_count": self.max_range_count,
            "outlier_count": self.outlier_count,
            "range_stats": self.range_stats.as_dict(),
            "error_stats": self.error_stats.as_dict(),
            "elapsed_s": time.time() - self.start_time,
        }

        # Publish as text for easy debugging and grading
        s = String()
        s.data = yaml.safe_dump(payload, sort_keys=False)
        self.pub_stats.publish(s)

        # Also log occasionally
        self.get_logger().info(
            f"n={self.range_stats.n} mean={self.range_stats.mean:.4f} std={self.range_stats.std:.4f} "
            f"bias={self.error_stats.mean:+.4f} outliers={self.outlier_count}"
        )

    def save_yaml(self) -> None:
        payload = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "target_distance": self.target_distance,
            "target_angle": self.target_angle,
            "angle_window": self.angle_window,
            "range_stats": self.range_stats.as_dict(),
            "error_stats": self.error_stats.as_dict(),
            "total_scans": self.total_scans,
            "valid_scans": self.valid_scans,
            "invalid_scans": self.invalid_scans,
            "outlier_count": self.outlier_count,
            "max_range_count": self.max_range_count,
            "scan_meta": self.last_scan_meta,
        }

        os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
        with open(self.results_path, "w") as f:
            yaml.safe_dump(payload, f, sort_keys=False)

        self.get_logger().info(f"Saved calibration results to {self.results_path}")


def main():
    rclpy.init()
    node = LidarCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_yaml()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
