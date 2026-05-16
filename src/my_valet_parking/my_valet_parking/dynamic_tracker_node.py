#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from my_valet_parking.slot_table import SLOT_TABLE


CONTROL_PERIOD_S   = 0.05
STATUS_LOG_PERIOD_S = 1.0
SLOT_SCAN_HALF_DEG  = 15.0  # per-slot ±window for distance measurement
SLOT_CLEAR_RATIO    = 1.3   # exit declared when current_dist > baseline * ratio
SLOT_CLEAR_ABS_M    = 0.20  # fallback: exit if current_dist > baseline + this
SLOT_CLEAR_HOLD_S   = 2.0   # seconds hold before declaring slot empty


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


@dataclass
class SideProjection:
    points_odom: np.ndarray
    ranges: np.ndarray
    close_points_odom: np.ndarray
    representative_clearance_m: float
    clear_ratio: float
    close_count: int
    forward_unit_odom: np.ndarray
    side_unit_odom: np.ndarray


class DynamicTrackerNode(Node):
    """Two-phase slot detector.

    BASELINE: after trigger, captures median distance to each of the 3 parked
              vehicles using a per-slot ±15° body-frame scan window.
    MONITOR:  watches for a slot's distance to exceed baseline * SLOT_CLEAR_RATIO
              held for SLOT_CLEAR_HOLD_S seconds, then declares that slot empty.
    DONE:     publishes target PoseStamped at 10 Hz.
    """

    def __init__(self) -> None:
        super().__init__("dynamic_tracker_node")

        self.scan_topic = self.declare_parameter("scan_topic", "/limo1/scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/limo1/odom").value
        self.target_topic = self.declare_parameter(
            "target_topic", "/limo1/target_slot"
        ).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value

        self.side_scan_center_deg = float(
            self.declare_parameter("side_scan_center_deg", -90.0).value
        )
        self.side_scan_half_deg = float(
            self.declare_parameter("side_scan_half_deg", 45.0).value
        )
        self.min_range_m = float(self.declare_parameter("min_range_m", 0.05).value)
        self.max_range_m = float(self.declare_parameter("max_range_m", 10.0).value)
        self.occupied_distance_m = float(
            self.declare_parameter("occupied_distance_m", 0.45).value
        )
        self.baseline_capture_distance_m = float(
            self.declare_parameter("baseline_capture_distance_m", 0.75).value
        )
        self.clear_distance_m = float(
            self.declare_parameter("clear_distance_m", 0.65).value
        )
        min_clear_ratio_default = float(
            self.declare_parameter("min_clear_ratio", 0.25).value
        )
        self.min_clear_ratio = float(
            self.declare_parameter("clear_ratio_min", min_clear_ratio_default).value
        )
        self.side_percentile = float(
            self.declare_parameter("side_percentile", 70.0).value
        )
        self.min_side_rays = int(self.declare_parameter("min_side_rays", 3).value)
        self.baseline_min_points = int(
            self.declare_parameter("baseline_min_points", 3).value
        )
        self.track_box_length_m = float(
            self.declare_parameter("track_box_length_m", 0.35).value
        )
        self.track_box_width_m = float(
            self.declare_parameter("track_box_width_m", 0.30).value
        )
        self.clear_count_fraction = float(
            self.declare_parameter("clear_count_fraction", 0.25).value
        )
        self.clear_max_points = int(
            self.declare_parameter("clear_max_points", 2).value
        )
        self.clear_hold_s = float(self.declare_parameter("clear_hold_s", 0.70).value)
        self.slot_center_offset_m = float(
            self.declare_parameter("slot_center_offset_m", 0.30).value
        )
        self.target_yaw_rad = float(
            self.declare_parameter("target_yaw_rad", -math.pi / 2.0).value
        )
        self.use_side_axis_yaw = bool(
            self.declare_parameter("use_side_axis_yaw", False).value
        )
        self.allow_fallback_without_baseline = bool(
            self.declare_parameter("allow_fallback_without_baseline", False).value
        )
        self.fallback_target_x = float(
            self.declare_parameter("fallback_target_x", 0.0).value
        )
        self.fallback_target_y = float(
            self.declare_parameter("fallback_target_y", -0.75).value
        )
        self.publish_period_s = float(
            self.declare_parameter("publish_period_s", 0.10).value
        )

        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, qos_profile_sensor_data
        )
        self.target_pub = self.create_publisher(PoseStamped, self.target_topic, 10)
        self.create_subscription(
            Bool,
            "/limo1/start_detection",
            self._detection_trigger_cb,
            10)
        self.timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)
        self.debug_timer = self.create_timer(STATUS_LOG_PERIOD_S, self.debug_log_loop)

        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_rx_time = None
        self.current_pose: Optional[Tuple[float, float, float]] = None

        # Phase state machine: IDLE → BASELINE → MONITOR → DONE
        self._phase = "IDLE"
        self._baseline_dist: dict = {}          # slot_name → float
        self._monitor_clear_since: dict = {}    # slot_name → rclpy time or None
        self._detected_slot: Optional[str] = None
        self.target_pose: Optional[PoseStamped] = None
        self.last_publish_time = self.get_clock().now()
        self._last_monitor_log = None

        self.get_logger().info(
            "dynamic_tracker_node ready: "
            f"scan={self.scan_topic}, odom={self.odom_topic}, "
            f"target={self.target_topic}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def scan_callback(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_rx_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def _detection_trigger_cb(self, msg: Bool) -> None:
        if msg.data and self._phase == "IDLE":
            self._phase = "BASELINE"
            self.get_logger().info("Phase: BASELINE — 기준값 캡처 시작")

    # ------------------------------------------------------------------
    # Main timer — phase dispatch
    # ------------------------------------------------------------------

    def control_loop(self) -> None:
        if self._phase == "IDLE":
            return
        if self.latest_scan is None or self.current_pose is None:
            return

        if self._phase == "BASELINE":
            self._run_baseline()
            return

        if self._phase == "MONITOR":
            self._run_monitor()
            return

        if self._phase == "DONE":
            self._publish_target()

    # ------------------------------------------------------------------
    # Phase: BASELINE — capture reference distance to all 3 slots
    # ------------------------------------------------------------------

    def _run_baseline(self) -> None:
        new_baselines = {}
        for slot_name, slot in SLOT_TABLE.items():
            center_deg = self._slot_body_angle(slot["x"], -0.75)
            d = self._sector_median(self.latest_scan, center_deg, SLOT_SCAN_HALF_DEG)
            if d is not None:
                new_baselines[slot_name] = d
        if len(new_baselines) == len(SLOT_TABLE):
            self._baseline_dist = new_baselines
            self._monitor_clear_since = {k: None for k in SLOT_TABLE}
            self._phase = "MONITOR"
            self.get_logger().info(
                "Phase: MONITOR — baseline captured: "
                + ", ".join(f"{k}={v:.3f}m" for k, v in self._baseline_dist.items())
            )

    # ------------------------------------------------------------------
    # Phase: MONITOR — detect which slot's vehicle exited
    # ------------------------------------------------------------------

    def _run_monitor(self) -> None:
        now = self.get_clock().now()

        dists = {}
        for slot_name, slot in SLOT_TABLE.items():
            center_deg = self._slot_body_angle(slot["x"], -0.75)
            d = self._sector_median(self.latest_scan, center_deg, SLOT_SCAN_HALF_DEG)
            dists[slot_name] = d

        for slot_name, d in dists.items():
            baseline = self._baseline_dist.get(slot_name)
            if d is None or baseline is None:
                self._monitor_clear_since[slot_name] = None
                continue
            slot_clear = (
                d > baseline * SLOT_CLEAR_RATIO
                or d > baseline + SLOT_CLEAR_ABS_M
            )
            if slot_clear:
                if self._monitor_clear_since[slot_name] is None:
                    self._monitor_clear_since[slot_name] = now
                elif (now - self._monitor_clear_since[slot_name]).nanoseconds / 1e9 >= SLOT_CLEAR_HOLD_S:
                    self._detected_slot = slot_name
                    self._phase = "DONE"
                    self.get_logger().info(
                        f"[TRACKER] 빈 슬롯 감지: {slot_name} "
                        f"(d={d:.3f}m, baseline={baseline:.3f}m, "
                        f"ratio={d/baseline:.2f}x, delta={d-baseline:+.3f}m)"
                    )
                    self._build_target_pose()
                    return
            else:
                self._monitor_clear_since[slot_name] = None

        # Debug log every ~1 s
        if (
            self._last_monitor_log is None
            or (now - self._last_monitor_log).nanoseconds / 1e9 >= 1.0
        ):
            self._last_monitor_log = now
            dist_str = ", ".join(
                f"{k}={f'{v:.3f}' if v is not None else 'None'}m"
                for k, v in dists.items()
            )
            hold_str = ", ".join(
                f"{k}={f'{(now - t).nanoseconds / 1e9:.1f}' if t is not None else '0.0'}s"
                for k, t in self._monitor_clear_since.items()
            )
            self.get_logger().info(f"[MONITOR] dist=[{dist_str}] hold=[{hold_str}]")

    # ------------------------------------------------------------------
    # Phase: DONE — build and republish target
    # ------------------------------------------------------------------

    def _build_target_pose(self) -> None:
        slot = self._detected_slot
        msg = PoseStamped()
        msg.header.frame_id = slot
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = SLOT_TABLE[slot]["x"]
        msg.pose.position.y = -0.103
        msg.pose.position.z = 0.0
        msg.pose.orientation = yaw_to_quaternion(-math.pi / 2.0)
        self.target_pose = msg
        self.target_pub.publish(msg)
        self.last_publish_time = self.get_clock().now()

    def _publish_target(self) -> None:
        if self.target_pose is None:
            return
        now = self.get_clock().now()
        if (now - self.last_publish_time).nanoseconds / 1e9 < self.publish_period_s:
            return
        self.target_pose.header.stamp = now.to_msg()
        self.target_pub.publish(self.target_pose)
        self.last_publish_time = now

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _slot_body_angle(self, slot_x: float, slot_y: float) -> float:
        """Body-frame angle (degrees) from LIMO1's current pose to a world point."""
        px, py, yaw = self.current_pose
        dx = slot_x - px
        dy = slot_y - py
        world_angle = math.atan2(dy, dx)
        body_angle = normalize_angle(world_angle - yaw)
        return math.degrees(body_angle)

    def _sector_median(
        self, scan: LaserScan, center_deg: float, half_deg: float
    ) -> Optional[float]:
        """Median of valid ranges in a body-frame degree window. None if < 3 rays."""
        lo = center_deg - half_deg
        hi = center_deg + half_deg
        vals = []
        for i, raw in enumerate(scan.ranges):
            ang_deg = math.degrees(scan.angle_min + i * scan.angle_increment)
            if lo <= ang_deg <= hi:
                d = self.usable_scan_range(raw, scan)
                if d is not None:
                    vals.append(d)
        if len(vals) < 3:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def usable_scan_range(
        self, distance: float, msg: LaserScan
    ) -> Optional[float]:
        if math.isnan(distance):
            return None
        if math.isinf(distance):
            return self.max_range_m
        min_range = max(msg.range_min, self.min_range_m)
        max_range = self.max_range_m
        if msg.range_max > 0.0 and math.isfinite(msg.range_max):
            max_range = min(max_range, msg.range_max)
        if distance < min_range:
            return None
        return min(distance, max_range)

    @staticmethod
    def angle_in_sector(angle: float, low: float, high: float) -> bool:
        low_n = normalize_angle(low)
        high_n = normalize_angle(high)
        if low_n <= high_n:
            return low_n <= angle <= high_n
        return angle >= low_n or angle <= high_n

    @staticmethod
    def normalize_vector(vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-6:
            return vector
        return vector / norm

    # ------------------------------------------------------------------
    # Status log
    # ------------------------------------------------------------------

    def debug_log_loop(self) -> None:
        scan_received = self.latest_scan is not None
        scan_age_s = math.inf
        if self.last_scan_rx_time is not None:
            scan_age_s = (
                self.get_clock().now() - self.last_scan_rx_time
            ).nanoseconds / 1e9
        scan_alive = scan_received and scan_age_s <= STATUS_LOG_PERIOD_S * 2.0
        odom_received = self.current_pose is not None

        slot_angles_str = "N/A"
        if odom_received:
            parts = []
            for slot_name, slot in SLOT_TABLE.items():
                ang = self._slot_body_angle(slot["x"], -0.75)
                parts.append(f"{slot_name}={ang:.1f}°")
            slot_angles_str = ", ".join(parts)

        self.get_logger().info(
            f"[Tracker DEBUG] phase={self._phase}, "
            f"scan={scan_alive}(age={scan_age_s:.1f}s), "
            f"odom={odom_received}, "
            f"slot_angles=[{slot_angles_str}], "
            f"baseline={self._baseline_dist}, "
            f"detected={self._detected_slot}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicTrackerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
