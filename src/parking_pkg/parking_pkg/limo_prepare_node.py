#!/usr/bin/env python3

import math
from enum import Enum
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from my_valet_parking.slot_table import SLOT_TABLE

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

# =========================================================
# PARAMS
# =========================================================

CONTROL_PERIOD_S = 0.05

START_X_M = 0.0215
START_Y_M = -0.1030

EVADE_REVERSE_DISTANCE_M = 0.57
EVADE_SPEED_MPS = 0.08
EVADE_TOLERANCE_M = 0.05

RETURN_TO_START_TOLERANCE_M = 0.060
RETURN_TO_START_SPEED_MPS = 0.035

TARGET_SLOT_FRESHNESS_S = 20.0

SPOT_VALIDATION_PASS_HOLD_S = 0.3

SPOT_SIDE_SCAN_CENTER_DEG = 90.0
SPOT_SIDE_SCAN_HALF_DEG = 25.0
SPOT_SIDE_CLEAR_M = 0.50
SPOT_SIDE_MIN_RAYS = 3

MAX_LINEAR_SPEED_MPS = 0.05
LINEAR_RAMP_RATE = 0.020   # max linear.x change per 0.05s tick => 0.40 m/s²
ANGULAR_RAMP_RATE = 0.06   # max angular.z change per tick

AISLE_Y_M = -0.103


# =========================================================
# UTILS
# =========================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# =========================================================
# FSM
# =========================================================

class PrepareState(Enum):
    EVADE = "EVADE"
    WAIT_FOR_EXIT = "WAIT_FOR_EXIT"
    RETURN_TO_START = "RETURN_TO_START"
    SPOT_VALIDATION = "SPOT_VALIDATION"
    DONE = "DONE"


# =========================================================
# NODE
# =========================================================

class LimoPrepareNode(Node):

    def __init__(self):
        super().__init__("limo_prepare_node")

        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )

        self.ready_pub = self.create_publisher(
            Bool,
            "parking_ready",
            10
        )

        self.detection_trigger_pub = self.create_publisher(
            Bool, "start_detection", 10)

        self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            scan_qos
        )

        self.create_subscription(
            PoseStamped,
            "target_slot",
            self.target_slot_callback,
            10
        )

        self.timer = self.create_timer(
            CONTROL_PERIOD_S,
            self.control_loop
        )

        self.state = PrepareState.EVADE

        self.current_position = None
        self.latest_scan = None

        self.target_slot_received = False
        self.target_received_time = None

        self.evade_target = None
        self.return_target = None

        self.target_slot_name = "limo2"
        self.goal_x = 0.0215
        self.goal_y = AISLE_Y_M
        self._detection_triggered = False
        self._wait_log_time = None

        self.spot_pass_since = None

        self._cur_linear = 0.0
        self._cur_angular = 0.0
        self._ready_published = False

        self.get_logger().info("Prepare Node Started")

    # =====================================================
    # CALLBACKS
    # =====================================================

    def odom_callback(self, msg):

        p = msg.pose.pose.position

        self.current_position = (p.x, p.y)

    def scan_callback(self, msg):

        self.latest_scan = msg

    def target_slot_callback(self, msg):
        if self.state == PrepareState.DONE:
            return
        self.target_slot_received = True
        self.target_received_time = self.get_clock().now()
        slot_name = msg.header.frame_id
        if slot_name in SLOT_TABLE:
            if self.target_slot_name != slot_name:  # 변경될 때만 로그
                self.target_slot_name = slot_name
                self.goal_x = SLOT_TABLE[slot_name]["x"]
                self.goal_y = AISLE_Y_M
                self.get_logger().info(
                    f"목표 슬롯 확정: {slot_name} → "
                    f"goal=({self.goal_x:.3f}, {self.goal_y:.3f})")
            else:
                self.target_slot_name = slot_name
                self.goal_x = SLOT_TABLE[slot_name]["x"]
                self.goal_y = AISLE_Y_M

    # =====================================================
    # MAIN LOOP
    # =====================================================

    def control_loop(self):

        if self.current_position is None:
            self.publish_cmd(0.0)
            return

        if self.state == PrepareState.EVADE:

            if self.run_evade():

                self.transition(
                    PrepareState.WAIT_FOR_EXIT
                )

        elif self.state == PrepareState.WAIT_FOR_EXIT:
            self.publish_cmd(0.0)
            now = self.get_clock().now()
            if (
                self._wait_log_time is None
                or (now - self._wait_log_time).nanoseconds / 1e9 >= 3.0
            ):
                self._wait_log_time = now
                self.get_logger().info("WAIT_FOR_EXIT: 차량 이탈 대기 중...")

            if self.target_slot_is_fresh():

                self.transition(
                    PrepareState.RETURN_TO_START
                )

        elif self.state == PrepareState.RETURN_TO_START:

            if self.run_return_to_start():

                self.transition(
                    PrepareState.SPOT_VALIDATION
                )

        elif self.state == PrepareState.SPOT_VALIDATION:

            if self.run_spot_validation():

                self.transition(
                    PrepareState.DONE
                )

        elif self.state == PrepareState.DONE:
            if not self._ready_published:
                self.publish_cmd(0.0)
                if abs(self._cur_linear) < 0.001 and abs(self._cur_angular) < 0.001:
                    msg = Bool()
                    msg.data = True
                    self.ready_pub.publish(msg)
                    self._ready_published = True
                    self.get_logger().info("parking_ready=True 발행 → parking_node 핸드오프")

    # =====================================================
    # EVADE
    # =====================================================

    def run_evade(self):

        if self.evade_target is None:

            self.evade_target = (
                self.current_position[0]
                + EVADE_REVERSE_DISTANCE_M
            )

        dist = abs(
            self.evade_target
            - self.current_position[0]
        )

        if dist <= EVADE_TOLERANCE_M:

            self.publish_cmd(0.0)
            if abs(self._cur_linear) > 0.001:
                return False
            if not self._detection_triggered:
                trig = Bool()
                trig.data = True
                self.detection_trigger_pub.publish(trig)
                self._detection_triggered = True
                self.get_logger().info("EVADE 완료 → 슬롯 감지 트리거 발행")

            return True

        speed = clamp(
            dist * 0.5,
            0.015,
            EVADE_SPEED_MPS
        )

        self.publish_cmd(-speed)

        return False

    # =====================================================
    # RETURN
    # =====================================================

    def run_return_to_start(self):

        dx = self.goal_x - self.current_position[0]
        dy = self.goal_y - self.current_position[1]
        dist = math.hypot(dx, dy)

        if dist <= RETURN_TO_START_TOLERANCE_M:

            self.publish_cmd(0.0)

            self.get_logger().info(
                f"RETURN_TO_START 완료 | "
                f"pos=({self.current_position[0]:.3f}, "
                f"{self.current_position[1]:.3f})")
            return True

        speed = clamp(dist * 0.5, 0.015, RETURN_TO_START_SPEED_MPS)

        self.publish_cmd(speed)

        return False

    # =====================================================
    # VALIDATION
    # =====================================================

    def run_spot_validation(self):

        self.publish_cmd(0.0)

        if not self.target_slot_received:

            return False

        side_m = self.measure_side_clearance()

        if side_m < SPOT_SIDE_CLEAR_M:

            self.spot_pass_since = None

            return False

        now = self.get_clock().now()

        if self.spot_pass_since is None:

            self.spot_pass_since = now

        held = (
            now - self.spot_pass_since
            >= Duration(
                seconds=SPOT_VALIDATION_PASS_HOLD_S
            )
        )

        return held

    def measure_side_clearance(self):

        if self.latest_scan is None:
            return 999.0

        ranges = self.sector_ranges(
            self.latest_scan,
            SPOT_SIDE_SCAN_CENTER_DEG
            - SPOT_SIDE_SCAN_HALF_DEG,
            SPOT_SIDE_SCAN_CENTER_DEG
            + SPOT_SIDE_SCAN_HALF_DEG
        )

        if len(ranges) < SPOT_SIDE_MIN_RAYS:
            return 999.0

        ranges.sort()

        return ranges[len(ranges)//2]

    # =====================================================
    # HELPERS
    # =====================================================

    def target_slot_is_fresh(self):

        if not self.target_slot_received:
            return False

        age = (
            self.get_clock().now()
            - self.target_received_time
        )

        return age <= Duration(
            seconds=TARGET_SLOT_FRESHNESS_S
        )

    def sector_ranges(
        self,
        msg,
        start_deg,
        end_deg
    ):

        out = []

        for i, d in enumerate(msg.ranges):

            if math.isnan(d):
                continue

            deg = math.degrees(
                msg.angle_min
                + i * msg.angle_increment
            )

            if start_deg <= deg <= end_deg:

                out.append(d)

        return out

    def publish_cmd(self, linear, angular=0.0):

        diff_l = clamp(linear, -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS) \
                 - self._cur_linear
        if abs(diff_l) <= LINEAR_RAMP_RATE:
            self._cur_linear += diff_l
        else:
            self._cur_linear += LINEAR_RAMP_RATE * (1.0 if diff_l > 0.0 else -1.0)

        diff_a = angular - self._cur_angular
        if abs(diff_a) <= ANGULAR_RAMP_RATE:
            self._cur_angular += diff_a
        else:
            self._cur_angular += ANGULAR_RAMP_RATE * (1.0 if diff_a > 0.0 else -1.0)

        cmd = Twist()
        cmd.linear.x = self._cur_linear
        cmd.angular.z = self._cur_angular
        self.cmd_pub.publish(cmd)

    def transition(self, new_state):

        self.get_logger().info(
            f"{self.state.value} -> {new_state.value}"
        )

        self.state = new_state


def main(args=None):

    rclpy.init(args=args)

    node = LimoPrepareNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
