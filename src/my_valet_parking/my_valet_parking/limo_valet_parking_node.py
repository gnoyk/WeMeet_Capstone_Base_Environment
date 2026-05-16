#!/usr/bin/env python3
"""
limo_valet_parking_node.py
==========================================================================
FSM 흐름:

  [1단계 — prepare: 조향 없이 직진/후진만]
    EVADE           : 후진으로 공간 확보 (직진 이동만)
    WAIT_FOR_EXIT   : LIMO2 이탈 신호 대기 (정지)
    RETURN_TO_START : 출발 위치로 전진 복귀 (직진 이동만)
    SPOT_VALIDATION : 빈칸 유효성 확인 (정지)

  [2단계 — parking: 실제 주차 방식]
    FORWARD_TURN    : 전진하며 슬롯 방향으로 조향 (호 궤적)
                      → 슬롯 입구 측면 위치(align_point)까지 이동
    REVERSE_ALIGN   : 약간 후진하며 슬롯과 평행 정렬
    FORWARD_PARK    : 직진으로 슬롯 진입 → 주차 완료
    FINISH / ABORT
"""

import math
from enum import Enum
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


# ══════════════════════════════════════════════════════════
# 타이밍
# ══════════════════════════════════════════════════════════
CONTROL_PERIOD_S                 = 0.05
STATUS_LOG_PERIOD_S              = 0.5
WAIT_FOR_EXIT_DEBUG_LOG_PERIOD_S = 1.0
IDLE_WARN_PERIOD_S               = 5.0
ABORT_ALERT_PERIOD_S             = 2.0

# ══════════════════════════════════════════════════════════
# 초기 위치 (world frame)
# ══════════════════════════════════════════════════════════
START_X_M     = 0.0215
START_Y_M     = -0.1030
START_YAW_RAD = math.pi

# ══════════════════════════════════════════════════════════
# 슬롯 정보
# ══════════════════════════════════════════════════════════
SLOT_WORLD_X_M      = 0.0
SLOT_WORLD_Y_M      = -0.75
SLOT_TARGET_YAW_RAD = -math.pi / 2.0

# ══════════════════════════════════════════════════════════
# 전역 속도 상한
# ══════════════════════════════════════════════════════════
MAX_LINEAR_SPEED_MPS    = 0.05
MAX_ANGULAR_SPEED_RADPS = 0.30

# Gazebo ODE 크래시 방지: 선속도 급변 억제
# 매 tick(0.05s) 최대 0.008 m/s 변화 → 실효 가속도 약 0.16 m/s²
LINEAR_RAMP_RATE = 0.008

# ══════════════════════════════════════════════════════════
# E-STOP (FORWARD_PARK 단계에서만 활성화)
# ══════════════════════════════════════════════════════════
ESTOP_FRONT_START_DEG = -30.0
ESTOP_FRONT_END_DEG   =  30.0
ESTOP_DISTANCE_M      =  0.30
RECOVERY_SPEED_MPS    = -0.03
RECOVERY_SECONDS      =  0.8

# ══════════════════════════════════════════════════════════
# Depth 카메라
# ══════════════════════════════════════════════════════════
DEPTH_INVALID_REPLACEMENT_M = 10.0

# ══════════════════════════════════════════════════════════
# [1단계] EVADE
# ══════════════════════════════════════════════════════════
EVADE_REVERSE_DISTANCE_M = 0.60
EVADE_SPEED_MPS          = 0.035
EVADE_TOLERANCE_M        = 0.025

# ══════════════════════════════════════════════════════════
# [1단계] RETURN_TO_START
# ══════════════════════════════════════════════════════════
RETURN_TO_START_TOLERANCE_M = 0.030
RETURN_TO_START_SPEED_MPS   = 0.035

# ══════════════════════════════════════════════════════════
# [1단계] WAIT_FOR_EXIT / SPOT_VALIDATION
# ══════════════════════════════════════════════════════════
TARGET_SLOT_FRESHNESS_S     = 2.0
SPOT_VALIDATION_PASS_HOLD_S = 0.3

# 측면 라이다 빈칸 확인 (슬롯이 왼쪽 → body 기준 +90°)
SPOT_SIDE_SCAN_CENTER_DEG =  90.0
SPOT_SIDE_SCAN_HALF_DEG   =  25.0
SPOT_SIDE_CLEAR_M         =   0.50
SPOT_SIDE_MIN_RAYS        =    3

# ══════════════════════════════════════════════════════════
# [2단계] FORWARD_TURN
# ══════════════════════════════════════════════════════════
# align_point 계산:
#   x: entry_x + left_normal_x * TURN_LATERAL_OFFSET_M = entry_x + 0.25
#      (+x 방향, LIMO 후방 쪽 → 전진+조향으로 호 궤적을 그리며 도달)
#   y: entry_y + TURN_Y_BACK_M
#      (슬롯 입구에서 위쪽으로 올려서 LIMO3 앞에서 완료)
#
# TURN_Y_BACK_M 을 크게 하면 → LIMO3와 더 멀리 떨어진 위치에서 완료
#                              → REVERSE_ALIGN 후진 거리가 길어짐
# TURN_Y_BACK_M = 0.50 → align_y ≈ -0.252 (LIMO3 y≈-0.30 보다 위)
SLOT_DEPTH_M             = 0.40
TURN_LATERAL_OFFSET_M    = 0.6   # entry_x 에서 +x 방향 오프셋
TURN_Y_BACK_M            = 0.4   # entry_y 에서 +y 방향 오프셋 (LIMO3 위쪽)
FORWARD_TURN_SPEED_MPS   = 0.030
FORWARD_TURN_KP_YAW      = 2.5
FORWARD_TURN_MAX_ANGULAR = 0.80
FORWARD_TURN_XY_TOL_M    = 0.060  # align_point 도달 판정 거리 (여유 있게)

# ══════════════════════════════════════════════════════════
# [2단계] REVERSE_ALIGN
# ══════════════════════════════════════════════════════════
REVERSE_ALIGN_SPEED_MPS   = 0.025
REVERSE_ALIGN_KP_YAW      = 2.0
REVERSE_ALIGN_MAX_ANGULAR = 0.60
REVERSE_ALIGN_DISTANCE_M  = 0.70   # align_point → entry_point 거리 커버 (약 0.50m + 여유)
REVERSE_ALIGN_YAW_TOL     = math.radians(5.0)
REVERSE_ALIGN_HOLD_S      = 0.3

# ══════════════════════════════════════════════════════════
# [2단계] FORWARD_PARK
# ══════════════════════════════════════════════════════════
FORWARD_PARK_SPEED_MPS   = 0.030
FORWARD_PARK_KP_LATERAL  = 1.4
FORWARD_PARK_KP_YAW      = 2.2
FORWARD_PARK_LAT_CLAMP   = 0.18
FORWARD_PARK_FINISH_HOLD = 0.6
FORWARD_PARK_FINE_M      = 0.04
FORWARD_PARK_X_PREC_M    = 0.015
FORWARD_PARK_YAW_PREC    = math.radians(3.0)
FORWARD_PARK_CREEP_MPS   = 0.012


# ══════════════════════════════════════════════════════════
# 유틸
# ══════════════════════════════════════════════════════════
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def shortest_angular_distance(target: float, current: float) -> float:
    return normalize_angle(target - current)


def compute_slot_points(
    slot_x: float, slot_y: float, slot_yaw: float
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    """
    entry_point 와 align_point 를 계산해 반환.

    slot_yaw = -π/2 기준:
      heading     = (0, -1)  → 슬롯 진입 방향(아래)
      left_normal = (1,  0)  → +x 방향

      entry_point = (slot_x, slot_y + SLOT_DEPTH/2)  ← 슬롯 입구

      align_point:
        x = entry_x + left_normal_x * TURN_LATERAL_OFFSET_M  (+x, LIMO 후방 쪽)
        y = entry_y + TURN_Y_BACK_M  (+y, LIMO3 위쪽으로 올림)
        → LIMO가 전진+조향 호 궤적으로 도달하고, 여기서 후진 정렬 시작
    """
    heading     = (math.cos(slot_yaw), math.sin(slot_yaw))
    left_normal = (-math.sin(slot_yaw), math.cos(slot_yaw))

    entry_x = slot_x - heading[0] * (SLOT_DEPTH_M / 2.0)
    entry_y = slot_y - heading[1] * (SLOT_DEPTH_M / 2.0)

    align_x = entry_x + left_normal[0] * TURN_LATERAL_OFFSET_M
    align_y = entry_y + TURN_Y_BACK_M   # LIMO3 통과 없이 도달 가능한 높이

    return (entry_x, entry_y), (align_x, align_y)


# ══════════════════════════════════════════════════════════
# FSM 상태
# ══════════════════════════════════════════════════════════
class MissionState(Enum):
    EVADE           = "EVADE"
    WAIT_FOR_EXIT   = "WAIT_FOR_EXIT"
    RETURN_TO_START = "RETURN_TO_START"
    SPOT_VALIDATION = "SPOT_VALIDATION"
    FORWARD_TURN    = "FORWARD_TURN"
    REVERSE_ALIGN   = "REVERSE_ALIGN"
    FORWARD_PARK    = "FORWARD_PARK"
    FINISH          = "FINISH"
    ABORT           = "ABORT"


# ══════════════════════════════════════════════════════════
# 노드
# ══════════════════════════════════════════════════════════
class LimoValetParkingNode(Node):

    def __init__(self) -> None:
        super().__init__("limo_valet_parking_node")

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub         = self.create_publisher(Twist, "cmd_vel", 10)
        self.debug_image_pub = self.create_publisher(Image, "debug/valet_exit_gate", 10)

        self.create_subscription(Odometry,    "odom",            self.odom_callback,       qos_profile_sensor_data)
        self.create_subscription(LaserScan,   "scan",            self.scan_callback,        scan_qos)
        self.create_subscription(Image,       "rgb/image_raw",   self.image_callback,       qos_profile_sensor_data)
        self.create_subscription(Image,       "depth/image_raw", self.depth_callback,       qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "target_slot",     self.target_slot_callback, 10)

        self.control_timer = self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.state        = MissionState.EVADE
        self.state_logged: set = set()

        now = self.get_clock().now()
        self.last_status_log_time              = now
        self.last_wait_for_exit_debug_log_time: Optional[rclpy.time.Time] = None
        self.wait_for_exit_enter_time: Optional[rclpy.time.Time]          = None
        self.last_idle_warn_time               = now
        self.last_abort_alert_time: Optional[rclpy.time.Time]             = None
        self.abort_reason: str                 = ""

        self.odom_received  = False
        self.scan_received  = False
        self.image_received = False
        self.depth_received = False

        self.current_position: Optional[Tuple[float, float]] = None
        self.current_yaw: Optional[float]                    = None

        self.latest_scan: Optional[LaserScan]   = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_image_header                = None
        self.latest_depth: Optional[np.ndarray] = None

        self.target_slot_received                        = False
        self.target_slot_position: Optional[Tuple[float, float]] = None
        self.target_slot_yaw: Optional[float]            = None
        self.latest_target_slot: Optional[PoseStamped]   = None
        self.target_received_time                        = None

        self.front_obstacle_distance_m = math.inf
        self.recovery_active           = False
        self.recovery_start_time       = None

        self.last_cmd_linear  = 0.0
        self.last_cmd_angular = 0.0
        self._ramped_linear   = 0.0   # Gazebo 크래시 방지 ramping 상태

        self.evade_target: Optional[Tuple[float, float]]    = None
        self.evade_start_pos: Optional[Tuple[float, float]] = None

        self.return_to_start_target: Optional[Tuple[float, float]] = None

        self.spot_pass_since  = None
        self.spot_block_since = None
        self.last_spot_lidar_clear_m = math.inf
        self.last_spot_depth_clear_m = math.inf

        self.ft_align_point: Optional[Tuple[float, float]] = None

        self.ra_start_pos: Optional[Tuple[float, float]] = None
        self.ra_stable_since                             = None

        self.fp_finish_since = None

        self.get_logger().info(
            "limo_valet_parking_node 시작\n"
            "  1단계(조향 없음): EVADE → WAIT_FOR_EXIT → RETURN_TO_START → SPOT_VALIDATION\n"
            "  2단계(실제 주차): FORWARD_TURN → REVERSE_ALIGN → FORWARD_PARK → FINISH"
        )

    # ══════════════════════════════════════════════════════
    # ROS 콜백
    # ══════════════════════════════════════════════════════

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_received = True
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.current_position = (p.x, p.y)
        self.current_yaw = self._quat_to_yaw(o.x, o.y, o.z, o.w)

    def scan_callback(self, msg: LaserScan) -> None:
        self.scan_received = True
        self.latest_scan = msg
        front = self._sector_ranges(msg, ESTOP_FRONT_START_DEG, ESTOP_FRONT_END_DEG)
        self.front_obstacle_distance_m = min(front) if front else math.inf

    def image_callback(self, msg: Image) -> None:
        frame = self._decode_image(msg)
        if frame is None:
            return
        self.image_received = True
        self.latest_frame = frame
        self.latest_image_header = msg.header

    def depth_callback(self, msg: Image) -> None:
        depth = self._decode_depth(msg)
        if depth is None:
            return
        self.depth_received = True
        self.latest_depth = depth

    def target_slot_callback(self, msg: PoseStamped) -> None:
        o = msg.pose.orientation
        yaw = self._quat_to_yaw(o.x, o.y, o.z, o.w)
        self.latest_target_slot   = msg
        self.target_received_time = self.get_clock().now()
        self.target_slot_position = (msg.pose.position.x, msg.pose.position.y)
        self.target_slot_yaw      = yaw
        first = not self.target_slot_received
        self.target_slot_received = True
        if first:
            self.get_logger().info(
                f"[TARGET] target_slot 수신: "
                f"x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, "
                f"yaw={math.degrees(yaw):.1f}°"
            )

    # ══════════════════════════════════════════════════════
    # 메인 제어 루프
    # ══════════════════════════════════════════════════════

    def control_loop(self) -> None:
        self._log_state_once()

        if self.state == MissionState.ABORT:
            self._run_abort()
            self._maybe_log_status()
            return

        if not self._inputs_ready():
            self._handle_waiting()
            return

        # E-STOP: FORWARD_PARK 단계에서만 활성화
        # FORWARD_TURN 은 LIMO3/4 옆을 호 궤적으로 통과하는 단계이므로 제외
        if self.state == MissionState.FORWARD_PARK:
            if self._handle_estop():
                self._maybe_log_status()
                return

        # ── 1단계 ──
        if self.state == MissionState.EVADE:
            if self._run_evade():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.WAIT_FOR_EXIT)

        elif self.state == MissionState.WAIT_FOR_EXIT:
            if self._run_wait_for_exit():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.RETURN_TO_START)

        elif self.state == MissionState.RETURN_TO_START:
            if self._run_return_to_start():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.SPOT_VALIDATION)

        elif self.state == MissionState.SPOT_VALIDATION:
            verdict = self._run_spot_validation()
            if verdict == "PASS":
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.FORWARD_TURN)
            elif verdict == "ABORT":
                self.publish_cmd(0.0, 0.0)
                self._trigger_abort("슬롯 유효성 검사 실패")

        # ── 2단계 ──
        elif self.state == MissionState.FORWARD_TURN:
            if self._run_forward_turn():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.REVERSE_ALIGN)

        elif self.state == MissionState.REVERSE_ALIGN:
            if self._run_reverse_align():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.FORWARD_PARK)

        elif self.state == MissionState.FORWARD_PARK:
            if self._run_forward_park():
                self.publish_cmd(0.0, 0.0)
                self._transition(MissionState.FINISH)

        elif self.state == MissionState.FINISH:
            self.publish_cmd(0.0, 0.0)

        self._maybe_log_status()

    # ══════════════════════════════════════════════════════
    # [1단계] EVADE — 순수 후진, angular=0
    # ══════════════════════════════════════════════════════

    def _run_evade(self) -> bool:
        if self.current_position is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.evade_start_pos is None:
            self.evade_start_pos = self.current_position
            self.evade_target = (
                self.current_position[0] + EVADE_REVERSE_DISTANCE_M,
                self.current_position[1],
            )
            self.get_logger().info(
                f"[EVADE] 후진 시작: 목표 x={self.evade_target[0]:.3f}"
            )

        dist = abs(self.evade_target[0] - self.current_position[0])

        if dist <= EVADE_TOLERANCE_M:
            self.get_logger().info(f"[EVADE] 완료: x={self.current_position[0]:.3f}")
            return True

        speed = clamp(dist * 0.5, 0.015, EVADE_SPEED_MPS)
        self.publish_cmd(-speed, 0.0)
        return False

    # ══════════════════════════════════════════════════════
    # [1단계] WAIT_FOR_EXIT
    # ══════════════════════════════════════════════════════

    def _run_wait_for_exit(self) -> bool:
        self._maybe_log_wait_debug()
        self.publish_cmd(0.0, 0.0)
        return self._target_slot_is_fresh()

    # ══════════════════════════════════════════════════════
    # [1단계] RETURN_TO_START — 순수 전진, angular=0
    # ══════════════════════════════════════════════════════

    def _run_return_to_start(self) -> bool:
        if self.current_position is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.return_to_start_target is None:
            self.return_to_start_target = (START_X_M, START_Y_M)
            self.get_logger().info(f"[RETURN_TO_START] 복귀 목표: x={START_X_M:.4f}")

        dist = abs(self.return_to_start_target[0] - self.current_position[0])

        if dist <= RETURN_TO_START_TOLERANCE_M:
            self.get_logger().info(
                f"[RETURN_TO_START] 완료: "
                f"pose=({self.current_position[0]:.3f}, {self.current_position[1]:.3f})"
            )
            return True

        speed = clamp(dist * 0.5, 0.015, RETURN_TO_START_SPEED_MPS)
        self.publish_cmd(speed, 0.0)
        return False

    # ══════════════════════════════════════════════════════
    # [1단계] SPOT_VALIDATION — target_slot + 측면 라이다
    # ══════════════════════════════════════════════════════

    def _run_spot_validation(self) -> str:
        """
        두 조건 모두 만족 시 PASS:
          1) target_slot 수신 (dynamic_tracker 이탈 감지)
          2) 측면 라이다 (body +90°) 거리 >= SPOT_SIDE_CLEAR_M
        """
        self.publish_cmd(0.0, 0.0)

        if not self.target_slot_received or self.target_slot_position is None:
            return "WAIT"

        side_m = self._measure_side_clearance()
        self.last_spot_lidar_clear_m = side_m

        if side_m < SPOT_SIDE_CLEAR_M:
            self.spot_pass_since = None
            self.get_logger().info(
                f"[SPOT_VALIDATION] 대기: side={side_m:.3f} m "
                f"(필요 >= {SPOT_SIDE_CLEAR_M:.2f} m)"
            )
            return "WAIT"

        self.spot_pass_since, held = self._update_hold_gate(
            True, self.spot_pass_since, SPOT_VALIDATION_PASS_HOLD_S
        )
        if held:
            tx, ty = self.target_slot_position
            self.get_logger().info(
                f"[SPOT_VALIDATION] PASS: slot=({tx:.3f}, {ty:.3f}), "
                f"yaw={math.degrees(self.target_slot_yaw):.1f}°, "
                f"side={side_m:.3f} m"
            )
            return "PASS"
        return "WAIT"

    def _measure_side_clearance(self) -> float:
        """body 기준 왼쪽(+90°) 측면 라이다 중앙값 반환."""
        if self.latest_scan is None:
            return math.inf
        ranges = self._sector_ranges(
            self.latest_scan,
            SPOT_SIDE_SCAN_CENTER_DEG - SPOT_SIDE_SCAN_HALF_DEG,
            SPOT_SIDE_SCAN_CENTER_DEG + SPOT_SIDE_SCAN_HALF_DEG,
        )
        if len(ranges) < SPOT_SIDE_MIN_RAYS:
            return math.inf
        ranges.sort()
        return ranges[len(ranges) // 2]

    # ══════════════════════════════════════════════════════
    # [2단계] FORWARD_TURN — 원본 로직 유지
    # ══════════════════════════════════════════════════════

    def _run_forward_turn(self) -> bool:
        """
        전진(speed>0) + 최대 조향으로 호 궤적을 그리며 align_point 에 도달.

        align_point = (entry_x + 0.25, entry_y) = 약 (0.294, -0.752)
        LIMO 현재 위치(0.022, -0.103) 기준으로 후방(+x)에 있어서
        전진하면서 오른쪽으로 크게 꺾어 호를 그리며 슬롯 측면에 도달.
        """
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False
        if self.target_slot_position is None or self.target_slot_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.ft_align_point is None:
            sx, sy = self.target_slot_position
            syaw   = self.target_slot_yaw
            _, align = compute_slot_points(sx, sy, syaw)
            self.ft_align_point = align
            self.get_logger().info(
                f"[FORWARD_TURN] 시작: "
                f"pose=({self.current_position[0]:.3f}, {self.current_position[1]:.3f}, "
                f"{math.degrees(self.current_yaw):.1f}°), "
                f"align_point=({align[0]:.3f}, {align[1]:.3f})"
            )

        ax, ay = self.ft_align_point
        dx = ax - self.current_position[0]
        dy = ay - self.current_position[1]
        dist = math.hypot(dx, dy)

        if dist <= FORWARD_TURN_XY_TOL_M:
            self.get_logger().info(
                f"[FORWARD_TURN] 완료: "
                f"pose=({self.current_position[0]:.3f}, {self.current_position[1]:.3f}), "
                f"yaw={math.degrees(self.current_yaw):.1f}°"
            )
            return True

        desired_yaw = math.atan2(dy, dx)
        yaw_err = shortest_angular_distance(desired_yaw, self.current_yaw)
        speed   = clamp(dist * 0.5, 0.015, FORWARD_TURN_SPEED_MPS)
        angular = clamp(
            yaw_err * FORWARD_TURN_KP_YAW,
            -FORWARD_TURN_MAX_ANGULAR,
            FORWARD_TURN_MAX_ANGULAR,
        )
        self.publish_cmd(speed, angular, angular_limit=FORWARD_TURN_MAX_ANGULAR)
        return False

    # ══════════════════════════════════════════════════════
    # [2단계] REVERSE_ALIGN — 원본 로직 유지
    # ══════════════════════════════════════════════════════

    def _run_reverse_align(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False
        if self.target_slot_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        if self.ra_start_pos is None:
            self.ra_start_pos = self.current_position
            self.get_logger().info(
                f"[REVERSE_ALIGN] 시작: "
                f"pose=({self.current_position[0]:.3f}, {self.current_position[1]:.3f}), "
                f"목표 yaw={math.degrees(self.target_slot_yaw):.1f}°"
            )

        slot_yaw = self.target_slot_yaw
        yaw_err  = shortest_angular_distance(slot_yaw, self.current_yaw)
        aligned  = abs(yaw_err) <= REVERSE_ALIGN_YAW_TOL

        reversed_dist = math.hypot(
            self.current_position[0] - self.ra_start_pos[0],
            self.current_position[1] - self.ra_start_pos[1],
        )

        if reversed_dist >= REVERSE_ALIGN_DISTANCE_M:
            self.get_logger().info(
                f"[REVERSE_ALIGN] 최대 후진 도달 ({reversed_dist:.3f} m) — 완료"
            )
            return True

        now = self.get_clock().now()

        if aligned:
            if self.ra_stable_since is None:
                self.ra_stable_since = now
                self.get_logger().info(
                    f"[REVERSE_ALIGN] 정렬 도달: yaw_err={math.degrees(yaw_err):+.2f}°, "
                    f"hold {REVERSE_ALIGN_HOLD_S:.1f}s 대기..."
                )
            elapsed = (now - self.ra_stable_since).nanoseconds / 1e9
            self.publish_cmd(0.0, 0.0)
            if elapsed >= REVERSE_ALIGN_HOLD_S:
                self.get_logger().info(
                    f"[REVERSE_ALIGN] 완료: yaw={math.degrees(self.current_yaw):.1f}°, "
                    f"err={math.degrees(yaw_err):+.2f}°"
                )
                return True
            return False

        self.ra_stable_since = None

        angular = clamp(
            -yaw_err * REVERSE_ALIGN_KP_YAW,
            -REVERSE_ALIGN_MAX_ANGULAR,
            REVERSE_ALIGN_MAX_ANGULAR,
        )
        self.publish_cmd(
            -REVERSE_ALIGN_SPEED_MPS, angular,
            angular_limit=REVERSE_ALIGN_MAX_ANGULAR,
        )
        return False

    # ══════════════════════════════════════════════════════
    # [2단계] FORWARD_PARK — 원본 로직 유지
    # ══════════════════════════════════════════════════════

    def _run_forward_park(self) -> bool:
        if self.current_position is None or self.current_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False
        if self.target_slot_position is None or self.target_slot_yaw is None:
            self.publish_cmd(0.0, 0.0)
            return False

        tx, ty  = self.target_slot_position
        tyaw    = self.target_slot_yaw
        heading = (math.cos(tyaw), math.sin(tyaw))
        left_n  = (-math.sin(tyaw), math.cos(tyaw))

        to_slot   = (tx - self.current_position[0], ty - self.current_position[1])
        remaining = max(0.0, to_slot[0]*heading[0] + to_slot[1]*heading[1])
        x_err     = ((self.current_position[0]-tx)*left_n[0]
                     + (self.current_position[1]-ty)*left_n[1])
        yaw_err   = shortest_angular_distance(tyaw, self.current_yaw)

        if remaining > FORWARD_PARK_FINE_M:
            yaw_off = clamp(x_err * FORWARD_PARK_KP_LATERAL,
                            -FORWARD_PARK_LAT_CLAMP, FORWARD_PARK_LAT_CLAMP)
            yaw_e   = shortest_angular_distance(tyaw - yaw_off, self.current_yaw)
            linear  = clamp(remaining * 0.4, 0.015, FORWARD_PARK_SPEED_MPS)
            angular = clamp(yaw_e * FORWARD_PARK_KP_YAW,
                            -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS)
            self.publish_cmd(linear, angular)
            self.fp_finish_since = None
            return False

        yaw_ok = abs(yaw_err) <= FORWARD_PARK_YAW_PREC
        lat_ok = abs(x_err)   <= FORWARD_PARK_X_PREC_M
        done   = remaining    <= 0.005

        if yaw_ok and lat_ok and done:
            now = self.get_clock().now()
            if self.fp_finish_since is None:
                self.fp_finish_since = now
            elapsed = (now - self.fp_finish_since).nanoseconds / 1e9
            self.publish_cmd(0.0, 0.0)
            if elapsed >= FORWARD_PARK_FINISH_HOLD:
                self.get_logger().info(
                    f"[FORWARD_PARK] 주차 완료: "
                    f"x_err={x_err*1000:+.1f}mm, "
                    f"yaw_err={math.degrees(yaw_err):+.2f}°"
                )
                return True
            return False

        self.fp_finish_since = None

        if abs(yaw_err) > math.radians(2.0):
            angular = clamp(yaw_err * FORWARD_PARK_KP_YAW,
                            -MAX_ANGULAR_SPEED_RADPS*0.7,
                             MAX_ANGULAR_SPEED_RADPS*0.7)
            self.publish_cmd(0.0, angular)
            return False

        yaw_off = clamp(x_err * FORWARD_PARK_KP_LATERAL * 0.8,
                        -FORWARD_PARK_LAT_CLAMP*0.5, FORWARD_PARK_LAT_CLAMP*0.5)
        yaw_e   = shortest_angular_distance(tyaw - yaw_off, self.current_yaw)
        creep   = 0.0 if done else FORWARD_PARK_CREEP_MPS
        angular = clamp(yaw_e * FORWARD_PARK_KP_YAW,
                        -MAX_ANGULAR_SPEED_RADPS, MAX_ANGULAR_SPEED_RADPS)
        self.publish_cmd(creep, angular)
        return False

    # ══════════════════════════════════════════════════════
    # ABORT
    # ══════════════════════════════════════════════════════

    def _trigger_abort(self, reason: str) -> None:
        if self.state == MissionState.ABORT:
            return
        self.abort_reason = reason
        self.get_logger().error(f"[ABORT] 주차 불가 — {reason}")
        self._transition(MissionState.ABORT)

    def _run_abort(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if (self.last_abort_alert_time is None
                or now - self.last_abort_alert_time
                >= Duration(seconds=ABORT_ALERT_PERIOD_S)):
            self.get_logger().error(f"[ABORT] {self.abort_reason}")
            self.last_abort_alert_time = now

    # ══════════════════════════════════════════════════════
    # E-STOP
    # ══════════════════════════════════════════════════════

    def _handle_estop(self) -> bool:
        now = self.get_clock().now()
        if self.recovery_active:
            elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
            if elapsed < RECOVERY_SECONDS:
                self.publish_cmd(RECOVERY_SPEED_MPS, 0.0)
                return True
            if self.front_obstacle_distance_m <= ESTOP_DISTANCE_M:
                self.publish_cmd(0.0, 0.0)
                return True
            self.recovery_active     = False
            self.recovery_start_time = None
            self.publish_cmd(0.0, 0.0)
            return True

        if self.front_obstacle_distance_m <= ESTOP_DISTANCE_M:
            self.recovery_active     = True
            self.recovery_start_time = now
            self.publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f"[E-STOP] 전방 {self.front_obstacle_distance_m:.3f} m"
            )
            return True
        return False

    # ══════════════════════════════════════════════════════
    # publish_cmd — linear ramping 포함
    # ══════════════════════════════════════════════════════

    def publish_cmd(
        self,
        linear: float,
        angular: float,
        angular_limit: Optional[float] = None,
    ) -> None:
        max_ang = angular_limit if angular_limit is not None else MAX_ANGULAR_SPEED_RADPS
        linear  = clamp(linear,  -MAX_LINEAR_SPEED_MPS, MAX_LINEAR_SPEED_MPS)
        angular = clamp(angular, -max_ang, max_ang)

        # 선속도 급변 억제 (Gazebo ODE 크래시 방지)
        diff = linear - self._ramped_linear
        if abs(diff) <= LINEAR_RAMP_RATE:
            self._ramped_linear = linear
        else:
            self._ramped_linear += LINEAR_RAMP_RATE * (1.0 if diff > 0.0 else -1.0)

        cmd = Twist()
        cmd.linear.x  = self._ramped_linear
        cmd.angular.z = angular
        self.last_cmd_linear  = self._ramped_linear
        self.last_cmd_angular = angular
        self.cmd_pub.publish(cmd)

    # ══════════════════════════════════════════════════════
    # FSM 전환
    # ══════════════════════════════════════════════════════

    def _transition(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"[FSM] {self.state.value} → {new_state.value}")
        self.state = new_state
        self.state_logged.discard(new_state)

        if new_state == MissionState.WAIT_FOR_EXIT:
            self.last_wait_for_exit_debug_log_time = None
            self.wait_for_exit_enter_time = self.get_clock().now()
            self.target_slot_received = False
            self.target_received_time = None
        elif new_state == MissionState.RETURN_TO_START:
            self.return_to_start_target = None
        elif new_state == MissionState.SPOT_VALIDATION:
            self.spot_pass_since  = None
            self.spot_block_since = None
        elif new_state == MissionState.FORWARD_TURN:
            self.ft_align_point = None
        elif new_state == MissionState.REVERSE_ALIGN:
            self.ra_start_pos    = None
            self.ra_stable_since = None
        elif new_state == MissionState.FORWARD_PARK:
            self.fp_finish_since = None
        elif new_state == MissionState.FINISH:
            self.get_logger().info("✅ 주차 완료 (FINISH)")
        elif new_state == MissionState.ABORT:
            self.last_abort_alert_time = None

    # ══════════════════════════════════════════════════════
    # 헬퍼
    # ══════════════════════════════════════════════════════

    def _inputs_ready(self) -> bool:
        return (self.odom_received and self.scan_received
                and self.image_received and self.depth_received)

    def _handle_waiting(self) -> None:
        self.publish_cmd(0.0, 0.0)
        now = self.get_clock().now()
        if now - self.last_idle_warn_time < Duration(seconds=IDLE_WARN_PERIOD_S):
            return
        missing = []
        if not self.odom_received:  missing.append("odom")
        if not self.scan_received:  missing.append("scan")
        if not self.image_received: missing.append("rgb/image_raw")
        if not self.depth_received: missing.append("depth/image_raw")
        self.get_logger().warn("입력 대기: " + ", ".join(missing))
        self.last_idle_warn_time = now

    def _target_slot_is_fresh(self) -> bool:
        if not self.target_slot_received or self.target_received_time is None:
            return False
        if (self.wait_for_exit_enter_time is not None
                and self.target_received_time < self.wait_for_exit_enter_time):
            return False
        return (self.get_clock().now() - self.target_received_time
                <= Duration(seconds=TARGET_SLOT_FRESHNESS_S))

    def _target_slot_age_s(self) -> float:
        if self.target_received_time is None:
            return math.inf
        return (self.get_clock().now() - self.target_received_time).nanoseconds / 1e9

    def _update_hold_gate(self, raw_pass: bool, since_time, hold_seconds: float):
        now = self.get_clock().now()
        if not raw_pass:
            return None, False
        if since_time is None:
            since_time = now
        held = now - since_time >= Duration(seconds=hold_seconds)
        return since_time, held

    def _sector_ranges(
        self, msg: LaserScan, start_deg: float, end_deg: float
    ) -> List[float]:
        out = []
        for i, d in enumerate(msg.ranges):
            if math.isnan(d):
                continue
            if math.isinf(d):
                d = msg.range_max
            if d < msg.range_min:
                continue
            deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            if start_deg <= deg <= end_deg:
                out.append(min(d, msg.range_max))
        return out

    def _quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

    def _decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None
        channels = max(1, msg.step // msg.width)
        frame = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            frame = frame.reshape((msg.height, msg.width, channels))
        except ValueError:
            return None
        enc = msg.encoding.lower()
        if enc.startswith("rgb"):
            return cv2.cvtColor(frame[:, :, :3], cv2.COLOR_RGB2BGR)
        if enc.startswith("bgr"):
            return frame[:, :, :3].copy()
        if enc.startswith("mono") or channels == 1:
            return cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
        return frame[:, :, :3].copy()

    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        if msg.height == 0 or msg.width == 0 or not msg.data:
            return None
        enc = msg.encoding.lower()
        try:
            if enc in ("32fc1", "32fc"):
                buf = np.frombuffer(msg.data, dtype=np.float32)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32)
            elif enc in ("16uc1", "mono16"):
                buf = np.frombuffer(msg.data, dtype=np.uint16)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
            else:
                buf = np.frombuffer(msg.data, dtype=np.float32)
                depth = buf.reshape((msg.height, msg.width)).astype(np.float32)
        except (ValueError, TypeError):
            return None
        depth = np.nan_to_num(depth,
                              nan=DEPTH_INVALID_REPLACEMENT_M,
                              posinf=DEPTH_INVALID_REPLACEMENT_M,
                              neginf=DEPTH_INVALID_REPLACEMENT_M)
        depth = np.where(depth > 0.05, depth, DEPTH_INVALID_REPLACEMENT_M)
        return np.minimum(depth, DEPTH_INVALID_REPLACEMENT_M).astype(np.float32)

    def _log_state_once(self) -> None:
        if self.state not in self.state_logged:
            self.get_logger().info(f"[STATE] {self.state.value}")
            self.state_logged.add(self.state)

    def _maybe_log_wait_debug(self) -> None:
        now = self.get_clock().now()
        if (self.last_wait_for_exit_debug_log_time is not None
                and now - self.last_wait_for_exit_debug_log_time
                < Duration(seconds=WAIT_FOR_EXIT_DEBUG_LOG_PERIOD_S)):
            return
        self.get_logger().info(
            f"[WAIT_FOR_EXIT] received={self.target_slot_received}, "
            f"fresh={self._target_slot_is_fresh()}, "
            f"age={self._target_slot_age_s():.2f}s"
        )
        self.last_wait_for_exit_debug_log_time = now

    def _maybe_log_status(self) -> None:
        now = self.get_clock().now()
        if now - self.last_status_log_time < Duration(seconds=STATUS_LOG_PERIOD_S):
            return
        x   = self.current_position[0] if self.current_position else math.nan
        y   = self.current_position[1] if self.current_position else math.nan
        yaw = math.degrees(self.current_yaw) if self.current_yaw is not None else math.nan
        tx  = self.target_slot_position[0] if self.target_slot_position else math.nan
        ty  = self.target_slot_position[1] if self.target_slot_position else math.nan
        tyaw = math.degrees(self.target_slot_yaw) if self.target_slot_yaw else math.nan
        self.get_logger().info(
            f"State={self.state.value} "
            f"pose=({x:.3f}, {y:.3f}, {yaw:.1f}°) "
            f"cmd=({self.last_cmd_linear:.3f}, {self.last_cmd_angular:.3f}) "
            f"front={self.front_obstacle_distance_m:.3f} "
            f"target=({tx:.3f}, {ty:.3f}, {tyaw:.1f}°) "
            f"side={self.last_spot_lidar_clear_m:.3f} "
            f"fresh={self._target_slot_is_fresh()}"
        )
        self.last_status_log_time = now

    def stop(self) -> None:
        self.publish_cmd(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimoValetParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
