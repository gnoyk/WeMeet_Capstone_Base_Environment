#!/usr/bin/env python3
"""
limo_parking_node.py  —  Ackermann 앞바퀴 조향 기반 좌측 수직 주차
====================================================================

상황 가정:
  - 차량은 위쪽(+Y) 방향을 바라보고 있음
  - 빈 슬롯은 차량 왼쪽 90° 방향에 수직으로 존재
  - 슬롯 양쪽에 차량 존재 (좌: 차량, 우: 차량)
  - 앞바퀴 조향(Ackermann) 구조 → 제자리 회전 불가

기동 시퀀스:
  IDLE
    ↓ parking_ready=True
  PARK_STAGE_1   전진 + 최대 좌조향 → yaw 가 target 의 45° 이내로 접근
    ↓
  PARK_STAGE_2   후진 + 최대 우조향 → 0.8s 안정 후 지정 거리 후진
    ↓
  FORWARD_PARK   직진 진입
                 ├─ Yaw 오차 보정 (target_yaw 기준)
                 ├─ 우측 차량 LiDAR 선형 피팅 → 평행각 오차 보정  ← NEW
                 ├─ 좌우 간격 센터링 보조
                 └─ 전방 거리 감속·정지
    ↓
  FINISH
"""

import math
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data


# ─────────────────────────────────────────────────────────
#  파라미터
# ─────────────────────────────────────────────────────────

CONTROL_PERIOD_S     = 0.05        # 20 Hz

# 속도 한계
MAX_LINEAR_MPS       = 0.06
MAX_ANGULAR_RADPS    = 0.35
LINEAR_RAMP          = 0.008
ANGULAR_RAMP         = 0.07

# PARK_STAGE_1 : 전진 + 좌조향
STAGE1_LINEAR        = 0.035
STAGE1_STEER         = 0.32
STAGE1_EXIT_ERR_RAD  = math.radians(45)
STAGE1_SAFE_RIGHT_M  = 0.25
STAGE1_SAFE_FRONT_M  = 0.35

# PARK_STAGE_2 : 후진 + 우조향
STAGE2_LINEAR        = -0.030
STAGE2_STEER         = 0.32
STAGE2_REVERSE_DIST  = 0.4

# FORWARD_PARK : 직진 진입
PARK_LINEAR_MPS      = 0.030
PARK_STOP_FRONT_M    = 0.25
PARK_DECEL_DIST      = 0.15

# Yaw 보정 게인
PARK_KP_YAW          = 1.0

# 센터링 게인 및 조건
PARK_KP_SIDE         = 0.7
PARK_SIDE_WINDOW     = 15.0        # 좌우 센서 탐색 범위 (±deg)
PARK_MAX_WALL_DIST   = 1.0
SIDE_MAX_CORR        = 0.25

# ── 평행 정렬 (LiDAR 선형 피팅) ──────────────────────────
# 우측 차량 표면을 스캔할 각도 범위 (차체 기준, 도)
ALIGN_RIGHT_START    = -120.0      # 우측 후방에서
ALIGN_RIGHT_END      =  -60.0      # 우측 전방까지
ALIGN_LEFT_START     =   60.0      # 좌측 후방에서
ALIGN_LEFT_END       =  120.0      # 좌측 전방까지

ALIGN_MIN_POINTS     = 5           # 피팅에 필요한 최소 포인트 수
ALIGN_MAX_DIST       = 1.2         # 이 거리 이내 포인트만 사용 (m)
ALIGN_KP             = 0.6         # 평행 오차 → 조향 보정 게인
ALIGN_MAX_CORR       = 0.15        # 평행 보정 최대 조향량
ALIGN_ACTIVE_YAW_DEG = 12.0        # yaw 오차가 이 이하일 때만 평행 보정 개입


# ─────────────────────────────────────────────────────────
#  유틸
# ─────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def sector_median(scan: LaserScan, start_deg: float, end_deg: float):
    vals = []
    for i, d in enumerate(scan.ranges):
        if math.isnan(d) or math.isinf(d) or d <= 0.01:
            continue
        deg = math.degrees(scan.angle_min + i * scan.angle_increment)
        if start_deg <= deg <= end_deg:
            vals.append(d)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]


def sector_min(scan: LaserScan, start_deg: float, end_deg: float):
    vals = []
    for i, d in enumerate(scan.ranges):
        if math.isnan(d) or math.isinf(d) or d <= 0.01:
            continue
        deg = math.degrees(scan.angle_min + i * scan.angle_increment)
        if start_deg <= deg <= end_deg:
            vals.append(d)
    return min(vals) if vals else None


def fit_wall_angle(scan: LaserScan,
                   start_deg: float, end_deg: float,
                   max_dist: float) -> float | None:
    """
    지정 각도 구간의 LiDAR 점들을 극좌표 → XY 변환 후
    최소자승 선형 피팅(y = ax + b)으로 벽면 기울기 각도(rad)를 반환한다.

    반환값 해석:
      - 양수: 앞쪽이 벽에 가까워지고 있음 → 반시계 보정 필요
      - 음수: 뒤쪽이 벽에 가까워지고 있음 → 시계 보정 필요
      - None : 포인트 부족으로 피팅 불가
    """
    xs, ys = [], []
    for i, d in enumerate(scan.ranges):
        if math.isnan(d) or math.isinf(d) or d <= 0.01 or d > max_dist:
            continue
        angle_rad = scan.angle_min + i * scan.angle_increment
        deg = math.degrees(angle_rad)
        if start_deg <= deg <= end_deg:
            xs.append(d * math.cos(angle_rad))
            ys.append(d * math.sin(angle_rad))

    n = len(xs)
    if n < ALIGN_MIN_POINTS:
        return None

    # 최소자승: slope = (n·Σxy - Σx·Σy) / (n·Σx² - (Σx)²)
    sum_x  = sum(xs)
    sum_y  = sum(ys)
    sum_xx = sum(x * x for x in xs)
    sum_xy = sum(x * y for x, y in zip(xs, ys))

    denom = n * sum_xx - sum_x * sum_x
    if abs(denom) < 1e-9:
        return None

    slope = (n * sum_xy - sum_x * sum_y) / denom
    # arctan(slope) = 벽면이 센서 좌표계에서 이루는 각도
    # 이상적 정렬 시 우측 벽은 Y축에 평행(slope≈0)이어야 함
    return math.atan(slope)


# ─────────────────────────────────────────────────────────
#  FSM
# ─────────────────────────────────────────────────────────

class State(Enum):
    IDLE          = "IDLE"
    PARK_STAGE_1  = "PARK_STAGE_1"
    PARK_STAGE_2  = "PARK_STAGE_2"
    FORWARD_PARK  = "FORWARD_PARK"
    FINISH        = "FINISH"


# ─────────────────────────────────────────────────────────
#  노드
# ─────────────────────────────────────────────────────────

class LimoParkingNode(Node):

    def __init__(self):
        super().__init__("limo_parking_node")

        scan_qos = QoSProfile(depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.create_subscription(Odometry, "odom",
                                 self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan",
                                 self._scan_cb, scan_qos)
        self.create_subscription(Bool, "parking_ready",
                                 self._ready_cb, 10)

        self.create_timer(CONTROL_PERIOD_S, self._loop)

        # 센서
        self.pos      = None
        self.yaw      = None
        self.yaw_rate = 0.0
        self.scan     = None

        # 주차 파라미터
        self.target_yaw  = None
        self.park_start  = None
        self.park_dist   = 0.65

        # FSM
        self.state           = State.IDLE
        self.parking_ready   = False
        self._pending_start  = False
        self._stable_since   = None
        self._s2_phase       = "STEER"
        self._s2_start       = None
        self._s1_steer       = STAGE1_STEER

        # 속도 램프
        self._cur_lin = 0.0
        self._cur_ang = 0.0

        # 평행 보정 EMA 필터 (노이즈 제거)
        self._wall_angle_ema  = None
        self._EMA_ALPHA       = 0.25      # 낮을수록 부드러움

        self.get_logger().info("LimoParkingNode 준비 — /parking_ready 대기 중")

    # ─────────────────────────────────────────
    #  콜백
    # ─────────────────────────────────────────

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.pos = (p.x, p.y)
        new_yaw = math.atan2(2.0 * (o.w * o.z),
                             1.0 - 2.0 * (o.z * o.z))
        if self.yaw is not None:
            self.yaw_rate = norm_angle(new_yaw - self.yaw) / CONTROL_PERIOD_S
        else:
            self.yaw_rate = 0.0
        self.yaw = new_yaw

    def _scan_cb(self, msg):
        self.scan = msg

    def _ready_cb(self, msg):
        if msg.data and self.state == State.IDLE:
            self._pending_start = True
            self.get_logger().info("parking_ready 수신 — odom 대기 중")

    # ─────────────────────────────────────────
    #  시퀀스 시작
    # ─────────────────────────────────────────

    def _start_sequence(self):
        if self.yaw is None:
            self._pending_start = True
            return

        self.target_yaw = norm_angle(self.yaw + math.pi / 2.0)
        self.parking_ready = True
        self._wall_angle_ema = None   # EMA 초기화

        self.get_logger().info(
            f"주차 시작 | 현재 yaw={math.degrees(self.yaw):.1f}° "
            f"→ 목표 yaw={math.degrees(self.target_yaw):.1f}°"
        )
        self._transition(State.PARK_STAGE_1)

    # ─────────────────────────────────────────
    #  메인 루프
    # ─────────────────────────────────────────

    def _loop(self):
        if self._pending_start and self.yaw is not None:
            self._pending_start = False
            self._start_sequence()
            return

        if not self.parking_ready:
            self._pub(0.0, 0.0)
            return
        if self.pos is None or self.yaw is None:
            return

        if   self.state == State.PARK_STAGE_1: self._run_stage1()
        elif self.state == State.PARK_STAGE_2: self._run_stage2()
        elif self.state == State.FORWARD_PARK: self._run_forward_park()
        elif self.state == State.FINISH:       self._pub(0.0, 0.0)

    # ─────────────────────────────────────────
    #  STAGE 1 : 전진 + 최대 좌조향
    # ─────────────────────────────────────────

    def _run_stage1(self):
        err = norm_angle(self.target_yaw - self.yaw)

        right_dist  = sector_min(self.scan, -100.0, -80.0) if self.scan else None
        right_close = right_dist is not None and right_dist < STAGE1_SAFE_RIGHT_M

        front_dist  = sector_min(self.scan, -10.0, 45.0) if self.scan else None
        front_close = front_dist is not None and front_dist < STAGE1_SAFE_FRONT_M

        if right_close:
            self.get_logger().info(
                f"STAGE1 우측 근접 차단 | right={right_dist:.3f}m")
        if front_close:
            self.get_logger().info(
                f"STAGE1 전방 근접 차단 | front={front_dist:.3f}m")

        if abs(err) > STAGE1_EXIT_ERR_RAD and not right_close and not front_close:
            self._pub(STAGE1_LINEAR, self._s1_steer)
        else:
            self._pub(0.0, 0.0)
            self._transition(State.PARK_STAGE_2)

    # ─────────────────────────────────────────
    #  STAGE 2 : 후진 + 최대 우조향
    # ─────────────────────────────────────────

    def _run_stage2(self):
        if self._s2_phase == "STEER":
            self._pub(0.0, STAGE2_STEER)

            if self._stable_since is None:
                self._stable_since = self.get_clock().now()

            elapsed = (self.get_clock().now() - self._stable_since).nanoseconds / 1e9
            rate_ok = abs(self.yaw_rate) < 0.05
            time_ok = elapsed >= 0.8

            if rate_ok and time_ok:
                self._stable_since = None
                self._s2_phase = "REVERSE"
                self.get_logger().info(
                    f"STAGE2 조향 완료 | elapsed={elapsed:.2f}s → 후진 시작")
            return

        if self._s2_start is None:
            self._s2_start = self.pos

        sx, sy   = self._s2_start
        cx, cy   = self.pos
        traveled = math.hypot(cx - sx, cy - sy)

        if traveled >= STAGE2_REVERSE_DIST:
            self._pub(0.0, 0.0)
            self._s2_phase = "STEER"
            self._s2_start = None
            self.get_logger().info(
                f"STAGE2 후진 완료 | {traveled:.3f}m → FORWARD_PARK 진입")
            self._measure_slot_and_enter()
        else:
            self._pub(STAGE2_LINEAR, STAGE2_STEER)

    # ─────────────────────────────────────────
    #  FORWARD_PARK 진입 준비
    # ─────────────────────────────────────────

    def _measure_slot_and_enter(self):
        self.park_start      = self.pos
        self._wall_angle_ema = None   # EMA 리셋 (슬롯 진입 시 새로 시작)
        self.get_logger().info(
            f"FORWARD_PARK 진입 (최대 허용 거리: {self.park_dist}m)")
        self._transition(State.FORWARD_PARK)

    # ─────────────────────────────────────────
    #  FORWARD_PARK : 직진 진입 + 평행 정렬
    # ─────────────────────────────────────────

    def _run_forward_park(self):
        # 0. 이동 거리
        sx, sy   = self.park_start
        cx, cy   = self.pos
        traveled = math.hypot(cx - sx, cy - sy)

        # 1. Yaw 오차 (기본 각도 보정)
        yaw_err     = norm_angle(self.target_yaw - self.yaw)
        yaw_err_deg = math.degrees(yaw_err)
        if abs(yaw_err_deg) < 4.0:
            yaw_err     = 0.0
            yaw_err_deg = 0.0

        # 2. 평행 정렬: 우측 차량 LiDAR 선형 피팅 ─────────────────
        #    차체가 어느 정도 정렬됐을 때만(|yaw_err| < ALIGN_ACTIVE_YAW_DEG) 개입
        align_corr = 0.0
        if self.scan and abs(yaw_err_deg) < ALIGN_ACTIVE_YAW_DEG:
            # 우측 차량 피팅
            raw_right = fit_wall_angle(
                self.scan,
                ALIGN_RIGHT_START, ALIGN_RIGHT_END,
                ALIGN_MAX_DIST
            )
            # 좌측 차량 피팅 (우측 불가 시 보조)
            raw_left  = fit_wall_angle(
                self.scan,
                ALIGN_LEFT_START, ALIGN_LEFT_END,
                ALIGN_MAX_DIST
            )

            # 우측과 좌측 피팅값 평균 (둘 다 있을 때 더 안정적)
            if raw_right is not None and raw_left is not None:
                # 좌측 slope 는 부호가 반대이므로 음수로 반전 후 평균
                raw_angle = (raw_right + (-raw_left)) / 2.0
            elif raw_right is not None:
                raw_angle = raw_right
            elif raw_left is not None:
                raw_angle = -raw_left
            else:
                raw_angle = None

            if raw_angle is not None:
                # EMA 필터로 노이즈 제거
                if self._wall_angle_ema is None:
                    self._wall_angle_ema = raw_angle
                else:
                    self._wall_angle_ema = (
                        self._EMA_ALPHA * raw_angle
                        + (1.0 - self._EMA_ALPHA) * self._wall_angle_ema
                    )

                # 조향 보정량: 벽면이 앞으로 기울어지면(양수) → 좌조향으로 보정
                align_corr = clamp(
                    -ALIGN_KP * self._wall_angle_ema,
                    -ALIGN_MAX_CORR, ALIGN_MAX_CORR
                )

                self.get_logger().debug(
                    f"ALIGN | raw={math.degrees(raw_angle):.2f}° "
                    f"ema={math.degrees(self._wall_angle_ema):.2f}° "
                    f"corr={align_corr:.3f}"
                )

        # 3. 좌우 간격 센터링 (보조, yaw 거의 0일 때만)
        left_center_deg  =  90.0 + yaw_err_deg
        right_center_deg = -90.0 + yaw_err_deg

        left_dist  = sector_median(self.scan,
                                   left_center_deg  - PARK_SIDE_WINDOW,
                                   left_center_deg  + PARK_SIDE_WINDOW) \
                     if self.scan else None
        right_dist = sector_median(self.scan,
                                   right_center_deg - PARK_SIDE_WINDOW,
                                   right_center_deg + PARK_SIDE_WINDOW) \
                     if self.scan else None

        side_corr = 0.0
        if abs(yaw_err_deg) < 5.0 and align_corr == 0.0:
            # 평행 보정이 없을 때만 센터링 개입 (중복 방지)
            if left_dist is not None and right_dist is not None:
                if left_dist < PARK_MAX_WALL_DIST and right_dist < PARK_MAX_WALL_DIST:
                    side_err  = left_dist - right_dist
                    side_corr = clamp(side_err * PARK_KP_SIDE,
                                      -SIDE_MAX_CORR, SIDE_MAX_CORR)

        # 4. 전방 거리 및 종료 판정
        front_dist = sector_min(self.scan, -10.0, 10.0) if self.scan else 1.0
        is_aligned = abs(yaw_err_deg) <= 10.0

        if (is_aligned and front_dist is not None and front_dist <= PARK_STOP_FRONT_M) \
                or (traveled >= self.park_dist):
            self._pub(0.0, 0.0)
            self.get_logger().info(
                f"주차 완료! | 전방={front_dist:.3f}m  "
                f"이동={traveled:.3f}m  "
                f"yaw_err={yaw_err_deg:.1f}°  "
                f"align_ema={math.degrees(self._wall_angle_ema):.2f}° "
                    if self._wall_angle_ema is not None else
                f"주차 완료! | 전방={front_dist:.3f}m  이동={traveled:.3f}m"
            )
            self._transition(State.FINISH)
            return

        # 5. 감속 프로파일
        remaining = self.park_dist - traveled
        speed = PARK_LINEAR_MPS
        if is_aligned and front_dist is not None and front_dist < PARK_DECEL_DIST:
            speed = max(0.015, PARK_LINEAR_MPS * (front_dist / PARK_DECEL_DIST))
        elif remaining < PARK_DECEL_DIST:
            speed = max(0.015, PARK_LINEAR_MPS * (remaining / PARK_DECEL_DIST))

        # 6. 최종 조향 합산
        #    우선순위: Yaw 보정 > 평행 보정 > 센터링
        steer_cmd = PARK_KP_YAW * yaw_err + align_corr + side_corr

        self._pub(speed, steer_cmd)

        self.get_logger().debug(
            f"FWD_PARK | traveled={traveled:.3f}m speed={speed:.3f} "
            f"steer={steer_cmd:.3f} "
            f"(yaw={PARK_KP_YAW*yaw_err:.3f} align={align_corr:.3f} side={side_corr:.3f})"
        )

    # ─────────────────────────────────────────
    #  속도 발행 (램프)
    # ─────────────────────────────────────────

    def _pub(self, linear: float, angular: float = 0.0):
        tgt_lin = clamp(linear,  -MAX_LINEAR_MPS,   MAX_LINEAR_MPS)
        tgt_ang = clamp(angular, -MAX_ANGULAR_RADPS, MAX_ANGULAR_RADPS)

        dl = tgt_lin - self._cur_lin
        self._cur_lin += clamp(dl, -LINEAR_RAMP, LINEAR_RAMP)

        da = tgt_ang - self._cur_ang
        self._cur_ang += clamp(da, -ANGULAR_RAMP, ANGULAR_RAMP)

        cmd = Twist()
        cmd.linear.x  = self._cur_lin
        cmd.angular.z = self._cur_ang
        self.cmd_pub.publish(cmd)

    # ─────────────────────────────────────────
    #  FSM 전환
    # ─────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(f"[FSM] {self.state.value} → {new_state.value}")
        self.state = new_state


# ─────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LimoParkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
