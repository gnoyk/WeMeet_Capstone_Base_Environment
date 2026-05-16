#!/usr/bin/env python3

import curses
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

FORWARD_SPEED_MPS = 0.20        # [TASK0] 조정 완료 — 변경하지 말 것
BACKWARD_SPEED_MPS = -0.20      # [TASK0] 조정 완료 — 변경하지 말 것
TURN_ANGULAR_RADPS = 1.20       # [TASK0-1] 0.80 → 1.20 rad/s (각속도 상향)
TURN_ASSIST_LINEAR_MPS = 0.10   # [TASK0] 조정 완료 — 변경하지 말 것
CMD_VEL_TOPIC = "/limo2/cmd_vel"
ODOM_TOPIC = "/limo2/odom"
PUBLISH_RATE_HZ = 10.0          # [TASK0-2] 연속 발행 주기 (모델 소멸 방지)

# [TASK0-3] 소프트 경계: 이 좌표 밖으로 나가는 선속도 명령을 차단
# 실제 바닥 ±3.75 m × ±2.25 m에서 0.5 m 여유 확보
BOUNDARY_X_M = 3.25
BOUNDARY_Y_M = 1.75
# 매 10 Hz tick에 최대 이 값만큼 선속도 변경 → 0.5 m/s² 실효 가속도 제한
LINEAR_RAMP_RATE = 0.05
ANGULAR_RAMP_RATE = 0.08

KEY_BINDINGS = {
    curses.KEY_UP:    (FORWARD_SPEED_MPS,         0.0),
    curses.KEY_DOWN:  (BACKWARD_SPEED_MPS,         0.0),
    curses.KEY_LEFT:  (TURN_ASSIST_LINEAR_MPS,  TURN_ANGULAR_RADPS),
    curses.KEY_RIGHT: (TURN_ASSIST_LINEAR_MPS, -TURN_ANGULAR_RADPS),
    ord(' '):         (0.0,                         0.0),
}


class Limo2Teleop(Node):
    def __init__(self):
        super().__init__("limo2_arrow_teleop")
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        # [TASK0-3] odom 구독: 위치/헤딩 추적으로 경계 이탈 방지
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)
        self._limo2_pos = None   # (x, y) world frame
        self._limo2_yaw = 0.0   # world frame yaw

        # 현재 출력 속도와 목표 속도를 분리 — 타이머가 ramping 후 단일 발행
        self._linear = 0.0
        self._angular = 0.0
        self._target_linear = 0.0
        self._target_angular = 0.0
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._timer_publish)

    # -------- callbacks --------

    def _odom_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self._limo2_pos = (p.x, p.y)
        self._limo2_yaw = math.atan2(siny, cosy)

    def _timer_publish(self) -> None:
        # 선속도만 ramping: 급격한 속도 전환이 ODE solver 불안정 유발하므로
        # 매 tick 최대 LINEAR_RAMP_RATE m/s씩 목표 속도에 수렴
        diff = self._target_linear - self._linear
        if abs(diff) <= LINEAR_RAMP_RATE:
            self._linear = self._target_linear
        else:
            self._linear += LINEAR_RAMP_RATE * (1.0 if diff > 0.0 else -1.0)
        diff_a = self._target_angular - self._angular
        if abs(diff_a) <= ANGULAR_RAMP_RATE:
            self._angular = self._target_angular
        else:
            self._angular += ANGULAR_RAMP_RATE * (1.0 if diff_a > 0.0 else -1.0)
        self._publish(self._linear, self._angular)

    # -------- boundary check --------

    def _should_block_linear(self, linear: float) -> bool:
        """[TASK0-3] 이동 방향 기준으로 소프트 경계 이탈 여부 판단."""
        if linear == 0.0:
            return False
        if self._limo2_pos is None:
            return False
        x, y = self._limo2_pos
        # 0.15 s 후 예상 위치
        new_x = x + linear * math.cos(self._limo2_yaw) * 0.15
        new_y = y + linear * math.sin(self._limo2_yaw) * 0.15
        outside = abs(new_x) > BOUNDARY_X_M or abs(new_y) > BOUNDARY_Y_M
        if outside:
            self.get_logger().warn(
                f"[BOUNDARY] 이동 차단: 현재=({x:.2f}, {y:.2f}) "
                f"예상=({new_x:.2f}, {new_y:.2f}) 경계=±{BOUNDARY_X_M}m/±{BOUNDARY_Y_M}m"
            )
        return outside

    # -------- publish --------

    def _publish(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.pub.publish(msg)

    def send(self, linear: float, angular: float) -> None:
        if self._should_block_linear(linear):
            linear = 0.0
            angular = 0.0
        # 목표 속도만 갱신 — 실제 발행은 타이머가 ramping 후 처리 (이중 발행 제거)
        self._target_linear = float(linear)
        self._target_angular = float(angular)

    def stop(self) -> None:
        self._target_linear = 0.0
        self._target_angular = 0.0


def spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = Limo2Teleop()

    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    def run(stdscr):
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "=== LIMO 2 Teleop ===")
        stdscr.addstr(1, 0, f"  직진: {FORWARD_SPEED_MPS:.2f} m/s  각속도: {TURN_ANGULAR_RADPS:.2f} rad/s")
        stdscr.addstr(2, 0, "  ↑ : 전진   ↓ : 후진")
        stdscr.addstr(3, 0, "  ← : 좌회전  → : 우회전")
        stdscr.addstr(4, 0, "  Space : 정지   q : 종료")
        stdscr.addstr(6, 0, "현재: 정지")
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()
            if key == ord('q'):
                node.stop()
                break
            if key in KEY_BINDINGS:
                linear, angular = KEY_BINDINGS[key]
                node.send(linear, angular)
                label = {
                    curses.KEY_UP:    "전진",
                    curses.KEY_DOWN:  "후진",
                    curses.KEY_LEFT:  "좌회전",
                    curses.KEY_RIGHT: "우회전",
                    ord(' '):         "정지",
                }.get(key, "")
                stdscr.addstr(6, 0, f"현재: {label}   ")
                stdscr.refresh()
            time.sleep(0.05)

    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
