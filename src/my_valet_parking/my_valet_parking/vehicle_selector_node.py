#!/usr/bin/env python3

import curses
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

FORWARD_SPEED_MPS = 0.20
BACKWARD_SPEED_MPS = -0.20
TURN_ANGULAR_RADPS = 1.20
TURN_ASSIST_LINEAR_MPS = 0.10
PUBLISH_RATE_HZ = 10.0

BOUNDARY_X_M = 3.25
BOUNDARY_Y_M = 1.75
LINEAR_RAMP_RATE = 0.05
ANGULAR_RAMP_RATE = 0.08

KEY_BINDINGS = {
    curses.KEY_UP:    (FORWARD_SPEED_MPS,        0.0),
    curses.KEY_DOWN:  (BACKWARD_SPEED_MPS,        0.0),
    curses.KEY_LEFT:  (TURN_ASSIST_LINEAR_MPS,  TURN_ANGULAR_RADPS),
    curses.KEY_RIGHT: (TURN_ASSIST_LINEAR_MPS, -TURN_ANGULAR_RADPS),
    ord(' '):         (0.0,                        0.0),
}

VEHICLES = (2, 3, 4)


class VehicleSelectorNode(Node):
    def __init__(self):
        super().__init__("vehicle_selector_node")

        self._pubs = {
            n: self.create_publisher(Twist, f"/limo{n}/cmd_vel", 10)
            for n in VEHICLES
        }
        self._positions = {n: None for n in VEHICLES}
        self._yaws = {n: 0.0 for n in VEHICLES}

        for n in VEHICLES:
            self.create_subscription(
                Odometry,
                f"/limo{n}/odom",
                lambda msg, v=n: self._odom_cb(msg, v),
                10,
            )

        self._active = 2
        self._linear = 0.0
        self._angular = 0.0
        self._target_linear = 0.0
        self._target_angular = 0.0
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._timer_publish)

    def _odom_cb(self, msg: Odometry, vehicle: int) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        self._positions[vehicle] = (p.x, p.y)
        self._yaws[vehicle] = math.atan2(siny, cosy)

    def _timer_publish(self) -> None:
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
        self._publish_to(self._active, self._linear, self._angular)

    def _should_block_linear(self, vehicle: int, linear: float) -> bool:
        if linear == 0.0:
            return False
        pos = self._positions[vehicle]
        if pos is None:
            return False
        x, y = pos
        yaw = self._yaws[vehicle]
        new_x = x + linear * math.cos(yaw) * 0.15
        new_y = y + linear * math.sin(yaw) * 0.15
        outside = abs(new_x) > BOUNDARY_X_M or abs(new_y) > BOUNDARY_Y_M
        if outside:
            self.get_logger().warn(
                f"[BOUNDARY] limo{vehicle} 이동 차단: 현재=({x:.2f},{y:.2f}) "
                f"예상=({new_x:.2f},{new_y:.2f}) 경계=±{BOUNDARY_X_M}m/±{BOUNDARY_Y_M}m"
            )
        return outside

    def _publish_to(self, vehicle: int, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self._pubs[vehicle].publish(msg)

    def switch_vehicle(self, new_vehicle: int) -> None:
        if new_vehicle == self._active:
            return
        self._publish_to(self._active, 0.0, 0.0)
        self._active = new_vehicle
        self._linear = 0.0
        self._angular = 0.0
        self._target_linear = 0.0
        self._target_angular = 0.0

    def send(self, linear: float, angular: float) -> None:
        if self._should_block_linear(self._active, linear):
            linear = 0.0
            angular = 0.0
        self._target_linear = float(linear)
        self._target_angular = float(angular)

    def stop_all(self) -> None:
        for n in VEHICLES:
            self._publish_to(n, 0.0, 0.0)


def spin_thread(node):
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSelectorNode()

    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    def run(stdscr):
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.clear()
        stdscr.addstr(0, 0, "=== LIMO Vehicle Selector ===")
        stdscr.addstr(1, 0, "Press 2/3/4 to select vehicle, then use arrow keys to drive.")
        stdscr.addstr(2, 0, "Space=stop, Q=quit")
        stdscr.addstr(3, 0, f"  forward: {FORWARD_SPEED_MPS:.2f} m/s  angular: {TURN_ANGULAR_RADPS:.2f} rad/s")
        stdscr.addstr(4, 0, "  UP: forward  DOWN: reverse  LEFT: arc-left  RIGHT: arc-right")
        stdscr.addstr(6, 0, f"Active vehicle: limo{node._active}  |  status: stopped   ")
        stdscr.refresh()

        while rclpy.ok():
            key = stdscr.getch()
            if key == ord('q'):
                node.stop_all()
                break
            if key in (ord('2'), ord('3'), ord('4')):
                new_v = int(chr(key))
                node.switch_vehicle(new_v)
                stdscr.addstr(6, 0, f"Active vehicle: limo{node._active}  |  status: stopped   ")
                stdscr.refresh()
            elif key in KEY_BINDINGS:
                linear, angular = KEY_BINDINGS[key]
                node.send(linear, angular)
                label = {
                    curses.KEY_UP:    "forward",
                    curses.KEY_DOWN:  "reverse",
                    curses.KEY_LEFT:  "arc-left",
                    curses.KEY_RIGHT: "arc-right",
                    ord(' '):         "stopped",
                }.get(key, "")
                stdscr.addstr(6, 0, f"Active vehicle: limo{node._active}  |  status: {label}   ")
                stdscr.refresh()
            time.sleep(0.05)

    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
