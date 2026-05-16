#!/usr/bin/env python3
"""
[REQ-5] LIMO 위치 리셋 — spawn_entity.py 우회, rclpy로 서비스 직접 호출
Gazebo를 재시작하지 않고 limo1/limo2/limo3/limo4를 초기 위치에 재스폰한다.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from gazebo_msgs.srv import DeleteEntity, GetModelList, SpawnEntity
from std_msgs.msg import String
from std_srvs.srv import Empty

LIMO1 = dict(name='limo1', ns='limo1', x=0.0215,  y=-0.1030, z=0.1450, yaw=3.1362)
LIMO2 = dict(name='limo2', ns='limo2', x=0.0,     y=-0.75,   z=0.1450, yaw=1.5708)
LIMO3 = dict(name='limo3', ns='limo3', x=-0.340909, y=-0.75,   z=0.1450, yaw=1.5708)
LIMO4 = dict(name='limo4', ns='limo4', x= 0.340909, y=-0.75,   z=0.1450, yaw=1.5708)

DELETE_RETRIES = 3
DELETE_RETRY_INTERVAL_S = 1.0
DELETE_SPAWN_GAP_S = 1.5
INTER_SPAWN_DELAY_S = 1.5


class ResetNode(Node):
    def __init__(self):
        super().__init__('limo_reset_node')
        self.urdf = None
        # robot_state_publisher는 transient_local QoS로 퍼블리시 — 반드시 동일하게 맞춰야 수신됨
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, '/robot_description', self._urdf_cb, qos)

    def _urdf_cb(self, msg):
        self.urdf = msg.data

    def wait_urdf(self, timeout=10.0):
        self.get_logger().info('/robot_description 대기 중...')
        deadline = time.monotonic() + timeout
        while self.urdf is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.urdf is not None

    def pause_physics(self):
        """삭제/스폰 중 물리 엔진 간섭을 차단하기 위해 physics 일시정지."""
        cli = self.create_client(Empty, '/pause_physics')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/pause_physics 서비스 없음 — 일시정지 생략')
            return
        fut = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('물리 엔진 일시정지')

    def unpause_physics(self):
        """스폰 완료 후 물리 엔진 재개."""
        cli = self.create_client(Empty, '/unpause_physics')
        if not cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/unpause_physics 서비스 없음 — 재개 생략')
            return
        fut = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info('물리 엔진 재개')

    def entity_exists(self, name: str) -> bool:
        """/get_model_list로 entity 존재 여부 확인. 서비스 불가 시 True(보수적) 반환."""
        cli = self.create_client(GetModelList, '/get_model_list')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/get_model_list 서비스 없음 — 존재 확인 생략')
            return True
        fut = cli.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() and fut.result().success:
            exists = name in fut.result().model_names
            self.get_logger().info(
                f'[모델 확인] {name}: {"아직 존재" if exists else "삭제 확인"}'
            )
            return exists
        # 응답 없으면 보수적으로 존재한다고 가정
        return True

    def delete(self, name: str) -> bool:
        """entity 삭제. success=False 시 최대 DELETE_RETRIES회 재시도. bool 반환."""
        cli = self.create_client(DeleteEntity, '/delete_entity')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/delete_entity 서비스 없음')
            return False

        for attempt in range(1, DELETE_RETRIES + 1):
            req = DeleteEntity.Request()
            req.name = name
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)

            if not fut.result():
                self.get_logger().warn(
                    f'{name} 삭제 응답 없음 (시도 {attempt}/{DELETE_RETRIES})'
                )
            elif fut.result().success:
                self.get_logger().info(f'{name} 삭제 성공 (시도 {attempt})')
                return True
            else:
                status_msg = fut.result().status_message
                # "does not exist / not found" 류 → 이미 없는 것이므로 성공으로 처리
                if any(kw in status_msg.lower() for kw in ('not exist', 'not found', 'no model')):
                    self.get_logger().info(f'{name} 이미 없음 — 삭제 불필요')
                    return True
                self.get_logger().warn(
                    f'{name} 삭제 실패: "{status_msg}" (시도 {attempt}/{DELETE_RETRIES})'
                )

            if attempt < DELETE_RETRIES:
                time.sleep(DELETE_RETRY_INTERVAL_S)

        self.get_logger().warn(
            f'{name} {DELETE_RETRIES}회 재시도 후에도 삭제 실패 — 스폰 충돌 가능, 계속 진행'
        )
        return False

    def spawn(self, name, ns, x, y, z, yaw):
        cli = self.create_client(SpawnEntity, '/spawn_entity')
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/spawn_entity 서비스 없음')
            return False
        req = SpawnEntity.Request()
        req.name = name
        req.xml = self.urdf
        req.robot_namespace = ns
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.z = math.sin(yaw / 2)
        req.initial_pose.orientation.w = math.cos(yaw / 2)
        req.reference_frame = 'world'
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        if fut.result():
            self.get_logger().info(f'{name} 스폰 완료: {fut.result().status_message}')
            return fut.result().success
        return False


def main():
    rclpy.init()
    node = ResetNode()

    if not node.wait_urdf(timeout=10.0):
        node.get_logger().error('/robot_description 없음 — robot_state_publisher 확인 필요')
        rclpy.shutdown()
        sys.exit(1)

    # 삭제/스폰 중 물리 엔진 간섭 차단
    node.pause_physics()

    ALL_LIMOS = (LIMO1, LIMO2, LIMO3, LIMO4)

    # [1단계] 전체 삭제 (재시도 포함) — mbot 먼저 삭제
    failed_deletes = []
    ok = node.delete('mbot')
    if not ok:
        failed_deletes.append('mbot')
        node.get_logger().warn('[주의] mbot 삭제 실패 — 스폰 시 충돌 가능성 있음')

    for e in ALL_LIMOS:
        ok = node.delete(e['name'])
        if not ok:
            failed_deletes.append(e['name'])
            node.get_logger().warn(f"[주의] {e['name']} 삭제 실패 — 스폰 시 충돌 가능성 있음")

    if failed_deletes:
        node.get_logger().warn(f'삭제 실패 entity: {failed_deletes}')

    # [2단계] Gazebo physics가 삭제를 완전히 처리하도록 대기 (0.5 → 1.5초)
    node.get_logger().info(f'삭제 후 Gazebo 처리 대기 {DELETE_SPAWN_GAP_S}s...')
    time.sleep(DELETE_SPAWN_GAP_S)

    # [3단계] /get_model_list로 실제 제거 여부 확인
    still_alive = []
    for e in ALL_LIMOS:
        if node.entity_exists(e['name']):
            still_alive.append(e['name'])
            node.get_logger().warn(
                f"[경고] {e['name']} 가 아직 Gazebo 월드에 남아 있음 — 스폰 충돌 가능"
            )

    # [4단계] 스폰 (순서: limo1 → limo2 → limo3 → limo4)
    for i, e in enumerate(ALL_LIMOS):
        node.spawn(**e)
        if i < len(ALL_LIMOS) - 1:
            node.get_logger().info(f"{e['name']} 스폰 완료 — 다음 스폰까지 {INTER_SPAWN_DELAY_S}s 대기")
            time.sleep(INTER_SPAWN_DELAY_S)

    # 물리 엔진 재개
    node.unpause_physics()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
