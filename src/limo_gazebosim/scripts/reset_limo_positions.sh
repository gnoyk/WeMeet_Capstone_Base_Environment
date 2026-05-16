#!/usr/bin/env bash
# [REQ-5] 시뮬레이션 리셋 — Gazebo 유지, LIMO 위치만 초기화
# spawn_entity.py 우회: reset_limo.py (rclpy 직접 서비스 호출)

set -eo pipefail

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${HOME}/parking_ws/parking_ws/install/setup.bash"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "${ROS_SETUP}"
[ -f "${WS_SETUP}" ] && source "${WS_SETUP}"

echo "[1/3] 자율주차 노드 종료 중..."
for pattern in limo_valet_parking_node dynamic_tracker_node car_exit_controller limo2_exit_handler limo2_arrow_teleop; do
    while IFS= read -r pid; do
        [ -n "${pid}" ] && kill -SIGINT "${pid}" 2>/dev/null || true
    done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done

# SIGINT 후 2초 대기 (1초에서 증가)
sleep 2

# 잔존 프로세스 확인 후 SIGKILL 강제 종료
echo "[1/3] 잔존 프로세스 확인 중..."
found_remaining=0
for pattern in limo_valet_parking_node dynamic_tracker_node car_exit_controller limo2_exit_handler limo2_arrow_teleop; do
    while IFS= read -r pid; do
        if [ -n "${pid}" ]; then
            echo "  → SIGKILL: ${pattern} (PID ${pid})"
            kill -SIGKILL "${pid}" 2>/dev/null || true
            found_remaining=1
        fi
    done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done

# SIGKILL 후 추가 대기
[ "${found_remaining}" -eq 1 ] && sleep 1

echo "[2/3] limo1 / limo2 위치 리셋 중..."
python3 "${SCRIPT_DIR}/reset_limo.py"

echo ""
echo "[3/3] 완료. 자율주차 노드 재시작:"
echo "  cd ~/parking_ws/parking_ws"
echo "  ./src/my_valet_parking/scripts/run_parking_algorithm.sh"
