#!/bin/bash
set -e

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_ROOT="${HOME}/YSM_ws"
WS_SETUP="${WS_ROOT}/install/setup.bash"
WORLD_PATH="${WS_ROOT}/install/limo_gazebosim/share/limo_gazebosim/worlds/parking_lot_scaled_vehicles.world"
LOG_DIR="${WS_ROOT}/log"

mkdir -p "${LOG_DIR}"
source "${ROS_SETUP}"

echo "[1] 기존 프로세스 정리..."
for p in gazebo limo rviz; do
  pkill -f "${p}" 2>/dev/null || true
done
sleep 1

if [ ! -f "${WS_SETUP}" ]; then
  echo "[2] 빌드 중..."
  cd "${WS_ROOT}"
  colcon build --symlink-install \
    > "${LOG_DIR}/build.log" 2>&1
fi

source "${WS_SETUP}"

PACKAGE_MODEL_PATH="${WS_ROOT}/install/limo_gazebosim/share/limo_gazebosim/models"
export GAZEBO_MODEL_PATH="${HOME}/.gazebo/models:${PACKAGE_MODEL_PATH}"

echo "[3] Gazebo 시작..."
setsid bash -lc "
  source '${ROS_SETUP}' && source '${WS_SETUP}' &&
  export GAZEBO_MODEL_PATH='${GAZEBO_MODEL_PATH}' &&
  ros2 launch gazebo_ros gazebo.launch.py \
    gui:=true server:=true world:='${WORLD_PATH}'
" > "${LOG_DIR}/gazebo.log" 2>&1 < /dev/null &

echo "[4] robot_state_publisher 시작..."
setsid bash -lc "
  source '${ROS_SETUP}' && source '${WS_SETUP}' &&
  ros2 launch limo_car ackermann.launch.py use_sim_time:=true
" > "${LOG_DIR}/rsp.log" 2>&1 < /dev/null &

echo "[5] /spawn_entity 대기 중 (최대 90s)..."
for ((i=1; i<=90; i++)); do
  ros2 service list 2>/dev/null | grep -Fxq "/spawn_entity" && break
  sleep 1
done

echo "[6] LIMO 스폰..."
source "${WS_SETUP}"
python3 "${HOME}/YSM_ws/src/limo_gazebosim/scripts/reset_limo.py"

echo ""
echo "=== YSM_ws 환경 준비 완료 ==="
echo "주차 알고리즘 실행:"
echo "  source ~/YSM_ws/install/setup.bash"
echo "  ros2 launch parking_pkg valet.launch.py"
