#!/usr/bin/env bash
# [REQ-5] 시작 스크립트 최적화
# - 기본값: 빌드 건너뜀 (install/ 있으면 재사용). --rebuild 플래그로 강제 재빌드 가능
# - sleep 대신 서비스/토픽 폴링으로 대기 시간 단축
#
# 사용법:
#   ./start_parking_limo_master.sh           # 빠른 재시작 (이미 빌드된 경우)
#   ./start_parking_limo_master.sh --rebuild # 클린 빌드 후 시작

set -e

ROS_SETUP="/opt/ros/humble/setup.bash"
WS_ROOT="${HOME}/parking_ws/parking_ws"
WS_SETUP="${HOME}/parking_ws/parking_ws/install/setup.bash"
WORLD_PATH="${HOME}/parking_ws/parking_ws/install/limo_gazebosim/share/limo_gazebosim/worlds/parking_lot_scaled_vehicles.world"
LOG_DIR="${HOME}/parking_ws/parking_ws/log"
GAZEBO_LOG="${LOG_DIR}/gzserver_debug.log"
BUILD_LOG="${LOG_DIR}/colcon_build.log"
LIMO_DESCRIPTION_LOG="${LOG_DIR}/limo_description.log"
GZCLIENT_LOG="${LOG_DIR}/gzclient.log"
SPAWN_LOG="${LOG_DIR}/spawn_entity.log"
SPAWN_SERVICE="/spawn_entity"
SPAWN_TIMEOUT_SECONDS=90
ROBOT_DESCRIPTION_TOPIC="/robot_description"

DO_REBUILD=false
for arg in "$@"; do
  [ "${arg}" = "--rebuild" ] && DO_REBUILD=true
done

mkdir -p "${LOG_DIR}"

if [ ! -f "${ROS_SETUP}" ]; then
  echo "Missing ROS setup: ${ROS_SETUP}" >&2
  exit 1
fi

source "${ROS_SETUP}"

echo "[1] 기존 gazebo / limo / rviz 프로세스 정리..."
for pattern in gazebo limo rviz; do
  while read -r pid; do
    if [ -n "${pid}" ] && [ "${pid}" != "$$" ]; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  done < <(pgrep -f "${pattern}" 2>/dev/null || true)
done
sleep 1

if [ "${DO_REBUILD}" = true ]; then
  echo "[2] --rebuild 플래그 감지: 클린 빌드 시작..."
  cd "${WS_ROOT}"
  rm -rf build/ install/ log/
  mkdir -p "${LOG_DIR}"
  colcon build --symlink-install >"${BUILD_LOG}" 2>&1
  echo "[2] 빌드 완료."
else
  echo "[2] 빌드 건너뜀 (재사용). 코드 변경 시 --rebuild 사용."
  cd "${WS_ROOT}"
fi

if [ ! -f "${WS_SETUP}" ]; then
  echo "install/setup.bash 없음. --rebuild 로 다시 실행하세요." >&2
  exit 1
fi

if [ ! -f "${WORLD_PATH}" ]; then
  echo "월드 파일 없음: ${WORLD_PATH}. --rebuild 로 다시 실행하세요." >&2
  exit 1
fi

echo "[3] 워크스페이스 소싱..."
source "${WS_SETUP}"

PACKAGE_MODEL_PATH="${HOME}/parking_ws/parking_ws/install/limo_gazebosim/share/limo_gazebosim/models"
export GAZEBO_MODEL_PATH="${HOME}/.gazebo/models:${PACKAGE_MODEL_PATH}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"

echo "[4] gzserver 시작..."
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && export GAZEBO_MODEL_PATH='${GAZEBO_MODEL_PATH}' && ros2 launch gazebo_ros gazebo.launch.py gui:=false server:=true init:=true factory:=true verbose:=true world:='${WORLD_PATH}'" \
  >"${GAZEBO_LOG}" 2>&1 < /dev/null &
GAZEBO_LAUNCH_PID=$!

echo "[4] gzclient 시작..."
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && gzclient" \
  >"${GZCLIENT_LOG}" 2>&1 < /dev/null &
GZCLIENT_PID=$!

echo "[5] robot_state_publisher 시작..."
existing_rsp_count=$(bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 node list 2>/dev/null | grep -Fxc '/robot_state_publisher' || true")
if [ "${existing_rsp_count}" -gt 1 ]; then
  echo "[WARN] /robot_state_publisher 중복 감지 (${existing_rsp_count}개). 정리 중..."
  while read -r pid; do
    [ -n "${pid}" ] && [ "${pid}" != "$$" ] && kill -9 "${pid}" 2>/dev/null || true
  done < <(pgrep -f "robot_state_publisher" 2>/dev/null || true)
  sleep 1
  existing_rsp_count=0
fi

if [ "${existing_rsp_count}" -eq 0 ]; then
  setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 launch limo_car ackermann.launch.py use_sim_time:=true" \
    >"${LIMO_DESCRIPTION_LOG}" 2>&1 < /dev/null &
  LIMO_DESCRIPTION_PID=$!
else
  echo "[INFO] 기존 /robot_state_publisher 감지. 재사용."
  LIMO_DESCRIPTION_PID=""
fi

echo "[6] /spawn_entity 서비스 대기 중 (최대 ${SPAWN_TIMEOUT_SECONDS}s)..."
spawn_ready=0
for ((i=1; i<=SPAWN_TIMEOUT_SECONDS; i++)); do
  if ros2 service list 2>/dev/null | grep -Fxq "${SPAWN_SERVICE}"; then
    spawn_ready=1
    echo "[6] /spawn_entity 준비 완료 (${i}s 경과)"
    break
  fi
  printf "\r[6] 대기 중... %ds" "${i}"
done
echo ""

if [ "${spawn_ready}" -ne 1 ]; then
  echo "타임아웃: ${SPAWN_SERVICE} 서비스 없음. 로그: ${GAZEBO_LOG}" >&2
  exit 1
fi

echo "[7] /robot_description 토픽 대기 중 (최대 30s)..."
topic_ready=0
for ((i=1; i<=30; i++)); do
  if ros2 topic list 2>/dev/null | grep -Fxq "${ROBOT_DESCRIPTION_TOPIC}"; then
    topic_ready=1
    echo "[7] /robot_description 준비 완료 (${i}s 경과)"
    break
  fi
  sleep 1
done

if [ "${topic_ready}" -ne 1 ]; then
  echo "타임아웃: ${ROBOT_DESCRIPTION_TOPIC} 토픽 없음. 로그: ${LIMO_DESCRIPTION_LOG}" >&2
  exit 1
fi

echo "[8] limo1 스폰 중..."
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity limo1 -robot_namespace limo1 -x 0.0215 -y -0.1030 -z 0.1450 -Y 3.1362 -timeout 120.0" \
  >"${SPAWN_LOG}" 2>&1 < /dev/null &
wait $!

echo "[8] limo2 스폰 중..."
setsid bash -lc "source '${ROS_SETUP}' && source '${WS_SETUP}' && ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity limo2 -robot_namespace limo2 -x 0.0 -y -0.75 -z 0.1450 -Y 1.5708 -timeout 120.0" \
  >>"${SPAWN_LOG}" 2>&1 < /dev/null &
wait $!

echo ""
echo "=== 시뮬레이션 환경 준비 완료 ==="
echo "Gazebo PID: ${GAZEBO_LAUNCH_PID}"
echo "LIMO description PID: ${LIMO_DESCRIPTION_PID}"
echo "gzclient PID: ${GZCLIENT_PID}"
echo "로그 디렉토리: ${LOG_DIR}"
echo ""
echo "다음 터미널에서 자율주차 노드를 실행하세요:"
echo "  ./src/my_valet_parking/scripts/run_parking_algorithm.sh"

wait
