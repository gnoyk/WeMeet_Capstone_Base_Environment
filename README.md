# Remote Parking Simulation Environment

ROS 2와 Gazebo 기반 LIMO 원격 주차 시뮬레이션 환경입니다. 다중 차량 월드 구성, 차량 텔레옵, 주차 알고리즘 실행과 위치 초기화 기능을 제공합니다.

## 주요 기능

- Gazebo 주차 월드와 LIMO 차량 모델 실행
- 슬롯 추적기와 주차 제어 노드 통합 실행
- LIMO2 키보드 텔레옵 및 주행 경계 제한
- Gazebo 재시작 없이 차량 위치 초기화

## 패키지

| 패키지 | 기능 |
| --- | --- |
| `limo_car` | LIMO 모델, Ackermann 설정, RViz 구성 |
| `limo_gazebosim` | 시뮬레이션 월드, 차량 스폰·초기화, 실행 지원 |
| `my_valet_parking` | 슬롯 추적, 텔레옵, 출차 및 주차 제어 |
| `parking_pkg` | 주차 시나리오 상태 관리 및 통합 실행 |

## 실행

실행 환경: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic

```bash
source /opt/ros/humble/setup.bash
cd ~/YSM_ws
colcon build --symlink-install
source install/setup.bash
./start_YSM.sh
```

시뮬레이션이 준비되면 주차 시나리오를 실행합니다.

```bash
ros2 launch parking_pkg valet.launch.py
```

주요 실행 도구:

- `src/my_valet_parking/scripts/run_parking_algorithm.sh`: 슬롯 추적 및 주차 제어 실행
- `src/my_valet_parking/my_valet_parking/limo2_arrow_teleop.py`: LIMO2 키보드 제어
- `src/limo_gazebosim/scripts/reset_limo_positions.sh`: Gazebo를 유지한 채 차량 위치 초기화

## 출처

외부 패키지와 라이선스 정보는 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)에 정리되어 있습니다.
