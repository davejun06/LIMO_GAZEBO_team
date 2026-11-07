# LIMO_GAZEBO_DWA

Gazebo에서 LIMO(애커만)를 two-stage DWA로 장애물 3개 사이로 주행시키는 코드입니다.
이 폴더 하나만 받아 빌드하면 실행됩니다.

## 필요한 환경
ROS 2 Humble, Gazebo Classic, nav2(`nav2_bringup`), `ackermann_msgs`, `gazebo_ros_pkgs`.
GUI 창을 띄우려면 화면 연결(X11)이 필요합니다.

## 빌드
```bash
cd LIMO_GAZEBO_DWA
source /opt/ros/humble/setup.bash
colcon build
```

## 실행
**모든 터미널에서 먼저 실행하세요.** 빠뜨리면 노드끼리 서로 보이지 않습니다.
```bash
cd LIMO_GAZEBO_DWA && source env.sh
```

```bash
# 창 1: Gazebo + RViz (장애물 월드, 차는 흑백선 뒤)
ros2 launch limo_car roboracer_sim.launch.py

# 창 2: 지도 + AMCL (초기 위치 자동 설정)
ros2 launch nav2_bringup localization_launch.py \
  map:=$PWD/src/Raceline-Optimization/maps/track_map3.yaml \
  use_sim_time:=true params_file:=$PWD/tools/exp/amcl_start.yaml

# 창 3: 전역 경로 발행
python3 src/limo_pp/pp_path_pub.py --ros-args \
  -p csv_path:=$PWD/src/Raceline-Optimization/outputs/track_map3_line_pp.csv -p use_sim_time:=true

# 창 4: Pure Pursuit
ros2 run pure_pursuit pure_pursuit_node --ros-args \
  --params-file src/pure_pursuit/config/limo_params.yaml -p use_sim_time:=true 2>&1 | tee /tmp/pp.log

# 창 5: 플래너
python3 src/dwa_planner.py --ros-args -p use_sim_time:=true -p dwa_keep_gap:=0.06 -p max_speed:=1.0

# 창 6: 명령 변환
python3 src/ackermann_to_twist.py --ros-args \
  -r /pure_pursuit/ackermann_cmd:=/final_ackermann_cmd -p use_sim_time:=true
```
창 1~6을 띄우고 약 30초 뒤 출발합니다.

### 출발 방법 1: `go.py` (권장)
```bash
python3 tools/exp/go.py          # 1랩, AMCL 정지 기준 0.25 m
python3 tools/exp/go.py 1 0.30   # 1랩, 0.30 m (안전 규칙 완화)
```
장애물 확인, 시작 위치 배치, 사전 점검, 출발, 랩타임 측정을 하고 **충돌이나 근접 시 차를 세웁니다.**
결과는 `logs/exp/my-run/`에 저장됩니다. `Ctrl-C`로 멈추며, 즉시 서지 않고 조금 더 갑니다.

### 출발 방법 2: 직접 신호
```bash
ros2 topic pub --once /control_selector std_msgs/msg/String "{data: 'pure_pursuit'}"   # 출발
ros2 topic pub --once /control_selector std_msgs/msg/String "{data: 'stop'}"           # 정지
python3 tools/exp/place_start.py                                                       # 멈춘 뒤 시작 위치로
```
**이 방법은 충돌 감지와 랩 측정이 없습니다.** 차가 닿아도 멈추지 않고, 한 바퀴 뒤에도 계속 돕니다.

## 값
| 인자 | 값 | 뜻 |
|---|---|---|
| `max_speed` | 1.0 | 속도 상한 [m/s] |
| `dwa_keep_gap` | 0.06 | 차체가 이보다 가까우면 틈을 줄이는 조향을 피함 [m]. 기본값은 0(꺼짐)이라 창 5에서 꼭 지정 |

## 기록 (개발 환경, 사실)
흑백선 뒤 출발, `track_map3`, 충돌 0회.

| 설정 | 랩타임 |
|---|---|
| `max_speed` 0.8, AMCL 기준 0.30 | 132.4초, 136.2초 |
| `max_speed` 1.0, AMCL 기준 0.30, 창 켬 | 129.6초 |
| 같은 값, 이 폴더를 새로 빌드, 창 끔 | 135.3초 |

## 알려진 한계
- **매번 완주하지는 않습니다.** 같은 값으로 AMCL 오차나 장애물 옆 근접으로 멈춘 적이 있고, 위 표는 완주한 주행의 기록입니다. 3연속 완주는 없습니다.
- **AMCL 오차가 큽니다.** 최대 0.293 m까지 나왔습니다. 0.25 m 기준(`go.py` 기본값)이면 o3를 지난 뒤 멈출 수 있고, 0.30 m는 안전 규칙을 완화한 값입니다.
- **50초 목표는 달성하지 못했습니다.** 평균 속도가 약 0.33 m/s이고, 장애물 옆에서 0.06 m/s로 기어갑니다.
- **값은 이 월드에 맞춰졌습니다.** `dwa_keep_gap` 0.06은 이 통로의 틈을 보고 정해서 다른 월드에서는 검증되지 않았습니다.
- **Gazebo를 오래 켜 두면** 접촉 정보가 끊기는 경우가 있었습니다(원인 미상). `go.py`가 알려 주면 Gazebo를 다시 띄우세요.
- **RViz의 차 모양**은 메시 경로 문제로 박스 형태로 보입니다. 주행과는 무관합니다.

## 구성
| 경로 | 내용 |
|---|---|
| `env.sh` | 터미널 환경 설정 |
| `src/dwa_planner.py` | 로컬 플래너 (DWA) |
| `src/ackermann_to_twist.py` | 명령 변환 |
| `src/limo_pp/pp_path_pub.py` | 전역 경로 발행 |
| `src/pure_pursuit/` | Pure Pursuit 패키지 |
| `src/limo_ros2/limo_car/` | 차량 모델, 월드, launch |
| `src/Raceline-Optimization/` | 지도 `track_map3`, 경로 csv |
| `tools/exp/` | `go.py`, 모니터, 사전 점검, AMCL 설정 |
