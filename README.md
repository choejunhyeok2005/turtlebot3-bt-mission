# TurtleBot3 BT 과제: 이동 · 촬영 · 복귀

ROS 2 Jazzy, Webots, Nav2를 사용한 Behaviour Tree 과제입니다.

목표 수신 → 출발 위치 저장 → 목표로 이동 → 사진 촬영·저장 → 출발 위치 복귀

## 설치

ROS 2 Jazzy와 Webots ROS 2 TurtleBot3·Nav2가 설치된 Ubuntu 환경이 필요합니다.

```bash
mkdir -p ~/robot_bt_ws/src
cd ~/robot_bt_ws/src
git clone https://github.com/choejunhyeok2005/turtlebot3-bt-mission.git py_bt_ros
cd py_bt_ros
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Webots·Nav2·RViz 실행 (터미널 1)

기존 과제 환경의 workspace와 Webots 설치 경로를 기준으로 합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/webots_ros2_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
export WEBOTS_HOME=/usr/local/webots
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true use_sim_time:=true
```

RViz의 **2D Pose Estimate**로 로봇의 현재 위치와 방향을 지정하세요.


기존 Webots와 Nav2를 실행하고 RViz의 `2D Pose Estimate`로 초기 위치를 지정합니다.
RViz에 `SetGoal` 도구를 추가하고 Tool Properties의 Topic을 `/bt/goal_pose`로
설정합니다. Fixed Frame은 `map`을 사용합니다.

## 터미널 2: 사진 저장 서비스

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_bt_ws/src/py_bt_ros
source .venv/bin/activate
python scenarios/example_turtlebot3/capture_image_server.py --ros-args \
  -p use_sim_time:=true
```

기본 영상 토픽은 `/TurtleBot3Burger/front_camera/image_color`입니다.
실제 토픽이 다르면 `-p image_topic:=/실제/영상/토픽`을 추가합니다.
사진은 `~/robot_bt_ws/photos`에 PNG로 저장합니다.
`-p output_dir:=/원하는/폴더`로 저장 경로를 변경할 수 있습니다.

## 터미널 3: BT

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_bt_ws/src/py_bt_ros
source .venv/bin/activate
python main.py --config=scenarios/example_turtlebot3/config.yaml
```

RViz에서 `/bt/goal_pose`를 사용하는 도구로 목표 위치와 방향을 지정합니다.
BT는 목표 수신 → `map`에서 로봇 출발 위치 저장 → Nav2 이동 → 촬영 서비스 호출
→ 출발 위치 복귀 순서로 실행합니다. 완료 후 다음 목표를 기다립니다.

진행 중 받은 새 목표는 무시됩니다. 목표·출발 위치는 실행 중 변경하지 않습니다.
실패하면 자동 재시도하지 않고 다음 목표를 기다립니다. 촬영에 실패하면 복귀도
실행하지 않습니다. 필요하면 RViz의 Nav2 도구로 직접 복귀시키세요.
서비스 요청은 취소할 수 없으므로 BT의 촬영 타임아웃 뒤에도 서비스가 사진을
저장할 수 있습니다.

BT 창의 `q` 또는 창 닫기는 종료와 진행 중 Nav2 목표 취소를 요청합니다.
`p`는 BT tick만 일시 정지합니다. **이미 실행 중인 Nav2 이동은 계속됩니다.**

## 연결 확인

```bash
ros2 action info /navigate_to_pose
ros2 topic info /TurtleBot3Burger/front_camera/image_color
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /bt/goal_pose --once
```

TF 로봇 프레임이 다른 경우 `config.yaml`의 `robot_frame`을 변경합니다.
Nav2 액션이나 촬영 서비스 이름도 같은 파일에서 변경할 수 있습니다.
BT와 시뮬레이터 터미널은 같은 ROS_DOMAIN_ID와 RMW 설정을 사용해야 합니다.

사진 서비스만 먼저 확인하려면 다음을 실행합니다.

```bash
ros2 service call /bt/capture_image std_srvs/srv/Trigger '{}'
```

이 호출은 로봇 이동과 관계없이 사진을 저장합니다.
요청 후 새 카메라 메시지를 기다리며 5초 안에 수신하지 못하면 실패합니다.

## 시뮬레이터 없이 검증

프로젝트 루트에서 가상환경을 활성화한 뒤 실행합니다.

```bash
python -m unittest scenarios.example_turtlebot3.test_mission -v
```

## 검증 상태

단위 테스트와 실제 Webots·Nav2 연동 검증은 별개입니다. 실제 시뮬레이터 연동은 실행 후 확인해야 합니다.

## 기반 프로젝트와 라이선스

[inmo-jang/py_bt_ros](https://github.com/inmo-jang/py_bt_ros)를 기반으로 TurtleBot3 과제 시나리오와 실행·종료 처리를 추가했습니다.
기존 turtlesim 예제, [원본 README](docs/UPSTREAM_README.md), [GPL-3.0 라이선스](LICENSE)를 보존했습니다.
