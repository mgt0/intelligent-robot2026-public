# robot_ws

2026 지능형로봇경진대회 로봇 소프트웨어 (ROS 2 Jazzy, ament_python).

## 설치

```bash
# [Pi] 로봇            : bash tools/setup_pi.sh
# [노트북] 개발 PC      : bash tools/setup_laptop.sh
# [Pi] udev            : tools/99-robot.rules의 VID/PID를 채운 뒤 /etc/udev/rules.d/에 복사
```

## 빌드

```bash
# [Pi]/[노트북] ~/robot_ws
rosdep install --from-paths src --ignore-src -y --skip-keys=libcamera
colcon build --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=42
```

## 시험 (ROS 없이)

```bash
# 워크스페이스 루트에서
python3 -m pytest src -q
```

## 실행

```bash
# [Pi] 종목 1 / 종목 2
ros2 launch robot_bringup drive.launch.py
ros2 launch robot_bringup follow.launch.py
# 출발 (노트북 또는 Pi)
ros2 service call /mission/start std_srvs/srv/Trigger
# 하드웨어 없이 구동 노드만 확인
ros2 launch robot_bringup base.launch.py use_fake_hw:=true
# 인식 튜닝
ros2 launch robot_bringup vision_test.launch.py
ros2 run robot_vision hsv_tuner <이미지|영상|카메라번호>
```

GUI (상태 보기, 모드 전환, 출발·정지. 수동 조종 없음). Pi와 노트북은 같은 `ROS_DOMAIN_ID`를 쓴다:

```bash
# [Pi] 센서·모터 + mode_manager (미션 스택은 mode_manager가 관리)
ros2 launch robot_bringup robot.launch.py default_mode:=drive
# [노트북] GUI 창 (파라미터 파일은 선택)
ros2 run robot_gui gui --ros-args --params-file $(ros2 pkg prefix robot_bringup)/share/robot_bringup/config/gui.yaml
```

launch 목록: base, sensors, bringup, teleop(시험 전용), drive, follow, slam, vision_test, robot(Pi용 GUI 운용), mission_stack(mode_manager가 실행).
파라미터는 `src/robot_bringup/config/*.yaml`에서 바꾼다.
