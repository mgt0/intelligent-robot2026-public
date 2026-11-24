#!/usr/bin/env bash
# [Pi] Ubuntu 24.04 + ROS 2 Jazzy(ros-base) 설치 후 실행. 로봇 탑재용 패키지.
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-pytest \
  python3-numpy python3-opencv python3-yaml \
  ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-slam-toolbox \
  ros-jazzy-dynamixel-sdk ros-jazzy-camera-ros ros-jazzy-rplidar-ros \
  ros-jazzy-teleop-twist-keyboard

# 아래 apt 패키지가 없으면 소스 빌드로 대체한다 (계획서 3.3절, 7장 위험 1). 확인 필요
#   rplidar_ros : git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
#   camera_ros  : raspberrypi 포크 libcamera 소스 빌드 후 camera_ros 소스 빌드
#   dynamixel_sdk: pip install dynamixel-sdk

# 0.10.18 arm64 py3.12 패키지가 PyPI에 있는 것은 확인했다(2026-10-21). Pi에서 실행·FPS는 [확인 필요].
# 실패해도 hand_node가 color로 전환한다.
# mediapipe는 0.10.18로 고정한다 (계획서 3.3절).
#  - 0.10.30 이상(1.x 포함)은 mp.solutions가 삭제돼 hand_detector가 쓸 수 없다.
#  - 0.10.20/0.10.21은 arm64(Pi)용 py3.12 패키지가 없다.
#  - 의존성을 그대로 깔면 numpy 2 / opencv-contrib가 들어와 ROS cv_bridge(numpy 1.26)를 깰 수 있어
#    --no-deps로 필요한 것만 설치한다(jax, opencv-contrib, sentencepiece 제외).
pip install --user --break-system-packages --no-deps \
  mediapipe==0.10.18 absl-py flatbuffers "protobuf>=4.25.3,<5" sounddevice \
  || echo "mediapipe 설치 실패: backend=color 사용"

sudo usermod -aG dialout "$USER"
echo "udev 규칙: sudo cp tools/99-robot.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger"
echo "로그아웃 후 다시 로그인해야 dialout 그룹이 적용된다."
