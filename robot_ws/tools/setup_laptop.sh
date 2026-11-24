#!/usr/bin/env bash
# [노트북] Ubuntu 24.04 + ROS 2 Jazzy(desktop) 설치 후 실행. 개발·시각화용 패키지.
set -euo pipefail

sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip python3-pytest \
  python3-numpy python3-opencv python3-yaml \
  ros-jazzy-cv-bridge ros-jazzy-tf2-ros ros-jazzy-slam-toolbox \
  ros-jazzy-dynamixel-sdk ros-jazzy-rqt-image-view ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-rosbag2

# 실물 센서 없이 개발하므로 rplidar_ros / camera_ros는 선택. 설치할 때만 주석을 푼다. 확인 필요
# sudo apt install -y ros-jazzy-rplidar-ros ros-jazzy-camera-ros

# mediapipe는 0.10.18로 고정한다 (계획서 3.3절).
#  - 0.10.30 이상(1.x 포함)은 mp.solutions가 삭제돼 hand_detector가 쓸 수 없다.
#  - 0.10.20/0.10.21은 arm64(Pi)용 py3.12 패키지가 없다.
#  - 의존성을 그대로 깔면 numpy 2 / opencv-contrib가 들어와 ROS cv_bridge(numpy 1.26)를 깰 수 있어
#    --no-deps로 필요한 것만 설치한다(jax, opencv-contrib, sentencepiece 제외).
pip install --user --break-system-packages --no-deps \
  mediapipe==0.10.18 absl-py flatbuffers "protobuf>=4.25.3,<5" sounddevice \
  || echo "mediapipe 설치 실패: backend=color 사용"
