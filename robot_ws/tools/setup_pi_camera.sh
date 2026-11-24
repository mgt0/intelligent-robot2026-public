#!/usr/bin/env bash
# [Pi 5] CSI 카메라(모듈 3 / imx708)를 ROS에서 쓰기 위한 설치.
#
# 배경: Pi 5는 PiSP ISP를 쓰는데 Ubuntu 24.04 기본 libcamera(0.2.0)에는 PiSP 파이프라인/IPA가 없다
#       (ipa_rpi_vc4만 있고 ipa_rpi_pisp 없음 -> "No IPA found", 카메라 목록 비어 있음).
#       그래서 라즈베리파이 포크 libcamera(0.7.x, PiSP 지원)를 소스 빌드하고
#       camera_ros도 그 libcamera에 맞춰 소스 빌드한다. (계획서 3.3절, 7장 위험 1)
#
# 2026-10-23 at@G(Pi 5, Ubuntu 24.04.5)에서 실제로 성공한 절차. /image_raw 30Hz(bgra8) 확인.
set -euo pipefail

# 0) noble-updates 저장소가 빠져 있으면 추가 (없으면 -dev 패키지 의존성 충돌)
SRC=$(grep -rl "Suites:" /etc/apt/sources.list.d/*.sources | head -1)
if ! grep -q "noble-updates" "$SRC"; then
  sudo sed -i "s/^Suites: noble noble-security/Suites: noble noble-updates noble-security/; s/^Suites: noble$/Suites: noble noble-updates/" "$SRC"
fi
sudo apt-get update

# 1) libcamera 빌드 의존성
sudo apt-get install -y git meson ninja-build pkg-config python3-pip python3-jinja2 \
  python3-yaml python3-ply libboost-dev libgnutls28-dev openssl libtiff-dev \
  pybind11-dev libglib2.0-dev libevent-dev cmake libcamera-ipa

# 2) 라즈베리파이 포크 libcamera 소스 빌드 (PiSP + VC4 파이프라인)
cd ~
[ -d libcamera ] || git clone --depth=1 https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release \
  -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp \
  -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
  -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=disabled
ninja -C build
sudo ninja -C build install

# 포크 libcamera(/usr/local)를 기본 링커 경로에 등록
echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/libcamera-local.conf >/dev/null
sudo ldconfig

# 확인: ipa_rpi_pisp.so가 있어야 하고, cam -l에 imx708이 보여야 한다
ls /usr/local/lib/aarch64-linux-gnu/libcamera/ipa/ipa_rpi_pisp.so

# 3) 사용자를 video/render/dialout 그룹에 (카메라·시리얼 접근). 로그아웃 후 재로그인 필요
sudo usermod -aG video,render,dialout "$USER"

# 4) camera_ros 소스 빌드 (포크 libcamera 0.7.x에 맞춰 패치)
#    - -Werror로 deprecated(libcamera::Span) 경고가 에러가 되므로 그 항목만 해제
#    - libcamera::dynamic_extent -> std::dynamic_extent (포크에서 제거됨)
cd ~/robot_ws/src
[ -d camera_ros ] || git clone --depth=1 https://github.com/christianrauch/camera_ros.git
cd camera_ros
sed -i "s/add_compile_options(-Wall -Wextra -Wpedantic -Werror)/add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-error=deprecated-declarations)/" CMakeLists.txt
sed -i "s/libcamera::dynamic_extent/std::dynamic_extent/g" src/ParameterHandler.cpp src/type_extent.hpp

# camera_ros의 ROS 의존성 (rosdep은 무암호 sudo가 안 돼서 직접 설치)
sudo apt-get install -y ros-jazzy-camera-info-manager ros-jazzy-image-view \
  ros-jazzy-ament-cmake-clang-format ros-jazzy-ament-cmake-mypy ros-jazzy-ament-cmake-pyflakes

cd ~/robot_ws
source /opt/ros/jazzy/setup.bash
export PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig
colcon build --symlink-install --packages-select camera_ros --cmake-args -DCMAKE_PREFIX_PATH=/usr/local

echo "완료. 확인:"
echo "  source ~/robot_ws/install/setup.bash"
echo "  ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -r /camera/image_raw:=/image_raw"
echo "  ros2 topic hz /image_raw   # 약 30Hz"
