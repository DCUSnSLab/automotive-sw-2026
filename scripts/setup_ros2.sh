#!/usr/bin/env bash
# 오토모티브SW프로그래밍 — 실습 환경 일괄 설치 (Ubuntu 22.04 / WSL2)
# 2주차 강의자료 25~31p의 수동 절차와 동일. 실패 시 수동 절차로 진행할 것.
set -e

echo "== [1/5] locale =="
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US.UTF-8

echo "== [2/5] ROS 2 apt 저장소 =="
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "== [3/5] ROS 2 Humble + 개발 도구 =="
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools

echo "== [4/5] Gazebo Classic 11 + 실습 패키지 =="
sudo apt install -y ros-humble-gazebo-ros-pkgs \
                    ros-humble-teleop-twist-keyboard \
                    ros-humble-ackermann-msgs \
                    terminator git x11-apps

echo "== [5/5] .bashrc 등록 =="
grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc \
  || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# Gazebo 온라인 모델 DB 조회 끄기 — 빈 월드 첫 실행이 약 2분 멈추는 것을 막는다 (학생이 따로 설정하지 않도록)
grep -qF "GAZEBO_MODEL_DATABASE_URI" ~/.bashrc \
  || echo 'export GAZEBO_MODEL_DATABASE_URI=""' >> ~/.bashrc

echo ""
echo "설치 완료. 새 터미널을 열거나 'source ~/.bashrc' 실행 후:"
echo "  ros2 run demo_nodes_cpp talker"
