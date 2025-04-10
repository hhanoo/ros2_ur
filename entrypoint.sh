#!/bin/bash
set -e

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 워크스페이스가 빌드되어 있다면 setup.bash를 source 하기 위한 조건문
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source /root/ros2_ws/install/setup.bash
fi

# 입력된 명령어 실행
exec "$@"