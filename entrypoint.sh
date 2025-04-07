#!/bin/bash
set -e

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 사용자가 자신의 워크스페이스를 colcon build 했다면 이 부분도 추가할 수 있어
if [ -f "/code/install/setup.bash" ]; then
    source /code/install/setup.bash
fi

exec "$@"