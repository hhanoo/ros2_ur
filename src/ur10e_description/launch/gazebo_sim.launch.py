# Launch 및 ROS 노드를 위한 모듈 임포트
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

# 패키지 경로 탐색을 위한 함수 임포트
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로 및 URDF 경로 설정
    pkg_path = get_package_share_directory('ur10e_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'ur10e.urdf')

    # Gazebo 실행 (빈 월드 + ROS2 통합 플러그인 포함)
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # Gazebo Harmonic (Ignition) 에 로봇 spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur10e', '-file', urdf_file],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_robot
    ])
