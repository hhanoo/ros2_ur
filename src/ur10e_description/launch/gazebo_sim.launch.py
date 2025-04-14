# ROS2 런치 파일 실행에 필요한 기본 모듈
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

# ROS2 패키지 경로를 찾기 위한 도구
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 패키지 경로 및 URDF 경로 설정
    pkg_path = get_package_share_directory('ur10e_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'ur10e.urdf')

    # Gazebo 모델 경로 설정
    gazebo_models_path = os.path.join(pkg_path, 'meshes')
    
    # 환경 변수 설정
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = gazebo_models_path
    os.environ['IGN_FILE_PATH'] = gazebo_models_path
    os.environ['ROS_PACKAGE_PATH'] = os.path.dirname(pkg_path)

    # URDF 파일 내용 읽기
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo에 로봇 spawn
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'ur10e', '-file', urdf_file],
        output='screen'
    )

    # 실행할 노드들을 순서대로 반환
    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', gazebo_models_path),
        SetEnvironmentVariable('IGN_FILE_PATH', gazebo_models_path),
        SetEnvironmentVariable('ROS_PACKAGE_PATH', os.path.dirname(pkg_path)),
        gazebo,
        robot_state_publisher_node,
        spawn_robot
    ])
