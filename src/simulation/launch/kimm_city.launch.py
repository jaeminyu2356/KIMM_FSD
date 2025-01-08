import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue
import xacro
from launch.event_handlers import OnProcessStart
from launch.actions import TimerAction

def generate_launch_description():
    # GUI 관련 환경 변수 설정
    gui_env = SetEnvironmentVariable(
        'QT_X11_NO_MITSHM', 
        '1'
    )
    
    # XDG_RUNTIME_DIR 설정
    runtime_dir = SetEnvironmentVariable(
        'XDG_RUNTIME_DIR',
        '/tmp/runtime-root'
    )

    # Scout 경로 먼저 정의
    scout_path = '/home/KIMM_FSD/src/scout_ros2'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_simulation = get_package_share_directory('simulation')
    
    # 환경 변수 설정 - 모델 경로를 필요한 디렉토리만 포함하도록 수정
    gazebo_setup = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', 
        os.path.join(pkg_simulation, 'models') + ':' +
        '/usr/share/gazebo-11/models'
    )
    
    # Gazebo 리소스 경로 설정
    additional_gazebo_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH',
        os.path.join(pkg_simulation, 'worlds') + ':' +
        '/usr/share/gazebo-11'
    )

    # Scout 로봇의 URDF 파일 로드
    xacro_file = os.path.join(scout_path, 'scout_description/urdf/scout_v2.xacro')
    
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                ' lidar_enable:=true',
                ' imu_enable:=true',
                ' gps_enable:=true',
                ' lidar_visual:=false',
                ' add_materials:=true']),
        value_type=str
    )

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'world': os.path.join(pkg_simulation, 'worlds', 'kimm_city.world'),
            'verbose': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Scout 로봇 스폰 - Gazebo 로드 후 10초 대기
    spawn_robot = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'scout',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gui_env,
        runtime_dir,
        gazebo_setup,
        additional_gazebo_path,
        gazebo,
        robot_state_publisher,
        spawn_robot
    ]) 