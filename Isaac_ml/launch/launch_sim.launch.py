#!/isaac-sim/kit/python/bin/python3

import os
from os.path import join
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    state_machine = Node(
            package='isaac_ml',
            executable='state_machine',
            name='state_machine'
        )
    
    four_ws_control = Node(
            package='isaac_ml',
            executable='four_ws_control.py',
            name='four_ws_control'
        )
    
    four_ws_control_pos = Node(
        package='isaac_ml',
        executable='four_ws_control_pos.py',
        name='four_ws_control_pos'
    )

    drone_local_path = Node(
        package='isaac_ml',
        executable='drone_local_path.py',
        name='drone_local_path'
    )
    
    return LaunchDescription([
        state_machine,
        # four_ws_control,
        # four_ws_control_pos,
        drone_local_path
    ])