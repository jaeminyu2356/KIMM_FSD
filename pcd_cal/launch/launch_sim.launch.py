#!/usr/bin/env python3
#  Copyright (c) 2022 Jonas Mahler

#  This file is part of pcl_example.

#  pcl_example is free software: you can redistribute it and/or modify it under the terms 
#  of the GNU General Public License as published by the Free Software Foundation, 
#  either version 3 of the License, or (at your option) any later version.

#  pcl_example is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
#  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
#  See the GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License along 
#  with pcl_example. If not, see <https://www.gnu.org/licenses/>. 

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    yaml_path = os.path.join(
        get_package_share_directory('pcd_cal'),
        'config',
        'tree_grid_sim.yaml'
    )
    
    rviz_path = os.path.join(
        get_package_share_directory('pcd_cal'),
        'rviz',
        'visualization_pcd.rviz'
    )
    
    return LaunchDescription([
        Node(
            package='pcd_cal',
            executable='map_to_pcl_parser_sim',
            name='pcd_publisher',
            parameters= [yaml_path]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path]  # RViz 설정 파일 경로 (선택 사항)
        )
    ])