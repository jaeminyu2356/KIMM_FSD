#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    #----------------------------------------------------------------------
    # 1) LaunchConfigurations
    #----------------------------------------------------------------------
    namespace        = LaunchConfiguration('namespace')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    autostart        = LaunchConfiguration('autostart')
    params_file      = LaunchConfiguration('params_file')
    use_composition  = LaunchConfiguration('use_composition')
    container_name   = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    rviz_config_file = LaunchConfiguration('rviz_config_file')  # <-- RViz 설정

    lifecycle_nodes = [
        'map_server',
        'filter_mask_server',
        'costmap_filter_info_server',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'smoother_server'
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    param_substitutions = {
        'use_sim_time': use_sim_time,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    #----------------------------------------------------------------------
    # 2) Declare Launch Arguments
    #----------------------------------------------------------------------
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('keepout'),
            'params',
            'keepout_params.yaml'
        ),
        description='Full path to the merged keepout+nav2 params file'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False', description='Use composed bringup if True'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='Name of container for nodes if using composition'
    )

    # -- (새로 추가) RViz 설정 파일 인자 --
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            get_package_share_directory('nav2_bringup'),
            'rviz',
            'nav2_default_view.rviz'
        ),
        description='Full path to the RVIZ config file to use'
    )

    #----------------------------------------------------------------------
    # 3) Standalone Nodes (use_composition=False)
    #----------------------------------------------------------------------
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # (1) 실제 전역 맵(OccupancyGrid) 퍼블리시 - map_server 노드
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                namespace=namespace,
                output='screen',
                parameters=[configured_params],
                # remappings=[('map', 'map')]  # 필요시 명시적 remap 가능
            ),

            # (2) 필터 마스크를 퍼블리시하는 filter_mask_server
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                namespace=namespace,
                output='screen',
                parameters=[configured_params]
            ),

            # (3) costmap_filter_info_server
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                namespace=namespace,
                output='screen',
                parameters=[configured_params]
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[configured_params],
                remappings=remappings + [
                    ('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]
            )
        ]
    )

    #----------------------------------------------------------------------
    # 4) Composable Nodes (use_composition=True)
    #----------------------------------------------------------------------
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            PushRosNamespace(
                condition=IfCondition(PythonExpression([LaunchConfiguration('namespace'), " != ''"])),
                namespace=namespace
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    # (1) 실제 전역 맵(OccupancyGrid) 퍼블리시 - map_server
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='map_server',
                        parameters=[configured_params]
                        # remappings=[('map', 'map')]  # 필요하면 추가
                    ),

                    # (2) filter_mask_server
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='filter_mask_server',
                        parameters=[configured_params]
                    ),

                    # (3) costmap_filter_info_server
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='costmap_filter_info_server',
                        parameters=[configured_params]
                    ),

                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='nav2_behaviors::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_params],
                        remappings=remappings + [
                            ('cmd_vel', 'cmd_vel_nav'),
                            ('cmd_vel_smoothed', 'cmd_vel')
                        ]
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'autostart': autostart,
                            'node_names': lifecycle_nodes
                        }]
                    )
                ]
            )
        ]
    )

    #----------------------------------------------------------------------
    # 5) RViz Node (자동 실행)
    #----------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    #----------------------------------------------------------------------
    # 6) LaunchDescription 구성
    #----------------------------------------------------------------------
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Declare launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_rviz_config_cmd)

    # Add group actions
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    # Finally, add the RViz node (always launched)
    ld.add_action(rviz_node)

    return ld
