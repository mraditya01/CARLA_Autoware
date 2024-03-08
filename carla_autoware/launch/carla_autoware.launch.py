import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='carla_autoware',
            executable='carla_autoware',
            name=['carla_autoware'],
            output='screen',
            emulate_tty=True,
            parameters=[
            ],
        ),
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='carla_to_lidar_concatenated',
            output='screen',
            arguments=['/carla/ego_vehicle/lidar', '/carla_pointcloud']
        ),
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='gnss',
            output='screen',
            arguments=['/carla/ego_vehicle/gnss', '/carla_nav_sat_fix']
        ),
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='initialpose',
            output='screen',
            arguments=['/initialpose', '/initialpose3d']
        ),
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='camera_info',
            output='screen',
            arguments=['/carla/ego_vehicle/rgb_front/camera_info', '/sensing/camera/traffic_light/camera_info']
        ),
        launch_ros.actions.Node(
            package='topic_tools',
            executable='relay',
            name='camera',
            output='screen',
            arguments=['/carla/ego_vehicle/rgb_front/image', '/sensing/camera/camera5/image_rect_color']
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ego_vehicle2base_link',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '/ego_vehicle', '/base_link']
        ),
        launch_ros.actions.Node(
            package='gnss_interface',
            executable='gnss_interface_node',
            name='gnss_interface',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='pointcloud_interface',
            executable='pointcloud_interface_node',
            name='pointcloud_interface',
            output='screen'
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
