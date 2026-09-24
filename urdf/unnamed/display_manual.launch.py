import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/chakradhar/Nurobots_kerela/Code-for-india/urdf/unnamed/urdf/unnamed_gazebo.urdf'
    rviz_config = '/home/chakradhar/Nurobots_kerela/Code-for-india/urdf/unnamed/robot.rviz'

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. Robot State Publisher (publishes tf from /joint_states)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # 2. Standalone RViz2 connected to /joint_states
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2_node
    ])
