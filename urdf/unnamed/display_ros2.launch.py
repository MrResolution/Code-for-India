import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context).lower()
    simulate = LaunchConfiguration('simulate').perform(context).lower()

    if robot == 'quard':
        urdf_file = '/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/urdf/quard_bot.urdf'
        rviz_config = '/home/sabo/Documents/learn_/Hardware/urdf/quard_bot/rviz/quard_bot.rviz'
    elif robot == 'dual':
        urdf_file = '/home/sabo/Documents/learn_/Hardware/urdf/dual_robots.urdf'
        rviz_config = '/home/sabo/Documents/learn_/Hardware/urdf/dual_robots.rviz'
    else:
        urdf_file = '/home/sabo/Documents/learn_/Hardware/urdf/arm/urdf/arm.urdf'
        rviz_config = '/home/sabo/Documents/learn_/Hardware/urdf/arm/rviz/arm.rviz'

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    nodes = []

    # 1. Robot State Publisher
    nodes.append(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    ))

    if simulate == 'true':
        if robot != 'quard':
            nodes.append(ExecuteProcess(
                cmd=['python3', '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/arm_simulator.py'],
                output='screen'
            ))
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ))
    else:
        nodes.append(Node(
            package='servo_joint_publisher',
            executable='calibrated_joint_gui',
            name='calibrated_joint_gui',
            output='screen',
            additional_env={
                'ROBOT_TARGET': robot,
                'RVIZ_CONFIG': rviz_config
            }
        ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulate',
            default_value='false',
            description='Launch mode: true for automated motion simulator, false for manual sliders GUI'
        ),
        DeclareLaunchArgument(
            'robot',
            default_value='arm',
            description='Target robot: arm (6-DOF Robotic Arm) or quard (Sesame Quadruped)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
