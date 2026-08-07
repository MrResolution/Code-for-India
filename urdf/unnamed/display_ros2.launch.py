import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    urdf_file = '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/urdf/unnamed_gazebo.urdf'

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    simulate_arg = DeclareLaunchArgument(
        'simulate',
        default_value='true',
        description='Launch mode: true for automated motion simulator, false for manual sliders GUI'
    )

    simulate = LaunchConfiguration('simulate')

    # 1. Robot State Publisher
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

    # 2. Automated Motion Simulator Node (simulate == 'true')
    arm_simulator_node = ExecuteProcess(
        cmd=['python3', '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/arm_simulator.py'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", simulate, "' == 'true'"]))
    )

    # 3. Manual Joint State Publisher GUI (simulate == 'false')
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(PythonExpression(["'", simulate, "' == 'false'"]))
    )

    # 4. RViz2
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/sabo/Documents/learn_/Hardware/urdf/unnamed/robot.rviz']
    )

    return LaunchDescription([
        simulate_arg,
        robot_state_publisher,
        arm_simulator_node,
        joint_state_publisher_gui,
        rviz2_node
    ])
