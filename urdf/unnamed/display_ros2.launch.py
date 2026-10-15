import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Repo root: $ROBOT_PROJECT_DIR, else two levels above this file (urdf/unnamed/).
PROJECT_DIR = os.environ.get(
    'ROBOT_PROJECT_DIR',
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
URDF_DIR = os.path.join(PROJECT_DIR, 'urdf')
# The URDFs point their meshes at this checkout path; swap in the real one so
# RViz finds them wherever the repository lives.
URDF_MESH_PREFIX = 'file:///home/sabo/Documents/learn_/Hardware'


def attach_to_world(urdf):
    """Hang a model off a `world` link when its tree starts elsewhere.

    Every robot then shares one root frame, so the desktop app can swap the
    model in robot_state_publisher (its RViz follows the robot tab) without
    RViz losing its Fixed Frame. calibrated_joint_gui.py has the same helper.
    """
    root = ET.fromstring(urdf)
    links = {link.get('name') for link in root.findall('link')}
    children = {j.find('child').get('link') for j in root.findall('joint') if j.find('child') is not None}
    roots = sorted(links - children)
    if 'world' in links or len(roots) != 1:
        return urdf
    joint = (f'  <link name="world"/>\n'
             f'  <joint name="world_to_{root.get("name", "robot")}" type="fixed">\n'
             f'    <parent link="world"/>\n    <child link="{roots[0]}"/>\n  </joint>\n')
    end = urdf.rindex('</robot>')
    return urdf[:end] + joint + urdf[end:]


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context).lower()
    simulate = LaunchConfiguration('simulate').perform(context).lower()

    if robot == 'quard':
        urdf_file = os.path.join(URDF_DIR, 'quard_bot', 'urdf', 'quard_bot.urdf')
        rviz_config = os.path.join(URDF_DIR, 'quard_bot', 'rviz', 'quard_bot.rviz')
    elif robot == 'dogu':
        urdf_file = os.path.join(URDF_DIR, 'dogu', 'urdf', 'dogu.urdf')
        rviz_config = os.path.join(URDF_DIR, 'dogu', 'rviz', 'dogu.rviz')
    elif robot == 'dual':
        urdf_file = os.path.join(URDF_DIR, 'dual_robots.urdf')
        rviz_config = os.path.join(URDF_DIR, 'dual_robots.rviz')
    else:
        urdf_file = os.path.join(URDF_DIR, 'arm', 'urdf', 'arm.urdf')
        rviz_config = os.path.join(URDF_DIR, 'arm', 'rviz', 'arm.rviz')

    with open(urdf_file, 'r') as f:
        robot_desc = attach_to_world(f.read().replace(URDF_MESH_PREFIX, 'file://' + PROJECT_DIR))

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
        if robot in ('arm', 'dual'):      # arm_simulator only moves the arm
            nodes.append(ExecuteProcess(
                cmd=['python3', os.path.join(URDF_DIR, 'unnamed', 'arm_simulator.py')],
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
                'RVIZ_CONFIG': rviz_config,
                'ROBOT_PROJECT_DIR': PROJECT_DIR
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
            description='Target robot: arm (Robotic Arm), quard (Sesame Quadruped), dogu (12-DOF quadruped) or dual'
        ),
        OpaqueFunction(function=launch_setup)
    ])
