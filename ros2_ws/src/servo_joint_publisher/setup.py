from setuptools import find_packages, setup

package_name = 'servo_joint_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='ROS 2 Developer',
    maintainer_email='user@todo.todo',
    description='ROS 2 node reading microcontroller servo angle via USB Serial and publishing JointState',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_serial_publisher = servo_joint_publisher.servo_serial_publisher:main',
            'servo_simulator = servo_joint_publisher.servo_simulator:main',
            'servo_serial_commander = servo_joint_publisher.servo_serial_commander:main',
            'calibrated_joint_gui = servo_joint_publisher.calibrated_joint_gui:main',
            'gesture_teleop_node = servo_joint_publisher.gesture_teleop_node:main',
            'unified_robot_commander = servo_joint_publisher.unified_robot_commander:main',
        ],
    },
)
