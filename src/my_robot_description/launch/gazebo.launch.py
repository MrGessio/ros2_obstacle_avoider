from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() :
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')

    return LaunchDescription([
            ExecuteProcess(
                cmd = ['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
                output = 'screen'
            ),

            Node(
                package = 'robot_state_publisher',
                executable = 'robot_state_publisher',
                name = 'robot_state_publisher',
                output = 'screen',
                parameters = [{
            'robot_description': Command(['xacro', xacro_file])
        }]
            ),

            Node(
                package = 'gazebo_ros',
                executable = 'spawn_entity.py',
                arguments = [
                    '-entity', 'my_robot',
                        '-topic', 'robot_description'
                ],
                output = 'screen'
            )
    ])

