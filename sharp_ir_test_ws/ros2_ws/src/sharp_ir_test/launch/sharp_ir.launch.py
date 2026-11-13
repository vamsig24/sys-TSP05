from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino Uno serial device'),
        DeclareLaunchArgument('baud', default_value='9600',
                              description='Arduino serial baud rate'),
        Node(
            package='sharp_ir_test',
            executable='sharp_ir_serial_node',
            name='sharp_ir_serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }],
        ),
    ])
