from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0',
                              description='Arduino serial device'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('safety_threshold_cm', default_value='-1.0',
                              description='<0 keeps firmware default (20 cm), 0 disables'),
        DeclareLaunchArgument('initial_mode', default_value='',
                              description="'', 'GUI' or 'SENSOR'"),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description='Also start foxglove_bridge'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

        Node(
            package='tsp05_bridge',
            executable='tsp05_serial_node',
            name='tsp05_serial_node',
            output='screen',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
                'safety_threshold_cm': ParameterValue(
                    LaunchConfiguration('safety_threshold_cm'), value_type=float),
                'initial_mode': ParameterValue(
                    LaunchConfiguration('initial_mode'), value_type=str),
            }],
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'), 'launch',
                'foxglove_bridge_launch.xml'])),
            launch_arguments={'port': LaunchConfiguration('foxglove_port')}.items(),
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
