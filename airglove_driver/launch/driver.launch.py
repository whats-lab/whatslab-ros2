from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument('pair', default_value='1', description='Spine pair'),
        DeclareLaunchArgument('listen_port', default_value='4040'),
        DeclareLaunchArgument('spine_port', default_value='4042'),
        DeclareLaunchArgument('publish_hz', default_value='120.0'),
        DeclareLaunchArgument(
            'haptics_timeout',
            default_value='1.0',
            description='진동 중 명령이 없으면 끄는 시간 [s], 0 = 끔',
        ),
    ]
    return LaunchDescription(
        args
        + [
            Node(
                package='airglove_driver',
                executable='airglove_driver',
                name='airglove_driver',
                output='screen',
                parameters=[
                    {
                        'pair': LaunchConfiguration('pair'),
                        'listen_port': LaunchConfiguration('listen_port'),
                        'spine_port': LaunchConfiguration('spine_port'),
                        'publish_hz': LaunchConfiguration('publish_hz'),
                        'haptics_timeout': LaunchConfiguration('haptics_timeout'),
                    }
                ],
            ),
        ]
    )
