from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """단일 래퍼 노드 실행 (수신+리타게팅+발행). OSC 포트는 한 번만 bind 되므로
    노드는 하나만 띄우고 hand_type='both' 로 양손을 함께 처리한다."""

    hand_type_arg = DeclareLaunchArgument(
        'hand_type',
        default_value='both',
        description="처리할 손: 'left' | 'right' | 'both'",
    )
    robot_config_arg = DeclareLaunchArgument(
        'robot_config',
        default_value='base_hand',
        description='로봇 손 설정 (whatslab CONFIG_REGISTRY 키)',
    )
    input_source_arg = DeclareLaunchArgument(
        'input_source',
        default_value='atlas',
        description="입력 소스: 'atlas'(AirGlove) | 'meta_quest'(Quest)",
    )

    return LaunchDescription([
        hand_type_arg,
        robot_config_arg,
        input_source_arg,
        Node(
            package='atlas_hand',
            executable='atlas_hand',
            name='atlas_hand',
            output='screen',
            parameters=[{
                'input_source': LaunchConfiguration('input_source'),
                'hand_type':    LaunchConfiguration('hand_type'),
                'robot_config': LaunchConfiguration('robot_config'),
            }],
        ),
    ])
