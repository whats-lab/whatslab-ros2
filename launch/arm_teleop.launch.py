from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """팔+손 텔레옵 (whatslab GloveModel) → /arm/joint_trajectory + /joint_states."""
    args = [
        DeclareLaunchArgument('rig', default_value='rigs/nero_orca_right.yaml',
                              description='whatslab rig (whatslab configs 기준 상대경로)'),
        DeclareLaunchArgument('hand_retarget', default_value='orca_hand'),
        DeclareLaunchArgument('controller_side', default_value='left',
                              description='Quest 컨트롤러 좌/우'),
        DeclareLaunchArgument('hand_side', default_value='right',
                              description='글러브 좌/우'),
    ]
    return LaunchDescription(args + [
        Node(
            package='atlas_hand',
            executable='atlas_arm_teleop',
            name='atlas_arm_teleop',
            output='screen',
            parameters=[{
                'rig': LaunchConfiguration('rig'),
                'hand_retarget': LaunchConfiguration('hand_retarget'),
                'controller_side': LaunchConfiguration('controller_side'),
                'hand_side': LaunchConfiguration('hand_side'),
            }],
        ),
    ])
