import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from airglove_retargeting.assets import (
    absolute_mesh_uris,
    description_env,
    resolve_urdf,
    root_link,
)


def _launch_setup(context):
    config = context.launch_configurations
    side = config['side']
    spec = config['urdf'] or config['model']
    urdf_path = resolve_urdf(spec, side)
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(
            f'URDF 를 찾을 수 없습니다: {spec} ({side}) -> {urdf_path!r}. urdf:=<경로> 로 지정'
        )

    with open(urdf_path, 'r') as f:
        robot_desc = absolute_mesh_uris(f.read(), os.path.dirname(urdf_path))

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
    ]
    if config['gui'] == 'true':
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
            )
        )
    if config['rviz'] == 'true':
        rviz_config = os.path.join(
            get_package_share_directory('airglove_retargeting'), 'rviz', 'hand_view.rviz'
        )
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config, '-f', root_link(robot_desc)],
                additional_env=description_env(),
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='orca_hand',
                description='dexhand_description 의 손 폴더 이름 (orca_hand, allegro_hand, ...)',
            ),
            DeclareLaunchArgument(
                'side',
                default_value='right',
                choices=['left', 'right'],
                description='Hand side (left / right)',
            ),
            DeclareLaunchArgument(
                'urdf',
                default_value='',
                description='URDF 경로 (절대 또는 dexhand_description 기준, {side} 치환)',
            ),
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                choices=['true', 'false'],
                description='joint_state_publisher_gui 슬라이더. 라이브 텔레옵 시 false.',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                choices=['true', 'false'],
                description='rviz2 실행 (디스플레이가 없으면 false)',
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
