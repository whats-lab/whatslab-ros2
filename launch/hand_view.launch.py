import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from whatslab.teleop.hand.hand_configs import CONFIG_REGISTRY


def _launch_setup(context):
    model = context.launch_configurations['model']
    side  = context.launch_configurations['side']
    # gui=false: joint_state_publisher_gui 를 띄우지 않는다 — 라이브 텔레옵(외부에서
    # /joint_states 발행) 시 GUI 슬라이더 발행과 충돌하므로 뷰어만 필요할 때 끈다.
    gui   = context.launch_configurations.get('gui', 'true').lower() != 'false'
    desc_share = get_package_share_directory('dexhand_description')

    # dexhand_description 패키지(URDF/메쉬/rviz)를 자산 소스로 사용 (atlas 는 로직만)
    config    = CONFIG_REGISTRY[model](urdf_root=desc_share)
    urdf_path = config._get_urdf_path(side)

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF 파일을 찾을 수 없습니다: {urdf_path}")

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
    ]
    if gui:                                  # 수동 슬라이더 뷰 (텔레옵과 함께 쓰면 gui:=false)
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ))

    rviz_dir  = os.path.join(desc_share, config._MODEL_SUBDIR, 'rviz')
    rviz_args = []
    # _RVIZ_FILENAME 명시 → 해당 파일 사용
    # 미설정 → {model}_{side}.rviz → {model}.rviz 순으로 자동 탐색
    explicit = config._RVIZ_FILENAME.get(side, '')
    candidates = (
        [explicit] if explicit
        else [f'{config._MODEL_SUBDIR}_{side}.rviz', f'{config._MODEL_SUBDIR}.rviz']
    )
    for name in candidates:
        path = os.path.join(rviz_dir, name)
        if os.path.exists(path):
            rviz_args = ['-d', path]
            break

    nodes.append(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=rviz_args,
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='base_hand',
            choices=list(CONFIG_REGISTRY.keys()),
            description='Hand model: ' + ' | '.join(CONFIG_REGISTRY.keys()),
        ),
        DeclareLaunchArgument(
            'side',
            default_value='left',
            choices=['left', 'right'],
            description='Hand side (left / right)',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            choices=['true', 'false'],
            description='joint_state_publisher_gui 슬라이더. 라이브 텔레옵 시 false.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
