import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from airglove_retargeting.assets import description_env, description_share

# 모델 → (dexhand_description 하위 디렉토리, URDF 상대경로 규칙, rviz 파일 후보).
# rviz 후보가 없으면 rviz/{subdir}_{side}.rviz → rviz/{subdir}.rviz 순으로 찾는다.
HAND_MODELS = {
    'base_hand': ('base_hand', 'urdf/{side}.urdf', ['base_{side}.rviz']),
    'orca_hand': ('orca_hand', 'urdf/{side}.urdf', ['orca_hand.rviz']),
    'allegro_hand': ('allegro_hand', 'allegro_hand_{side}.urdf', ['allegro_hand.rviz']),
    'tesollo_dg5f': ('tesollo_dg5f', 'dg5f_{side}.urdf', ['tesollo_dg5f_{side}.rviz']),
    'robotis_hx5_d20': (
        'robotis_hx5_d20',
        'urdf/hx5_d20_{side}.urdf',
        ['robotis_hx5_d20_{side}.rviz', 'robotis.rviz'],
    ),
    'leap_hand': ('leap_hand', 'leap_hand_{side}.urdf', []),
    'ability_hand': ('ability_hand', 'ability_hand_{side}.urdf', []),
    'schunk_hand': ('schunk_hand', 'schunk_svh_hand_{side}.urdf', []),
}


def _launch_setup(context):
    model = context.launch_configurations['model']
    side = context.launch_configurations['side']
    # gui=false: joint_state_publisher_gui 를 띄우지 않는다 — 라이브 텔레옵(외부에서
    # /joint_states 발행) 시 GUI 슬라이더 발행과 충돌하므로 뷰어만 필요할 때 끈다.
    gui = context.launch_configurations.get('gui', 'true').lower() != 'false'
    desc_share = description_share()
    if not desc_share:
        raise FileNotFoundError(
            'dexhand_description 이 없습니다 (colcon 패키지나 pip dexhand-description)'
        )

    # dexhand_description 패키지(URDF/메쉬/rviz)를 자산 소스로 사용 (이 패키지는 로직만)
    subdir, urdf_rule, rviz_names = HAND_MODELS[model]
    urdf_path = os.path.join(desc_share, subdir, urdf_rule.format(side=side))

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'URDF 파일을 찾을 수 없습니다: {urdf_path}')

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
    if gui:  # 수동 슬라이더 뷰 (텔레옵과 함께 쓰면 gui:=false)
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
            )
        )

    rviz_dir = os.path.join(desc_share, subdir, 'rviz')
    rviz_args = []
    candidates = [n.format(side=side) for n in rviz_names]
    candidates += [f'{subdir}_{side}.rviz', f'{subdir}.rviz']
    for name in candidates:
        path = os.path.join(rviz_dir, name)
        if os.path.exists(path):
            rviz_args = ['-d', path]
            break

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=rviz_args,
            additional_env=description_env(),  # pip 설치일 때 메쉬 package:// 해석용
        )
    )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'model',
                default_value='orca_hand',
                choices=list(HAND_MODELS.keys()),
                description='Hand model: ' + ' | '.join(HAND_MODELS.keys()),
            ),
            DeclareLaunchArgument(
                'side',
                default_value='right',
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
        ]
    )
