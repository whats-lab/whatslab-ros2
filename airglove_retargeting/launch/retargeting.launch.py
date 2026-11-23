from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# driver.launch.py 로 그대로 넘기는 인자 (pair 는 따로 선언)
DRIVER_DEFAULTS = (
    ('listen_port', '4040'),
    ('spine_port', '4042'),
    ('publish_hz', '120.0'),
    ('haptics_timeout', '1.0'),
)


def generate_launch_description() -> LaunchDescription:
    """AirGlove → GeoRT 손 리타게팅 → /joint_states.

    driver:=true 면 이 PC 에서 airglove_driver 도 띄운다(Spine 과 같은 PC). 드라이버가 다른 PC 에
    있으면 driver:=false 로 리타게팅만 띄운다.
    """
    args = [
        DeclareLaunchArgument(
            'hand_type', default_value='right', description="처리할 손: 'left' | 'right' | 'both'"
        ),
        DeclareLaunchArgument(
            'retarget',
            default_value='geort:orca_right',
            description=(
                '손 리타게팅: geort:<설정> (GeoRT) | dex:<손> (DexPilot, whatslab-sdk[hand])'
            ),
        ),
        DeclareLaunchArgument(
            'robot_urdf',
            default_value='orca_hand/urdf/{side}.urdf',
            description='0-fill 발행용 URDF (dexhand_description 기준, {side} = left/right)',
        ),
        DeclareLaunchArgument(
            'input_namespace',
            default_value='/airglove',
            description='{ns}/{side}/joints 를 구독 (다른 HandJoints 소스면 바꾼다)',
        ),
        DeclareLaunchArgument(
            'driver', default_value='true', description='airglove_driver 도 띄울지'
        ),
        DeclareLaunchArgument(
            'pair', default_value='1', description='Spine pair (driver:=true 일 때)'
        ),
        *[
            DeclareLaunchArgument(name, default_value=default, description='driver:=true 일 때')
            for name, default in DRIVER_DEFAULTS
        ],
    ]
    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('airglove_driver'), 'launch', 'driver.launch.py']
            )
        ),
        launch_arguments={
            name: LaunchConfiguration(name) for name in ['pair', *dict(DRIVER_DEFAULTS)]
        }.items(),
        condition=IfCondition(LaunchConfiguration('driver')),
    )
    return LaunchDescription(
        args
        + [
            driver,
            Node(
                package='airglove_retargeting',
                executable='airglove_retargeting',
                name='airglove_retargeting',
                output='screen',
                parameters=[
                    {
                        'hand_type': LaunchConfiguration('hand_type'),
                        'retarget': LaunchConfiguration('retarget'),
                        'robot_urdf': LaunchConfiguration('robot_urdf'),
                        'input_namespace': LaunchConfiguration('input_namespace'),
                    }
                ],
            ),
        ]
    )
