from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'atlas_hand'

# URDF/메쉬/rviz 는 dexhand_description 패키지가 설치한다 (ament depend). atlas 는
# 로직만 — models 자산을 더 이상 직접 설치하지 않는다.

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'scripts', 'standalone']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whatslab',
    maintainer_email='sdh@whatslab.co.kr',
    description='Atlas Hand Retargeting - ROS 2 Python Package',
    license='CC BY-NC-ND 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 손 전용: 수신(whatslab receiver) → 리타게팅(whatslab) → /joint_states
            'atlas_hand = atlas_hand.retargeting_node:main',
            # 팔+손: whatslab GloveModel → /arm/joint_trajectory + /joint_states
            'atlas_arm_teleop = atlas_hand.arm_teleop_node:main',
            # 안전 게이트: command_raw → whatslab.safety(clamp·rate·watchdog·estop) → command_safe
            'safety_gate = atlas_hand.safety_gate_node:main',
        ],
    },
)
