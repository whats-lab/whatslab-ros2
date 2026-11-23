import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'airglove_retargeting'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whatslab',
    maintainer_email='sdh@whatslab.co.kr',
    description='AirGlove hand retargeting (whatslab GeoRT) + safety gate',
    license='CC BY-NC-ND 4.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airglove_retargeting = airglove_retargeting.retargeting_node:main',
            'safety_gate = airglove_retargeting.safety_gate_node:main',
        ],
    },
)
