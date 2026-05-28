from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'openarm_quest_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
        ('share/' + package_name + '/web',
            glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='air-lab-ncsu',
    maintainer_email='air-lab-ncsu@todo.todo',
    description='OpenArm Quest 3 bimanual teleoperation',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quest_bridge = openarm_quest_teleop.quest_bridge:main',
        ],
    },
)
