from setuptools import find_packages, setup
import glob
import sys
import os
from glob import glob

package_name = 'hand_cmd_pub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agilex',
    maintainer_email='agilex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'hand_cmd = hand_cmd_pub.hand_cmd:main',
        'multi_hand_cmd = hand_cmd_pub.multi_hand_cmd:main',

        ],
    },
)
