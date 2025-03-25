import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'task_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch','*launch.py'))),
        # first argument specifies where will the files be installed. In this case, the destination is share/package_name(task_1)/launch
        # second argument specifies what files will be installed. In this case, it will find all launch.py files in the package's /launch directory
        # (os.path.join('share','task_1','launch'),glob(os.path.join('launch','*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='albert',
    maintainer_email='fualbert20030507@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = task_1.time_recorder_pub:main',   # the name 'talker' does not need to match the name from launch file or the source code's node names
            'listener = task_1.doubler_sub:main'        # it is the name when we run the command "ros2 run task_1 talker" 
        ],
    },
)
