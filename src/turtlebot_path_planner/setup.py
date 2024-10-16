from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot_path_planner'

setup(
    name=package_name,
    version='1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='Khang.Nguyen@skoltech.ru',
    description='Turtlebot path planner with multiple trajectories',
    license='Abc',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_planner = turtlebot_path_planner.path_planner:main',
        ],
    },
)
