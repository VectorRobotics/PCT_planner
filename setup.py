from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'pct_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('pct_planner/planner/config/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kushal Agarwal',
    maintainer_email='kushalagarwal444@gmail.com',
    description='ROS2 compatible package for PCT planner',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pct_planner_node.py = pct_planner.scripts.pct_planner_node:main',
            'pct_planner_visualizer.py = pct_planner.scripts.pct_planner_visualizer:main',
            'pcd_to_tomogram.py = pct_planner.scripts.pcd_to_tomogram:main',
        ],
    },
)