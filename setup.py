from setuptools import setup

package_name = 'pct_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kushal Agarwal',
    maintainer_email='kushalagarwal444@gmail.com',
    description='ROS2 compatible package for PCT planner',
    license='Apache License 2.0',
    tests_require=['pytest']
)