from glob import glob
import os

from setuptools import setup

package_name = 'nav2_api_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_nav_to_pose = nav2_api_examples.example_nav_to_pose:main',
            'example_nav_through_poses = nav2_api_examples.example_nav_through_poses:main',
            'example_waypoint_follower = nav2_api_examples.example_waypoint_follower:main',
            'example_follow_path = nav2_api_examples.example_follow_path:main',
            'demo_picking = nav2_api_examples.demo_picking:main',
            'demo_inspection = nav2_api_examples.demo_inspection:main',
            'demo_security = nav2_api_examples.demo_security:main',
            'demo_recoveries = nav2_api_examples.demo_recoveries:main',
            'example_assisted_teleop = nav2_api_examples.example_assisted_teleop:main',
        ],
    },
)
