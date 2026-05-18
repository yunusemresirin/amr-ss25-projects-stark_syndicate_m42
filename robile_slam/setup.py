from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'robile_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install package.xml file
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        # Install map files
        (os.path.join('share', 'robile_navigation', 'maps'),
         glob(os.path.join('maps', '*'))),

         # Install ekf configuration files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner_node = robile_slam.global_planner_node:main',
            'waypoint_follower_node = robile_slam.waypoint_follower_node:main',
            'potential_field_node = robile_slam.potential_field_node:main',
            'particle_filter_node = robile_slam.monte_carlo_pf_node:main',
            'nav_stack = robile_slam.navigation_stack:main',
            'mcl = robile_slam.mcl:main',
            # 'astar_explorer = robile_slam.astar_explore:main',
            # 'random_goal = robile_slam.random_goal:main',
            'exploration_node = robile_slam.exploration_node:main',
            'updated_global_planner = robile_slam.updated_global_planner:main',
        ],
    },
)
