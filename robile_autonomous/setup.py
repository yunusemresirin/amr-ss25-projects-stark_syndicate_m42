import os
from setuptools import setup
from glob import glob

package_name = 'robile_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], # This line is critical! It tells setuptools to find the 'robile_autonomous' (inner) directory as a Python package.
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ensure your config and launch files are also installed correctly
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name', # Replace with your name
    maintainer_email='your.email@example.com', # Replace with your email
    description='ROS2 package for robot autonomous navigation.',
    license='TODO: License declaration', # Choose a license, e.g., 'Apache-2.0'
    tests_require=['pytest'],
    # --- THIS 'entry_points' SECTION IS THE KEY TO EXECUTABLE SCRIPTS ---
    entry_points={
        'console_scripts': [
            # Each line defines an executable:
            # 'executable_command_name = python_module_path:main_function'
            # The 'python_module_path' is your_package_name.your_script_file_name_without_py_extension
            'A_star_planner = robile_autonomous.A_star_planner:main',
            'navigation_controller = robile_autonomous.navigation_controller:main',
            'potential_field_planner = robile_autonomous.potential_field_planner:main',
        ],
    },
)
