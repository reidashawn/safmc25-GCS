from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        where='.',  # Search for packages in the current directory
        include=['controller', 'controller.helpers', 'controller.madgwick_py']  # Ensure helpers is included
    ),
    data_files=[  # Make sure the resource files and launch files are included
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # ✅ Add this to include launch files
    ],
    install_requires=['setuptools','rclpy', 'std_msgs', 'sensor_msgs', 'geometry_msgs', 'std_srvs'],
    zip_safe=True,
    maintainer='sleepyyrei',
    maintainer_email='sleepyyrei@todo.todo',  # Replace with your email
    description='This package provides control logic for robot systems.',  # Provide a real description
    license='MIT',  # Replace with the license you're using
    tests_require=['pytest'],
    entry_points={  # Specify ROS 2 node scripts here
        'console_scripts': [
            'controller_pub = controller.controller_pub:main', 
            'imu_converter = controller.imu_converter:main',
            'pot_converter = controller.pot_converter:main',
            'button_manager = controller.button_manager:main',
            'drone_movement = controller.drone_movement:main',  # Adjust the node entry point accordingly
        ],
    },
)
