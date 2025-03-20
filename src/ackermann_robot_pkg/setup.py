from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ackermann_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        # Include world files
        (os.path.join('share', package_name, 'worlds'),
            glob(os.path.join('worlds', '*.sdf'))),
        # Include model files
        (os.path.join('share', package_name, 'models', 'ackermann_robot'),
            glob(os.path.join('models', 'ackermann_robot', '*'))),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mw',
    maintainer_email='marc.wester@gmail.com',
    description='Gazebo test for the ackermann robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'control_ackermann = ackermann_robot_pkg.control_ackermann:main',
            'auto_mode_toggle = ackermann_robot_pkg.auto_mode_toggle:main',
        ],
    },
)
