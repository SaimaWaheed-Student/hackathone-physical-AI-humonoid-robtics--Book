from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # Add config files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS 2 package for Nav2 configuration for the humanoid robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_bringup_node = humanoid_nav2.nav2_bringup_node:main', # Placeholder
        ],
    },
)
