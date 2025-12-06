from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), glob('launch/*.py')), # Placeholder for launch files
        (os.path.join('share', package_name, 'plugins'), glob('plugins/*.xml')), # For plugin description XML
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Custom path planner plugin for Nav2 suitable for bipedal locomotion',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_planner_node = humanoid_planner.humanoid_planner_node:main', # Placeholder
        ],
    },
)
