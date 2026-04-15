import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'surface_wiping_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sihan',
    maintainer_email='sihandong14@gmail.com',
    description='Nodes for the surface wiping assignment',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_setup = surface_wiping_nodes.scene_setup:main',
            'surface_ik_server = surface_wiping_nodes.surface_ik_server:main',
            'reachability_map = surface_wiping_nodes.reachability_map:main',
            'coverage_planner = surface_wiping_nodes.coverage_planner:main',
            'contact_wiping_controller = surface_wiping_nodes.contact_wiping_controller:main',
            'trajectory_playback = surface_wiping_nodes.trajectory_playback:main',
        ],
    },
)
