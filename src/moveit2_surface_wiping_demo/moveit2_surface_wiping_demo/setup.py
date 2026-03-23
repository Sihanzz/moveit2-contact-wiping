from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'moveit2_surface_wiping_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sihan',
    maintainer_email='sihandong14@gmail.com',
    description='Surface wiping demo with MoveIt 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_setup = moveit2_surface_wiping_demo.scene_setup:main',
            'ik_service = moveit2_surface_wiping_demo.ik_service:main',
            'reachability_map = moveit2_surface_wiping_demo.reachability_map:main',
            'coverage_planner = moveit2_surface_wiping_demo.coverage_planner:main',
            'wiping_controller = moveit2_surface_wiping_demo.wiping_controller:main',
            'wiping_visualization = moveit2_surface_wiping_demo.wiping_visualization:main',
            'plot_wiping_logs = moveit2_surface_wiping_demo.plot_wiping_logs:main',
        ],
    },

)
