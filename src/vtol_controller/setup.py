"""from setuptools import find_packages, setup

package_name = 'vtol_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asambhav',
    maintainer_email='sambhava06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)"""
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vtol_controller'

setup(
    name=package_name,

    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[

        # ======================================================
        #                AMENT PACKAGE INDEX
        # ======================================================

        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # ======================================================
        #                    PACKAGE XML
        # ======================================================

        (
            'share/' + package_name,
            ['package.xml']
        ),

        # ======================================================
        #                   CONFIG FILES
        # ======================================================

        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),

        # ======================================================
        #                  LAUNCH FILES
        # ======================================================

        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=['setuptools','numpy','scipy'],

    zip_safe=True,

    maintainer='asambhav',

    maintainer_email='sambhava06@gmail.com',

    description='VTOL controller package for ROS2 and Gazebo simulation',

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'thrust_allocator = vtol_controller.thrust_allocator:main',
            # ==================================================
            #            FUTURE FLIGHT CONTROLLER
            # ==================================================

            # Example:
            #
            'controller_node = vtol_controller.controller_node:main',

        ],
    },
)