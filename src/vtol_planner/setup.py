from setuptools import find_packages, setup

from glob import glob

import os


package_name = 'vtol_planner'


setup(

    name=package_name,

    version='0.0.0',

    packages=find_packages(

        exclude=['test']

    ),

    data_files=[

        # ==================================================
        #            AMENT PACKAGE INDEX
        # ==================================================

        (

            'share/ament_index/resource_index/packages',

            ['resource/' + package_name]

        ),

        # ==================================================
        #                PACKAGE.XML
        # ==================================================

        (

            'share/' + package_name,

            ['package.xml']

        ),

        # ==================================================
        #                LAUNCH FILES
        # ==================================================

        (

            os.path.join(

                'share',
                package_name,
                'launch'

            ),

            glob('launch/*.py')

        ),

        # ==================================================
        #                 RVIZ FILES
        # ==================================================

        (

            os.path.join(

                'share',
                package_name,
                'rviz'

            ),

            glob('rviz/*.rviz')

        ),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='asambhav',

    maintainer_email='sambhava06@gmail.com',

    description='VTOL Planner Package',

    license='Apache License 2.0',

    extras_require={

        'test': [

            'pytest',

        ],

    },

    entry_points={

        'console_scripts': [

            # ==============================================
            #               PLANNER NODE
            # ==============================================

            'planner_node = '

            'vtol_planner.planner_node:main',

            # ==============================================
            #         TRAJECTORY VISUALIZER
            # ==============================================

            'trajectory_visualizer = '

            'vtol_planner.trajectory_visualizer:main',

        ],

    },

)