from setuptools import setup

package_name = 'vtol_plant'

setup(

    name=package_name,

    version='0.0.1',

    packages=[package_name],

    install_requires=[

        'setuptools',
        'numpy'

    ],

    zip_safe=True,

    maintainer='asambhav',

    maintainer_email='sambhava06@gmail.com',

    description='Nonlinear VTOL UAV Plant Model',

    license='Apache-2.0',

    tests_require=['pytest'],

    data_files=[

        (

            'share/ament_index/resource_index/packages',

            ['resource/' + package_name]

        ),

        (

            'share/' + package_name,

            ['package.xml']

        ),

    ],

    entry_points={

        'console_scripts': [

            'plant_node = vtol_plant.plant_node:main',
            'state_visualizer = vtol_plant.state_visualizer:main',

        ],

    },

)