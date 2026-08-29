import os

from glob import glob
from setuptools import find_packages, setup


package_name = 'mission_formalism_evaluation'


setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [
                'resource/' + package_name,
            ],
        ),

        (
            'share/' + package_name,
            [
                'package.xml',
            ],
        ),

        (
            os.path.join(
                'share',
                package_name,
                'missions',
            ),
            glob('missions/*.yaml'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'missions',
                'debug',
            ),
            glob('missions/debug/*.yaml'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'scenarios',
            ),
            glob('scenarios/*.yaml'),
        ),

        (
            os.path.join(
                'share',
                package_name,
                'experiments',
            ),
            glob('experiments/*.yaml'),
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='bishal',
    maintainer_email='bishal@example.com',

    description=(
        'Evaluation of autonomous UAV mission '
        'implementation formalisms'
    ),

    license='Apache-2.0',

    tests_require=[
        'pytest',
    ],

    entry_points={
        'console_scripts': [

            'waypoint_executor = '
            'mission_formalism_evaluation.'
            'waypoint.waypoint_executor:main',

            'fsm_executor = '
            'mission_formalism_evaluation.'
            'fsm.fsm_executor:main',
            
            'bt_executor = '
            'mission_formalism_evaluation.'
            'bt.bt_executor:main',
            
            'htn_executor = '
            'mission_formalism_evaluation.'
            'htn.htn_executor:main',

            'scenario_injector = '
            'mission_formalism_evaluation.'
            'experiment.scenario_injector:main',
            
            'gazebo_executor = '
            'mission_formalism_evaluation.'
            'simulation.gazebo_executor:main',
            
            
        ],
    },
)

