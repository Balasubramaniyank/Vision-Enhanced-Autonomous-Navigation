from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    navigation_params_path_1=os.path.join(
        get_package_share_directory('autonomous_navigation'),
        'config',
        'navigation_2.yaml'
    )

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                
                'use_sim_time':'true',
                'params_file': navigation_params_path_1,
                # 'slam': 'True',

            }.items()

        )

    ])        