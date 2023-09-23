import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    package = "microsoft7scenes"

    ####### DATA INPUT END ##########

    dataset_params = os.path.join(
        get_package_share_directory(package),
        'params',
        'params.yaml'
        )

    dataset_node = Node(
            package=package,
            executable='seven_scenes_node',
            output='screen',
            name='seven_scenes_node',
            parameters=[dataset_params])

    # create and return launch description object
    return LaunchDescription(
        [            
            dataset_node
        ]
    )