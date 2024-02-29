import os
# import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('my_simulations'),'worlds','Arena_1.world'),
        description='Full path to new world.'
    )

    ign_gazebo_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')])
    )    

    return LaunchDescription([
        # world_arg,
        ign_gazebo_arg
    ])
