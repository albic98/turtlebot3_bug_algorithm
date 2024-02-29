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

    gazebo_arg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world':world}.items()
    )    

    launch_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'turtlebot3_state_publisher.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'spawn_turtlebot3.launch.py')]),
    )

    return LaunchDescription([
        world_arg,
        launch_turtlebot3,
        gazebo_arg,
        spawn_turtlebot3,
    ])
