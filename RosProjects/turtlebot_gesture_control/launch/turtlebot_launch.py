#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get TurtleBot3 package share directory
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # Include the standard TurtleBot3 world launch file
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    
    # Add hand gesture detector node
    hand_gesture_node = Node(
        package='turtlebot',
        executable='hand_gesture_detector.py',
        name='hand_gesture_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Launch TurtleBot3 in Gazebo world
        turtlebot3_world,
        
        # Add hand gesture control
        hand_gesture_node,
    ])


if __name__ == '__main__':
    generate_launch_description()