import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare arguments
    # use_sim_time should be 'true' if running in Gazebo
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 2. Define the Hybrid A* Planner Node
    # This node listens to /map and /goal_pose to generate a path
    planner_node = Node(
        package='hybrid_astar_planner',
        executable='planner_node',
        name='hybrid_astar_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Define the Pure Pursuit Controller Node
    # This node listens to the /planned_path and sends /cmd_vel to the robot
    controller_node = Node(
        package='hybrid_astar_planner',
        executable='teb_controller',
        name='teb_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 4. Return the LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        planner_node,
        controller_node
    ])