import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    pkg_share = pkg

    xacro_file = os.path.join(pkg, 'robot_description', 'robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    rviz_config_file = os.path.join(pkg, 'rviz', 'my_robot.rviz')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', os.path.join(pkg, 'worlds', 'maze_world.sdf')],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'publish_frequency': 50.0,
            'ignore_timestamp': True,
        }]
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name',  'my_robot',
            '-x', '-8.0',
            '-y', '-8.0',
            '-z', '0.1',
            '-Y', '0.0',
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='main_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/my_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/maze_world/model/my_robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/model/my_robot/odometry', '/odom'),
            ('/world/maze_world/model/my_robot/joint_state', '/joint_states'),
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(pkg, 'config', 'mapper_params_online_async.yaml'),
            {'use_sim_time': True}
        ]
    )

    global_costmap_node = Node(
        package='my_robot_description',
        executable='global_costmap',
        name='global_costmap',
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['slam_toolbox'],
            'bond_timeout': 4.0,
        }]
    )


    human_detector = Node(
        package='my_robot_description',
        executable='human_node',
        name='human_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    human_fusion = Node(
        package='my_robot_description',
        executable='human_fusion',
        name='human_fusion',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    social_costmap = Node(
        package='my_robot_description',
        executable='social_costmap',
        name='social_costmap',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            spawn,
            bridge,
            human_detector,   
            human_fusion,     
            social_costmap,   
        ]
    )

    delayed_nodes_2 = TimerAction(
        period=3.0,
        actions=[ekf_node, rviz, slam_toolbox, global_costmap_node, lifecycle_manager]
    )

    return LaunchDescription([
        gz_sim,         # 1. Gazebo first
        clock_bridge,   # 2. /clock bridge immediately — RSP needs this
        rsp,            # 3. RSP immediately — now has clock, ignore_timestamp keeps it alive
        delayed_nodes_2,  
        delayed_nodes,  # 4. Everything else after 3 s
    ])