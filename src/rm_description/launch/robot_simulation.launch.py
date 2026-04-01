import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    world_file_name = 'casa_mobiliada.sdf'
    xacro_file = "robot.urdf.xacro"
    rviz_config_file = "robot_vis.rviz"
    description_package_name = "rm_description"
    description_package_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_file_path = os.path.join(description_package_path, 'urdf', xacro_file)

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    world_file = os.path.join(
        get_package_share_directory(description_package_name),
        'world',
        world_file_name
    )
    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # Runs the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_config_file)
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
        )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.1',
        ],
        output='screen',
    )
    
    bridge_params = os.path.join(get_package_share_directory(description_package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz_node,
        spawn_entity,
        ros_gz_bridge,
        ros_gz_image_bridge,
    ])