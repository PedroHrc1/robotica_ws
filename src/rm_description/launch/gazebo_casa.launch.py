import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Nome do pacote que contém o robô e o mundo
    pkg_name = 'rm_description'
    pkg_share = get_package_share_directory(pkg_name)

    # Caminho para o arquivo do mundo
    world_file = os.path.join(pkg_share, 'world', 'casa.sdf')

    # Processar o arquivo XACRO para obter o URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = doc.toxml()

    # 1. Lançar o Gazebo Sim com o mundo da casa
    #    Utiliza a launch file do pacote ros_gz_sim
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

    # 2. Publicar a descrição do robô no tópico /robot_description
    #    Necessário para que o Gazebo consiga ler o modelo do robô
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # 3. Inserir o robô no mundo já carregado
    #    Lê o modelo do robô a partir do tópico /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'meu_robo',
            '-topic', 'robot_description',
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.1',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])