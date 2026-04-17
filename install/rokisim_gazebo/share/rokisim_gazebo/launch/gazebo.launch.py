import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rokisim = get_package_share_directory('rokisim_gazebo')
    
    urdf_file = os.path.join(pkg_rokisim, 'urdf', 'simple_arm.urdf')
    world_file = os.path.join(pkg_rokisim, 'worlds', 'laboratuvar.world')

    # URDF Dosyasını Oku
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 1. Gazebo'yu Laboratuvar Dünyasıyla Başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. EKSİK OLAN PARÇA: Robot State Publisher (Robotun DNA'sını yayınlar)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Robotu Gazebo'ya Topic Üzerinden Doğur
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'rokisim_basit_kol'],
        output='screen'
    )

    # 4. Broadcaster (Robotun anlık durumunu okur)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 5. Controller (Motorlara güç verir)
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller
    ])