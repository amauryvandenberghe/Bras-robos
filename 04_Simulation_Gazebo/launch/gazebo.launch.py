import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_bras = get_package_share_directory('mon_bras_robot')
    pkg_share = os.path.dirname(pkg_bras)

    set_ign_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=f'{pkg_share}:{os.path.expanduser("~/ros2_ws/src")}')
    set_gz_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=f'{pkg_share}:{os.path.expanduser("~/ros2_ws/src")}')

    # robot:=Assemblage1 (defaut, bras actuel) ou robot:=bras_50cm (bras long sur le robot roulant)
    arg_robot = DeclareLaunchArgument('robot', default_value='Assemblage1', description='Fichier de urdf/ sans .urdf')

    def publier_robot(context):
        with open(os.path.join(pkg_bras, 'urdf', LaunchConfiguration('robot').perform(context) + '.urdf')) as f:
            robot_desc = f.read()
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        )]

    rsp = OpaqueFunction(function=publier_robot)

    # gui:=true (defaut) : fenetre 3D ; gui:=false : physique seule, sans affichage (beaucoup moins de processeur)
    arg_gui = DeclareLaunchArgument('gui', default_value='true', description='Afficher la fenetre 3D de Gazebo')
    # monde:=scene_balle (defaut, une balle et un bac) ou monde:=scene_deux_coupes (deux balles, deux coupes perchees)
    arg_monde = DeclareLaunchArgument('monde', default_value='scene_balle', description='Fichier de worlds/ sans .world')
    chemin_monde = [os.path.join(os.path.expanduser('~/ros2_ws/src/mon_bras_robot/worlds'), ''),
                    LaunchConfiguration('monde'), '.world']
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={'gz_args': ['-r '] + chemin_monde}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )
    gazebo_sans_fenetre = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={'gz_args': ['-r -s '] + chemin_monde}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mon_bras_robot', '-z', '0.0'],
        output='screen'
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        arg_gui, arg_monde, arg_robot, set_ign_path, set_gz_path, rsp, gazebo, gazebo_sans_fenetre, spawn_robot, jsb_spawner,
        arm_spawner
    ])
