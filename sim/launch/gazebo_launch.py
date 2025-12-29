from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue

# launch gazebo with swerve controller
def generate_launch_description():
    use_sim_time = True # this must match with other launch files

    my_pckg_path = FindPackageShare('sim')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    world_sdf_file = PathJoinSubstitution([my_pckg_path, 'sdf/world.sdf'])

    #convert robot model xacro file into urdf
    xacro_file = PathJoinSubstitution([my_pckg_path,'urdf','robot_model.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    bridge_config_file = PathJoinSubstitution([my_pckg_path, 'configs', 'gazebo_bridge.yaml'])
    controllers_file = PathJoinSubstitution([my_pckg_path,'configs','controllers.yaml'])


    # set environment variables
    set_gz_sim_resource_path_env = SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([my_pckg_path, 'urdf'])
        )
    set_gz_sim_plugin_path_env = SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([my_pckg_path, 'plugins'])
        )
    # launch gazebo with the given world map
    launch_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': world_sdf_file, 
                'on_exit_shutdown': 'True',
                # physics overrides: ensure physics update faster than controller (> 50 Hz)
                'physics_max_step_size': '0.001',         # seconds
                'physics_real_time_update_rate': '1000',  # Hz
                'physics_real_time_factor': '1.0'
            }.items(),
        )
        # bridge and remap Gazebo topics to ROS 2 
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'config_file': bridge_config_file,
            'use_sim_time': use_sim_time,
            'use_own_container': True,
            'use_composition': False
        }],
        output='screen',
    )
    # spawn robot model to the gazebo world
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'robot_model',
            '-x','0',
            '-y','0',
            '-z','1'
        ],
        output='screen'
    )
    start_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='both'
    )
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description)
        }],
        output='both'
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    spawn_swerve_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller', '--param-file', controllers_file]
    )
    # delay starting swerve_controller after joint_state_broadcaster
    delay_swerve_controller_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_swerve_controller]
        )
    )
    return LaunchDescription([
        set_gz_sim_resource_path_env,
        set_gz_sim_plugin_path_env,
        launch_gazebo,
        gz_bridge,
        gz_spawn_robot,
        start_controller_manager,
        start_robot_state_publisher,
        spawn_joint_state_broadcaster,
        delay_swerve_controller_spawn
    ])