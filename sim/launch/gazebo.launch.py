from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

# launch gazebo with controllers
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    my_pckg_path = FindPackageShare('sim')
    ros_gz_sim_pckg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pckg_path, 'launch', 'gz_sim.launch.py'])
    world_sdf = PathJoinSubstitution([my_pckg_path, 'sdf/world.sdf'])

    # convert xacro into urdf 
    robot_description = Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([my_pckg_path, 'urdf', 'robot_model.xacro.urdf'])
        ]
    )
    bridge_config_file = PathJoinSubstitution([my_pckg_path, 'config', 'gazebo_bridge.yaml'])
    robot_controllers = PathJoinSubstitution([my_pckg_path, 'config', 'controllers.yaml'])


    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    gz_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'crackvac',
            '-allow_renaming', 'true',
            '-x','0',
            '-y','0',
            '-z','1'
        ],
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    spawn_swerve_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_controller', 
            '--param-file', robot_controllers,
            '--controller-ros-args',
            '-r /swerve_controller/odom:=/odom',
            '--controller-ros-args',
            '-r /swerve_controller/cmd_vel:=/cmd_vel'
        ],
    )
    # remap Gazebo topics to ROS 2 
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{
            'config_file': bridge_config_file,
            'use_own_container': False,
            'use_composition': False
        }],
    )
    delay_joint_state_broadcaster_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_robot,
            on_exit=[spawn_joint_state_broadcaster]
        )
    )
    delay_swerve_controller_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_swerve_controller]
        )
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': world_sdf, 
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        delay_joint_state_broadcaster_spawn,
        delay_swerve_controller_spawn,
        bridge,
        start_robot_state_publisher,
        gz_spawn_robot,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
    ])
