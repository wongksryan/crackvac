from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit

# launch rviz with controllers
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    my_pckg_path = FindPackageShare('sim')
    controllers_file = PathJoinSubstitution([my_pckg_path,'config','controllers.yaml'])
    rviz_file = PathJoinSubstitution([my_pckg_path,'config','display.rviz'])

    # convert xacro into urdf 
    robot_description = Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([my_pckg_path, 'urdf', 'robot_model.xacro.urdf'])
        ]
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
        output='both',
        parameters=[{
            'robot_description': robot_description
        }],
    )
    launch_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_file]
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
    # delay starting rviz after joint_state_broadcaster
    delay_rviz_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[launch_rviz]
        )
    )
    # delay starting swerve_controller after joint_state_broadcaster
    delay_swerve_controller_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_swerve_controller]
        )
    )

    return LaunchDescription([
        start_controller_manager,
        delay_rviz_launch,
        delay_swerve_controller_spawn,
        start_robot_state_publisher,
        spawn_joint_state_broadcaster,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
    ])