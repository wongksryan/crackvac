from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

# launch rviz with swerve controller
def generate_launch_description():
    use_sim_time = True

    my_pckg_path = FindPackageShare('sim')
    controllers_file = PathJoinSubstitution([my_pckg_path,'configs','controllers.yaml'])
    rviz_file = PathJoinSubstitution([my_pckg_path,'configs','display.rviz'])

    # load robot model, get urdf via xacro
    xacro_file = PathJoinSubstitution([my_pckg_path,'urdf','robot_model.urdf.xacro'])
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
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
            'use_sim_time': use_sim_time,
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
        start_robot_state_publisher,
        spawn_joint_state_broadcaster,
        delay_rviz_launch,
        delay_swerve_controller_spawn
    ])