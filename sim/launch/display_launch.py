from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    use_sim_time = True

    pckg_path = FindPackageShare('sim')
    xacro_file = PathJoinSubstitution([pckg_path,'urdf','robot_model.urdf.xacro'])
    controllers_file = PathJoinSubstitution([pckg_path,'configs','controllers.yaml'])
    rviz_file = PathJoinSubstitution([pckg_path,'configs','display.rviz'])

    # load robot model, get urdf via xacro
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # starts ros2 control
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='both'
    )
    # starts ros2 robot_state_publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )
    # starts rviz view
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_file]
    )
    # starts ros2 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )
    # starts swerve_controller
    swerve_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_controller', '--param-file', controllers_file]
    )
    # delay starting rviz after joint_state_broadcaster
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node]
        )
    )
    # delay starting swerve_controller after joint_state_broadcaster
    delay_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner]
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz,
        delay_swerve_controller_spawner
    ])