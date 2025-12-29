from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = True

    my_pckg_path = FindPackageShare('sim')
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])

    #convert robot model xacro file into urdf
    model_xacro_file = PathJoinSubstitution([my_pckg_path,'urdf','robot_model.urdf.xacro'])
    robot_description = Command(['xacro ', model_xacro_file])

    return LaunchDescription([
        # set environment variables
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([my_pckg_path, 'urdf'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([my_pckg_path, 'plugins'])
        ),
        # launch gazebo with the given world map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': PathJoinSubstitution([my_pckg_path, 'sdf/world.sdf']), 
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        # bridge and remap Gazebo topics to ROS 2 
        # we need to remap clock topic for controller_manager to function correctly
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            parameters=[{
                "qos_overrides./tf_static.publisher.durability": "transient_local",
                "use_sim_time": use_sim_time
            }],
            output="screen",
        ),
        # spawn robot model to the gazebo world
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-string', robot_description,
                '-name', 'robot_model'
                '-x','0',
                '-y','0',
                '-z','1'
            ]
        )
    ])