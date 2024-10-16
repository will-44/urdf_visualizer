from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get the URDF file name from the launch argument
    urdf_filename = LaunchConfiguration('urdf_filename').perform(context)

    # Get the package share directory
    package_share_directory = get_package_share_directory('urdf_visualizer')

    # Construct the full path to the URDF file
    urdf_file_path = os.path.join(package_share_directory, 'urdf', urdf_filename)

    # Read the URDF file
    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    # Create the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return [robot_state_publisher_node, rviz_node]

def generate_launch_description():
    # Declare the URDF file name as a launch argument with a default value
    declare_urdf_filename_cmd = DeclareLaunchArgument(
        'urdf_filename',
        default_value='robot.urdf',
        description='Name of the URDF file in the urdf directory')

    return LaunchDescription([
        declare_urdf_filename_cmd,
        OpaqueFunction(function=launch_setup)
    ])
