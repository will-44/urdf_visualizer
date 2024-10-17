from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro

def launch_setup(context, *args, **kwargs):
    
    declare_xacro_file_cmd = DeclareLaunchArgument(
        'filename',
        default_value='robot.xacro',
        description='Nom du fichier Xacro dans le dossier urdf')

    filename = LaunchConfiguration('filename')#.perform(context)
 
    # Get the package share directory
    package_share_directory = get_package_share_directory('urdf_visualizer')

    # Construct the full path to the URDF file
    file_path = os.path.join(package_share_directory, 'urdf', filename.perform(context))
    
    if file_path.endswith('.xacro'):
        # robot_description_content = Command(command=[
        # "python3 -c '\n"
        # "import yaml \n"
        # "with open(\"", file_path, "\") as f: \n"
        # "    print(yaml.dump(f.read()))"
        # "'"
        # ])
        robot_description_content = Command(['xacro ', file_path])
        # robot_description_content = xacro.process_file(robot_description_content).toxml()
        print(robot_description_content)
    elif file_path.endswith('.urdf'):
        # Read the URDF file
        with open(file_path, 'r') as infp:
            robot_description_content = infp.read()
            print(robot_description_content)
    else:
        # print error log ros2
        print("Error: file extension is not xacro or urdf")

    # robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_node',
        emulate_tty=True,
        output="screen"
    )

    # Create the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return [robot_state_publisher_node, rviz_node, joint_state_publisher_node]

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
