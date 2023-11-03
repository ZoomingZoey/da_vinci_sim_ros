import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # create two variables, one for the package name and one for the filepath of the urdf/xacro file
    pkg_name = 'da_vinci_sim'
    xacro_subpath = 'description/da_vinci_instrument_detailed.urdf.xacro'

    # Process the urdf/xacro file using xacro
    xacro_file = os.path.join(get_package_share_directory(pkg_name), xacro_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_bot'],
                    output='screen')
    
    ld = LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])

    # Run the node
    return ld