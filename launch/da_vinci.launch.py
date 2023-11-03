import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # create two variables, one for the package name and one for the filepath of the urdf/xacro file
    packageName = 'da_vinci_sim'
    xacroFilePath = 'description/da_vinci_instrument.urdf.xacro'

    # Process the urdf/xacro file using xacro
    xacroFile = os.path.join(get_package_share_directory(packageName), xacroFilePath)
    robot_description_raw = xacro.process_file(xacroFile).toxml()

    # Configure the node
    robotStatePublisherNode = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Run the node
    return LaunchDescription([
        robotStatePublisherNode
    ])