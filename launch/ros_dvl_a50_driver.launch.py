import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ros_dvl_a50_driver_node = Node(
            package='ros_dvl_a50_driver',
            executable='ros_dvl_a50_driver_node',
            name='ros_dvl_a50_driver_node',
            parameters=[os.path.join(get_package_share_directory('ros_dvl_a50_driver'),'config','ros_dvl_a50_driver_config.yaml')],
            output='screen',
        )
    return LaunchDescription([
        ros_dvl_a50_driver_node
    ])