from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('urc_rover')
    
    return LaunchDescription([
        # Include main rover system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'rover_system.launch.py')
            ),
            launch_arguments={'mission_mode': 'autonomous'}.items()
        ),
        
        # Autonomous Navigation Node
        Node(
            package='urc_rover',
            executable='autonomous_nav',
            name='autonomous_nav',
            output='screen'
        ),
    ])
