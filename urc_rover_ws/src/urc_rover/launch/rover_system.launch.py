from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Declare mission mode argument
    mission_mode_arg = DeclareLaunchArgument(
        'mission_mode',
        default_value='science',
        description='Mission mode: science, delivery, autonomous, servicing'
    )
    
    mission_mode = LaunchConfiguration('mission_mode')
    
    return LaunchDescription([
        mission_mode_arg,
        
        # Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Mission Manager
        Node(
            package='urc_rover',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'mission_mode': mission_mode}]
        ),
        
        # Navigation Controller
        Node(
            package='urc_rover',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen'
        ),
        
        # Camera Handler
        Node(
            package='urc_rover',
            executable='camera_handler',
            name='camera_handler',
            output='screen'
        ),
        
        # GNSS Simulator
        Node(
            package='urc_rover',
            executable='gnss_simulator',
            name='gnss_simulator',
            output='screen'
        ),
        
        # LED Indicator
        Node(
            package='urc_rover',
            executable='led_indicator',
            name='led_indicator',
            output='screen'
        ),
    ])
