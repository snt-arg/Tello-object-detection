import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    pkg_dir = get_package_share_directory("person_tracking")

    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")

    parameters = DeclareLaunchArgument("params_file", default_value=str(default_param_file))

    param_file = LaunchConfiguration("params_file")

    person_detector = Node(
            package='person_tracking',
            executable='person_detector_node',
            parameters=[param_file],
            prefix='gnome-terminal --',
        )
    
    pilot_person_selector = Node(
            package='person_tracking',
            executable='pilot_person_selector_node',
            parameters=[param_file],
            prefix='gnome-terminal --',
        )
    
    person_tracker = Node(
            package='person_tracking',
            executable='person_tracker_node',
            parameters=[param_file],
            prefix='gnome-terminal --',
        )
    
    pilot_person_drawer = Node(
            package='person_tracking',
            executable='pilot_person_drawer_node',
            parameters=[param_file],
            prefix='gnome-terminal --',
        )
    
    ld = LaunchDescription()
    ld.add_action(parameters)
    ld.add_action(person_detector)
    ld.add_action(pilot_person_selector)
    ld.add_action(person_tracker)
    ld.add_action(pilot_person_drawer)

    return ld
        
    	
        
    	
        
    
