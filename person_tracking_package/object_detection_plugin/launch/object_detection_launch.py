import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    pkg_dir = get_package_share_directory("object_detection_plugin")

    default_param_file = os.path.join(pkg_dir, "config", "params.yaml")

    parameters = DeclareLaunchArgument("params_file", default_value=str(default_param_file))

    param_file = LaunchConfiguration("params_file")

    object_detection_node = Node(
            package='object_detection_plugin',
            executable='object_detection_node',
            parameters=[param_file],)
            #prefix='gnome-terminal --',)
 
            
    ld = LaunchDescription()
    ld.add_action(parameters)
    ld.add_action(object_detection_node)
    
   

    return ld
        
    	
        
    	
        
    


