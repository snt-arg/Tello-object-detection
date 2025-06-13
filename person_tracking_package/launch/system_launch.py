import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def create_tello_driver_launch(ld: LaunchDescription) -> None:
    tello_driver_pkg_dir = get_package_share_directory("tello_driver")

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tello_driver_pkg_dir, "launch/tello_driver.launch.py")
            ),
        )
    )


def create_robot_bt_launch(ld: LaunchDescription) -> None:
    ld.add_action(Node(package="robot_bt", executable="bt_server", output="screen", prefix="gnome-terminal --",))


def create_tello_control_station_launch(ld: LaunchDescription) -> None:
    ld.add_action(
        Node(
            package="tello_control_station",
            executable="control_station",
            output="screen",
        )
    )


def create_hand_tracker_plugin_launch(ld: LaunchDescription) -> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    hand_tracker_pck_dir = get_package_share_directory("hand_gestures")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hand_tracker_pck_dir, "launch/hand_gestures_launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
                "run_annotator": "true",
            }.items(),
        )
    )

def create_object_detection_plugin_launch(ld: LaunchDescription) -> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    object_detection_pck_dir = get_package_share_directory("object_detection_plugin")
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(object_detection_pck_dir, "launch/object_detection_launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
            }.items(),
        )
    )
def create_person_object_association_plugin_launch(ld:LaunchDescription)-> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    pck_dir = get_package_share_directory("person_object_association")
    ld.add_action(
        Node(
            package='person_object_association',
            executable='associator_node',
            parameters=[params_file],)
            #prefix='gnome-terminal --',)
    )

def create_object_following_plugin_launch(ld:LaunchDescription)-> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    #pck_dir = get_package_share_directory("object_following_plugin")
    ld.add_action(
        Node(
            package='object_following_plugin',
            executable='tracker_node',
            parameters=[params_file],
            prefix='gnome-terminal --',)
    )
    ld.add_action(
        Node(
            package='object_following_plugin',
            executable='following_commands_node',
            parameters=[params_file],
            prefix='gnome-terminal --',)
    )
    
    #ld.add_action(
    #    Node( package='object_following_plugin',
    #        executable='collision_avoiding_node',
    #        parameters=[params_file],
    #        prefix='gnome-terminal --',)
    #)

def create_sign_filter_plugin_launch(ld:LaunchDescription)-> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    #pck_dir = get_package_share_directory("sign_filter_plugin")
    ld.add_action(
        Node(
            package='sign_filter_plugin',
            executable='sign_filter_node',
            parameters=[params_file],)
            #prefix='gnome-terminal --',)
    )
    ld.add_action(
        Node(
            package='sign_filter_plugin',
            executable='tello_sign_interpreter_node',
            parameters=[params_file],
        )
    )
    

def create_land_takeoff_plugin_launch(ld:LaunchDescription)-> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    #pck_dir = get_package_share_directory("object_following_plugin")
    ld.add_action(
        Node(
            package='object_following_plugin',
            executable='takeoff_node',
            parameters=[params_file],)
            #prefix='gnome-terminal --',)
    )
    ld.add_action(
        Node(
            package='object_following_plugin',
            executable='land_node',
            parameters=[params_file],
        )
    )

def create_video_interface_plugin_launch(ld:LaunchDescription)-> None:
    pkg_dir = get_package_share_directory("robot_bringup")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")
    #pck_dir = get_package_share_directory("object_following_plugin")
    ld.add_action(
        Node(
            package='video_interface_plugin',
            executable='video_interface_node',
            parameters=[params_file],)
            #prefix='gnome-terminal --',)
    )
    ld.add_action(
        Node(
            package='drawer_plugin',
            executable='drawer_node',
            parameters=[params_file],
        )
    )


def generate_launch_description():
    ld = LaunchDescription()

    create_tello_driver_launch(ld)
    create_robot_bt_launch(ld)
    #create_tello_control_station_launch(ld)

    # ------------------
    # -    Plugins     -
    # ------------------

    create_hand_tracker_plugin_launch(ld)
    create_object_detection_plugin_launch(ld)
    create_person_object_association_plugin_launch(ld)
    create_object_following_plugin_launch(ld)
    create_sign_filter_plugin_launch(ld)
    create_land_takeoff_plugin_launch(ld)
    create_video_interface_plugin_launch(ld)

    return ld
