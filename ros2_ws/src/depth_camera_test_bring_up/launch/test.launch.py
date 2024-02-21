from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    launch_description = LaunchDescription()
    ### Launch arguments
    launch_args = []
    launch_args.append(DeclareLaunchArgument(
        "offset_x",
        default_value="0.0"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "offset_y",
        default_value="0.0"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "offset_z",
        default_value="0.0"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "tf_parent",
        default_value="tf_map"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "tf_child",
        default_value="camera_link"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "pointcloud_filter_min_distance",
        default_value="0.5"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "pointcloud_filter_max_distance",
        default_value="2.5"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "pointcloud_freespace_resolution",
        default_value="0.1"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "map_width",
        default_value="10.0"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "map_height",
        default_value="12.0"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "occupancy_grid_resolution",
        default_value="0.2"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "frame_map",
        default_value="tf_map"
        )
                       )
    launch_args.append(DeclareLaunchArgument(
        "frame_camera",
        default_value="tf_map"
        )
                       )

    for arg in launch_args:
        launch_description.add_action(arg)

    param_offset_x = LaunchConfiguration("offset_x")
    param_offset_y = LaunchConfiguration("offset_y")
    param_offset_z = LaunchConfiguration("offset_z")
    param_tf_parent = LaunchConfiguration("tf_parent")
    param_tf_child = LaunchConfiguration("tf_child")
    param_pointcloud_filter_min_distance = LaunchConfiguration("pointcloud_filter_min_distance")
    param_pointcloud_filter_max_distance = LaunchConfiguration("pointcloud_filter_max_distance")
    param_pointcloud_freespace_resolution = LaunchConfiguration("pointcloud_freespace_resolution")
    param_map_width = LaunchConfiguration("map_width")
    param_map_height = LaunchConfiguration("map_height")
    param_occupancy_grid_resolution = LaunchConfiguration("occupancy_grid_resolution")
    param_frame_map = LaunchConfiguration("frame_map")
    param_frame_camera = LaunchConfiguration("frame_camera")

    ### TOPICS
    topic_odometry = "/odometry"
    topic_depth_camera_pointcloud = "/camera/depth/color/points"
    topic_vicon = "/vicon/camera/camera"
    topic_occupancy_grid = "/map_occupancy_grid"
    topic_debug_pointcloud = "/debug_pointcloud"
    
    ### NODES
    nodes = []

    nodes.append(Node(
        package = "vicon_manager",
        executable = "vicon_to_odometry",
        name = "odometry_converter",
        parameters = [
            {"offset_x" : param_offset_x},
            {"offset_y" : param_offset_y},
            {"offset_z" : param_offset_z},
            {"tf_parent" : param_tf_parent},
            {"tf_child" : param_tf_child}
            ],
        remappings = [
            ("/topic_odometry", topic_odometry),
            ("/topic_vicon", topic_vicon)
            ],
        prefix = []
        )
                 )

    nodes.append(Node(
        package = "camera_to_occupancy",
        executable = "camera_to_occupancy",
        name = "occupancy_map_manager",
        parameters = [
            {"pointcloud_filter_min_distance" : param_pointcloud_filter_min_distance},
            {"pointcloud_filter_max_distance" : param_pointcloud_filter_max_distance},
            {"pointcloud_freespace_resolution" : param_pointcloud_freespace_resolution},
            {"map_width" : param_map_width},
            {"map_height" : param_map_height},
            {"occupancy_grid_resolution" : param_occupancy_grid_resolution},
            {"frame_map" : param_frame_map},
            {"frame_camera" : param_frame_camera}
            ],
        remappings = [
            ("/in_cloud", topic_depth_camera_pointcloud),
            ("/in_odometry", topic_odometry),
            ("/out_occupancy_grid", topic_occupancy_grid),
            ("/out_debug_pointcloud", topic_debug_pointcloud)
            ]
        )
                 )

    for node in nodes:
        launch_description.add_action(node)

    launch_files = []
    launch_file_path = os.path.join(
            get_package_share_directory("realsense2_camera"),
            'launch',
            'rs_launch.py'
            )
    launch_files.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments={
            'pointcloud.enable' : "true",
            'pointcloud.stream_filter' : "0"
            }.items()
        )
            )

    launch_file_path = os.path.join(
            get_package_share_directory("vicon_receiver"),
            'launch',
            'client.launch.py'
            )
    launch_files.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        )
            )
    
    for file in launch_files:
        launch_description.add_action(file)

    return launch_description
