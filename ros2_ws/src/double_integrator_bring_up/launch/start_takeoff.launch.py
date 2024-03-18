from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
        ### Rviz
        rviz_configuration = get_package_share_directory("double_integrator_bring_up") + "/rviz/log_gpis_test.rviz"

        ### Create LaunchDescription
        launch_description = LaunchDescription()

        ### Declare launch arguments
        launch_args = []

        launch_args.append(DeclareLaunchArgument(
            "pointcloud_filter_min_distance",
            default_value="2.0"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "pointcloud_filter_max_distance",
            default_value="3.3"
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
            "frame_drone",
            default_value="tf_drone"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "frame_depth_camera",
            default_value="D435_link"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "frame_tracking_camera",
            default_value="T265_link"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "tracking_serial_number",
            default_value='213322071066'
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "depth_serial_number",
            default_value='046222071652'
            )
                           )

        for arg in launch_args:
            launch_description.add_action(arg)

        param_pointcloud_filter_min_distance = LaunchConfiguration("pointcloud_filter_min_distance")
        param_pointcloud_filter_max_distance = LaunchConfiguration("pointcloud_filter_max_distance")
        param_pointcloud_freespace_resolution = LaunchConfiguration("pointcloud_freespace_resolution")
        param_map_width = LaunchConfiguration("map_width")
        param_map_height = LaunchConfiguration("map_height")
        param_occupancy_grid_resolution = LaunchConfiguration("occupancy_grid_resolution")
        param_frame_map = LaunchConfiguration("frame_map")
        param_frame_drone = LaunchConfiguration("frame_drone")
        param_frame_depth = LaunchConfiguration("frame_depth_camera")
        param_frame_tracking = LaunchConfiguration("frame_tracking_camera")
        param_tracking_serial_number = LaunchConfiguration("tracking_serial_number")
        param_depth_serial_number = LaunchConfiguration("depth_serial_number")

        ### TOPICS
        topic_drone_odometry = "/drone_odometry"
        topic_depth_camera_pointcloud = "/D435/depth/color/points"
        topic_tracking_camera_odometry = "/camera/pose/sample"
        topic_occupancy_grid = "/map_occupancy_grid"
        topic_debug_pointcloud = "/debug_pointcloud"       ### Launch parameters

        ### Nodes
        nodes = []
        nodes.append(Node(
            package = "double_integrator_test",
            executable = "governor_takeoff",
            name = "governor",
            parameters = [
                ],
            remappings = [
                ("/odometry", topic_drone_odometry)
                ],
            prefix = ["xterm -hold -e"]
            )
                     )
        nodes.append(Node(
            package = "px4_to_ros_bridge",
            executable = "px4_msgs_bridge",
            name = "px4_bridge",
            parameters = [],
            remappings = [
                ("/simulation_vehicle_odometry", topic_drone_odometry)
                ]
            )
                     )

        nodes.append(Node(
            package = "visual_to_px4",
            executable = "T265_to_px4",
            name = "tracking_manager",
            parameters = [
                ],
            remappings = [
                ("/camera/pose/sample", topic_tracking_camera_odometry)
                ]
            )
                     )

        nodes.append(Node(
            package = "tf2_manager",
            executable = "ldc_tf_manager",
            name = "tf2_manager",
            parameters = [
                {"map_frame" : param_frame_map},
                {"drone_frame" : param_frame_drone},
                {"tracking_frame" : param_frame_tracking},
                {"depth_frame" : param_frame_depth}
                ],
            remappings = [
                ("/drone_odometry", topic_drone_odometry)
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
                'camera_name' : 'T265',
                'serial_number' : param_tracking_serial_number
                }.items()
            )
                            )
        launch_files.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file_path),
            launch_arguments={
                'camera_name' : 'D435',
                'serial_number' : param_depth_serial_number
                }.items()
            )
                            )

        for file in launch_files:
            launch_description.add_action(file)

        return launch_description
