from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ### Rviz
        rviz_configuration = get_package_share_directory("double_integrator_bring_up") + "/rviz/log_gpis_test.rviz"

        ### Create LaunchDescription
        launch_description = LaunchDescription()

        ### Declare launch arguments
        launch_args = []
        launch_args.append(DeclareLaunchArgument(
            "use_sim_time",
            default_value=TextSubstitution(text="true")
            )
                           )

        launch_args.append(DeclareLaunchArgument(
            "tf_world_name",
            default_value="map"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "tf_odometry_name",
            default_value="odometry"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "tf_drone_name",
            default_value="drone"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "world_name",
            default_value="lgpis_test_1"
            )
                           )

        launch_args.append(DeclareLaunchArgument(
            "gp_lambda_whittle",
            default_value="7.5"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "gp_resolution",
            default_value="0.1"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "gp_error_variance",
            default_value="0.01"
            )
                           )

        launch_args.append(DeclareLaunchArgument(
            "load_world",
            default_value="true"
            )
                           )
        launch_args.append(DeclareLaunchArgument(
            "save_world",
            default_value="false"
            )
                           )

        for arg in launch_args:
            launch_description.add_action(arg)

        ### Launch parameters
        # Global
        use_sim_time = LaunchConfiguration("use_sim_time")

        # Simulation manager
        param_tf_world_name = LaunchConfiguration("tf_world_name")
        param_tf_odometry_name = LaunchConfiguration("tf_odometry_name")
        param_tf_drone_name = LaunchConfiguration("tf_drone_name")
        param_world_name = LaunchConfiguration("world_name")

        param_gp_lamba_whittle = LaunchConfiguration("gp_lambda_whittle")
        param_gp_resolution = LaunchConfiguration("gp_resolution")
        param_gp_error_variance = LaunchConfiguration("gp_error_variance")

        param_load_world = LaunchConfiguration("load_world")
        param_save_world = LaunchConfiguration("save_world")

        ### Namespaces
        drone_namespace = "/drone"
        log_gpis_namespace = "/log_gpis"

        ### Topics
        topic_drone_odometry = drone_namespace + "/odometry"
        topic_log_gpis_visualization = log_gpis_namespace + "/visualization"

        ### Nodes
        nodes = []
        nodes.append(Node(
            package = "double_integrator_test",
            executable = "governor_node_2",
            name = "governor",
            namespace = drone_namespace,
            parameters = [
                {"use_sim_time" : use_sim_time}
                ],
            remappings = [
                ("/odometry", topic_drone_odometry)
                ],
            prefix = ["xterm -hold -e"]
            )
                     )

        nodes.append(Node(
            package = "simulation_manager",
            executable = "gazebo_manager",
            name = "simulation_manager",
            namespace = log_gpis_namespace,
            parameters = [
                {"use_sim_time" : use_sim_time},
                {"topic_ros_drone_odometry" : topic_drone_odometry},
                {"topic_log_gpis_visualization" : topic_log_gpis_visualization},
                {"tf_world_name" : param_tf_world_name},
                {"tf_odometry_name" : param_tf_odometry_name},
                {"tf_drone_name" : param_tf_drone_name},
                {"gp_lambda_whittle" : param_gp_lamba_whittle},
                {"gp_resolution" : param_gp_resolution},
                {"gp_error_variance" : param_gp_error_variance},
                {"load_world" : param_load_world},
                {"save_world" : param_save_world}
                ],
            prefix = ["xterm -hold -e"]
            )
                     )

        nodes.append(Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            name = "ros_gz_bridge",
            namespace = log_gpis_namespace,
            parameters = [
                {"use_sim_time" : use_sim_time}
                ],
            arguments = ["/model/x500_vision_0/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
            prefix = ["xterm -hold -e"]
            )
                     )

        nodes.append(Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            namespace = log_gpis_namespace,
            arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            prefix = ["xterm -hold -e"]
            )
                     )

        nodes.append(Node(
            package = "rviz2",
            executable = "rviz2",
            name = "rviz2",
            parameters = [
                {"use_sim_time" : use_sim_time}
                ],
            arguments = ["-d " + rviz_configuration]
            )
                     )

        for node in nodes:
            launch_description.add_action(node)

        return launch_description
