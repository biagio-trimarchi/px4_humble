import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data

class DronePathVisualizer(Node):
    def __init__(self):
        super().__init__('drone_path_visualizer')
        
        self.subscription = self.create_subscription(
            Odometry,
            'camera/pose/sample',
            self.odometry_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        
        # Publisher for the current position
        self.position_publisher = self.create_publisher(
            Marker,
            'drone_visualization_marker',
            10)

        # Publisher for the trajectory
        self.trajectory_publisher = self.create_publisher(
            Marker,
            'drone_trajectory_marker',
            10)

        # Initialize the marker points for the trajectory
        self.trajectory_points = Marker()
        self.trajectory_points.header.frame_id = "camera_link"
        self.trajectory_points.type = Marker.POINTS
        self.trajectory_points.action = Marker.ADD
        self.trajectory_points.pose.orientation.w = 1.0

        self.trajectory_points.scale.x = 0.025 # width of the line
        self.trajectory_points.scale.y = 0.025

        self.trajectory_points.color.r = 0.0
        self.trajectory_points.color.g = 0.0
        self.trajectory_points.color.b = 1.0
        self.trajectory_points.color.a = 1.0

    def odometry_callback(self, msg):
        current_position = msg.pose.pose.position

        # Create a marker for the current position
        position_marker = Marker()
        position_marker.header.frame_id = "camera_link"
        position_marker.type = Marker.SPHERE
        position_marker.action = Marker.ADD
        position_marker.pose.position = current_position
        position_marker.pose.orientation.w = 1.0
        position_marker.scale.x = 0.1
        position_marker.scale.y = 0.1
        position_marker.scale.z = 0.1
        position_marker.color.r = 0.0
        position_marker.color.g = 0.0
        position_marker.color.b = 1.0
        position_marker.color.a = 1.0

        # Add the current position to the trajectory
        self.trajectory_points.points.append(Point(
            x=current_position.x,
            y=current_position.y,
            z=current_position.z
        ))

        # Publish the markers
        self.position_publisher.publish(position_marker)
        self.trajectory_publisher.publish(self.trajectory_points)

def main(args=None):
    rclpy.init(args=args)
    drone_path_visualizer = DronePathVisualizer()
    rclpy.spin(drone_path_visualizer)
    drone_path_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
