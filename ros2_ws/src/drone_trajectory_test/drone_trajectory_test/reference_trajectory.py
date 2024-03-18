import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class DesiredTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('desired_trajectory_publisher')
        self.publisher = self.create_publisher(
            Marker,
            'desired_trajectory_marker',
            10)
        self.timer_period = 1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Define the desired trajectory points
        trajectory_points = [
            Point(x=0.0, y=0.0, z=0.0),  # Start position
            Point(x=0.0, y=0.0, z=1.0),  # Takeoff and rise to 1m
            Point(x=2.0, y=0.0, z=1.0),  # Move 2m forward
            Point(x=2.0, y=1.0, z=1.0),  # Move 1m to the left
            Point(x=3.0, y=1.0, z=1.0),  # Move 1m to the right
            Point(x=3.0, y=-2.0, z=1.0),  # Move 3m to the right
        ]

        # Create a marker for the desired trajectory
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "camera_link"
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.points = trajectory_points
        trajectory_marker.pose.orientation.w = 1.0
        trajectory_marker.scale.x = 0.05  # Width of the line
        trajectory_marker.color.r = 1.0
        trajectory_marker.color.g = 0.0
        trajectory_marker.color.b = 0.0
        trajectory_marker.color.a = 1.0

        # Publish the marker
        self.publisher.publish(trajectory_marker)

def main(args=None):
    rclpy.init(args=args)
    desired_trajectory_publisher = DesiredTrajectoryPublisher()
    rclpy.spin(desired_trajectory_publisher)
    desired_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
