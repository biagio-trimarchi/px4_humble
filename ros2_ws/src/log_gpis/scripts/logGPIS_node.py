#!/usr/bin/env python3

### LIBRARIES
# ROS2 Libraries
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geometry_msgs
import visualization_msgs.msg as visualization_msgs
import tf2_ros

# Python Standard Libraries
import numpy as np

# Custom Libraries
from log_gpis.logGPIS import logGPIS as logGPIS

class logGPISServer(Node):
    def __init__(self):
        super().__init__("logGPIS_Server")

        ### PARAMETERS
        self.debug = True

        lamba_whittle = 3.0
        resolution = 0.2

        # logGPIS
        self.log_map = logGPIS(3, lamba_whittle, resolution)

        if self.debug:
            # Add timer
            self.timer_debug = self.create_timer(1.0, self.debugCallback)

            # Add publisher
            self.publisher_debug = self.create_publisher(visualization_msgs.Marker, 'log_map', 10)

            # Add tf broadcaster
            self.tf_brodcaster = tf2_ros.TransformBroadcaster(self)

            # Add ground
            for xx in np.arange(-10.0, 10.0, 1.0):
                for yy in np.arange(-10.0, 10.0, 1.0):
                    self.log_map.addSample(np.array([xx, yy, 0.0]))

            # Add sphere
            sphere_center = np.array([2.0, 2.0, 3.0])
            sphere_radius = 2.0

            for psi in np.arange(0.0, np.pi, 0.2):
                for theta in np.arange(0.0, 2*np.pi, 0.2):
                    sphere_coordinate = np.array([np.sin(psi) * np.cos(theta),
                                                  np.sin(psi) * np.sin(theta),
                                                  np.cos(psi)
                                                  ])
                    self.log_map.addSample(sphere_center + sphere_radius * sphere_coordinate)

            # Add cylinder
            cylinder_center_ground = np.array([-1.0, -3.0, 0.0])
            cylinder_radius = 1.5
            for zz in np.arange(0.1, 2.0, 0.1):
                for theta in np.arange(0.0, 2*np.pi, 0.5):
                    cylinder_coordinate = np.array([np.cos(theta),
                                                    np.sin(theta),
                                                               zz
                                                    ])
                    self.log_map.addSample(cylinder_center_ground + cylinder_radius * cylinder_coordinate)

            print("Training... ", self.log_map.getSamplesNumber(), " samples.")
            self.log_map.train()
            print("Done!")

    def pointCloudCallback(self):
        pass
    
    def scanRegion(self):
        resolution = 0.2
        for xx in np.arange(-5.0, 5.0, resolution):
            print("xx: ", xx)
            for yy in np.arange(-5.0, 5.0, resolution):
                for zz in np.arange(-0.5, 4.0, resolution):
                    if self.log_map.evaluate(np.array([xx, yy, zz])) < 0.2:
                        # Add message point
                        point = geometry_msgs.Point()
                        point.x = xx
                        point.y = yy
                        point.z = zz
                        self.debug_message.points.append(point)

    def debugCallback(self):
        t = geometry_msgs.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        self.tf_brodcaster.sendTransform(t)

        print("HEY!!")
        self.debug_message = visualization_msgs.Marker()
        self.debug_message.header.frame_id = "map"
        self.debug_message.header.stamp = self.get_clock().now().to_msg()
        self.debug_message.type = visualization_msgs.Marker.POINTS
        self.debug_message.action = visualization_msgs.Marker.ADD
        self.debug_message.points.clear()
        self.scanRegion()
        self.debug_message.pose.position.x = 0.0
        self.debug_message.pose.position.y = 0.0
        self.debug_message.pose.position.z = 0.0
        self.debug_message.pose.orientation.w = 1.0
        self.debug_message.pose.orientation.x = 0.0
        self.debug_message.pose.orientation.y = 0.0
        self.debug_message.pose.orientation.z = 0.0
        self.debug_message.scale.x = 0.1
        self.debug_message.scale.y = 0.1
        self.debug_message.scale.z = 0.1
        self.debug_message.color.r = 0.0
        self.debug_message.color.g = 1.0
        self.debug_message.color.b = 0.0
        self.debug_message.color.a = 1.0

        self.publisher_debug.publish(self.debug_message)
        print("OOY!!")

def main(args=None):
    rclpy.init(args=args)

    node = logGPISServer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
