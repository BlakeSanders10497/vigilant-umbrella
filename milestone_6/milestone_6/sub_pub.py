# Node:
# Written by:
# Date Created: Jan 2023
# Description: Python ROS2 node for converting joystick commands to msgs to send to the controller.

# Import all Necessary Python Modules Here

import math

# Import all Necessary ROS2 Modules Here, this includes messages (std_msgs.msg for Int16, geometry_msgs.msg for Pose) Look at this
# for all message data: link

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import utm

class NodeTemplate(Node):
    """Node Description"""

    x = 0.0
    y = 0.0
    

    error_data = list()

    def __init__(self):
        super().__init__("my_sub_pub")

        self.subscription = self.create_subscription(msg_type = Odometry, topic = 'odometry', callback = self.odometry_callback, qos_profile = 1)
        self.subscription = self.create_subscription(msg_type = NavSatFix, topic = 'gps', callback = self.gps_callback, qos_profile = 1)
        self.publisher = self.create_publisher(msg_type = PoseStamped, topic = 'vehicle_pose', qos_profile = 1)

    def odometry_callback(self, msg: Odometry):

        delta_t_s = 0.05

        v_x = msg.twist.twist.linear.x
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        theta_vehicle = 2*math.atan2(z, w)

        self.x += v_x*math.cos(theta_vehicle)*delta_t_s
        self.y += v_x*math.sin(theta_vehicle)*delta_t_s


        msgEst = PoseStamped()
        msgEst.pose.position.x = self.x
        msgEst.pose.position.y = self.y
        msgEst.pose.orientation.z = z
        msgEst.pose.orientation.w = w
        msgEst.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msgEst)




    def gps_callback(self, msg: NavSatFix):
        east, north, _, _ = utm.from_latlong(msg.latitude, msg.longitude)
        self.x = east #todo check if this is the right way around
        self.y = north

        #self.get_logger().info(f"x: {self.x}")
        #self.get_logger().info(f"y: {self.y}")

def main(args=None):
    rclpy.init(args=args)
    node_template = NodeTemplate()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
