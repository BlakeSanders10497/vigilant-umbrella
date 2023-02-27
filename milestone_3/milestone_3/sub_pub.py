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
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class NodeTemplate(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("my_sub_pub")

        self.subscription = self.create_subscription(msg_type = LaserScan, topic = 'scan', callback = self.one_came_in_function, qos_profile = 1)
        self.publisher = self.create_publisher(msg_type = AckermannDriveStamped, topic = 'vehicle_command_ackermann', qos_profile = 1)

    def one_came_in_function(self, msg: LaserScan):
        """Description of Callback"""

        #Define AckermannDriveStamped message variable
        out_msg = AckermannDriveStamped()
        drive = AckermannDrive()

        #Get distances from Lidar
        d = msg.ranges[180]
        d_offset = msg.ranges[220]

        #Calculate angle alpha, perpendicular distance to the wall, and look ahead perpendicular distance to the wall
        alpha = math.atan2((d_offset * math.cos(20*math.pi/180) - d), (d_offset*math.sin(20*math.pi/180)))
        D_perp = d * math.cos(alpha)
        D_perpL = 0.2*math.sin(alpha) + D_perp

        #Calculate error from wall
        error = 1 - D_perpL

        #Set constant speed
        speed = 1.0
        drive.speed = speed

        #Set static turn angle for BB and convert to rad
        turn_angle = -20*math.pi/180

        if error > 0:
            drive.steering_angle = turn_angle
        else:
            drive.steering_angle = -turn_angle

        #Add AckermannDrive to AckermannDriveStamped
        out_msg.drive = drive

        #Publish AckermannDriveStamped message
        self.publisher.publish(out_msg)

        #Print out useful info
        self.get_logger().info(f"Turn error: = {error}")
        self.get_logger().info(f"Perpendicular distance: = {D_perpL}")

def main(args=None):
    rclpy.init(args=args)
    node_template = NodeTemplate()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
