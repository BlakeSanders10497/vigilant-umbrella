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
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class NodeTemplate(Node):
    """Node Description"""

    error_data = list()

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

        #Set target distance and calculate error from wall
        d_setpoint = 0.5
        error = d_setpoint - D_perpL 

        #Proportional Controller
        K_p = 45 #Tune this value
        u_1 = K_p*error

        #Integral Controller
        K_i = 0 #Tune this value
        delta_t = 0.1

        self.error_data.append(error)
        if len(self.error_data) > 100:
            self.error_data.pop(0)

        error_sum = sum(self.error_data)


        u_2 = K_i*error_sum*delta_t

        #Sum the commands
        u = u_1 + u_2

        if u > 45:
            u = 45

        if u < -45:
            u = -45    

        #Set turn angle for PI and convert to rad
        u_rad = u*math.pi/180
        drive.steering_angle = u_rad

        #Set constant speed
        speed = 1.0
        drive.speed = speed

        #Add AckermannDrive to AckermannDriveStamped
        out_msg.drive = drive

        #Publish AckermannDriveStamped message
        self.publisher.publish(out_msg)

        #Print out useful info
        self.get_logger().info(f"Error: = {error}")
        self.get_logger().info(f"Error sum: = {error_sum}")
        self.get_logger().info(f"U_1: = {u_1}")
        self.get_logger().info(f"U_2: = {u_2}")
        self.get_logger().info(f"Turn Command: = {u}")

def main(args=None):
    rclpy.init(args=args)
    node_template = NodeTemplate()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
