# Node:
# Written by:
# Date Created: Jan 2023
# Description: Python ROS2 node for converting joystick commands to msgs to send to the controller.

# Import all Necessary Python Modules Here

#import something

# Import all Necessary ROS2 Modules Here, this includes messages (std_msgs.msg for Int16, geometry_msgs.msg for Pose) Look at this
# for all message data: link

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32, String
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class NodeTemplate(Node):
    """Node Description"""

    def __init__(self):
        super().__init__("joy_mapper")

        self.subscription = self.create_subscription(msg_type = Joy, topic = "joy", callback = self.callback, qos_profile = 1)
        self.publisher = self.create_publisher(msg_type = AckermannDriveStamped, topic = "vehicle_command_ackermann", qos_profile = 1)
        # self.timer = self.create_timer(timer_period_secs = , callback = )

    def callback(self, msg: Joy):
        """Description of Callback"""
        out_msg = AckermannDriveStamped()
        drive = AckermannDrive()

        #Right trigger mapped to speed of 0-100
        left_stick_y = msg.axes[0]
        lt = (1-msg.axes[2]) * 0.5
        rt = (1-msg.axes[5]) * 0.5
        max_speed = 6.35
        
        speed = (rt-lt)*max_speed

        #Left Joystick mapped from -45 to 45 for steering
        turn_angle = -msg.axes[0]*45

        #Add parameters to AckermannDrive
        drive.steering_angle = turn_angle
        drive.speed = speed

        #Add AckermannDrive to AckermannDriveStamped
        out_msg.drive = drive

        self.publisher.publish(out_msg)

        self.get_logger().info(f"Speed = {speed}, Turn angle = {turn_angle}")

    def callback2(self):
        """Description of Callback"""
        # You need replace this

        # Do Something

        # self.get_logger().info(" ") # Use this if it is just a string
        # self.get_logger().info(f"Something {var}") # Use this if you want to print a variable to the screen


def main(args=None):
    rclpy.init(args=args)
    node_template = NodeTemplate()
    rclpy.spin(node_template)
    node_template.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
