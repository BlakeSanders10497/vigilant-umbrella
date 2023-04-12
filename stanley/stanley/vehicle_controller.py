import math
import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

from gps_nav_interfaces.msg import CurrentGoalPose

from gps_nav.uf_support.route_support import get_rad_of_curvature_to_carrot

class VehicleController(Node):
    def __init__(self):
        super().__init__("vehicle_controller")

        self.declare_parameter("L_wheelbase_m", 0.33)

        self.subscription1 = self.create_subscription(PoseStamped, "vehicle_pose", self.vehicle_pose_callback, 1)

        self.subscription2 = self.create_subscription(
            CurrentGoalPose, "current_goal_pose", self.current_goal_pose_callback, 1
        )

        self.subscription3 = self.create_subscription(Int8, "e_stop", self.e_stop_callback, 10)

        self.publisher = self.create_publisher(AckermannDriveStamped, "vehicle_command_ackermann", 10)

        # set up the timer (0.1 sec) to send over the current_carrot message to the vehicle controller
        self.main_timer = self.create_timer(timer_period_sec=0.1, callback=self.main_timer_callback)

        # define the variables that will store the data from the two message inputs
        self.current_goal_point = np.array([0.0, 0.0, 0.0])
        self.current_goal_heading_rad = 0.0
        self.closest_point = np.array([0.0, 0.0, 0.0])
        self.closest_heading_rad = 0.0
        self.speed = 0.0
        self.state = 0.0

        self.vehicle_point = np.array([0.0, 0.0, 0.0])
        self.vehicle_heading_rad = 0.0

        #add terms specific to controller
        self.last_steering_angle_rad = 0.0
        self.heading_error_rad = 0.0
        self.heading_error_m1 = 0.0
        self.heading_error_m2 = 0.0
        self.Kp = 3.0
        self.Kd = 0.0
        self.Ki = 0.1
        self.steering_ang_max_rad = 45.0*math.pi/180.0

        self.pause = False
        self.last_pause_value = False

        self.have_vehicle_pose = False
        self.get_logger().info(f"vehicle pose = false")
        self.have_goal_pose = False
        self.get_logger().info(f"goal pose = false")

    def vehicle_pose_callback(self, msg):
        self.have_vehicle_pose = True
        self.get_logger().info(f"vehicle pose = true")

        self.vehicle_point[0] = msg.pose.position.x
        self.vehicle_point[1] = msg.pose.position.y
        self.vehicle_point[2] = 0.0
        self.vehicle_heading_rad = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

    def current_goal_pose_callback(self, msg):
        self.have_goal_pose = True
        self.get_logger().info(f"goal pose = true")

        self.current_goal_point[0] = msg.current_goal_pose.pose.position.x
        self.current_goal_point[1] = msg.current_goal_pose.pose.position.y
        self.current_goal_point[2] = 0.0
        self.current_goal_heading_rad = 2.0 * math.atan2(
            msg.current_goal_pose.pose.orientation.z, msg.current_goal_pose.pose.orientation.w
        )

        self.closest_point[0] = msg.closest_pose.pose.position.x
        self.closest_point[1] = msg.closest_pose.pose.position.y
        self.closest_point[2] = 0.0
        self.closest_heading_rad = 2.0 * math.atan2(
            msg.closest_pose.pose.orientation.z, msg.closest_pose.pose.orientation.w
        )

        self.speed = msg.speed
        self.state = msg.state

        if self.speed > 2.5:
            self.speed = 2.5
        elif self.speed < -2.5:
            self.speed = -2.5

    def e_stop_callback(self, msg):
        if msg.data == 0:
            self.pause = True
        elif msg.data == 1:
            self.pause = False

    def main_timer_callback(self):

        if self.pause == False:
            self.last_pause_value = False

        if self.pause == True:
            if self.last_pause_value == False:
                self.last_pause_value = True

                # send out a zero velocity twist
                out_msg = AckermannDriveStamped()
                out_msg.drive.speed = 0.0
                out_msg.drive.steering_angle = 0.0

                self.publisher.publish(out_msg)
                return

        # This will only publish after each subscription has occured at least once
        if self.have_goal_pose and self.have_vehicle_pose:

            self.get_logger().info(f"Stage 1")

            #Create vector from current vehicle pose to goal pose
            vec_to_carrot = self.current_goal_point - self.vehicle_point
            #Unitize Vector
            vec_to_carrot /= np.linalg.norm(vec_to_carrot)
            #Get angle from car to goal pose
            heading_to_carrot_rad = math.atan2(vec_to_carrot[1], vec_to_carrot[0])
            #Calculate error in radians of vehicle heading to carrot heading
            heading_error_rad = heading_to_carrot_rad - self.vehicle_heading_rad

            self.get_logger().info(f"Stage 2")

            #Add bounds for heading error
            while heading_error_rad > math.pi:
                heading_error_rad -= 2.0*math.pi
            while heading_error_rad < -math.pi:
                heading_error_rad += 2.0*math.pi

            self.get_logger().info(f"Stage 3")

            #If car is moving, calculate new steering angle based on PID, if car not moving then change is zero
            if(self.speed > 0.0):
                change_in_steering_ang = self.Kp * (heading_error_rad - self.heading_error_m1) + self.Kd * (heading_error_rad - 2*self.heading_error_m1 + self.heading_error_m2) + self.Ki * heading_error_rad
                new_steering_ang = self.last_steering_angle_rad + change_in_steering_ang
            else:
                change_in_steering_ang = 0.0
                new_steering_ang = self.last_steering_angle_rad

            #Define out message and apply speed to it
            out_msg = AckermannDriveStamped()
            out_msg.drive.speed = self.speed

            #Add bounds to steering angle
            if(new_steering_ang > self.steering_ang_max_rad):
                new_steering_ang = self.steering_ang_max_rad
            elif(new_steering_ang < -self.steering_ang_max_rad):
                new_steering_ang = -self.steering_ang_max_rad

            print(f'{heading_to_carrot_rad*180.0/math.pi}')

            #Apply steering angle to out message
            out_msg.drive.steering_angle = new_steering_ang

            #Update stored variables for next iteration
            self.last_steering_angle_rad = new_steering_ang
            self.heading_error_m2 = self.heading_error_m1
            self.heading_error_m1 = heading_error_rad

            #Publish message to update speed and steering angle of vehicle
            self.get_logger().info(f"publisher update!")
            self.publisher.publish(out_msg)
        else:
            self.get_logger().info(f"not publishing.")


def main(args=None):
    rclpy.init(args=args)

    vehicle_controller_carrot = VehicleController()

    rclpy.spin(vehicle_controller_carrot)

    vehicle_controller_carrot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
