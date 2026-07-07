#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from actuator_msgs.msg import Actuators 

class HuitzilinTeleopNode(Node):

    def __init__(self):
        super().__init__("huitzilin_teleop")

        # /cmd_vel subscriber
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.callback_cmd_vel,
            10)

        # /huitzilin/motor_speed publisher
        self.publisher_ = self.create_publisher(
            Actuators,
            "huitzilin/motor_speed",
            10)
        
        self.get_logger().info("Huitzilin Teleop has been started.")



    def callback_cmd_vel(self, msg: Twist):

        # Base RPM
        base_rpm = 1600

        # Extracts the inputs and translates them into RPM
        pitch = msg.linear.x * base_rpm
        roll = msg.linear.y * base_rpm
        yaw = msg.angular.z * base_rpm
        throttle = msg.linear.z * base_rpm

        # Drone kInematics

        # Front right
        rotor_0 = throttle - pitch - roll - yaw

        # Rear left
        rotor_1 = throttle + pitch + roll - yaw 

        # Front left
        rotor_2 = throttle - pitch + roll + yaw

        # Rear right
        rotor_3 = throttle + pitch - roll + yaw

        # Actuator message
        actuator_msg = Actuators()
        actuator_msg.velocity = [rotor_0, rotor_1, rotor_2, rotor_3]
        
        # The message is published
        self.publisher_.publish(actuator_msg)

def main(args=None):
    rclpy.init(args=args)
    huitzilin_teleop = HuitzilinTeleopNode()
    rclpy.spin(huitzilin_teleop)
    rclpy.shutdown()


if __name__ == '__main__':
    main()