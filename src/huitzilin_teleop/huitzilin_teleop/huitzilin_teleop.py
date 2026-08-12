#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Instructions 
MSG = """
==============================================
         huitzilin_teleop             
==============================================
   w : Pitch Forward    |   s : Pitch Backward
   a : Roll Left        |   d : Roll Right
   i : Ascend           |   k : Descend 
   j : Yaw Left         |   l : Yaw Right

   SPACE : Reset 
   CTRL+C : Quit
==============================================
"""

class HuitzilinTeleopNode(Node):
    def __init__(self):
        super().__init__('huitzilin_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Setpoints
        self.pitch_step = 0.15   # rad 
        self.roll_step  = 0.15   # rad 
        self.yaw_step   = 0.3    # rad/s
        self.throttle_step = 1.0 

        # Save terminal settings 
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info("huitzilin_teleop initialized.")
        print(MSG)

    def getKey(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.getKey(timeout=0.05)
                msg = Twist()

                if key == 'w':
                    msg.linear.x = self.pitch_step    # Forward pitch
                elif key == 's':
                    msg.linear.x = -self.pitch_step   # Backward pitch
                elif key == 'a':
                    msg.angular.z = self.roll_step    # Left roll
                elif key == 'd':
                    msg.angular.z = -self.roll_step   # Right roll
                elif key == 'j':
                    msg.linear.y = self.yaw_step      # Yaw left
                elif key == 'l':
                    msg.linear.y = -self.yaw_step     # Yaw riight
                elif key == 'i':
                    msg.linear.z = self.throttle_step # Ascend
                elif key == 'k':
                    msg.linear.z = -self.throttle_step# Descend
                elif key == ' ':
                    # Spacebar zeroing
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.z = 0.0
                elif key == '\x03': # CTRL+C
                    break
                else:
                    # No key pressed
                    msg.linear.x = 0.0
                    msg.linear.y = 0.0
                    msg.linear.z = 0.0
                    msg.angular.z = 0.0

                self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error in teleop loop: {e}")
        finally:
            # Restore settings
            stop_msg = Twist()
            self.publisher_.publish(stop_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = HuitzilinTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()