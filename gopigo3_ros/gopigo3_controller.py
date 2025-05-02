#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from easygopigo3 import EasyGoPiGo3

class GoPiGo3Controller(Node):
    def __init__(self):
        super().__init__('gopigo3_controller')
        self.gpg = EasyGoPiGo3()
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("GoPiGo3 Controller Started!")

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x * 100  # Scale m/s to % (adjust as needed)
        angular = msg.angular.z * 50 # Scale rad/s to %

        # Motor control logic
        if angular > 0:    # Turn right
            self.gpg.set_speed(int(angular))
            self.gpg.right()
        elif angular < 0:  # Turn left
            self.gpg.set_speed(int(-angular))
            self.gpg.left()
        else:             # Move forward/backward
            self.gpg.set_speed(int(linear))
            if linear >= 0:
                self.gpg.forward()
            else:
                self.gpg.backward()

def main(args=None):
    rclpy.init(args=args)
    node = GoPiGo3Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()