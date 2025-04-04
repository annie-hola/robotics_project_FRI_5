import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.get_logger().info("Movement Control Node Initialized")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.cmd_vel_pub.publish(msg)

    def move_backward(self, speed):
        msg = Twist()
        msg.linear.x = -speed
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, angular_speed):
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, angular_speed):
        msg = Twist()
        msg.angular.z = -angular_speed
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)