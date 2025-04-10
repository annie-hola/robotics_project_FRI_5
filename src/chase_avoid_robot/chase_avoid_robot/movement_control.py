import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.get_logger().info("Movement Control Node Initialized")
        self.mvt = Twist()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def move_forward(self, speed):
        msg = self.mvt
        msg.linear.x = speed
        self.cmd_vel_pub.publish(msg)

    def move_backward(self, speed):
        msg = self.mvt
        msg.linear.x = -speed
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, angular_speed):
        msg = self.mvt
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, angular_speed):
        msg = self.mvt
        msg.angular.z = -angular_speed
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        msg = self.mvt
        self.cmd_vel_pub.publish(msg)
    
    def cmd_vel_callback(self, msg):
        self.mvt = msg