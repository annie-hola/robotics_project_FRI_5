import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovementControl(Node):
    def __init__(self):
        super().__init__('movement_control')
        self.get_logger().info("Movement Control Node Initialized")
        self.cmd_vel_pub = self.create_publisher(Twist, '/Robot5/cmd_vel', 10)

    def move_forward(self, speed):
        speed = min(speed, 1.0)  # Limit max speed
        self.get_logger().debug(f"Moving forward with speed: {speed}")
        msg = Twist()
        msg.linear.x = speed
        self.cmd_vel_pub.publish(msg)

    def move_backward(self, speed):
        speed = min(speed, 1.0)  # Limit max speed
        self.get_logger().debug(f"Moving backward with speed: {speed}")
        msg = Twist()
        msg.linear.x = -speed
        self.cmd_vel_pub.publish(msg)

    def turn_left(self, angular_speed):
        angular_speed = min(angular_speed, 2.0)  # Limit max angular speed
        self.get_logger().debug(f"Turning left with angular speed: {angular_speed}")
        msg = Twist()
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg)

    def turn_right(self, angular_speed):
        angular_speed = min(angular_speed, 2.0)  # Limit max angular speed
        self.get_logger().debug(f"Turning right with angular speed: {angular_speed}")
        msg = Twist()
        msg.angular.z = -angular_speed
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.get_logger().debug("Stopping the robot")
        # Ensure robot stops completely
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def set_speed(self, speed):
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def set_turn(self, angular_speed):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_speed
        self.cmd_vel_pub.publish(msg)
