import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorFSM(Node):
    RANDOM_ROAMING = "RANDOM_ROAMING"
    CHASING = "CHASING"
    AVOIDING = "AVOIDING"

    def __init__(self):
        super().__init__('sensor_fsm')
        self.current_state = self.RANDOM_ROAMING
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.process_lidar_data,
            10
        )

    def process_lidar_data(self, msg):
        # Example: Process LiDAR data and update state
        closest_distance = min(msg.ranges)
        if closest_distance < 0.5:
            self.set_state(self.AVOIDING)
        elif closest_distance < 2.0:
            self.set_state(self.CHASING)
        else:
            self.set_state(self.RANDOM_ROAMING)

    def set_state(self, state):
        self.get_logger().info(f"Transitioning to state: {state}")
        self.current_state = state

    def get_state(self):
        return self.current_state