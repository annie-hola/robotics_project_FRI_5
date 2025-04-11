import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import IrIntensityVector

class SensorFSM(Node):
    RANDOM_ROAMING = "RANDOM_ROAMING"
    START_CHASING = "START_CHASING"
    CHASING = "CHASING"
    START_AVOIDING = "START_AVOIDING"
    AVOIDING = "AVOIDING"

    def __init__(self):
        super().__init__('sensor_fsm')
        self.get_logger().info("Sensor Control Node Initialized")
        self.current_state = self.RANDOM_ROAMING
        self.dir = "NULL"
        self.lidar_sub = self.create_subscription(
            IrIntensityVector,
            '/ir_intensity',
            self.process_lidar_data,
            10
        )

    def process_lidar_data(self, msg):
        # Example: Process LiDAR data and update state
        max_value = 0
        max_id = "NULL"
        for reading in msg.readings:
            value = reading.value
            if value >= max_value:
                max_value = value
                max_id = reading.header.frame_id
        """
        self.get_logger().info(f"Lidar max value: {max_value}")
        self.dir = max_id
        if max_value > 500:
            self.set_state(self.AVOIDING)
        elif max_value > 20:
            self.set_state(self.CHASING)
        else:
            self.set_state(self.RANDOM_ROAMING)"""
        
        if self.current_state == self.RANDOM_ROAMING:
            if max_value > 20:
                self.set_state(self.START_CHASING)
            


    def set_state(self, state):
        self.get_logger().info(f"Transitioning to state: {state}")
        self.current_state = state

    def get_state(self):
        return self.current_state
    
    def get_direction(self):
        return self.dir