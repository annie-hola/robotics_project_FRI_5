import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import InterfaceButtons, HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector
from rclpy.qos import qos_profile_sensor_data

class SensorFSM(Node):
    RANDOM_ROAMING = "RANDOM_ROAMING"
    CHASING = "CHASING"
    AVOIDING = "AVOIDING"

    def __init__(self):
        super().__init__('sensor_fsm')
        self.get_logger().info("Sensor Control Node Initialized")
        self.current_state = self.RANDOM_ROAMING
        self.mode = self.CHASING  # Default mode
        self.dir = "NULL"
        self.lidar_sub = self.create_subscription(
            IrIntensityVector,
            '/Robot5/ir_intensity',
            self.process_lidar_data,
            qos_profile_sensor_data
        )
        self.button_sub = self.create_subscription(
            InterfaceButtons,
            '/Robot5/joy',
            self.handle_button_press,
            qos_profile_sensor_data
        )
        self.bumper_sub = self.create_subscription(
            HazardDetectionVector,
            '/Robot5/hazard_detection',
            self.handle_bumper_event,
            qos_profile_sensor_data
        )

    def process_lidar_data(self, msg):
        self.get_logger().info("aaaaaaaa")
        # Ensure LiDAR data is valid and update state
        closest_distance = 0
        max_id = "NULL"
        for reading in msg.readings:
            value = reading.value
            if value >= closest_distance:
                closest_distance = value
                max_id = reading.header.frame_id
        self.dir = max_id
        self.get_logger().info(f"Lidar max value: {closest_distance}")


        self.get_logger().info(f"{closest_distance}")
        if closest_distance > 20:
            self.set_state(self.AVOIDING if self.mode == self.AVOIDING else self.CHASING)
        else:
            self.set_state(self.RANDOM_ROAMING)

    def handle_button_press(self, msg):
        # Cycle between CHASING and AVOIDING modes
        if msg.button_1.is_pressed:
            self.mode = self.AVOIDING if self.mode == self.CHASING else self.CHASING
            self.get_logger().info(f"Mode switched to: {self.mode}")

    def handle_bumper_event(self, msg):
        # Ensure bumper events are handled correctly
        if msg.detections:
            if self.current_state == self.CHASING:
                self.mode = self.AVOIDING  # Switch to AVOIDING mode if chasing
                self.set_state(self.AVOIDING)
                self.get_logger().info(f"Mode switched to: {self.mode} due to collision")
            else:
                self.mode = self.CHASING if self.mode == self.AVOIDING else self.AVOIDING
                self.get_logger().info(f"Mode toggled to: {self.mode} due to collision")

    def set_state(self, state):
        self.get_logger().info(f"Transitioning to state: {state}")
        self.current_state = state

    def get_state(self):
        return self.current_state