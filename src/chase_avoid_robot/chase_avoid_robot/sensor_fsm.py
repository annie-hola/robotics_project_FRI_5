import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import InterfaceButtons, HazardDetectionVector
from irobot_create_msgs.msg import LightringLeds, LedColor
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import Undock
from rclpy.qos import qos_profile_sensor_data

from enum import Enum

# Enumerating the states of the FSM
class State(Enum):
    UNINITIALIZED= -1
    INITIALIZED = 0
    UNDOCK = 1
    RANDOM_ROAMING = 2
    CHASING = 3
    AVOIDING = 4
class SensorFSM(Node):
    UNINITIALIZED= -1
    INITIALIZED = 0
    UNDOCK = 1
    RANDOM_ROAMING = 2
    CHASING = 3
    AVOIDING = 4

    def __init__(self):
        super().__init__('sensor_fsm')
        self.get_logger().info("Sensor Control Node Initialized")
        self.current_state = self.UNINITIALIZED
        self.mode = self.CHASING  # Default mode
        self.dir = "NULL"
        
        self.led_publisher = self.create_publisher(LightringLeds, '/cmd_lightring', 10)
        
    def intialize(self):
        self.get_logger().info("Initializing...")
        self.set_state(self.INITIALIZED)
        
        # Perform undocking after initialization
        self.perform_undock()

    def perform_undock(self):
        self.set_state(self.UNDOCK)

        undock_client = ActionClient(self, Undock, '/undock')

        self.get_logger().info("Waiting for Undock action server...")
        if not undock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Undock action server not available!")
            self.perform_undock() # Retry undocking
            return  

        goal_msg = Undock.Goal()
        future = undock_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future)
        if not future.result().accepted: # Check if the goal was rejected
            self.get_logger().error("Undock goal was rejected!")
            self.perform_undock() # Retry undocking
            return 

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().status != 4:  # 4 = SUCCEEDED -> NOT SUCCEEDED
            self.perform_undock()  # Retry undocking
            return
        else:
            self.get_logger().info("Undocking succeeded!")
            self.set_state(self.RANDOM_ROAMING)  # Transition to RANDOM_ROAMING after successful undocking

        self.initialize_subscriptions()

    def initialize_subscriptions(self):
        self.lidar_sub = self.create_subscription(
            IrIntensityVector,
            '/Robot5/ir_intensity',
            self.process_lidar_data,
            qos_profile_sensor_data
        )

        self.bumper_sub = self.create_subscription(
            HazardDetectionVector,
            '/Robot5/hazard_detection',
            self.handle_bumper_event,
            qos_profile_sensor_data
        )
        
    def process_lidar_data(self, msg):
        if self.current_state not in [self.RANDOM_ROAMING, self.CHASING, self.AVOIDING]:
            return
        
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

    def handle_bumper_event(self, msg):
        if self.current_state not in [self.RANDOM_ROAMING, self.CHASING, self.AVOIDING]:
            return
        
        # Ensure bumper events are handled correctly
        if msg.detections:
            if self.current_state == self.CHASING:
                self.mode = self.AVOIDING  # Switch to AVOIDING mode if chasing
                self.set_state(self.AVOIDING)
                self.get_logger().info(f"Mode switched to: {self.mode} due to collision")
            else:
                self.mode = self.CHASING if self.mode == self.AVOIDING else self.AVOIDING
                self.get_logger().info(f"Mode toggled to: {self.mode} due to collision")


    def set_led_color(self, red, green, blue):
        msg = LightringLeds()

        color = LedColor()
        color.red = int(red * 255)
        color.green = int(green * 255)
        color.blue = int(blue * 255)

        msg.leds = [color] * 6 
        self.led_publisher.publish(msg)  # Publish the message to the cmd_lightring topic

    def set_state(self, state):
        self.get_logger().info(f"Transitioning to state: {state}")
        self.current_state = state
        
        if state == self.RANDOM_ROAMING:
            self.set_led_color(0.0, 1.0, 0.0)  # Green for RANDOM_ROAMING
        elif state == self.CHASING:
            self.set_led_color(1.0, 0.0, 0.0)  # Red for CHASING
        elif state == self.AVOIDING:
            self.set_led_color(0.0, 0.0, 1.0)  # Blue for AVOIDING
        else:
            self.set_led_color(1.0, 1.0, 1.0)  # White for other states

    def get_state(self):
        return self.current_state