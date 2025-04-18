import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from irobot_create_msgs.msg import InterfaceButtons, HazardDetectionVector, HazardDetection
from irobot_create_msgs.msg import LightringLeds, LedColor
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import Dock
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from enum import Enum
import time

qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

class SensorFSM(Node):
    UNINITIALIZED= -1
    INITIALIZED = 0
    UNDOCK = 1
    RANDOM_ROAMING = 2
    CHASING = 3
    AVOIDING = 4
    PUSHING = 5
    DOCKING = 6

    def __init__(self):
        super().__init__('sensor_fsm')
        self.get_logger().info("Sensor Control Node Initialized")
        self.current_state = self.UNINITIALIZED
        self.active_state = [self.RANDOM_ROAMING, self.CHASING, self.AVOIDING, self.PUSHING]
        self.mode = self.CHASING  # Default mode
        self.dir = "NULL"
        self.distance = 1000
        self.angle = 0
        self.init_avoiding = 0
        
        # Exploration timer to return to dock
        self.exploration_timer = None
        self.exploration_duration = 180.0 # 3 mins
        self.create_timer(1.0, self.check_exploration_timeout)
        
        self.led_publisher = self.create_publisher(LightringLeds, '/Robot5/cmd_lightring', 10)
        
    def intialize(self):
        self.get_logger().info("Initializing...")
        self.set_state(self.INITIALIZED)
        
        # Start counting the exploration time
        self.exploration_start_time = self.get_clock().now()
        
        # Perform undocking after initialization
        self.perform_undock()

    def check_exploration_timeout(self):
        if self.current_state in self.active_state:
            if self.exploration_start_time :
                # Check if the exploration duration has elapsed
                elapsed_time = (self.get_clock().now() - self.exploration_start_time).nanoseconds * 1e-9
                if elapsed_time > self.exploration_duration:
                    self.perform_dock()
    
    def perform_dock(self):
        self.set_state(self.DOCKING)
        
        dock_client = ActionClient(self, Dock, '/Robot5/dock')
        
        self.get_logger().info("Waiting for Dock action server...")
        if not dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock action server not available!")
            return
    
        goal_msg = Dock.Goal()
        future = dock_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future) # Wait for the goal to be accepted
        if not future.result().accepted:
            self.get_logger().error("Dock goal was rejected!")
            return

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future) # Wait for the result

        if result_future.result().status == 4:
            self.get_logger().info("Docking succeeded!")
        else:
            self.get_logger().error("Docking failed!")
        
    def perform_undock(self):
        self.set_state(self.UNDOCK)

        undock_client = ActionClient(self, Undock, '/Robot5/undock')

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
            self.process_hazard_detection,
            qos_profile_sensor_data
        )

        
    def process_lidar_data(self, msg):
        if self.current_state not in [self.RANDOM_ROAMING, self.CHASING]:
            return
        
        # Ensure LiDAR data is valid and update state
        max_value = 0
        max_id = "NULL"
        for reading in msg.readings:
            if "dock" in reading.header.frame_id:
                self.get_logger().info("Dock detected")
                continue
            
            value = reading.value
            if value >= max_value:
                max_value = value
                max_id = reading.header.frame_id
        self.set_distance_angle(max_value, max_id)

        if max_value > 20:
            #self.get_logger().info(f"Lidar value, angle, distance: {max_value}, {self.angle}, {self.distance}")
            self.set_state(self.PUSHING)
        else:
            self.set_state(self.RANDOM_ROAMING)
            
    def process_hazard_detection(self, msg):
        if self.current_state not in self.active_state:
            return
        
        if msg.detections:
            for hazard in msg.detections:
                #self.get_logger().warn(f"Hazard detected: {hazard.type}.")

                # Handle CLIFF or WHEEL_DROP hazards
                if hazard.type in [HazardDetection.CLIFF, HazardDetection.WHEEL_DROP]:
                    self.get_logger().warn(f"Executing hazard handling.")
                    self.behavior_logic.handle_hazard()
                    return

                # Handle BUMP hazards
                elif hazard.type == HazardDetection.BUMP:
                    self.get_logger().info("Bump detected. Switching to AVOIDING state.")
                    self.set_state(self.AVOIDING)
                    return


    def set_led_color(self, colors):
        msg = LightringLeds()
        msg.override_system = True
        
        for color in colors:
            led_color = LedColor()
            led_color.red = color["red"]
            led_color.green = color["green"]
            led_color.blue = color["blue"]
            msg.leds.append(led_color)

        print(f"Setting LED color: {msg.leds}")
        self.led_publisher.publish(msg)

    def set_state(self, state):
        if self.current_state == self.AVOIDING:
            if time.time() - self.init_avoiding < 10:
                self.get_logger().info(f"Staying to state: {self.current_state}")
                return
        elif state == self.AVOIDING:
            self.init_avoiding = time.time()
        if self.current_state != state:
            self.current_state = state
            self.get_logger().info(f"Transitioning to state: {state}")
        
        if state == self.RANDOM_ROAMING:
            self.set_led_color([{"red": 0, "green": 255, "blue": 0}] * 6)  # Green for RANDOM_ROAMING
        elif state == self.CHASING:
            self.set_led_color([{"red": 255, "green": 0, "blue": 0}] * 6)  # Red for CHASING
        elif state == self.AVOIDING:
            self.set_led_color([{"red": 0, "green": 0, "blue": 255}] * 6)  # Blue for AVOIDING
        elif state == self.DOCKING:
            self.set_led_color([{"red": 255, "green": 255, "blue": 0}] * 6)  # Yellow for DOCKING
        else:
            self.set_led_color([{"red": 255, "green": 255, "blue": 255}] * 6)  # White for other states

    def get_state(self):
        return self.current_state
    
    def set_distance_angle(self, max_val, dir):
        if max_val < 20:
            self.angle = 0
        elif dir == "ir_intensity_side_left":
            self.angle = 90
        elif dir == "ir_intensity_left":
            self.angle = 60
        elif dir == "ir_intensity_front_left":
            self.angle = 30
        elif dir == "ir_intensity_front_center_left":
            self.angle = 10
        elif dir == "ir_intensity_front_center_right":
            self.angle = -10
        elif dir == "ir_intensity_front_right":
            self.angle = -30
        elif dir == "ir_intensity_right":
            self.angle = -60
        self.distance = 15/max_val