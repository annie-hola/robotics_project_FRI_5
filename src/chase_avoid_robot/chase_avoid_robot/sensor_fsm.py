import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.msg import HazardDetectionVector, HazardDetection
from irobot_create_msgs.msg import IrIntensityVector, IrOpcode
from irobot_create_msgs.action import Undock
from irobot_create_msgs.action import Dock
from irobot_create_msgs.action import NavigateToPosition
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
    DOCKING = 5

    def __init__(self):
        super().__init__('sensor_fsm')
        self.get_logger().info("Sensor Control Node Initialized")
        self.current_state = self.UNINITIALIZED
        self.active_state = [self.RANDOM_ROAMING, self.CHASING, self.AVOIDING]
        self.dir = "NULL"
        self.distance = 1000
        self.angle = 0
        self.init_avoiding = 0
        self.see_dock = 0
        
        # Exploration timer to return to dock
        self.exploration_timer = None
        self.exploration_duration = 60.0 # 3 mins
        self.create_timer(1.0, self.check_exploration_timeout)
                
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

        # Navigate to the docking station
        navigate_client = ActionClient(self, NavigateToPosition, '/Robot5/navigate_to_position')

        self.get_logger().info("Waiting for NavigateToPosition action server...")
        if not navigate_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateToPosition action server not available!")
            self.perform_dock() # Retry docking
            return  

        goal_msg = NavigateToPosition.Goal()
        goal_msg.achieve_goal_heading = True
        goal_msg.goal_pose.pose.position.x = -0.15
        goal_msg.goal_pose.pose.position.y = 0.
        goal_msg.goal_pose.pose.position.z = 0.
        goal_msg.goal_pose.pose.orientation.x = 0.
        goal_msg.goal_pose.pose.orientation.y = 0.
        goal_msg.goal_pose.pose.orientation.z = 0.
        goal_msg.goal_pose.pose.orientation.w = 1.
        future = navigate_client.send_goal_async(goal_msg)

        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        except Exception as e:
            self.get_logger().error(f"Error while waiting for NavigateToPosition goal result: {e}")
            self.execute_dock_action()  # Skip navigation and directly attempt docking
            return

        time.sleep(20)
        self.execute_dock_action()
    
    def execute_dock_action(self):
        dock_client = ActionClient(self, Dock, '/Robot5/dock')

        self.get_logger().info("Waiting for Dock action server...")
        if not dock_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Dock action server not available!")
            return

        goal_msg = Dock.Goal()
        future = dock_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, future) 
        if not future.result().accepted:
            self.get_logger().error("Dock goal was rejected!")
            return

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)  # Wait for the result

        if result_future.result().status == 4:  # 4 = SUCCEEDED
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
            self.perform_dock() # dock if undock is rejected
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

        self.opcode = self.create_subscription(
            IrOpcode,
            '/Robot5/ir_opcode',
            self.process_opcode,
            qos_profile_sensor_data
        )

    def process_lidar_data(self, msg):
        if self.current_state not in self.active_state:
            return
        
        # Find closest object to the robot using IR sensors
        max_value = 0
        max_reading = "NULL"
        for reading in msg.readings:
            value = reading.value
            if value >= max_value:
                max_value = value
                max_reading = reading # reading.header.frame_id

        self.set_distance_angle(max_value, max_reading.header.frame_id)

        # No object => max_value < 15
        if max_value > 20:  
            if time.time() - self.see_dock < 2:
                self.set_state(self.AVOIDING)
            else:
                self.set_state(self.CHASING)
        else:
            self.set_state(self.RANDOM_ROAMING)
            
    def process_hazard_detection(self, msg):
        if self.current_state not in self.active_state:
            return
        
        if msg.detections:
            for hazard in msg.detections:
                # Handle CLIFF or WHEEL_DROP hazards
                if hazard.type in [HazardDetection.CLIFF, HazardDetection.WHEEL_DROP]:
                    self.get_logger().warn(f"Void detected !")
                    self.set_state(self.AVOIDING)
                    return

                # Handle BUMP hazards
                elif hazard.type == HazardDetection.BUMP:
                    self.get_logger().warn("Bump detected !")
                    self.set_state(self.AVOIDING)
                    return
    
    def process_opcode(self, msg):
        self.see_dock = time.time()

    def set_state(self, state):
        if self.current_state == self.AVOIDING:
            if time.time() - self.init_avoiding < 2:
                return
        elif state == self.AVOIDING:
            self.init_avoiding = time.time()
        if self.current_state != state:
            self.current_state = state
            self.get_logger().info(f"Transitioning to state: {state}")

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