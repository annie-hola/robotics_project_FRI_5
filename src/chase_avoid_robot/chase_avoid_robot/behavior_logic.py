from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
import random
import time

class BehaviorLogic:
    def __init__(self, fsm: SensorFSM, movement: MovementControl):
        self.fsm = fsm
        self.movement = movement

    def execute_behavior(self):
        state = self.fsm.get_state()
        self.fsm.get_logger().debug(f"Executing behavior for state: {state}")
        if state == SensorFSM.RANDOM_ROAMING:
            self.roaming()
        elif state == SensorFSM.CHASING:
            print("Should be chasing")
            self.movement.move_forward(0.5)
        elif state == SensorFSM.AVOIDING:
            self.avoiding()

    def roaming(self):
        self.fsm.get_logger().info("Roaming state")
        direction = 'left' #random.choice(['left', 'right'])
        turn_speed = random.uniform(0.5, 2.0)  # Adjusted range for better turning
        self.fsm.get_logger().debug(f"Roaming: Turning {direction} for {turn_speed} seconds")

        if direction == 'left':
            self.movement.turn_left(1.0)
        else:
            self.movement.turn_right(1.0)
        time.sleep(turn_speed)

        self.movement.move_forward(2.0)
        time.sleep(2.0)
        self.movement.stop()

    def chase_object(self, distance, angle):
        self.fsm.get_logger().info("Chasing state")
        if distance > 0.5:
            speed = min(distance * 0.5, 1.0)
            self.movement.set_speed(speed)
            self.movement.set_turn(angle)
        else:
            self.movement.set_speed(0.1)

    def avoiding(self):
        self.fsm.get_logger().info("Avoiding state")
        direction = random.choice(['left', 'right'])
        self.fsm.get_logger().debug(f"Avoiding: Moving backward and turning {direction}")

        self.movement.move_backward(0.5)
        time.sleep(1.5)
        self.movement.stop()
        if direction == 'left':
            self.movement.turn_left(1.0)
        else:
            self.movement.turn_right(1.0)
        time.sleep(1.0)
        self.movement.stop()
        self.movement.move_forward(0.6)
        time.sleep(2.0)
        self.movement.stop()
        
        # TODO: Resume roaming after avoiding
        # self.fsm.set_state(SensorFSM.RANDOM_ROAMING)
        
    def handle_hazard(self):
        self.fsm.get_logger().info("Handling hazard...")
        self.movement.stop()

        # Move backward to avoid the hazard
        self.movement.move_backward(0.3)
        time.sleep(1.5)
        self.movement.stop()

        # Turn away from the hazard
        direction = random.choice(['left', 'right'])
        if direction == 'left':
            self.movement.turn_left(1.0)
        else:
            self.movement.turn_right(1.0)
        time.sleep(1.0)
        self.movement.stop()

        # Resume roaming after avoiding the hazard
        self.fsm.set_state(SensorFSM.RANDOM_ROAMING)
        
    def pushing(self):
        self.fsm.get_logger().info("Pushing object")

        # Move forward to push the object
        self.movement.move_forward(0.5)
        while True:
            # Check if cliff sensors detect a drop
            if self.fsm.current_state == SensorFSM.AVOIDING: 
                self.fsm.get_logger().info("Cliff detected. Object likely pushed off.")
                self.movement.stop()
                self.fsm.set_state(SensorFSM.RANDOM_ROAMING)
                break