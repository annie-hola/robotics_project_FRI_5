from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
import random
import time

class BehaviorLogic:
    def __init__(self, fsm: SensorFSM, movement: MovementControl):
        self.fsm = fsm
        self.movement = movement
        self.roaming_mode = "forward"
        self.roaming_start_time = time.time()
        self.current_turn_direction = "left"
        self.started_avoiding = False

    def execute_behavior(self):
        state = self.fsm.get_state()
        if state == SensorFSM.RANDOM_ROAMING:
            self.roaming()
        elif state == SensorFSM.CHASING:
            angle = self.fsm.angle
            distance = self.fsm.distance
            self.chase_object(distance, angle)
                # self.movement.move_forward(0.5)
        elif state == SensorFSM.AVOIDING:
            self.avoiding()
        elif state == SensorFSM.PUSHING:
            self.pushing()

    def roaming(self):
        speed = 1.0
        turn_speed = 2.
        self.movement.move_forward(speed)
        self.movement.turn_left(turn_speed)

    def chase_object(self, distance, angle):
        speed = min(distance * 0.5, 1.0)
        turn_speed = angle/90
        self.movement.turn_left(turn_speed)
        self.movement.move_forward(speed)

    def avoiding(self):
        if not self.started_avoiding:
            self.started_avoiding = True
            self.movement.stop()
        turn_speed = 2.
        self.movement.turn_left(turn_speed)
        
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