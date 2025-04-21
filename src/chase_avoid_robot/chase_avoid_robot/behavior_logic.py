from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
import random

class BehaviorLogic:
    def __init__(self, fsm: SensorFSM, movement: MovementControl):
        self.fsm = fsm
        self.movement = movement
        self.started_avoiding = False
        self.started_roaming = False
        self.turn_speed = 1.

    def execute_behavior(self):
        state = self.fsm.get_state()
        if state == SensorFSM.RANDOM_ROAMING:
            self.roaming()
        elif state == SensorFSM.CHASING:
            angle = self.fsm.angle
            distance = self.fsm.distance
            self.chase_object(distance, angle)
        elif state == SensorFSM.AVOIDING:
            self.avoiding()

    def roaming(self):
        """
        Roam in circle
        """
        if not self.started_roaming:
            self.started_roaming = True
            self.turn_speed = random.random() + 0.2
        speed = 0.2
        self.movement.move_forward(speed)
        self.movement.turn_left(self.turn_speed)

    def chase_object(self, distance, angle):
        """
        Turn the robot toward the nearest object.
        Adapt the speed with respect to the distance to the object
        """
        speed = min(distance*0.5 + 0.1, 1.0)
        turn_speed = angle/90
        self.movement.turn_left(turn_speed)
        self.movement.move_forward(speed)

    def avoiding(self):
        """
        Make a random turn to move in the opposite direction
        """
        if not self.started_avoiding:
            self.started_avoiding = True
            self.turn_speed = random.random() + 1.
        self.movement.set_turn(self.turn_speed)
