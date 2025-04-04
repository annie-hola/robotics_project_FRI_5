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
        if state == SensorFSM.RANDOM_ROAMING:
            self.roaming(self.movement)
        elif state == SensorFSM.CHASING:
            self.movement.move_forward(0.5)
        elif state == SensorFSM.AVOIDING:
            self.avoiding(self.movement)

    def roaming(self):
        direction = random.choice(['left', 'right'])
        #speed = 0.2 #m/s from 0.2 to 1.0 maximum
        turn_speed = 0.0 # rad/s 0.5 - 2.0

        turn_speed = random.uniform(0.0, 2.0)
        if direction == 'left':
            self.movement.turn_left(1.0)
        else: 
            self.movement.turn_right(1.0)
        time.sleep(turn_speed)

        self.movement.move_forward(0.2)
        time.sleep(2.0)
        self.movement.stop()


    def chase_object(self, distance, angle):
        if distance > 0.5:
            speed = min(distance * 0.5, 1.0)
            self.movement.set_speed(speed)
            self.movement.set_turn(angle)
        else:
            self.movement.set_speed(0.1)


    def avoiding(self):
        direction = random.choice(['left', 'right'])

        self.movement.move_backward(0.5)
        time.sleep(1.5)
        self.movement.stop()
        if direction == 'left':
            self.movement.turn_left(1.0)
        else:
            self.movement.turn_rigt(1.0)
        time.sleep(1) #adjust later
        self.movement.stop()
        self.movement.move_forward(0.6)
        time.sleep(2) #adjust later
        self.movement.stop()