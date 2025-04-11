from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
import random
import time

class BehaviorLogic:
    def __init__(self, fsm: SensorFSM, movement: MovementControl):
        self.fsm = fsm
        self.movement = movement
        self.dir = "NULL"
        self.count = 0

    def execute_behavior(self):
        state = self.fsm.get_state()
        self.fsm.get_logger().info(f"Executing state: {state}")
        if state == SensorFSM.RANDOM_ROAMING:
            self.roaming()
        elif state == SensorFSM.CHASING:
            self.chase_object(0.6, 1.0)
        elif state == SensorFSM.AVOIDING:
            self.avoiding()

    def roaming(self):
        direction = 'left' # random.choice(['left', 'right'])
        speed = 1. # m/s from 0.2 to 1.0 maximum
        turn_speed = 0.1*self.count # rad/s 0.5 - 2.0

        self.movement.move_forward(speed)
        if direction == 'left':
            self.movement.turn_left(turn_speed)
        else: 
            self.movement.turn_right(turn_speed)
        self.count+=1
        time.sleep(0.5)


    def chase_object(self):
        speed = 1.
        turn_speed = 2.
        direction = self.fsm.get_direction()
        self.movement.move_forward(speed)

        if direction == "ir_intensity_side_left":
            self.movement.turn_left(1.0*turn_speed)

        if direction == "ir_intensity_left":
            self.movement.turn_left(0.7*turn_speed)

        if direction == "ir_intensity_front_left":
            self.movement.turn_left(0.4*turn_speed)

        if direction == "ir_intensity_front_center_left":
            self.movement.turn_left(0.2*turn_speed)

        if direction == "ir_intensity_front_center_right":
            self.movement.turn_right(0.2*turn_speed)

        if direction == "ir_intensity_front_right":
            self.movement.turn_right(0.4*turn_speed)

        if direction == "ir_intensity_right":
            self.movement.turn_right(0.7*turn_speed)
        time.sleep(0.5)



    def avoiding(self):
        self.movement.stop()
        if direction == 'left':
            self.movement.turn_left(1.0)
        else:
            self.movement.turn_right(1.0)
        time.sleep(1) #adjust later
        self.movement.stop()
        self.movement.move_forward(0.6)
        time.sleep(2) #adjust later

        self.movement.stop()