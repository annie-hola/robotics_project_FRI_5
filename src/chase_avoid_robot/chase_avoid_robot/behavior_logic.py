from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl

class BehaviorLogic:
    def __init__(self, fsm: SensorFSM, movement: MovementControl):
        self.fsm = fsm
        self.movement = movement

    def execute_behavior(self):
        state = self.fsm.get_state()
        if state == SensorFSM.RANDOM_ROAMING:
            self.movement.move_forward(0.2)
        elif state == SensorFSM.CHASING:
            self.movement.move_forward(0.5)
        elif state == SensorFSM.AVOIDING:
            self.movement.move_backward(0.3)