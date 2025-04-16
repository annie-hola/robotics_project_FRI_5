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
            angle = self.fsm.get_angle()
            distance = self.fsm.get_distance()
            self.chase_object(distance, angle)
                # self.movement.move_forward(0.5)
        elif state == SensorFSM.AVOIDING:
            self.avoiding()

    def roaming(self):
        now = time.time()

        if self.roaming_mode == "turning":
            if now - self.roaming_start_time < 1.5:
                if self.current_turn_direction == "left":
                    self.movement.turn_left(1.0)
                else:
                    self.movement.turn_right(1.0)
            else:
                self.roaming_mode = "forward"
                self.roaming_start_time = now
                self.movement.stop()

        elif self.roaming_mode == "forward":
            if now - self.roaming_start_time < 2.0:
                self.movement.move_forward(0.5)
            else:
                self.roaming_mode = "turning"
                self.roaming_start_time = now
                self.movement.stop()
                self.current_turn_direction = random.choice(["left", "right"])


    def chase_object(self, distance, angle):
        self.fsm.get_logger().info(f"Chasing: angle={angle:.2f}, distance={distance:.2f}")

        speed = min(distance * 0.5, 1.0)

        if abs(angle) < 5:
            self.movement.set_speed(speed)
            self.movement.set_turn(0.0)

        elif abs(angle) < 20:
            self.movement.set_speed(speed * 0.8) 
            turn_speed = angle * 0.3             
            self.movement.set_turn(turn_speed)

        else:
            self.movement.set_speed(0.0)          
            turn_speed = angle * 0.6              
            self.movement.set_turn(turn_speed)


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