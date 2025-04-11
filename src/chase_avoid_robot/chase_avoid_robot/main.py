import rclpy
from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
from chase_avoid_robot.behavior_logic import BehaviorLogic

def main(args=None):
    rclpy.init(args=args)

    fsm = SensorFSM()
    movement = MovementControl()
    behavior = BehaviorLogic(fsm, movement)

    try:
        while rclpy.ok():
            rclpy.spin_once(fsm, timeout_sec=0.1)
            behavior.execute_behavior()
            print("Main loop running...")
    except KeyboardInterrupt:
        pass
    finally:
        fsm.destroy_node()
        movement.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()