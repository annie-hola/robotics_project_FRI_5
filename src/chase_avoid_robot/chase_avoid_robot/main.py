import rclpy
from chase_avoid_robot.sensor_fsm import SensorFSM
from chase_avoid_robot.movement_control import MovementControl
from chase_avoid_robot.behavior_logic import BehaviorLogic

def main(args=None):
    rclpy.init(args=args)

    fsm = SensorFSM() # Initialize the state machine
    movement = MovementControl() # Initialize the movement control
    behavior = BehaviorLogic(fsm, movement) # Initialize the behavior logic
    fsm.intialize()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(fsm, timeout_sec=0.1) # spin the state machine every 0.1 seconds
            behavior.execute_behavior()
    except Exception as e:
        fsm.get_logger().error(f"An error occurred: {e}")
    except KeyboardInterrupt:
        fsm.get_logger().info("Shutting down...")
    finally:
        fsm.destroy_node()
        movement.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()