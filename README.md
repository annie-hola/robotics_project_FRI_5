# Project Robotics

## Introduction

This project involves programming an iRobot Create® 3 to explore a tabletop, detect objects, and push them off the table while avoiding hazards like cliffs and the dock. The robot uses a Finite State Machine (FSM) to manage its behavior, transitioning between states such as roaming, chasing, avoiding, and pushing.

Our slides available in this [link](https://docs.google.com/presentation/d/1wqL3F1s97yaeExnx8migyhF6UZM9Uc-kJlXCrEvAmGI/edit?usp=sharing)

## Project Idea

### Objective:
- Develop a robot capable of:
  - Exploring a tabletop systematically.
  - Detecting objects using IR intensity sensors.
  - Pushing objects off the table while avoiding the dock and cliffs.

### Key Features:
- **Object Detection**: Use IR intensity sensors to detect objects and determine their direction.
- **Cliff and Dock Avoidance**: Use hazard detection sensors to avoid falling off the table and to recognize the dock.
- **State Machine**: Implement a Finite State Machine (FSM) to manage robot behavior.
- **Exploration Timer**: Return to the dock after a fixed duration of exploration.

## How It Works

### Architecture:
- **Sensors**:
  - IR intensity sensors for object detection.
  - Hazard detection sensors for cliffs, bumps, and wheel drops.
- **Actuators**:
  - Motors for movement (forward, backward, turning).
- **State Machine**:
  - Controls robot behavior based on sensor inputs.
  - States: `RANDOM_ROAMING`, `CHASING`, `AVOIDING`, `DOCKING`.

### Code Organization:
- **`sensor_fsm.py`**:
  - Implements the state machine.
  - Handles sensor data and state transitions.
  - Publishes LED feedback.
- **`movement_control.py`**:
  - Provides movement commands (e.g., forward, backward, turning).
  - Publishes velocity commands to `/cmd_vel`.
- **`behavior_logic.py`**:
  - Implements behavior for each state (e.g., roaming, chasing, avoiding, pushing).
  - Handles hazard avoidance and object pushing.
- **`main.py`**:
  - Initializes the state machine, movement control, and behavior logic.
  - Spins the ROS 2 nodes.

### Features:
1. **Object Detection**:
   - Detects objects using IR intensity sensors and determines their direction.
2. **Cliff and Dock Avoidance**:
   - Uses hazard detection sensors to avoid cliffs and recognize the dock.
3. **State Machine**:
   - Manages transitions between states based on sensor inputs.
   - Ensures safe and efficient operation.
4. **Exploration Timer**:
   - Returns to the dock after a fixed duration of exploration.


## Steps to Run

Build package

```
colcon build
```

Source changes

```
source install/setup.bash
```

Run the node

```
ros2 run chase_avoid_robot chase_avoid_robot
```

## Difficulties and Misc Observations

### Challenges:

1. **State Machine Design**:
   - Managing smooth transitions between states.
   - Avoiding getting stuck in certain states (e.g., `AVOIDING`).

2.**Simulation Setup**:
   - Setting up a simulation environment for the iRobot Create® 3 in ROS 2 was challenging due to:
     - Limited documentation for integrating the robot's specific sensors and actuators.
     - Differences between simulated and real-world behavior, requiring additional calibration and testing.

## Directory Structure

```
chase_avoid_robot/
├── package.xml
├── setup.cfg
├── setup.py
├── chase_avoid_robot/
│   ├── __init__.py
│   ├── sensor_fsm.py
│   ├── movement_control.py
│   ├── behavior_logic.py
│   └── main.py
├── resource/
│   └── chase_avoid_robot
├── test/
│   ├── test_sensor_fsm.py
│   ├── test_movement_control.py
│   ├── test_behavior_logic.py
│   └── test_integration.py
```

## Contributors
- HA Kieu Anh
- DENIEL Théo
- Mariia Soltys
- Kaidiliya Dilixiati

