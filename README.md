# Project Robotics

## Introduction

Robot has two modes, chase and avoid. Robot will run around randomly until it detects an object (like a person or a ball). If it is in a chase mode, it will go towards the object, following it (chasing). In avoiding mode, it will turn and try to run away from the object. We can add switching the mode after the robot makes contact with the object. We can implement it with a Finite State Machine (FSM), using sensors to detect an object.

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
