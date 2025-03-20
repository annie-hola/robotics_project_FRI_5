# Documentation

## Team 1: Sensor, Movement, and FSM

### Responsibilities

- **Sensor Data Acquisition and Processing**
- **Basic Movement Control**
- **Finite State Machine (FSM) Implementation**
- **Integration of Sensor Data with FSM**

### Detailed Implementation

#### Sensor Data Acquisition

- **Topic**: Subscribe to the LiDAR scan topic (`/scan`).
- **Processing**:
  - Filter out irrelevant data points.
  - Identify potential object locations based on distance thresholds.
  - Calculate the relative position and distance of detected objects.

#### Basic Movement Control

- **Topic**: Publish `Twist` messages to the velocity command topic (`/cmd_vel`).
- **Functions**:
  - `move_forward(speed)`
  - `move_backward(speed)`
  - `turn_left(angular_speed)`
  - `turn_right(angular_speed)`
  - `stop()`

#### FSM Implementation

- **States**:
  - `RANDOM_ROAMING`
  - `CHASING`
  - `AVOIDING`
- **Transitions**:
  - `RANDOM_ROAMING` → `CHASING`: Object detected within range.
  - `RANDOM_ROAMING` → `AVOIDING`: Object detected too close.
  - `CHASING` → `AVOIDING`: Object becomes too close or collision sensor is triggered.
  - `AVOIDING` → `RANDOM_ROAMING`: Object moves out of close range.
- **Functions**:
  - `set_state(state)`
  - `get_state()`

#### Sensor-FSM Integration

- Use processed sensor data to trigger FSM state transitions.
- Ensure smooth integration between sensor data and FSM logic.

---

## Team 2: Behavior and Logic

### Responsibilities

- **Random Roaming Behavior**
- **Chasing Behavior**
- **Avoiding Behavior**
- **Behavior Integration with FSM**

### Detailed Implementation

#### Random Roaming Behavior (`RANDOM_ROAMING`)

- **Functionality**:
  - Generate random movement commands (random turn angles and distances).
  - Use movement functions from Team 1 to execute commands.
  - Implement obstacle avoidance using LiDAR data.

#### Chasing Behavior (`CHASING`)

- **Functionality**:
  - Control the robot to move towards the detected object.
  - Use position data from Team 1 to calculate direction and distance.
  - Adjust speed based on the distance to the object.

#### Avoiding Behavior (`AVOIDING`)

- **Functionality**:
  - Control the robot to move away from the detected object.
  - Calculate a direction away from the object using position data from Team 1.
  - Ensure the robot avoids other obstacles while moving away.

#### Behavior Integration with FSM

- Call the appropriate behavior functions based on the current FSM state.
- Ensure seamless integration between behavior logic and FSM transitions.

---

## General Notes

- **ROS 2 Communication**:
  - `/scan`: LiDAR data topic.
  - `/cmd_vel`: Velocity command topic.
- **Testing**:
  - Unit tests for individual functions.
  - Integration tests for the overall system.
