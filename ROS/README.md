# ROS framework and package communication

This folder contains the ROS packages responsible for communication, navigation,
manual control, and cooperation between the AeroCat V2 ASV and the UAV. The
system was developed using ROS and operates through a Wi-Fi network connecting
the onboard computer, the ASV, and the UAV.

The framework enables autonomous navigation, real-time telemetry exchange,
remote operation, and cooperative missions such as UAV inspection and landing on
the moving ASV platform.

## ASV Quick start

### 1. Clone the repository

Clone the project to your machine:

```bash
git clone https://github.com/aerocat24/aerocat-v2.git
```

### 2. Flash the ESP32 firmware

The ESP32 firmware was developed using the PlatformIO plugin for VS Code. Open
VS Code, import the project located in the `boat_esp32` folder, build it, and
flash the firmware to your ESP32.

### 3. Build the ROS workspace

The project was developed using ROS Noetic. Go to the workspace:

```bash
cd aerocat-v2/ROS/boat_drone_ws
```

Build the workspace:

```bash
catkin build
```

### 4. Test the boat communication

With the ESP32 powered on and connected to the Wi-Fi network, start
`rosbridge`:

```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

In two other terminals, check whether the boat topics are being received:

```bash
rostopic echo /boat/compass
```

```bash
rostopic echo /boat/gps
```

If no data appears in the topics, check that:

- The computer and the ESP32 are connected to the same Wi-Fi network.
- The GPS module has signal. This is usually indicated by a blinking red light.

### 5. Check the joystick mapping

The joystick button mapping only needs to be checked once. To connect the
Bluetooth joystick to ROS, run:

```bash
roslaunch boat remote_control_joy.launch
```

In another terminal, monitor the joystick messages:

```bash
rostopic echo /boat/joy
```

Compare the received mapping with the one defined in
`remote_control_joy_node.py`. Then check whether the commands below are working
correctly:

| Control | Action |
| --- | --- |
| Left analog stick up | Sends PWM to the right motor |
| Right analog stick up | Sends PWM to the left motor |
| `L1` | Decreases the maximum power |
| `R1` | Increases the maximum power |
| `select` | Toggles between manual and autonomous mode |

When moving each analog stick, check that the corresponding motor is activated.

### 6. Start the position node

Start the boat position node:

```bash
rosrun boat position_node.py
```

Before starting autonomous navigation, update the latitude and longitude
variables in the code with the desired destination.

### 7. Start the control node

Then start the boat control node:

```bash
rosrun boat control_node.py
```

For safety, the boat does not start moving automatically. Press the `select`
button on the joystick to switch to autonomous mode. In this mode, the boat will
navigate to the GPS coordinate defined in the position node.

If the behavior is not as expected, press `select` again. The boat will return
to manual mode and stop autonomous navigation.

## Communication overview

The system architecture is divided into three main groups of nodes:

- ASV-related nodes.
- UAV-related nodes.
- Cooperative nodes responsible for interaction between both platforms.

The communication structure is based on ROS topics, where nodes publish and
subscribe to information in real time.

## Main ROS nodes

| Node | Function |
| --- | --- |
| `joy_node` | Reads Bluetooth gamepad inputs |
| `remote_control_boat` | Interprets manual commands for the ASV |
| `remote_control_tello` | Interprets manual commands for the UAV |
| `tello_driver` | Communication interface with the DJI Tello UAV |
| `aux_odom` | Applies offset corrections to UAV odometry |
| `position_p2p` | Calculates waypoint navigation parameters |
| `control_p2p` | Generates PWM commands for ASV motors |
| `interface` | Graphical interface for mission monitoring |
| `recognition` | QR code detection and landing assistance |

## Main ROS topics

| Topic | Description |
| --- | --- |
| `/Joy1` | Gamepad commands for the ASV |
| `/Joy2` | Gamepad commands for the UAV |
| `/cmd_vel` | UAV velocity commands |
| `/takeoff` | UAV takeoff command |
| `/land` | UAV landing command |
| `/status` | UAV status information |
| `/odom` | UAV visual odometry |
| `/gps` | ASV GPS data |
| `/compass` | ASV heading and orientation |
| `/distance` | Distance to the target waypoint |
| `/delta_theta` | Heading error relative to the waypoint |
| `/point_reached` | Indicates when a waypoint is reached |
| `/next_point` | Triggers navigation to the next waypoint |
| `/pwm` | PWM commands for ASV motors |
| `/camera` | UAV camera video stream |
| `/points` | Relative coordinates generated for UAV landing |

## System operation

### 1. Manual control

1. Two Bluetooth gamepads are connected to the onboard computer.
2. The `joy_node` receives joystick inputs and publishes them on the `/Joy1`
   and `/Joy2` topics.
3. The `remote_control_boat` and `remote_control_tello` nodes interpret the
   commands independently for each vehicle.
4. For the UAV, commands are converted into velocity references and published
   on `/cmd_vel`.
5. Takeoff and landing are executed through the `/takeoff` and `/land`
   topics.

## 2. ASV autonomous navigation

1. The ASV continuously publishes GPS and compass data through the `/gps`
   and `/compass` topics.
2. The `position_p2p` node calculates:
   - Distance to the target waypoint.
   - Required heading angle.
   - Waypoint completion status.
3. The computed information is published on:
   - `/distance`
   - `/delta_theta`
   - `/point_reached`
4. The `control_p2p` node converts these values into PWM motor commands.
5. PWM signals are published on the `/pwm` topic and sent to the ASV motors.
6. Once a waypoint is reached, the system waits for a `/next_point`
   command before continuing the mission.

## 3. UAV localization and monitoring

1. The `tello_driver` node provides:
   - Vehicle telemetry.
   - Battery information.
   - Flight status.
   - Visual odometry data.
2. Odometry data from `/odom` are processed by the `aux_odom` node.
3. Offset corrections are applied to improve UAV position estimation.
4. The corrected data are used for cooperative operations and landing
   assistance.

## 4. Cooperative ASV-UAV mission

1. The `interface` node provides a real-time graphical interface for:
   - Vehicle monitoring.
   - Trajectory visualization.
   - Dynamic waypoint definition.
2. During the landing procedure, a QR code marker positioned on the ASV is
   detected by the UAV camera.
3. The UAV camera stream is published on the `/camera` topic.
4. The `recognition` node processes the video stream and estimates the
   relative position between the UAV and the QR marker.
5. Landing reference coordinates are generated and published on the
   `/points` topic.
6. Using these coordinates, the UAV performs a safe landing on the moving
   ASV platform.

## Operational modes

| Mode | Description |
| --- | --- |
| Manual mode | Direct operator control using Bluetooth gamepads |
| Autonomous ASV mode | Waypoint-based autonomous navigation |
| Cooperative mode | Simultaneous ASV-UAV mission execution |
| Assisted landing mode | UAV landing using QR-code visual tracking |

## Notes

- All communication is performed over a local Wi-Fi network.
- The ASV manual control system acts as a contingency mode during autonomous
  navigation.
- Real-time cooperation depends on stable wireless communication between all
  ROS nodes.
- The UAV landing system relies on proper QR code visibility and camera
  calibration.
