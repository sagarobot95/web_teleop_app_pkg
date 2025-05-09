# Web-based TurtleBot3 Teleoperation

A ROS2 package that provides web-based teleoperation for TurtleBot3 (burger model) using a virtual joystick interface.

## Features

- Web-based virtual joystick control
- Real-time robot control via WebSocket
- Simple and intuitive user interface
- Compatible with TurtleBot3 burger model

## Requirements

- ROS2 (tested on Humble)
- Python 3.x
- Python packages:
  - rclpy
  - websockets

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <your-repo-url>
```

2. Install dependencies:
```bash
pip3 install websockets
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select web_teleop_app_pkg
```

## Usage

1. Source your ROS2 workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

2. Launch the web teleoperation node:
```bash
ros2 launch web_teleop_app_pkg web_teleop_app.launch.py
```

3. Open your web browser and navigate to:
```
http://localhost:8080
```

4. Use the virtual joystick to control your TurtleBot3:
   - Move the joystick up/down for forward/backward motion
   - Move left/right for turning
   - Release to stop the robot

## Architecture

- Frontend: HTML/JavaScript web interface with virtual joystick
- Backend: ROS2 node with WebSocket server
- Communication: WebSocket for real-time control commands

## License

MIT License

## Author

krissagar95 