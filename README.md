# TurtleBot3 Web Teleoperation Package

A ROS 2 package that provides a web-based interface for teleoperating a TurtleBot3 robot. The interface includes both joystick and keyboard controls, real-time odometry display, and 3D robot visualization.

## Features

- **Dual Control Modes**:
  - Virtual Joystick Control
  - Keyboard Control (WASD + Space)
  - Easy switching between control modes
- **Real-time Odometry Display**:
  - Position (X, Y, Z)
  - Linear Velocity
  - Angular Velocity
- **3D Robot Visualization**:
  - Live URDF model viewer
  - Interactive camera controls
- **Web-based Interface**:
  - Modern, responsive design
  - Real-time updates
  - Cross-platform compatibility

## Prerequisites

- ROS 2 (tested on Humble)
- Python 3.10 or higher
- TurtleBot3 packages
- Web browser with WebSocket support

## Installation

1. Clone the repository:
```bash
cd ~/ros2_ws/src
git clone <your-repo-url> web_teleop_app_pkg
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select web_teleop_app_pkg
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch the web teleoperation interface:
```bash
ros2 launch web_teleop_app_pkg web_teleop_app.launch.py
```

2. Open your web browser and navigate to:
```
http://localhost:8000
```

3. Control the robot:
   - Use the virtual joystick for intuitive control
   - Click "Switch to Keyboard Control" to use WASD keys
   - Monitor robot position and velocity in the odometry display
   - View the robot's 3D model in the URDF viewer

## Control Modes

### Joystick Control
- Drag the joystick to control the robot
- Release to stop
- Direction and distance from center determine speed and turning

### Keyboard Control
- W: Move forward
- S: Move backward
- A: Turn left
- D: Turn right
- Space: Stop

## Architecture

The package consists of three main components:

1. **Web Frontend** (`web_app/`):
   - HTML/CSS/JavaScript interface
   - Virtual joystick implementation
   - 3D visualization using Three.js
   - Real-time odometry display

2. **WebSocket Backend** (`web_teleop_app_pkg/websocket_backend.py`):
   - ROS 2 node for robot control
   - WebSocket server for frontend communication
   - Odometry data handling
   - Command velocity publishing

3. **Launch System** (`launch/web_teleop_app.launch.py`):
   - Starts the web server
   - Launches the WebSocket backend
   - Optional TurtleBot3 simulation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Authors

- krissagar95 (Krishna_Sagar@artc.a-star.edu.sg)

## Acknowledgments

- TurtleBot3 team for the robot platform
- ROS 2 community for the framework
- Three.js for 3D visualization
- NippleJS for the virtual joystick 