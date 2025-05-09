from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import ament_index_python.packages

pkg_share = ament_index_python.packages.get_package_share_directory('web_teleop_app_pkg')
WEB_APP_PATH = os.path.join(pkg_share, 'web_app')
BACKEND_PATH = os.path.join(pkg_share, 'web_teleop_app_pkg', 'websocket_backend.py')


def generate_launch_description():
    return LaunchDescription([
        # 1. Start the web server (Python HTTP server)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000'],
            cwd=WEB_APP_PATH,
            output='screen',
        ),
        # 2. Start backend websocket server
        ExecuteProcess(
            cmd=['python3.10', BACKEND_PATH],
            output='screen',
        ),
        # 3. (Optional) Start TurtleBot3 simulation (burger)
        # Uncomment the following block if you want to launch simulation as well
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py', 'model:=burger'],
        #     output='screen',
        # ),
    ]) 