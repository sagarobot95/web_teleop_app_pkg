import asyncio
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import websockets

node = None  # Global node variable
robot_description = None  # Global robot description variable
current_odom = None  # Global odometry variable

class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_backend')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to robot_description
        self.robot_description_sub = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            10
        )

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def robot_description_callback(self, msg):
        global robot_description
        robot_description = msg.data
        print("Received robot_description:", robot_description[:200], "...")

    def odom_callback(self, msg):
        global current_odom
        current_odom = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        }

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)

async def handler(websocket):
    global robot_description, current_odom
    async for message in websocket:
        try:
            data = json.loads(message)
            if data.get('type') == 'get_robot_description':
                if robot_description:
                    await websocket.send(json.dumps({
                        'type': 'robot_description',
                        'data': robot_description
                    }))
            elif data.get('type') == 'get_odom':
                if current_odom:
                    await websocket.send(json.dumps({
                        'type': 'odom',
                        'data': current_odom
                    }))
            else:
                linear = float(data.get('linear', 0.0))
                angular = float(data.get('angular', 0.0))
                node.publish_cmd_vel(linear, angular)
        except Exception as e:
            print(f"Error handling message: {e}")

async def main_async():
    global node
    rclpy.init()
    node = WebTeleopNode()
    start_server = websockets.serve(
        handler,
        '0.0.0.0', 8765
    )
    print("WebSocket server started on ws://0.0.0.0:8765")
    async with start_server:
        await asyncio.Future()  # run forever
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main_async()) 