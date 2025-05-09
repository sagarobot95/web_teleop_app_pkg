import asyncio
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import websockets

node = None  # Global node variable

class WebTeleopNode(Node):
    def __init__(self):
        super().__init__('web_teleop_backend')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)

async def handler(websocket):
    async for message in websocket:
        try:
            data = json.loads(message)
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