import rclpy
from rclpy.node import Node
from my_interface.srv import StopRobot  # Custom Service Message
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.cli = self.create_client(StopRobot, 'stop_robot_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service...')
        
        self.req = StopRobot.Request()
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.turtlebot_lidar_callback, 10)
    
    def turtlebot_lidar_callback(self, msg):
        distances = list(msg.ranges)
        min_distance = min([d for d in distances if not math.isinf(d) and not math.isnan(d)], default=float('inf'))
        min_angle = distances.index(min_distance) * 360 / len(distances) - 180
        
        self.get_logger().info(f"LiDAR Data: Min Distance = {min_distance:.2f}m at {min_angle:.2f}°")
        
        if min_distance < 0.18:
            self.get_logger().warn(f"Obstacle detected at {min_angle:.2f}° | {min_distance:.2f}m. Sending stop request.")
            self.req.stop = True
        else:
            self.req.stop = False
        
        self.call_service()
    
    def call_service(self):
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Service confirmed stop request.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = ServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
