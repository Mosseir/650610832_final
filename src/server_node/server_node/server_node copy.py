import rclpy
from rclpy.node import Node
from my_interface.srv import ControlRobot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        self.srv = self.create_service(ControlRobot, 'control_robot_service', self.handle_request)

        # Subscriber รับค่าจาก LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.turtlebot_lidar_callback,
            10)

        # Publisher ส่งคำสั่งควบคุมหุ่นยนต์
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stop_robot = False
        self.obstacle_angle = 0
        self.obstacle_distance = float('inf')

    def turtlebot_lidar_callback(self, msg):
        distances = list(msg.ranges)
        min_distance = min(distances)
        min_angle = distances.index(min_distance) * 360 / len(distances) - 180  # คำนวณองศา

        # แสดงค่าที่ได้รับจาก LiDAR
        self.get_logger().info(f"LiDAR Data: Min Distance = {min_distance:.2f}m at {min_angle:.2f}°")

        twist = Twist()

        if min_distance < 0.15:  # หยุดหุ่นยนต์หากมีสิ่งกีดขวางในระยะ < 150 มม.
            self.stop_robot = True
            self.obstacle_angle = min_angle
            self.obstacle_distance = min_distance
            self.get_logger().warn(f"Obstacle detected at {self.obstacle_angle:.2f}° | {self.obstacle_distance:.2f}m. Robot will stop.")

            # ส่งคำสั่งให้หยุด
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.stop_robot = False
            # ส่งคำสั่งให้หุ่นยนต์เคลื่อนที่ไปข้างหน้า
            twist.linear.x = 0.2  # ให้หุ่นยนต์เคลื่อนที่ด้วยความเร็ว 0.2 m/s

        self.cmd_vel_publisher.publish(twist)  # ส่งค่าไปยังหุ่นยนต์

    def handle_request(self, request, response):
        if self.stop_robot:
            self.get_logger().warn(f"Action: Stop (Obstacle detected at {self.obstacle_angle:.2f}° | {self.obstacle_distance:.2f}m)")
            response.action = "stop"
            # ส่งข้อมูลมุมและระยะห่างให้กับ service_node
            response.obstacle_distance = self.obstacle_distance
            response.obstacle_angle = self.obstacle_angle
        else:
            self.get_logger().info("Action: Move (No obstacle detected)")
            response.action = "move"
        
        return response

def main():
    rclpy.init()
    node = ServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
