import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        
        self.stop_robot = False  # กำหนดให้หุ่นยนต์เริ่มต้นเคลื่อนที่ได้

        # Subscriber จาก server_node
        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10)
        
        # Subscriber อ่านค่าจาก RPLiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rp_lidar_callback,
            10)

        # Publisher สำหรับส่งคำสั่งการเคลื่อนที่
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.Back_block = True  # ป้องกันการถอยหลังในบางกรณี

    def command_callback(self, msg):
        """ รับคำสั่งจาก server_node """
        if msg.data == "stop":
            self.get_logger().warn("❌ STOP command received! Sending [0.00, 0.00].")
            self.stop_robot = True  # หยุดหุ่นยนต์
        elif msg.data == "resume":
            self.get_logger().info("✅ RESUME command received! Returning to LiDAR-based control.")
            self.stop_robot = False  # กลับมาเคลื่อนที่

    def rp_lidar_callback(self, msg):
        """ ใช้ข้อมูล LiDAR ควบคุมหุ่นยนต์ """
        if self.stop_robot:
            self.get_logger().info("🛑 Robot stopped.")
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return  # ถ้าต้องหยุด ก็ไม่ต้องคำนวณ LiDAR

        # ถ้าไม่ถูกสั่งให้หยุด → ใช้ LiDAR ควบคุมการเคลื่อนที่
        self.process_lidar(msg)

    def process_lidar(self, msg):
        """ วิเคราะห์ข้อมูลจาก LiDAR แล้วกำหนดการเคลื่อนที่ """
        min_forward = float('inf')

        max_dist = 0.24
        active_threshold = 0.25
        max_linear_speed = 0.24

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance > max_dist:
                continue

            angle_radian = msg.angle_min + i * msg.angle_increment
            angle_degree = math.degrees(angle_radian) % 360

            if 175 <= angle_degree < 185:
                min_forward = min(min_forward, distance)

        self.get_logger().info(f"Min forward distance: {min_forward}")

        if min_forward < active_threshold:
            self.twist.linear.x = 0.0  # หยุดถ้ามีสิ่งกีดขวาง
        else:
            self.twist.linear.x = max_linear_speed  # เคลื่อนที่ไปข้างหน้า

        self.cmd_vel_pub.publish(self.twist)

def main():
    rclpy.init()
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
