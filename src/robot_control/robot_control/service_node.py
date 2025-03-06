import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # ใช้ข้อความแทน Service
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        
        # Subscriber รับคำสั่งจาก server_node
        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10)
        
        # Subscriber อ่านค่าจาก LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rp_lidar_callback,
            10)

        # Publisher ควบคุมการเคลื่อนที่
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.Back_block = True

    def command_callback(self, msg):
        """ รับคำสั่งจาก server_node และควบคุมหุ่นยนต์ """
        if msg.data == "stop":
            self.get_logger().warn("❌ STOP command received! Halting robot.")
            self.twist = Twist()
            self.cmd_vel_pub.publish(self.twist)

    def rp_lidar_callback(self, msg):
        """ ค้นหาอุปสรรคในแต่ละทิศทาง """
        # รีเซ็ตคำสั่งควบคุม
        self.twist = Twist()

        # กำหนดค่าเริ่มต้นของระยะขั้นต่ำ
        min_forward = float('inf')
        min_ccw = float('inf')
        min_cw = float('inf')
        min_back = float('inf')

        # พารามิเตอร์การตรวจจับ
        max_dist = 0.24
        active_threshold = 0.25
        max_linear_speed = 0.24
        max_angular_speed = 0.7

        # ตรวจสอบข้อมูล LiDAR
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance > max_dist:
                continue

            angle_radian = msg.angle_min + i * msg.angle_increment
            angle_degree = math.degrees(angle_radian) % 360

            # Forward: 175° - 185°
            if 175 <= angle_degree < 185:
                min_forward = min(min_forward, distance)
            # CCW: 265° - 275°
            elif 265 <= angle_degree < 275:
                min_ccw = min(min_ccw, distance)
            # CW: 85° - 95°
            elif 85 <= angle_degree < 95:
                min_cw = min(min_cw, distance)
            # Backward: 355°-360° หรือ 0°-5°
            elif ((355 <= angle_degree < 360) or (0 <= angle_degree < 5)) and self.Back_block:
                min_back = min(min_back, distance)

        # หาอุปสรรคที่ใกล้ที่สุด
        region_distances = {
            'forward': min_forward,
            'ccw': min_ccw,
            'cw': min_cw,
            'back': min_back
        }

        selected_region = None
        selected_distance = active_threshold  

        for region, dist in region_distances.items():
            if dist < selected_distance:
                selected_distance = dist
                selected_region = region

        # ถ้าไม่มีอุปสรรคที่ใกล้เกิน threshold → หยุดหุ่นยนต์
        if selected_region is None:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # คำนวณความเร็วตามทิศที่เลือก
        if selected_region == 'forward':
            self.twist.linear.x = max_linear_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'ccw':
            self.twist.angular.z = max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'cw':
            self.twist.angular.z = -max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'back':
            self.twist.linear.x = -max_linear_speed * (1 - (selected_distance / active_threshold))

        # แสดงผลลัพธ์
        self.get_logger().info(
            f"{selected_region} [{self.twist.linear.x:.2f} , {self.twist.angular.z:.2f}], Range:{selected_distance:.2f} m"
        )

        self.cmd_vel_pub.publish(self.twist)

def main():
    rclpy.init()
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
