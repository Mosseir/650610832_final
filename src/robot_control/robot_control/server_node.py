import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # ใช้ข้อความธรรมดาแทน Service
import math

class LidarServiceNode(Node):
    def __init__(self):
        super().__init__('server_node')
        
        # เปลี่ยนจาก Service Client เป็น Publisher
        self.publisher = self.create_publisher(String, '/robot_command', 10)

        # สร้าง Subscriber สำหรับรับข้อมูลจาก LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)
        
        self.stop_threshold = 0.18  # ระยะที่ต้องการหยุดหุ่นยนต์

    def lidar_callback(self, msg):
        """ ตรวจสอบว่ามีวัตถุใกล้เกินกำหนดหรือไม่ แล้วส่งคำสั่งผ่าน Topic """
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue  # ข้ามค่าที่ผิดปกติ
            
            if distance < self.stop_threshold:  # พบวัตถุที่เข้าใกล้เกินไป
                angle_radian = msg.angle_min + i * msg.angle_increment
                angle_degree = math.degrees(angle_radian) % 360
                
                # แสดงข้อมูลใน Terminal
                self.get_logger().warn(
                    f"⚠️ Obstacle Found : {angle_degree:.2f}° Range : {distance:.2f} m. ➡ STOP sent!"
                )
                
                # ส่งคำสั่งให้ service_node หยุดหุ่นยนต์
                msg = String()
                msg.data = "stop"
                self.publisher.publish(msg)
                break  # หยุด loop ทันทีที่พบอุปสรรคใกล้

def main():
    rclpy.init()
    node = LidarServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
