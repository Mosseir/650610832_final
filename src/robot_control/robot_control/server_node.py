import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from robot_control.srv import ControlRobot
import math

class LidarServiceNode(Node):
    def __init__(self):
        super().__init__('server_node')
        
        # สร้าง Service Client เพื่อส่งคำขอไปยัง service_node
        self.cli = self.create_client(ControlRobot, 'control_robot_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service...')
        
        # สร้าง Subscriber สำหรับรับข้อมูลจาก LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.lidar_callback,
            10)
        
        # กำหนดค่าระยะที่ต้องการหยุดหุ่นยนต์
        self.stop_threshold = 0.18  

    def lidar_callback(self, msg):
        """ ตรวจสอบว่ามีวัตถุใกล้เกินกำหนดหรือไม่ แล้วส่ง Service Request ถ้าจำเป็น """
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
                
                # ส่งคำขอไปยัง service_node ให้หยุดหุ่นยนต์
                self.call_service()
                break  # หยุด loop ทันทีที่พบอุปสรรคใกล้

    def call_service(self):
        """ ส่งคำขอให้หุ่นยนต์หยุดการเคลื่อนที่ """
        req = ControlRobot.Request()
        req.action = "stop"
        future = self.cli.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """ ตรวจสอบการตอบกลับจาก service_node """
        try:
            response = future.result()
            self.get_logger().info(f"✅ Service Response: {response.action}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = LidarServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()