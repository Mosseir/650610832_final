import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_control.srv import ControlRobot
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        
        self.cli = self.create_client(ControlRobot, 'control_robot_service')
        self.req = ControlRobot.Request()
        
        # ใช้ flag แทน wait_for_service() เพื่อป้องกันบล็อกโปรแกรม
        self.service_available = False  
        
        # ตรวจสอบ service ทุกๆ 2 วินาที
        self.create_timer(2.0, self.check_service_availability)

        self.subscription = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rp_lidar_callback,
            10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.Back_block = True

    def check_service_availability(self):
        """ ตรวจสอบว่าบริการ server_node ใช้งานได้หรือไม่ โดยไม่บล็อกโปรแกรม """
        if not self.service_available:
            if self.cli.service_is_ready():
                self.get_logger().info("✅ Server service is now available!")
                self.service_available = True
            else:
                self.get_logger().warn("Waiting for service...")

    def rp_lidar_callback(self, msg):
        """ ค้นหาอุปสรรคในแต่ละทิศและเลือกทิศที่มีอุปสรรคใกล้ที่สุด """
        self.call_service()
        
        # รีเซ็ตคำสั่งควบคุม
        self.twist = Twist()

        # กำหนดค่าเริ่มต้นของระยะขั้นต่ำสำหรับแต่ละทิศ
        min_forward = float('inf')
        min_ccw = float('inf')
        min_cw = float('inf')
        min_back = float('inf')

        # พารามิเตอร์การตรวจจับ
        max_dist = 0.24  
        active_threshold = 0.25  
        max_linear_speed = 0.24
        max_angular_speed = 0.7

        # วนลูปตรวจสอบข้อมูล LaserScan ทั้งหมด
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            if distance > max_dist:
                continue

            angle_radian = msg.angle_min + i * msg.angle_increment
            angle_degree = math.degrees(angle_radian) % 360

            # Forward: 175° - 185°
            if 175 <= angle_degree < 185:
                if distance < min_forward:
                    min_forward = distance
            # CCW: 265° - 275°
            elif 265 <= angle_degree < 275:
                if distance < min_ccw:
                    min_ccw = distance
            # CW: 85° - 95°
            elif 85 <= angle_degree < 95:
                if distance < min_cw:
                    min_cw = distance
            # Backward: 355°-360° หรือ 0°-5°
            elif ((355 <= angle_degree < 360) or (0 <= angle_degree < 5)) and self.Back_block:
                if distance < min_back:
                    min_back = distance

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

        # หากไม่มีอุปสรรคที่ใกล้เกิน threshold ให้หยุดหุ่นยนต์
        if selected_region is None:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # คำนวณความเร็วตามทิศที่เลือก
        if selected_region == 'forward':
            self.twist.linear.x = max_linear_speed * (1 - (selected_distance / active_threshold))
            self.twist.angular.z = 0.0
        elif selected_region == 'ccw':
            self.twist.linear.x = 0.0
            self.twist.angular.z = max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'cw':
            self.twist.linear.x = 0.0
            self.twist.angular.z = -max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'back':
            self.twist.linear.x = -max_linear_speed * (1 - (selected_distance / active_threshold))
            self.twist.angular.z = 0.0

        # แสดงผลลัพธ์
        self.get_logger().info(
            f"{selected_region} "
            f"[{self.twist.linear.x:.2f} , {self.twist.angular.z:.2f}], Range:{selected_distance:.2f} m"
        )

        self.cmd_vel_pub.publish(self.twist)

    def call_service(self):
        """ เรียกใช้ Service เพื่อเช็คว่าหุ่นยนต์ควรหยุดหรือไม่ """
        if not self.service_available:
            return  # ไม่ทำอะไรถ้า Service ยังไม่พร้อม
        
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """ ตรวจสอบคำตอบจาก ServerNode และหยุดหุ่นยนต์หากจำเป็น """
        try:
            response = future.result()
            if response.action == "stop":
                self.get_logger().warn("❌ Server: Stop detected, halting robot!")
                self.twist = Twist()
                self.cmd_vel_pub.publish(self.twist)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()