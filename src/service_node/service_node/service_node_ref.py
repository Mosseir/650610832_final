import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from my_interface.srv import ControlRobot  # ต้องสร้าง Service message ก่อนใช้งาน
import numpy as np
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.cli = self.create_client(ControlRobot, 'control_robot_service')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service...')
        
        self.req = ControlRobot.Request()
        self.subscription = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rp_lidar_callback,
            10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.Back_block = True  # กำหนดค่าเริ่มต้น

    def rp_lidar_callback(self, msg):
        """ ตรวจจับสิ่งกีดขวางและควบคุมการเคลื่อนที่ """
        self.call_service()  # เรียกใช้งาน Service ก่อนตัดสินใจเคลื่อนที่

        self.twist = Twist()
        obstacle_detected = False
        
        min_dist = 0.10
        max_dist = 0.40
        max_linear_speed = 0.24
        min_linear_speed = 0.10
        max_angular_speed = 0.50
        min_angular_speed = 0.10

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):  
                continue

            if distance < max_dist:  
                obstacle_detected = True
                angle_radian = msg.angle_min + i * msg.angle_increment
                angle_degree = math.degrees(angle_radian) % 360
                distance_rounded = round(distance, 2)

                if distance_rounded < min_dist:
                    linear_speed = max_linear_speed  # ถ้าระยะห่างน้อยกว่าค่า min_dist ก็ให้ถอยหลังด้วยความเร็วสูงสุด
                elif distance_rounded > max_dist:
                    linear_speed = 0  # ถ้าระยะห่างมากกว่า max_dist ก็หยุด
                else:
                    # คำนวณความเร็วในการถอยหลังตามระยะห่าง
                    linear_speed = ((distance_rounded - min_dist) / (max_dist - min_dist)) * (min_linear_speed - max_linear_speed) + max_linear_speed

                # คำนวณ angular_speed ให้หมุนได้ตามระยะห่าง
                angular_speed = (1 - (distance_rounded - min_dist) / (max_dist - min_dist)) * max_angular_speed

                self.get_logger().info(f'🚀 Backward Speed: Linear {linear_speed:.2f}, Angular {angular_speed:.2f}')

                if 255 <= angle_degree < 285:
                    self.get_logger().info(f"CCW: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.twist.angular.z = angular_speed  # หมุน CCW
                    self.twist.linear.x = 0.0 # ถอยหลัง
                elif 65 <= angle_degree < 105:
                    self.get_logger().info(f"CW: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.twist.angular.z = -angular_speed  # หมุน CW
                    self.twist.linear.x = 0.0 # ถอยหลัง
                elif 165 <= angle_degree < 195:
                    self.get_logger().info(f"Forward: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.twist.linear.x = linear_speed
                    self.twist.angular.z = 0.0
                elif (345 <= angle_degree < 360 or 0 <= angle_degree < 15) and self.Back_block:
                    self.get_logger().info(f"Back: {distance_rounded} m")
                    # ถอยหลังโดยใช้ความเร็วที่คำนวณ
                    self.twist.linear.x = -linear_speed
                    self.twist.angular.z = 0.0
                break

        self.cmd_vel_pub.publish(self.twist)

    def call_service(self):
        """ เรียกใช้ Service เพื่อเช็คว่าหุ่นยนต์ควรหยุดหรือไม่ """
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        """ ตรวจสอบคำตอบจาก ServerNode และหยุดหุ่นยนต์หากจำเป็น """
        try:
            response = future.result()
            if response.action == "stop":
                self.get_logger().warn("❌ Server: Stop detected, halting robot!")
                self.twist = Twist()  # หยุดหุ่นยนต์
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
