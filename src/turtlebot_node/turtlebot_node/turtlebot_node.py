#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class TurtleBotNode(Node):
    def __init__(self):
        super().__init__('turtlebot_node')
        
        # กำหนดค่า Threshold ของระยะใกล้สุด (150 มม. หรือ 0.15 เมตร)
        self.min_distance_threshold = 0.15  

        # Subscriber รับข้อมูล LiDAR จาก TurtleBot3 (/scan)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Subscriber รับคำสั่งการเคลื่อนที่จาก RPLidar Node (/cmd_vel)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publisher ส่งคำสั่งการเคลื่อนที่ไปยัง TurtleBot3
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # เก็บค่าคำสั่งล่าสุดที่ได้รับ (ค่าเริ่มต้นเป็น None)
        self.last_cmd = None
    
    def scan_callback(self, msg):
        """ ตรวจจับสิ่งกีดขวางและหยุดการเคลื่อนที่หากอยู่ใกล้กว่า 150 มม. """
        valid_ranges = [r for r in msg.ranges if r > 0 and r < float('inf')]  # กรองค่า inf และ 0 ออก
        
        if not valid_ranges:
            self.get_logger().warn("No valid LiDAR readings received.")
            return  # ถ้าไม่มีค่าที่ใช้ได้ก็ไม่ต้องทำอะไร

        min_distance = min(valid_ranges)  # หาค่าระยะต่ำสุดที่ตรวจจับได้

        if min_distance < self.min_distance_threshold:
            self.get_logger().warn(f"Obstacle detected! Distance: {min_distance:.3f} m. Stopping TurtleBot3.")

            # หยุดการเคลื่อนที่
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
        else:
            # ส่งคำสั่งการเคลื่อนที่ล่าสุด (ถ้ามีคำสั่งที่ได้รับมาก่อนหน้า)
            if self.last_cmd:
                self.cmd_vel_pub.publish(self.last_cmd)

    def cmd_vel_callback(self, msg):
        """ รับคำสั่งจาก RPLidar Node และเก็บไว้เป็นค่าล่าสุด """
        if msg.linear.x != 0 or msg.angular.z != 0:  # รับเฉพาะคำสั่งที่มีการเคลื่อนที่
            self.last_cmd = msg
            self.get_logger().info(f"Received movement command: linear={msg.linear.x}, angular={msg.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
