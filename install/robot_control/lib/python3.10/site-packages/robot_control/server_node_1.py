import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy

class LidarServiceNode(Node):
    def __init__(self):
        super().__init__('server_node')

        # Publisher สำหรับส่งคำสั่งไปยัง /robot_command
        self.publisher = self.create_publisher(String, '/robot_command', 10)

        # เปลี่ยน QoS เป็น BEST_EFFORT ให้ตรงกับ LiDAR
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # Subscriber สำหรับรับข้อมูลจาก LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        self.stop_threshold = 0.16  # ระยะที่ต้องหยุด
        self.prev_command = "resume"  # เริ่มต้นจากสถานะที่หุ่นยนต์เคลื่อนที่
        self.obstacle_detected = False  # สถานะการพบสิ่งกีดขวาง

    def lidar_callback(self, msg):
        """ ตรวจสอบข้อมูลจาก LiDAR และส่งคำสั่งเมื่อพบวัตถุใกล้เกินไป """
        self.get_logger().info(f"📡 LiDAR Callback Triggered! First 5 Ranges: {msg.ranges[:5]}")

        obstacle_detected = False  # ตัวแปรเช็คว่าพบวัตถุหรือไม่

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                self.get_logger().debug(f"⚠️ Ignoring invalid value at index {i}")
                continue  

            if distance < self.stop_threshold:
                obstacle_detected = True
                break  # ออกจาก loop ทันทีที่พบวัตถุ

        # หากพบสิ่งกีดขวาง
        if obstacle_detected:
            if not self.obstacle_detected:  # ถ้ายังไม่เคยส่ง stop มาก่อน
                self.send_command("stop")
                self.obstacle_detected = True
        else:
            if self.obstacle_detected:  # ถ้าสิ่งกีดขวางหายไป
                self.send_command("resume")
                self.obstacle_detected = False

    def send_command(self, command):
        """ ส่งคำสั่งใหม่เฉพาะเมื่อมีการเปลี่ยนแปลงคำสั่ง """
        if command != self.prev_command:
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
            self.prev_command = command
            self.get_logger().info(f"🚀 Command Sent: {command}")

def main():
    rclpy.init()
    node = LidarServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
