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

    def lidar_callback(self, msg):
        """ ตรวจสอบข้อมูลจาก LiDAR และส่งคำสั่งเมื่อพบวัตถุใกล้เกินไป """
        self.get_logger().info(f"📡 LiDAR Callback Triggered! First 5 Ranges: {msg.ranges[:5]}")

        obstacle_detected = False  # ตัวแปรเช็คว่าพบวัตถุหรือไม่

        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                self.get_logger().debug(f"⚠️ Ignoring invalid value at index {i}")
                continue  

            self.get_logger().info(f"🔍 Checking: {distance:.3f} m at index {i}")

            if distance < self.stop_threshold:
                angle_radian = msg.angle_min + i * msg.angle_increment
                angle_degree = math.degrees(angle_radian) % 360
                self.get_logger().warn(f"⚠️ Obstacle Found: {angle_degree:.2f}° ➡ STOP sent!")

                # ส่งคำสั่ง "stop" ไปที่ /robot_command
                stop_msg = String()
                stop_msg.data = "stop"
                self.publisher.publish(stop_msg)

                obstacle_detected = True
                break  # ออกจาก loop ทันทีที่พบวัตถุ

        if not obstacle_detected:
            self.get_logger().info("✅ No obstacle detected, robot continues moving.")

def main():
    rclpy.init()
    node = LidarServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
