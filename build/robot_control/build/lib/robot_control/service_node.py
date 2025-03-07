import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String  # ใช้ข้อความธรรมดาแทน Service
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        
        # Flag for stop LiDAR
        self.stop_robot = False  

        # Subscriber from server_node
        self.command_sub = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10)
        
        # Subscriber reading LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rp_lidar_callback,
            10)

        # Publisher for movement control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.Back_block = True

    def command_callback(self, msg):
        """ รับคำสั่งจาก server_node และควบคุมหุ่นยนต์ """
        if msg.data == "stop":
            self.get_logger().warn("❌ STOP command received! Sending [0.00, 0.00].")
            self.stop_robot = True  # Stop robot temporarily
        else:
            self.stop_robot = False  # Resume robot movement

    def rp_lidar_callback(self, msg):
        """ ใช้ LiDAR ควบคุมหุ่นยนต์ (แต่จะไม่ทำงานถ้าได้รับคำสั่งหยุด) """
        if self.stop_robot:
            # If stop always sending [0.00, 0.00]
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # Control order reset
        self.twist = Twist()

        # Define minimum value
        min_forward = float('inf')
        min_ccw = float('inf')
        min_cw = float('inf')
        min_back = float('inf')

        # Param
        max_dist = 0.24
        active_threshold = 0.25
        max_linear_speed = 0.24
        max_angular_speed = 0.7

        # Data validation from LiDAR
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
            # Backward: 355°-360° OR 0°-5°
            elif ((355 <= angle_degree < 360) or (0 <= angle_degree < 5)) and self.Back_block:
                min_back = min(min_back, distance)

        # Find Nearest Obstacle
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

        # no obstacle nearby threshold → parking robot
        if selected_region is None:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            return

        # Calculate Direction
        if selected_region == 'forward':
            self.twist.linear.x = max_linear_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'ccw':
            self.twist.angular.z = max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'cw':
            self.twist.angular.z = -max_angular_speed * (1 - (selected_distance / active_threshold))
        elif selected_region == 'back':
            self.twist.linear.x = -max_linear_speed * (1 - (selected_distance / active_threshold))

        # Disp Result
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
