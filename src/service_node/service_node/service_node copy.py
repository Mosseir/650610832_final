import rclpy
from rclpy.node import Node
from my_interface.srv import ControlRobot
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')

        # สร้าง client สำหรับเรียกใช้ service จาก server_node
        self.client = self.create_client(ControlRobot, '/control_robot')

        # Subscriber สำหรับ RPLidar A3
        self.rplidar_subscription = self.create_subscription(
            LaserScan,
            '/rp_scan',
            self.rplidar_callback,
            10
        )

        # Subscriber สำหรับ LiDAR บน TurtleBot3
        self.turtlebot_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.turtlebot_callback,
            10
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Publisher ไปที่ /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def rplidar_callback(self, msg):
        object_detected = self.check_obstacle_in_rplidar(msg)
        self.send_request(object_detected)

    def turtlebot_callback(self, msg):
        self.check_turtlebot_obstacle(msg)

    def send_request(self, object_detected):
        request = ControlRobot.Request()

        if object_detected == "front":
            request.linear_velocity = 1.0
            request.angular_velocity = 0.0
            print("move forward")
        elif object_detected == "back":
            request.linear_velocity = -1.0
            request.angular_velocity = 0.0
            print("move backward")
        elif object_detected == "left":
            request.linear_velocity = 0.0
            request.angular_velocity = 1.0
            print("Rotating CW")
        elif object_detected == "right":
            request.linear_velocity = 0.0
            request.angular_velocity = -1.0
            print("Rotating CCW")
        else:
            request.linear_velocity = 0.0
            request.angular_velocity = 0.0  # กรณี clear ให้หยุด
            print("Stop")

        future = self.client.call_async(request)
        future.add_done_callback(self.callback)

    def callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Move Command: Success! {response.message}')
                self.send_command_to_turtlebot(response.linear_velocity, response.angular_velocity)
            else:
                self.get_logger().info('Failed to move robot.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_command_to_turtlebot(self, linear_velocity, angular_velocity):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(msg)  # เพิ่ม publish command
        self.get_logger().info(f'Sending command to TurtleBot: Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}')

    def check_obstacle_in_rplidar(self, msg):
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue  # ข้ามค่าที่อ่านไม่ได้

            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)  # แปลงเป็นองศา

            if -170 <= angle_deg <= 170 and distance < 0.5:
                return "front"
            elif -10 <= angle_deg <= 10 and distance < 0.5:
                return "back"
            elif 80 <= angle_deg <= 100 and distance < 0.5:
                return "left"
            elif -100 <= angle_deg <= -80 and distance < 0.5:
                return "right"

        return "clear"

def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()
    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
