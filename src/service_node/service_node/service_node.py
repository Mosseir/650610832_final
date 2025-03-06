import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from my_interface.srv import StopRobot  # Custom Service Message
import math

class ServiceNode(Node):
    def __init__(self):
        super().__init__('service_node')
        self.srv = self.create_service(StopRobot, 'stop_robot_service', self.handle_stop_request)
        
        self.subscription = self.create_subscription(
            LaserScan, '/rp_scan', self.rp_lidar_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.force_stop = False  # Flag to determine if robot should stop
    
    def handle_stop_request(self, request, response):
        """ Handles stop request from server_node """
        if request.stop:
            self.force_stop = True
            self.get_logger().warn("Received stop request. Stopping robot.")
        else:
            self.force_stop = False
            self.get_logger().info("Resume robot movement.")
        response.success = True
        return response
    
    def rp_lidar_callback(self, msg):
        """ Controls robot movement based on RPLidar A3 data unless force_stop is True """
        if self.force_stop:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.get_logger().warn("Robot is stopped due to stop request.")
            return
        
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
                angle_radian = msg.angle_min + i * msg.angle_increment
                angle_degree = math.degrees(angle_radian) % 360
                distance_rounded = round(distance, 2)

                if distance_rounded < min_dist:
                    linear_speed = max_linear_speed
                elif distance_rounded > max_dist:
                    linear_speed = 0
                else:
                    linear_speed = ((distance_rounded - min_dist) / (max_dist - min_dist)) * (min_linear_speed - max_linear_speed) + max_linear_speed

                angular_speed = (1 - (distance_rounded - min_dist) / (max_dist - min_dist)) * max_angular_speed
                
                if 255 <= angle_degree < 285:
                    self.twist.angular.z = angular_speed
                    self.twist.linear.x = 0.0
                elif 65 <= angle_degree < 105:
                    self.twist.angular.z = -angular_speed
                    self.twist.linear.x = 0.0
                elif 165 <= angle_degree < 195:
                    self.twist.linear.x = linear_speed
                    self.twist.angular.z = 0.0
                elif (345 <= angle_degree < 360 or 0 <= angle_degree < 15):
                    self.twist.linear.x = -linear_speed
                    self.twist.angular.z = 0.0
                break

        self.cmd_vel_pub.publish(self.twist)


def main():
    rclpy.init()
    node = ServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
