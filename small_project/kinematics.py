import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class InverseKinematics(Node):
    def __init__(self):
        super().__init__('kinematics')
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'color_coordinates',
            self.calculate_angles,
            10
        )
        
        self.publisher = self.create_publisher(Float32MultiArray, 'desired_angles', 10)
        self.get_logger().info("Kinematics Node Ready")

    def calculate_angles(self, msg):
        x_pixel, y_pixel = msg.data
        
        x_real = (x_pixel - 320) / 320 * 10
        y_real = (y_pixel - 240) / 240 * 10
        
        distance = math.sqrt(x_real**2 + y_real**2)
        
        try:
            angle_x = math.degrees(math.atan2(x_real, y_real))
            height_diff = y_real - (8.0 + 4.0)
            angle_y = math.degrees(math.asin(min(max(height_diff/12.0, -1.0), 1.0)))
            
            angle_y = max(0.0, min(110.4, angle_y))
            
            self.get_logger().info(f"Calculated Angles: X={angle_x:.1f}°, Y={angle_y:.1f}°")
            
            angles_msg = Float32MultiArray()
            angles_msg.data = [angle_x, angle_y]
            self.publisher.publish(angles_msg)
            
            self.get_logger().info(f"Published Desired Angles: X={angle_x:.1f}°, Y={angle_y:.1f}°")
            
        except Exception as e:
            self.get_logger().error(f"Kinematics Calculation Failed: {str(e)}")

def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
