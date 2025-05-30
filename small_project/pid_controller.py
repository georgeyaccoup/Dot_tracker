import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        self.kp = 0.4
        self.ki = 0.0051
        self.kd = 0.12
        self.integral = 0.0
        self.previous_error = 0.0
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'desired_angles',
            self.control_callback,
            10
        )
        
        self.pub_y = self.create_publisher(Float32, 'motor_y_command', 10)
        self.pub_x = self.create_publisher(Float32, 'motor_x_command', 10)
        
        self.get_logger().info("PID Controller Ready")

    def control_callback(self, msg):
        if len(msg.data) != 2:
            self.get_logger().error("Received Invalid Angles Data")
            return
            
        angle_x, angle_y = msg.data
        
        self.get_logger().info(f"Received Desired Angles: X={angle_x:.1f}°, Y={angle_y:.1f}°")
        
        # Simpler: no PID control for X, just send it
        x_msg = Float32()
        x_msg.data = angle_x
        self.pub_x.publish(x_msg)
        
        # Simple PID for Y (optional here you could just send it too)
        error = angle_y - 55.2
        self.integral += error
        derivative = error - self.previous_error
        
        output = 55.2 + (self.kp * error + self.ki * self.integral + self.kd * derivative)
        output = max(0.0, min(110.4, output))
        
        y_msg = Float32()
        y_msg.data = output
        self.pub_y.publish(y_msg)
        
        self.previous_error = error
        
        self.get_logger().info(f"Sent X angle: {angle_x:.1f}°, Y angle: {output:.1f}°")

def main():
    rclpy.init()
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
