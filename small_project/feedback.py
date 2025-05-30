import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Feedback(Node):
    def __init__(self):
        super().__init__('feedback')
        self.initial_feedback_x = False
        self.initial_feedback_y = False
        self.operational_feedback_x = False
        self.operational_feedback_y = False
        self.system_initialized = False  # Initialize the attribute
        
        self.sub_x = self.create_subscription(
            Int32, 'motor_x_feedback', self.motor_x_callback, 10)
        self.sub_y = self.create_subscription(
            Int32, 'motor_y_feedback', self.motor_y_callback, 10)
        
        self.publisher = self.create_publisher(Int32, 'feedback', 10)
        self.get_logger().info("Feedback Node Ready")

    def motor_x_callback(self, msg):
        if not self.initial_feedback_x:
            self.initial_feedback_x = True
            self.get_logger().info("Received X initialization feedback")
        else:
            self.operational_feedback_x = (msg.data == 1)
        self.check_feedback()

    def motor_y_callback(self, msg):
        if not self.initial_feedback_y:
            self.initial_feedback_y = True
            self.get_logger().info("Received Y initialization feedback")
        else:
            self.operational_feedback_y = (msg.data == 1)
        self.check_feedback()

    def check_feedback(self):
        # Send initial ready signal (ONLY ONCE)
        if self.initial_feedback_x and self.initial_feedback_y and not self.system_initialized:
            feedback_msg = Int32()
            feedback_msg.data = 1
            self.publisher.publish(feedback_msg)
            self.get_logger().info("Sent system ready signal")
            self.system_initialized = True  # Mark system as initialized
    
        # Handle operational feedback (only after initialization)
        if self.system_initialized and self.operational_feedback_x and self.operational_feedback_y:
            feedback_msg = Int32()
            feedback_msg.data = 1
            self.publisher.publish(feedback_msg)
            self.get_logger().info("Sent operational ready signal")
            self.operational_feedback_x = False
            self.operational_feedback_y = False

def main():
    rclpy.init()
    node = Feedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()