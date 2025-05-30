import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import RPi.GPIO as GPIO
import time

DIR_PIN_X = 20  
STEP_PIN_X = 21
DEG_PER_STEP = 1.8

class StepperMotorXController(Node):
    def __init__(self):
        super().__init__('motor_x')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DIR_PIN_X, GPIO.OUT)
        GPIO.setup(STEP_PIN_X, GPIO.OUT)
        
        self.POINT_A = 0.0
        self.POINT_C = 110.4
        self.current_angle = self.POINT_A
        
        self.subscription = self.create_subscription(
            Float32, 'motor_x_command', self.move_callback, 10)
        self.feedback_publisher = self.create_publisher(Int32, 'motor_x_feedback', 10)
        
        self.get_logger().info("Motor X Controller Starting...")
        self.init_motor()

    def angle_to_steps(self, angle_diff):
        return int(abs(angle_diff) / DEG_PER_STEP)

    def move_stepper(self, target):
        target = max(self.POINT_A, min(self.POINT_C, target))
        delta = target - self.current_angle
        
        if delta == 0:
            return
        
        direction = GPIO.HIGH if delta > 0 else GPIO.LOW
        steps = self.angle_to_steps(delta)
        
        GPIO.output(DIR_PIN_X, direction)
        for _ in range(steps):
            GPIO.output(STEP_PIN_X, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(STEP_PIN_X, GPIO.LOW)
            time.sleep(0.001)
        
        self.current_angle = target
        self.get_logger().info(f"X moved to {self.current_angle:.1f}°")
        
        # Send movement feedback
        feedback_msg = Int32()
        feedback_msg.data = 1
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info("Sent X movement feedback")

    def init_motor(self):
        self.move_stepper(self.POINT_A)
        # Send initialization feedback
        feedback_msg = Int32()
        feedback_msg.data = 1
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info("Motor X Initialized and feedback sent")
        

    def move_callback(self, msg):
        self.move_stepper(msg.data)

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main():
    rclpy.init()
    node = StepperMotorXController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()