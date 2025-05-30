import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import RPi.GPIO as GPIO
import time

DIR_PIN_Y = 19
STEP_PIN_Y = 26
DEG_PER_STEP = 1.8  # 1.8° per step (200 steps/revolution)
STEPS_PER_DEG = 1/DEG_PER_STEP  # Steps per degree

class StepperMotorYController(Node):
    def __init__(self):
        super().__init__('motor_y')
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DIR_PIN_Y, GPIO.OUT)
        GPIO.setup(STEP_PIN_Y, GPIO.OUT)
        
        # Motor parameters
        self.POINT_A = 0.0      # Home position
        self.POINT_B = 140.0    # Increased to 180° as requested
        self.POINT_C = 330.0    # Maximum safe limit
        self.current_angle = self.POINT_A
        self.step_delay = 0.001  # 1ms delay between steps
        
        # ROS setup
        self.subscription = self.create_subscription(
            Float32, 'motor_y_command', self.move_callback, 10)
        self.feedback_publisher = self.create_publisher(Int32, 'motor_y_feedback', 10)
        
        self.get_logger().info("Motor Y Controller Starting...")
        self.initialize_motor()

    def angle_to_steps(self, angle_diff):
        """Convert angle difference to number of steps"""
        return int(abs(angle_diff) * STEPS_PER_DEG)

    def move_stepper(self, target_angle):
        """Move motor to specified angle"""
        # Constrain target within safe limits
        target_angle = max(self.POINT_A, min(self.POINT_C, target_angle))
        angle_diff = target_angle - self.current_angle
        
        if angle_diff == 0:
            return  # No movement needed
        
        # Set direction
        direction = GPIO.HIGH if angle_diff > 0 else GPIO.LOW
        GPIO.output(DIR_PIN_Y, direction)
        
        # Calculate steps needed
        steps = self.angle_to_steps(angle_diff)
        
        # Execute steps
        for _ in range(steps):
            GPIO.output(STEP_PIN_Y, GPIO.HIGH)
            time.sleep(self.step_delay)
            GPIO.output(STEP_PIN_Y, GPIO.LOW)
            time.sleep(self.step_delay)
        
        # Update current angle
        self.current_angle = target_angle
        self.get_logger().info(f"Motor Y moved to {self.current_angle:.1f}°")
        
        # Send feedback
        self.send_feedback()

    def send_feedback(self):
        """Send feedback that movement is complete"""
        feedback_msg = Int32()
        feedback_msg.data = 1
        self.feedback_publisher.publish(feedback_msg)
        self.get_logger().info("Sent Y feedback")

    def initialize_motor(self):
        """Initialize motor sequence"""
        self.get_logger().info("Homing Motor Y...")
        self.move_stepper(self.POINT_A)  # Home position
        time.sleep(0.5)
        
        self.get_logger().info(f"Moving to initial position {self.POINT_B}°...")
        self.move_stepper(self.POINT_B)  # Initial position at 180°
        
        self.get_logger().info("Motor Y Initialization Complete")
        self.send_feedback()

    def move_callback(self, msg):
        """Handle incoming movement commands"""
        target_angle = msg.data
        self.get_logger().info(f"Received command to move to {target_angle}°")
        self.move_stepper(target_angle)

    def destroy_node(self):
        """Cleanup GPIO on shutdown"""
        GPIO.cleanup()
        self.get_logger().info("Motor Y Controller Shutting Down...")
        super().destroy_node()

def main():
    rclpy.init()
    node = StepperMotorYController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()