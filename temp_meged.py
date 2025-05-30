import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node

class DotTrackerNode(Node):
    def __init__(self):
        super().__init__('dot_tracker_node')
        # Motor configuration for Y-axis
        self.DIR_PIN_Y = 19  # Direction pin for Motor Y
        self.STEP_PIN_Y = 26  # Step pin for Motor Y
        self.DELAY_Y = 0.008  # Delay between steps for Y (speed control)
        
        # Motor configuration for X-axis
        self.DIR_PIN_X = 20  # Direction pin for Motor X
        self.STEP_PIN_X = 21  # Step pin for Motor X
        self.DELAY_X = 0.015  # Delay between steps for X (increased for stability)
        
        self.CENTER_TOLERANCE = 20  # Tolerance for centering (same for both axes)
        self.STEPS = 5  # Fixed small steps to avoid overshooting
        self.MAX_PREV_CENTERS = 3  # Number of frames to average for X-axis smoothing

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN_Y, GPIO.OUT)
        GPIO.setup(self.STEP_PIN_Y, GPIO.OUT)
        GPIO.setup(self.DIR_PIN_X, GPIO.OUT)
        GPIO.setup(self.STEP_PIN_X, GPIO.OUT)

        # Camera setup
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            self.get_logger().error('Failed to open camera')
            raise RuntimeError('Camera initialization failed')

        # Position smoothing for X-axis
        self.prev_x_centers = []  # Store recent X coordinates for averaging

        self.get_logger().info('Dot Tracker Node Started')

        # Create a timer to run the tracking loop
        self.timer = self.create_timer(0.`1, self.track_and_align)

    def track_and_align(self):
        ret, frame = self.camera.read()
        if not ret:
            self.get_logger().warn('Failed to read from camera')
            return

        height, width, _ = frame.shape
        center_x = width // 2  # Vertical line for left/right
        center_y = height // 2  # Horizontal line for up/down

        # Convert frame to HSV and apply color filtering
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        red_lower = np.array([136, 87, 111], np.uint8)
        red_upper = np.array([180, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsv_frame, red_lower, red_upper)
        red_mask = cv2.dilate(red_mask, np.ones((5, 5), "uint8"))

        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('No red dot detected, stopping motors')
            self.prev_x_centers.clear()
            return

        # Process the first valid contour
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if area > 300 and circularity > 0.7:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                current_center = (int(x), int(y))

                # Smooth X position
                self.prev_x_centers.append(current_center[0])
                if len(self.prev_x_centers) > self.MAX_PREV_CENTERS:
                    self.prev_x_centers.pop(0)
                smoothed_x = int(np.mean(self.prev_x_centers))
                smoothed_center = (smoothed_x, current_center[1])

                # Determine motor movements
                x_movement = None
                y_movement = None

                # X-axis control (horizontal: left/right)
                if abs(smoothed_x - center_x) <= self.CENTER_TOLERANCE:
                    self.get_logger().info(f'Red dot centered horizontally at {smoothed_center}')
                elif smoothed_x > center_x:
                    self.get_logger().info(f'Red dot right of center at {smoothed_center}, moving X CW')
                    x_movement = {'ccw': False, 'steps': self.STEPS}
                else:
                    self.get_logger().info(f'Red dot left of center at {smoothed_center}, moving X CCW')
                    x_movement = {'ccw': True, 'steps': self.STEPS}

                # Y-axis control (vertical: up/down)
                if abs(current_center[1] - center_y) <= self.CENTER_TOLERANCE:
                    self.get_logger().info(f'Red dot centered vertically at {smoothed_center}')
                elif current_center[1] < center_y:
                    self.get_logger().info(f'Red dot above center at {smoothed_center}, moving Y CW')
                    y_movement = {'ccw': False, 'steps': self.STEPS}
                else:
                    self.get_logger().info(f'Red dot below center at {smoothed_center}, moving Y CCW')
                    y_movement = {'ccw': True, 'steps': self.STEPS}

                # Execute motor movements
                if x_movement:
                    self.rotate_motor_x(**x_movement)
                if y_movement:
                    self.rotate_motor_y(**y_movement)

                # Wait briefly to allow position to stabilize
                time.sleep(0.1)
                return  # Process one contour and wait
        self.get_logger().info('No valid red dot detected, stopping motors')
        self.prev_x_centers.clear()

    def rotate_motor_x(self, ccw=True, steps=3):
        GPIO.output(self.DIR_PIN_X, GPIO.HIGH if ccw else GPIO.LOW)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN_X, GPIO.HIGH)
            time.sleep(self.DELAY_X)
            GPIO.output(self.STEP_PIN_X, GPIO.LOW)
            time.sleep(self.DELAY_X)

    def rotate_motor_y(self, ccw=True, steps=3):
        GPIO.output(self.DIR_PIN_Y, GPIO.HIGH if ccw else GPIO.LOW)
        for _ in range(steps):
            GPIO.output(self.STEP_PIN_Y, GPIO.HIGH)
            time.sleep(self.DELAY_Y)
            GPIO.output(self.STEP_PIN_Y, GPIO.LOW)
            time.sleep(self.DELAY_Y)

    def destroy_node(self):
        self.get_logger().info('Shutting down Dot Tracker Node')
        self.camera.release()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DotTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()