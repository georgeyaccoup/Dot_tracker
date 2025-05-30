import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Int32
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'color_coordinates', 10)
        self.feedback_sub = self.create_subscription(
            Int32, 'feedback', self.feedback_callback, 10)
        
        self.system_initialized = False
        self.feedback_ready = True  # Start ready after initialization
        self.cap = cv2.VideoCapture(0)
        self.center_x = 320
        self.center_y = 240
        self.coordinate_buffer = []
        self.buffer_size = 4
        self.old_coordinates = None
        self.get_logger().info("Color Detector Initialized")

    def feedback_callback(self, msg):
        if self.system_initialized and msg.data == 1:
            self.system_initialized = True
            self.get_logger().info("System initialization complete - starting detection")
        self.feedback_ready = (msg.data == 1)

    def detect_red(self):
        if not self.system_initialized or not self.feedback_ready:
            if not self.system_initialized:
                self.get_logger().info("Waiting for system initialization...")
            else:
                self.get_logger().info("Waiting for feedback ready signal...")
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] != 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.coordinate_buffer.append((cx, cy))
                self.get_logger().info(f"Detected at ({cx}, {cy})")
        
        if len(self.coordinate_buffer) >= self.buffer_size:
            avg_x = int(sum(coord[0] for coord in self.coordinate_buffer) / self.buffer_size)
            avg_y = int(sum(coord[1] for coord in self.coordinate_buffer) / self.buffer_size)

            if self.old_coordinates:
                x_diff = abs(avg_x - self.old_coordinates[0])
                y_diff = abs(avg_y - self.old_coordinates[1])
                
                if x_diff < 10 and y_diff < 10:
                    self.publisher_.publish(Int32MultiArray(data=[0, 0]))
                    self.get_logger().info("Minimal movement - sent (0, 0)")
                else:
                    self.publisher_.publish(Int32MultiArray(data=[avg_x, avg_y]))
                    self.get_logger().info(f"Published coordinates: ({avg_x}, {avg_y})")
            
            self.old_coordinates = (avg_x, avg_y)
            self.coordinate_buffer = []
            self.feedback_ready = False  # Wait for next feedback

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        while rclpy.ok():
            node.detect_red()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()