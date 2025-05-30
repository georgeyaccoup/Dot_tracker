Robot Vision and Motor Control Project
Project Overview
This project implements a robot vision system with motor control using ROS2 for modularity and real-time communication. The system tracks a red object using a camera, processes the image to detect the object's coordinates, calculates the required motor angles using inverse kinematics, and controls two stepper motors (X and Y axes) using a PID controller to stabilize movement. The project is designed to be scalable, industry-ready, and leverages open-source tools with support for Python and C++.

Features

Color Detection: Uses OpenCV to capture frames from a camera, convert them to HSV color space, and filter for red objects.
Contour Detection: Identifies and filters contours to locate the largest red object and compute its center coordinates.
Inverse Kinematics: Converts pixel coordinates to real-world angles for motor control.
PID Control: Stabilizes motor movements with tuned PID parameters (Kp=0.4, Ki=0.0051, Kd=0.12) to reduce overshoot and improve accuracy.
ROS2 Integration: Modular nodes for color detection, kinematics, PID control, and motor control (X and Y axes).
Real-Time Feedback: Ensures synchronized operation between nodes using feedback signals.
Version Control: Iterative development with versions (V01–V26) covering node creation, kinematics, motor control, and PID tuning.


Main Libraries Used

OpenCV (cv2): For image processing and color detection.
NumPy: For numerical operations and coordinate calculations.
RPi.GPIO: For controlling stepper motors on Raspberry Pi.
rclpy: ROS2 Python client library for node creation and communication.
std_msgs: ROS2 message types for inter-node communication.


System Architecture
The system is composed of several ROS2 nodes, each handling a specific task:

Color Detector Node (color_detector.py):
Captures frames from a camera.
Converts frames to HSV and filters for red color.
Detects contours and calculates the center of the largest red object.
Publishes averaged coordinates to the color_coordinates topic.


Inverse Kinematics Node (kinematics.py):
Subscribes to color_coordinates topic.
Converts pixel coordinates to real-world angles using inverse kinematics.
Publishes desired angles to the desired_angles topic.


PID Controller Node (pid_controller.py):
Subscribes to desired_angles topic.
Applies PID control (Kp=0.4, Ki=0.0051, Kd=0.12) to stabilize motor commands.
Publishes motor commands to motor_x_command and motor_y_command topics.


Motor X Controller Node (Motor_x.py):
Controls the X-axis stepper motor.
Moves to target angles within safe limits (0° to 110.4°).
Sends feedback to the motor_x_feedback topic.


Motor Y Controller Node (Motor_y.py):
Controls the Y-axis stepper motor.
Moves to target angles within safe limits (0° to 330°).
Sends feedback to the motor_y_feedback topic.


Feedback Node (feedback.py):
Coordinates initialization and operational feedback between motor nodes and the color detector.
Publishes feedback signals to the feedback topic.




Package Tree
📁 small_project/
├── 📁 focus/
│   ├── 🐍 color_detector.py
│   ├── 🐍 feedback.py
│   ├── 🐍 kinematics.py
│   ├── 🐍 Motor_x.py
│   ├── 🐍 Motor_y.py
│   ├── 🐍 pid_controller.py
│   └── 🚀 focus_launch.py
├── 📁 focus_v2/
│   ├── 🐍 color_detector.py
│   ├── 🐍 kinematics.py
│   ├── 🐍 Motor_x.py
│   ├── 🐍 Motor_y.py
│   ├── 🐍 pid_controller.py
│   └── 🚀 foucs_v2_launch.py
├── 🐍 setup.py
├── 📜 LICENSE
└── 📝 README.md


Code
Below is an overview of the key code files in the small_project package, organized by their functionality:

📁 focus/

🐍 color_detector.py: Captures video frames, converts them to HSV, filters for red objects, detects contours, and publishes the object's center coordinates to the color_coordinates topic.
🐍 feedback.py: Manages initialization and operational feedback between motor nodes and the color detector, publishing signals to the feedback topic.
🐍 kinematics.py: Performs inverse kinematics calculations to convert pixel coordinates to motor angles, publishing results to the desired_angles topic.
🐍 Motor_x.py: Controls the X-axis stepper motor, moving it to target angles (0° to 110.4°) and sending feedback to the motor_x_feedback topic.
🐍 Motor_y.py: Controls the Y-axis stepper motor, moving it to target angles (0° to 330°) and sending feedback to the motor_y_feedback topic.
🐍 pid_controller.py: Implements PID control (Kp=0.4, Ki=0.0051, Kd=0.12) to stabilize motor movements, publishing commands to motor_x_command and motor_y_command topics.
🚀 focus_launch.py: ROS2 launch file to start all nodes in the focus package.


📁 focus_v2/

🐍 color_detector.py: Same functionality as in focus, with potential updates or optimizations for version 2.
🐍 kinematics.py: Same as in focus, with possible refinements for inverse kinematics calculations.
🐍 Motor_x.py: Controls the X-axis motor, identical to focus but part of the version 2 package.
🐍 Motor_y.py: Controls the Y-axis motor, identical to focus but part of the version 2 package.
🐍 pid_controller.py: Applies PID control, same as in focus, with potential tuning adjustments.
🚀 foucs_v2_launch.py: ROS2 launch file to start all nodes in the focus_v2 package (note: contains a typo in the filename, should be focus_v2_launch.py).


🐍 setup.py: Configures the small_project package, defining entry points for executables and dependencies (e.g., setuptools, pytest).



Installation

Prerequisites:

ROS2 (Humble or later recommended)
Python 3.8+
OpenCV (pip install opencv-python)
NumPy (pip install numpy)
RPi.GPIO (for Raspberry Pi motor control)
A compatible USB camera


Setup:
# Clone the repository
git clone <repository_url>
cd <repository_directory>

# Install dependencies
pip install -r requirements.txt

# Build the ROS2 package
colcon build --packages-select small_project

# Source the workspace
source install/setup.bash


Run the System:
# Launch the nodes (use either focus or focus_v2)
ros2 launch focus focus_launch.py
# OR
ros2 launch focus_v2 foucs_v2_launch.py




Usage

Ensure the camera is connected and the Raspberry Pi GPIO pins are correctly configured (DIR_PIN_X=20, STEP_PIN_X=21, DIR_PIN_Y=19, STEP_PIN_Y=26).
Run the launch file to start all nodes.
Place a red object in the camera's field of view. The system will:
Detect the object's coordinates.
Calculate the required motor angles.
Move the motors to track the object using PID control.


Monitor node logs for real-time feedback on detection, kinematics, and motor movements.


Development History
The project was developed iteratively across versions V01–V26:

V01–V04: Initial node creation and version control setup.
V05–V07: Kinematics tasks and equations defined.
V08–V10: Motor X and Y node creation and operation.
V11–V14: PI controller testing and implementation.
V15–V18: PD controller testing and implementation.
V19–V21: PID controller testing and creation.
V22–V25: PID tuning to address overshoot and improve stability.
V26: Final testing and documentation.


PID Control
The PID controller stabilizes motor movements with the following parameters:

Kp: 0.4 (Proportional gain)
Ki: 0.0051 (Integral gain)
Kd: 0.12 (Derivative gain)

The PID controller minimizes overshoot and ensures smooth motor operation. The Simulink block diagram (refer to presentation) illustrates the control loop, and time response analysis shows improved stability.

Why ROS2?

Modularity & Scalability: Nodes can be developed and tested independently.
Real-Time Communication: Efficient topic-based messaging.
Industry-Ready: Widely used in robotics with active community support.
Open Source: Free to use with Python and C++ support.


Results

The system successfully tracks red objects in real-time.
PID control reduces motor overshoot, improving tracking accuracy.
The modular ROS2 architecture ensures scalability and ease of maintenance.


Contributing
Contributions are welcome! Please fork the repository, create a new branch, and submit a pull request with your changes. Ensure all tests pass (pytest) before submitting.

License
This project is licensed under the Apache License 2.0. See the LICENSE file for details.

Contact
For questions or support, contact the maintainer:

Name: [Your Name]
Email: [your_email@example.com]

