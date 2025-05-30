Here’s a polished and professional rewrite of your project documentation, with improved clarity, formatting, and flow while preserving all technical details:

---

# **Robot Vision and Motor Control using ROS2**

## 📌 Project Overview

This project integrates **robot vision** with **motor control** using **ROS 2** to achieve real-time object tracking and precise actuation. It employs a modular architecture, where each component is implemented as an independent ROS 2 node. The system detects a red object in the camera frame, computes its coordinates, determines the required motion using inverse kinematics, and drives stepper motors using a PID controller to track the object. Built with scalability and industrial deployment in mind, it leverages open-source tools and supports both **Python** and **C++**.

---

## 🚀 Key Features

* **🎯 Color Detection:** Utilizes OpenCV to capture video frames, convert them to HSV, and detect red objects using color filtering.
* **📐 Inverse Kinematics:** Translates image pixel coordinates into real-world angles for motor positioning.
* **🧠 PID Control:** Fine-tuned PID controller (Kp=0.4, Ki=0.0051, Kd=0.12) ensures smooth and accurate motor motion.
* **🧩 Modular ROS2 Architecture:** Dedicated nodes for color detection, kinematics, control logic, and motor actuation.
* **🔄 Real-Time Feedback:** Feedback from motors ensures closed-loop control and synchronization across nodes.
* **📂 Versioned Development:** Progressive builds (V01–V26) ensure traceable and iterative improvements.

---

## 🔧 Technologies and Libraries

* **OpenCV (cv2):** For image processing and object detection.
* **NumPy:** Numerical operations and coordinate transformations.
* **RPi.GPIO:** GPIO control for stepper motors on Raspberry Pi.
* **rclpy:** ROS 2 Python client library.
* **std\_msgs:** ROS 2 standard message types for inter-node communication.

---

## 🧠 System Architecture

Each ROS 2 node is designed with single responsibility:

### `focus/`

* **`color_detector.py`:** Captures video, filters red objects, finds the largest contour, and publishes its center.
* **`kinematics.py`:** Converts coordinates into motor angles via inverse kinematics.
* **`pid_controller.py`:** Applies PID algorithm to minimize angle error and outputs motor commands.
* **`Motor_x.py` / `Motor_y.py`:** Controls X/Y stepper motors, maintaining angle limits and publishing feedback.
* **`feedback.py`:** Coordinates communication between motor controllers and vision system.
* **`focus_launch.py`:** Launch file to run the entire system.

### `focus_v2/`

Optimized or updated versions of the above with improved performance or added features.
⚠️ *Note:* Typo in launch file (`foucs_v2_launch.py`) should be corrected to `focus_v2_launch.py`.

---

## 📁 Directory Structure

```
small_project/
├── focus/
│   ├── color_detector.py
│   ├── feedback.py
│   ├── kinematics.py
│   ├── Motor_x.py
│   ├── Motor_y.py
│   ├── pid_controller.py
│   └── focus_launch.py
├── focus_v2/
│   ├── color_detector.py
│   ├── kinematics.py
│   ├── Motor_x.py
│   ├── Motor_y.py
│   ├── pid_controller.py
│   └── foucs_v2_launch.py  <-- should be renamed
├── setup.py
├── LICENSE
└── README.md
```

---

## ⚙️ Installation & Setup

### Prerequisites

* **ROS 2** (Humble or later)
* **Python 3.8+**
* Packages: `opencv-python`, `numpy`, `RPi.GPIO`

### Install Steps

```bash
# Clone repository
git clone <repository_url>
cd small_project

# Install Python dependencies
pip install -r requirements.txt

# Build the ROS2 package
colcon build --packages-select small_project

# Source the environment
source install/setup.bash
```

---

## ▶️ Running the System

### Launch with:

```bash
ros2 launch focus focus_launch.py
# OR for version 2
ros2 launch focus_v2 focus_v2_launch.py
```

### Hardware Setup

* Ensure a USB camera is connected.
* Motor GPIO pins configured as:

  * **DIR\_PIN\_X = 20, STEP\_PIN\_X = 21**
  * **DIR\_PIN\_Y = 19, STEP\_PIN\_Y = 26**

### Operation

* Place a red object in the camera's field of view.
* The system:

  1. Detects the object.
  2. Calculates motor angles.
  3. Moves motors via PID control to track the object.

---

## 📈 PID Controller

**Parameters:**

* **Kp = 0.4**
* **Ki = 0.0051**
* **Kd = 0.12**

The PID logic ensures stable, smooth tracking by minimizing error and overshoot. Time response and Simulink block diagrams are available in the project presentation.

---

## 🧪 Development Timeline

| Version | Milestone                                    |
| ------- | -------------------------------------------- |
| V01–V04 | ROS2 node creation and version control setup |
| V05–V07 | Inverse kinematics implementation            |
| V08–V10 | Motor node development                       |
| V11–V14 | PI control design                            |
| V15–V18 | PD controller testing                        |
| V19–V21 | Full PID controller development              |
| V22–V25 | PID tuning for stability                     |
| V26     | Final testing and documentation              |

---

## 🎯 Why ROS2?

* **Modularity:** Clean separation of functionality into reusable nodes.
* **Scalability:** Easy to expand or replace components.
* **Real-Time Communication:** Efficient, non-blocking pub-sub model.
* **Industry Adoption:** Ideal for robotics systems in real-world applications.
* **Open Source:** Python and C++ support with an active ecosystem.

---

## ✅ Results

* Real-time red object tracking successfully achieved.
* PID control significantly reduces motor overshoot.
* ROS 2 node-based architecture allows easy debugging, updates, and scaling.

---

## 🤝 Contributing

We welcome contributions!
To contribute:

1. Fork the repository.
2. Create a new branch.
3. Make your changes.
4. Submit a pull request.

Ensure all tests pass before submitting (`pytest`).

---

## 📄 License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

## 📬 Contact

**Maintainer:**
Name: *\[Your Name]*
Email: *\[[your\_email@example.com](mailto:your_email@example.com)]*

---

Let me know if you’d like this as a `README.md` file, or want help updating your GitHub repository directly.
