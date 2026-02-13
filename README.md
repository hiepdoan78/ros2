## 🚚 Autonomous Truck

Autonomous truck capable of:

* Lane following

* Obstacle avoidance

* Traffic sign recognition

* Automated container lifting

A semi-autonomous truck platform using ROS 2 on Raspberry Pi 5 for perception and navigation, combined with an STM32-based embedded controller for real-time vehicle actuation, sensor fusion, and computer vision.

## 🎥 Demo video:
https://youtu.be/xy0PPB-s8K4

## 🧠 System Architecture
```text
+----------------------+
|        Camera        |
| Lane + Sign Detection|
+----------+-----------+
           |
+----------v-----------+
|    Raspberry Pi 5    |
|  ROS2 Processing     |
|  Perception + Logic  |
+----------+-----------+
           | UART
+----------v-----------+
|        STM32         |
| Vehicle Controller   |
| Steering / Motor /   |
| Container Lifter     |
+----------+-----------+
           |
+------------------+------------------+
|                                     |
+-------v-------+           +--------v--------+
|     LiDAR     |           |    Actuators    |
|Obstacle Detect|           | Motor / Servo   |
+---------------+           | Lift Mechanism  |
                            +----------------+
```
## Key Technologies

### Robotics & Middleware
* ROS 2 Humble
* Sensor integration (Camera, LiDAR)
* Autonomous navigation logic

### Embedded Systems
* STM32 microcontroller vehicle control
* UART communication (Raspberry Pi ↔ STM32)
* Real-time steering and speed control

### Computer Vision
* Lane detection
* Traffic sign recognition
* Image processing pipeline

## 🚚 Main Features

* Autonomous lane following
* Traffic sign detection
* Obstacle avoidance using LiDAR
* Embedded vehicle motion control
* Automated container lifting mechanism

## 📂 Project Structure
ros2/
│
├── stm32_firmware/      # Vehicle control firmware
├── src/                 # ROS2 workspace
│   ├── truck/           # Truck model
│   ├── example_11/      # Switch between bicycle model and car model
│   ├── lane_detection/  # Lane detection algorithm
│   ├── rplidar_ros/     # Driver for Lidar
│   └── serial/          # Drive for UART comunication
└── README.md

## ⚙️ Hardware Setup

* Raspberry Pi 5 (Main processor / ROS node host)

 * STM32 MCU (Low-level vehicle controller)

* LiDAR sensor

* Camera module

* Motor driver + steering servo

* Container lifting mechanism

## 📈 Future Improvements

* Sensor fusion optimization

* Path planning enhancement

* CAN/Ethernet communication upgrade

* Improved perception accuracy

* Full autonomous logistics workflow

## 👨‍💻 Author

Doan Minh Hiep
Embedded Software Developer passionate about automotive, robotics, and low-level system programming.

---