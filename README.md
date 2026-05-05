# 🤖 ROS2 Fault Injector (Chaos Monkey for Robotics)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble/Foxy/Galactic-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](https://github.com/EngineerAbdullahBinZafar/ros-fault-injector/pulls)

**`ros-fault-injector`** is a high-performance, flexible tool designed to test the robustness of robotic systems by deliberately injecting faults into ROS2 communication channels. Inspired by "Chaos Engineering" (like Netflix's Chaos Monkey), this tool helps developers identify single points of failure and edge-case bugs before they happen in the field.

---

## ✨ Key Features

- ⏳ **Latent Delay Injection**: Simulates network congestion or CPU spikes by delaying messages.
- 📉 **Packet Drop**: Simulates signal loss or hardware instability by randomly dropping messages.
- 🔊 **Gaussian Noise Injection**: Adds noise to sensor data (Pose, Twist, Imu, etc.) to test filter robustness.
- 🔧 **Dynamic Configuration**: Change fault profiles on-the-fly using ROS2 parameters.
- 📦 **Zero Code Change**: Sit between your existing nodes without modifying their source code.

---

## 🚀 Quick Start

### 1. Installation
Clone this repository into your `colcon` workspace:
```bash
cd ~/ros2_ws/src
git clone https://github.com/EngineerAbdullahBinZafar/ros-fault-injector.git
cd ..
colcon build --packages-select ros_fault_injector
source install/setup.bash
```

### 2. Basic Usage
Run the injector to delay a Pose topic by 200ms:
```bash
ros2 run ros_fault_injector injector_node --ros-args \
  -p input_topic:=/odom \
  -p output_topic:=/odom_delayed \
  -p fault_type:=delay \
  -p delay_ms:=200
```

---

## 🛠️ Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | `string` | `/input_topic` | The topic to listen to. |
| `output_topic`| `string` | `/output_topic`| The topic to republish to. |
| `msg_type`    | `string` | `Pose` | Message type (Pose, Twist, LaserScan, Imu). |
| `fault_type`  | `string` | `delay` | Type of fault (`delay`, `drop`, `noise`). |
| `delay_ms`    | `int` | `100` | Delay duration in milliseconds. |
| `drop_probability` | `float` | `0.1` | Chance of dropping a message (0.0 to 1.0). |
| `noise_std_dev`| `float` | `0.05` | Standard deviation for Gaussian noise. |

---

## 🤝 Contributing
Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📄 License
Distributed under the MIT License. See `LICENSE` for more information.

## 👤 Author
**Engineer Abdullah Bin Zafar**
- GitHub: [@EngineerAbdullahBinZafar](https://github.com/EngineerAbdullahBinZafar)
- LinkedIn: [Abdullah Bin Zafar](https://www.linkedin.com/in/abdullah-bin-zafar/)
