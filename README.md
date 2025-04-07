# 🛠️ Autonomous Car Environment - ROS 2 + Python Project

## 📄 Project Description

This project is an educational exercise aimed at reinforcing knowledge of **Python** and **ROS 2**. It simulates an autonomous car navigating through cone-based tracks. The system is built with ROS 2 nodes, visualized with OpenCV, and structured for modular expansion.

---

## 🎯 Objective

The goal is to **develop a new node or set of nodes** (either in this package or in a new one) that allows the autonomous car to:

- Successfully navigate all available circuits **autonomously**.
- Use a **single algorithm** that works across all tracks (no per-track tuning allowed).
- **Stop the car at the end of the circuit** (near the finish area) **before** changing the state to `"finish"`.

---

## 📡 Topic Interface & Communication Rules

### ✅ Developers **can only interact with these topics**:

#### **Input topics to control the car:**

| Topic               | Type                | Description                          |
|--------------------|---------------------|--------------------------------------|
| `/car/steering`     | `std_msgs/Float32`  | Steering angle in radians. Positive = left, negative = right. |
| `/car/acceleration` | `std_msgs/Float32`  | Linear acceleration in m/s². Positive = forward, negative = braking. |
| `/car/state`        | `std_msgs/String`   | Can only be set to `"finish"`. Indicates completion of track. |


#### ⚠️ **Limited access to `/env` topics**

Topics starting with `/env` are **internal** and intended only for the environment's own use. 

However, you **can read from** the topic:

- `/env/car_position`: shows the current position of the car (`geometry_msgs/Point`).


---

## 📊 Sensor-like Data Topics

Developers **can read from these topics** to simulate sensor data:

### `/can/*` topics:

| Topic               | Type                | Description               |
|--------------------|---------------------|---------------------------|
| `/can/speed`        | `std_msgs/Float32`  | Current speed of the car. |
| `/can/yaw`          | `std_msgs/Float32`  | Current orientation (yaw).|

### `/vision/*` topics:

| Topic                          | Type                    | Description                          |
|-------------------------------|-------------------------|--------------------------------------|
| `/vision/all_cones/blue`      | `auria_msgs/PointArray` | All blue cones (array of positions). |
| `/vision/all_cones/yellow`    | `auria_msgs/PointArray` | All yellow cones.                    |
| `/vision/nearby_cones/blue`   | `auria_msgs/PointArray` | Blue cones within 8 meters.          |
| `/vision/nearby_cones/yellow` | `auria_msgs/PointArray` | Yellow cones within 8 meters.        |

> **Note**: `PointArray.msg` is a custom message defined in the `auria_msgs` package:
> ```msg
> geometry_msgs/Point[] points
> ```

---

## 🧠 Summary

Participants must write logic that:
- Consumes data from `/can/*` and `/vision/*` topics only.
- Publishes steering, acceleration, and state change commands via `/car/*`.
- Generalizes well to all tracks.
- Stops the car at the finish line before changing to `"finish"`.

---

## 📘 ROS 2 Command Dictionary

### 🔎 Topic introspection

```bash
ros2 topic list                     # List all topics
ros2 topic info /car/steering       # Show info about a topic
ros2 topic type /can/yaw            # Show message type of topic
ros2 interface show std_msgs/msg/Float32  # Show message definition
```

### 👀 Topic reading (echo)

```bash
ros2 topic echo /can/yaw
ros2 topic echo /vision/all_cones/blue
```

### 📤 Publishing manually

```bash
ros2 topic pub /car/steering std_msgs/Float32 "{data: 0.5}" --rate 10
ros2 topic pub /car/acceleration std_msgs/Float32 "{data: 1.0}" --rate 10
ros2 topic pub /car/state std_msgs/String "{data: 'finish'}"
```

---

## 🐳 Docker Launch Instructions

To launch the environment with full GUI (rqt_graph, OpenCV, etc.), simply run the `x` script:

```bash
./x
```

---

## 📎 Developer Notes

- Run `source install/setup.bash` **before launching any node**, in each terminal.
- You may use `rqt_graph` to visualize connections. 

---

## 🚀 Launch Instructions

### ▶️ Launch nodes individually

Make sure you have compiled and sourced your workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

Then run each node like this:

```bash
ros2 run environment env     # Launches the environment (map, cones, car view)
ros2 run environment car     # Launches the car node (publishes position)
```

### 🧩 Launch everything with one command

A launch file is provided for convenience. To run both the environment and car nodes together:

```bash
ros2 launch environment launch_env.py
```

> This will start both nodes in the same terminal, printing their logs to the screen.

---

Happy coding & enjoy the challenge! 🚗🧠✨

