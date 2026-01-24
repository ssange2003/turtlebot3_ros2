# Optimized ROS 2 Gripper Integration for TurtleBot3 via OpenCR Firmware

This repository features a high-performance integration of a 1-axis Dynamixel gripper into the TurtleBot3 platform. By leveraging deep-level firmware optimization and direct memory access,
this project achieves seamless control without the need for additional MCUs or external power distribution systems.

---

### 1. Register-Level Control & Operating Mode 5
To achieve a balance between grasping power and hardware safety, the actuator is configured at the register level:
* **Operating Mode (Addr 11):** Set to **Mode 5 (Current-based Position Control)**. This allows the gripper to maintain a steady grasp while preventing mechanical stalling.
* **Current Limit (Addr 38):** Capped at **300 units** to protect the Dynamixel from overcurrent while handling objects of varying densities.
* **Address 142 Integration:** Utilized for real-time state monitoring and status feedback within the control loop to ensure synchronization between the OpenCR and the actuator.

### 2. Zero-Copy Strategy via Direct Pointer Casting
To maintain a strict **100Hz real-time control loop**, I bypassed standard data buffering:
* **Implementation:** Used `(uint8_t*)&` to reference memory addresses directly.
* **Optimization:** By mapping the ROS 2 message data directly to the hardware registers, I eliminated redundant CPU cycles used for data copying.
* **Result:** Reduced control loop latency to **under 5ms**, ensuring high-speed responsiveness under limited computational resources.

### 3. Protocol Repurposing & Non-zero Signal Filtering
Instead of creating a custom ROS 2 message (which increases overhead), I repurposed the `linear.y` field of the standard `geometry_msgs/Twist` message.
* **Data Mapping:** Scaled `linear.y` values (e.g., `value * 100`) to pass integer commands through the existing navigation pipeline to **Address 154** (ADDR_CMD_VEL_LINEAR_Y).
* **Signal Filter:** Developed a **Non-zero Signal Filter** in the firmware to ignore high-frequency jitter (floating-point noise) from the Navigation stack. This prevents motor fatigue and overheating by only processing intentional control commands.

---


## ⚠️ [Caution] Hardware Integrity Constants
When utilizing this firmware on standard TurtleBot3 hardware, the following kinematic variables defined in the `turtlebot3_ros2` C++ source files **must remain at their default values**:
* `wheel_radius`: **0.033**
* `wheel_separation`: **0.160**
* *(And related kinematic constants)*

**Why?** These parameters are hard-linked to the odometry calculation. Modifying them to accommodate the gripper will result in critical errors in SLAM accuracy and navigation stability. Gripper control is handled purely through the repurposed `linear.y` field to maintain system integrity.

---

##  Author
Kim Minsang  Yeungnam University, Robot Engineering
Focus: Embedded Systems, ROS 2, and Optimized Control Architectures.
