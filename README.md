Platform: Jetson AGX Xavier | ROS 2 Foxy (Ubuntu 20.04)

Project Summary This project focuses on integrating a 1-axis Dynamixel gripper into the TurtleBot3 platform by optimizing existing OpenCR resources.
The core objective was to overcome hardware constraints and signal conflicts through firmware-level engineering, avoiding the complexity and latency of additional MCUs or external power systems.

1. Memory Optimization:
  Zero-copy Strategy via Direct Pointer Casting To maintain a strict 100Hz real-time control loop, I implemented a Zero-copy strategy using Direct Pointer Casting.
  By referencing memory addresses directly with (uint8_t*)& instead of copying data into temporary buffers, I eliminated redundant CPU cycles. This optimization reduced control loop latency to under 5ms, ensuring high-speed responsiveness despite limited computational resources.

4. Message Field Repurposing and Signal Filtering I repurposed the linear.y field of the standard ROS 2 Twist message for gripper control to maintain protocol efficiency.
5. To resolve high-frequency jitter caused by default navigation signals, I developed a Non-zero Signal Filter within the firmware. This logic ensures only intentional control signals are processed, effectively protecting the OpenCR board and actuator from overheating and mechanical fatigue.

6. Control Logic:
   Current-based Position Control The actuator operates in Operating Mode 5 (Current-based Position Control).
   By implementing a programmable current limit (300 units), the gripper handles objects of varying densities without mechanical damage. This achieves a critical balance between grasping power and hardware precision while protecting the motor from overcurrent.

8. Engineering Conclusion This project demonstrates that deep-level firmware optimization and protocol analysis are more effective than hardware expansion for solving complex integration problems. By removing redundancy, I established a robust, streamlined control architecture that maintains system integrity under strict operational constraints.

⚠️ [Caution]
When utilizing existing TurtleBot3 hardware, the four fundamental kinematic variables defined within the turtlebot3 C++ source files—such as wheel_radius, wheel_separation, and related constants—must be maintained at their original default values. Altering these parameters can lead to critical errors in odometry calculations and overall navigation stability.

Author: Kim Minsang

Department: Robot Engineering, Yeungnam University

Focus: Embedded Systems, ROS 2, and Optimized Control Architectures.
