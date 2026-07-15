# TurtleBot3 ROS 2 — Gripper-Extended OpenCR Firmware

**Adding a gripper axis to TurtleBot3 without a separate MCU, custom message types, or extra power distribution.**

This repository is a modified OpenCR firmware library that integrates a 1-axis Dynamixel gripper (XC430, ID 11) into the stock TurtleBot3 ROS 2 stack. The unused `linear.y` field of the standard `geometry_msgs/Twist` is repurposed as the gripper command channel — the gripper is driven through the **existing `/cmd_vel` pipeline** with no new message types.

> Built for the **2025 Gyeongnam Global Innovation Festa** autonomous robot competition (Battle Rumble & Mission Robot) — **Grand Prize (Governor's Award)**.

> **Companion change required:** the stock `turtlebot3_node` does not forward `linear.y` to the control table. One line in `cmd_vel_callback` enables it (see [How the Command Reaches the Motor](#how-the-command-reaches-the-motor)).

---

## System Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                      ROS 2 (Nav2 / Teleop)                     │
│                                                                │
│    geometry_msgs/Twist ──► /cmd_vel                            │
│    ├─ linear.x   → drive  (used by diff-drive)                 │
│    ├─ angular.z  → turn   (used by diff-drive)                 │
│    └─ linear.y   → ◆ GRIPPER  (unused on diff-drive)           │
│                     open: -1.0 · close: -0.3                   │
└───────────────────────────────┬────────────────────────────────┘
                                │
              modified turtlebot3_node (cmd_vel_callback)
              dword[1] = linear.y × 100   ← one-line change
                                │  USB / DYNAMIXEL Protocol 2.0
┌───────────────────────────────▼────────────────────────────────┐
│                     OpenCR (this firmware)                     │
│                                                                │
│   Control Table                                                │
│   ├─ Addr 152 : linear.x  ──► wheel motors (ID 1, 2)           │
│   ├─ Addr 154 : linear.y  ──► ◆ non-zero filter                │
│   │                            └► ×0.01 → rad → tick           │
│   │                               → gripper (XC430, ID 11)     │
│   └─ 50 Hz motor control loop (20 ms interval)                 │
└────────────────────────────────────────────────────────────────┘
```

---

## Key Engineering Decisions

### 1. Protocol Repurposing — `linear.y` as the Gripper Channel

On a differential-drive robot, `linear.y` (lateral velocity) is physically meaningless and always empty. This firmware registers it as a control-table item (**Addr 154**) and maps it to the gripper.

- **Why not a custom ROS 2 message?** A custom interface means new message definitions, build dependencies, and a separate communication path. Reusing an existing-but-unused field keeps Nav2/teleop untouched — the gripper is commanded with a plain `Twist` publish.
- **Trade-off (known and accepted):** field repurposing sacrifices interface self-documentation. In production, a dedicated message/service would be the right call; under competition constraints, minimal-change integration won.

### 2. Non-zero Signal Filter — One Choke Point for Every Publisher

Nav2 and teleop continuously publish `Twist` messages with `linear.y = 0` as their default. The firmware processes the gripper channel **only when the value is non-zero**:

```cpp
if (control_items.cmd_vel_linear[1] != 0)   // process only intentional commands
{
    float target_rad = (float)(control_items.cmd_vel_linear[1] * 0.01f);
    controllers.goal_pos_11 = (int32_t)(target_rad * 651.89) + 2048;  // rad → tick
    if (controllers.goal_pos_11 > 4000) controllers.goal_pos_11 = 4000;  // soft limit
    if (controllers.goal_pos_11 < 100)  controllers.goal_pos_11 = 100;   // soft limit
}
// value == 0 → block skipped → gripper HOLDS its last position
```

- Since 0 is never a gripper command (open `-1.0`, close `-0.3`), zero doubles as the *hold* signal: the previous goal is kept and the gripper keeps gripping. **Filtering is the hold logic.**
- Placed in firmware rather than ROS: patching on the ROS side would mean modifying every publisher (teleop, Nav2, any future node); one `if` statement at the firmware choke-point covers them all.

### How the Command Reaches the Motor

| Stage | Transform | Where |
|---|---|---|
| ROS 2 node | `float linear.y` → `int × 100` → `dword[1]` | modified `cmd_vel_callback`, `turtlebot3_node` |
| Firmware | `int × 0.01f` → radians | Addr 154 handler, `turtlebot3.cpp` |
| Firmware | `rad × 651.89 + 2048` → ticks (4096 ticks / 2π, centered) | same block |
| Firmware | clamp 100–4000 ticks | mechanical soft limits |

### 3. Gripper Drive — Position Control

The gripper Dynamixel (ID 11) runs in **Position Control Mode (3)** and is initialized at boot:

| Register | Value | Purpose |
|---|---|---|
| Addr 11 (Operating Mode) | **3** — Position Control | set with torque off (EEPROM area) |
| Addr 64 (Torque Enable) | 1 | enable output |
| Addr 116 (Goal Position) | 2048 | centered initial pose |

Open/close commands from `linear.y` map directly to goal positions; the firmware-side soft limits (100–4000 ticks) bound the mechanical range.

### 4. Hardware Integrity Constants — Do Not Touch

Kinematic constants must remain stock on standard TurtleBot3 hardware:

- `wheel_radius = 0.033`
- `wheel_separation = 0.160`

These feed odometry directly; changing them corrupts SLAM/Nav2 accuracy. That is exactly why gripper control lives in the repurposed `linear.y` field — **the drive kinematics stay untouched.**

---

## Verified Numbers (from this codebase)

| Item | Value | Where |
|---|---|---|
| Motor control loop | **50 Hz** (20 ms) | `INTERVAL_MS_TO_CONTROL_MOTOR = 20`, `turtlebot3.h` |
| Gripper channel | `linear.y` → **Addr 154** | `ADDR_CMD_VEL_LINEAR_Y`, `turtlebot3.cpp` |
| Command values | open **-1.0** · close **-0.3** | competition configuration |
| Scaling chain | ×100 (ROS 2 node) → ×0.01 (firmware) → rad | see table above |
| rad → tick | `× 651.89 + 2048` | 4096 ticks / 2π, centered |
| Soft limits | 100 – 4000 ticks | firmware-side clamp |
| Gripper motor | XC430, ID **11**, Mode **3** (Position Control) | `JOINT_ID_ADD`, `motor_driver.cpp` |

---

## Quick Start

```bash
# 1. Flash this firmware to OpenCR via Arduino IDE
#    (OpenCR board package required — same procedure as stock TB3 firmware).

# 2. Apply the one-line change in turtlebot3_node's cmd_vel_callback:
#      data.dword[1] = static_cast<int32_t>(msg->linear.y * 100);

# 3. Bring up TurtleBot3 as usual, then drive the gripper via /cmd_vel:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -1.0}}"  # open
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {y: -0.3}}"  # close
```

---

## Author

**Kim Minsang (김민상)** — Robotics Engineering, Yeungnam University
Embedded systems · ROS 2 · Control
Build log: https://blog.naver.com/kms031103 · LinkedIn: https://www.linkedin.com/in/kms2003
