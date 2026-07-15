# TurtleBot3 ROS 2 — Gripper-Extended OpenCR Firmware

**Adding a gripper axis to TurtleBot3 without a separate MCU, custom message types, or extra power distribution.**

Modified OpenCR firmware that integrates a 1-axis Dynamixel gripper (XC430, ID 11) into the stock TurtleBot3 ROS 2 stack, driven through the **existing `/cmd_vel` pipeline**.

> Built for the **2025 Gyeongnam Global Innovation Festa** autonomous robot competition (Battle Rumble & Mission Robot) — **Grand Prize (Governor's Award)**.

> **Companion change required:** the stock `turtlebot3_node` does not forward `linear.y` to the control table. One line in `cmd_vel_callback` enables it (see [How the Command Reaches the Motor](#how-the-command-reaches-the-motor)).

---

## System Architecture

```
ROS 2 (Nav2 / Teleop)
  geometry_msgs/Twist → /cmd_vel
   ├─ linear.x  → drive
   ├─ angular.z → turn
   └─ linear.y  → ◆ GRIPPER
        (unused on diff-drive)
        open -1.0 · close -0.3
              │
              ▼
modified turtlebot3_node
  cmd_vel_callback:
  dword[1] = linear.y × 100
  ← one-line change
              │
              ▼  USB / Protocol 2.0
OpenCR (this firmware)
  Addr 152: linear.x
    → wheel motors (ID 1, 2)
  Addr 154: linear.y
    → ◆ non-zero filter
    → ×0.01 → rad → tick
    → gripper (XC430, ID 11)
  50 Hz control loop (20 ms)
```

---

## Key Engineering Decisions

### 1. The Integration Problem — Three Ways to Add an Actuator

TurtleBot3's control path is closed around its two wheel motors: OpenCR firmware and `turtlebot3_node` exchange a fixed control table, and nothing in the stock stack expects a third actuator. The usual options for adding one:

| Option | Cost |
|---|---|
| Separate MCU (e.g., Arduino) for the gripper | extra board, extra power distribution, second comms path |
| Custom ROS 2 message/service | new interface definitions, build dependencies, changes in every consuming node |
| **Repurpose an unused field in the existing pipeline** | one control-table entry + one line in the ROS 2 node |

On a differential-drive robot, `linear.y` (lateral velocity) is physically meaningless — the field travels through the whole Nav2/teleop pipeline permanently empty. Registering it as a control-table item (**Addr 154**) and mapping it to the gripper gives a command channel with **zero new hardware and zero new interfaces**: the gripper is commanded with a plain `Twist` publish.

**Trade-off, accepted knowingly:** field repurposing sacrifices interface self-documentation — `linear.y = -0.3` doesn't announce that it closes a gripper. In production, a dedicated message/service is the right call; under competition constraints, minimal-change integration won.

### 2. Non-zero Signal Filter — One Choke Point Instead of Many Patches

A consequence of sharing `/cmd_vel`: the gripper channel now has **multiple publishers with conflicting defaults**. Nav2 and teleop continuously publish `Twist` messages with `linear.y = 0`, so an intentional gripper command from another terminal competes with a stream of zeros on the same topic.

Two possible fixes:
- Patch every publisher (teleop, Nav2, every future node) to stop emitting the default — invasive, and breaks again with each new node
- Filter once, in firmware, at the single point every command must pass through

This firmware takes the second path:

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

The design closes neatly because 0 is never a valid gripper command (open `-1.0`, close `-0.3`): zero doubles as the *hold* signal — the previous goal is kept and the gripper keeps gripping. **Filtering is the hold logic.**

### How the Command Reaches the Motor

| Stage | Transform | Where |
|---|---|---|
| ROS 2 node | `float linear.y` → `int × 100` → `dword[1]` | modified `cmd_vel_callback`, `turtlebot3_node` |
| Firmware | `int × 0.01f` → radians | Addr 154 handler, `turtlebot3.cpp` |
| Firmware | `rad × 651.89 + 2048` → ticks (4096 ticks / 2π, centered) | same block |
| Firmware | clamp 100–4000 ticks | mechanical soft limits |

### 3. Gripper Drive — Position Control Within the Motor's Limits

The XC430 has no current sensor and no current-control loop (its valid modes: velocity / position / extended position / PWM). That constraint sets the division of labor: **force-aware grasping cannot live in this motor's control loop**, so this firmware keeps the gripper's job simple and safe — position commands bounded by soft limits — and leaves force logic to a layer that can actually observe force. (That layer was built in the follow-up standalone gripper project, using Present Load feedback.)

Boot initialization (ID 11):

| Register | Value | Purpose |
|---|---|---|
| Addr 11 (Operating Mode) | **3** — Position Control | set with torque off (EEPROM area) |
| Addr 64 (Torque Enable) | 1 | enable output |
| Addr 116 (Goal Position) | 2048 | centered initial pose |

### 4. Hardware Integrity Constants — Do Not Touch

Kinematic constants must remain stock on standard TurtleBot3 hardware:

- `wheel_radius = 0.033`
- `wheel_separation = 0.160`

These feed odometry directly; changing them corrupts SLAM/Nav2 accuracy. This is the other half of the `linear.y` decision: the gripper rides in an unused field precisely so that **the drive kinematics stay untouched.**

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
