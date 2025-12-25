---
title: "Appendix A: Unitree Robot Hardware Specifications"
sidebar_position: 1
---

# Appendix A: Unitree Robot Hardware Specifications

This appendix provides detailed hardware specifications for the Unitree humanoid and quadruped robots referenced throughout this course. These specifications are essential for developing control algorithms, simulation models, and integration code.

## A.1 Unitree H1 Humanoid Robot

The Unitree H1 is a full-size humanoid robot designed for research and development. It features 19 degrees of freedom in its humanoid configuration and is capable of dynamic bipedal walking, manipulation, and human-robot interaction tasks.

### A.1.1 General Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Height | 1.01 m | Standing height |
| Total Mass | 47 kg | Including battery |
| DOF (Humanoid) | 19 | Standard configuration |
| DOF (Full) | 20+ | Including hands |
| Max Walking Speed | 2.0 m/s | Level ground |
| Operating Time | ~2 hours | Continuous walking |
| Battery | 15 Ah, 25.2V | Li-ion |
| Communication | Ethernet, CAN, USB | High-speed interfaces |
| Control Rate | 200 Hz | Default, configurable |
| Power Consumption | 450 W peak | Dynamic motions |
| Operating Temperature | -10C to 45C | Ambient |

### A.1.2 Joint Specifications

The H1 uses integrated brushless DC motors with harmonic drives for precise position control. Each joint includes absolute position sensing and current feedback.

#### Leg Joints

| Joint | Range (deg) | Velocity (deg/s) | Torque (Nm) | Gear Ratio |
|-------|-------------|------------------|-------------|------------|
| Hip Roll | -45 to 45 | 180 | 150 | 100:1 |
| Hip Pitch | -90 to 90 | 180 | 150 | 100:1 |
| Knee Pitch | -120 to 0 | 288 | 150 | 100:1 |
| Ankle Pitch | -45 to 45 | 216 | 120 | 100:1 |
| Ankle Roll | -30 to 30 | 180 | 80 | 100:1 |

#### Upper Body Joints

| Joint | Range (deg) | Velocity (deg/s) | Torque (Nm) | Gear Ratio |
|-------|-------------|------------------|-------------|------------|
| Waist Yaw | -90 to 90 | 108 | 100 | 100:1 |
| Waist Pitch | -45 to 45 | 108 | 80 | 100:1 |
| Shoulder Pitch | -180 to 180 | 216 | 80 | 100:1 |
| Shoulder Roll | -90 to 90 | 216 | 80 | 100:1 |
| Elbow Pitch | -150 to 0 | 288 | 50 | 100:1 |
| Wrist Pitch | -90 to 90 | 360 | 20 | 100:1 |
| Neck Pitch | -45 to 45 | 180 | 10 | 100:1 |
| Neck Yaw | -90 to 90 | 180 | 10 | 100:1 |

### A.1.3 Link Dimensions

| Link | Length (m) | Width (m) | Mass (kg) |
|------|------------|-----------|-----------|
| Pelvis | 0.15 | 0.25 | 5.0 |
| Upper Leg (femur) | 0.45 | 0.10 | 3.5 |
| Lower Leg (tibia) | 0.45 | 0.08 | 2.5 |
| Foot | 0.25 | 0.12 | 0.8 |
| Torso | 0.30 | 0.20 | 8.0 |
| Upper Arm | 0.30 | 0.08 | 1.5 |
| Forearm | 0.25 | 0.06 | 1.0 |
| Hand | 0.10 | 0.08 | 0.3 |
| Head | 0.15 (diameter) | - | 1.0 |

### A.1.4 Center of Mass Distribution

The H1 center of mass varies with posture. Approximate COM locations:

| Configuration | COM Height (m) | Notes |
|---------------|----------------|-------|
| Standing | 0.85 | Neutral stance |
| Walking (mid-stance) | 0.80 | Forward lean |
| Crouched | 0.50 | Maximum knee bend |
| Arms raised | 0.90 | Elevated COM |

### A.1.5 Sensor Suite

#### Inertial Measurement Unit

| Parameter | Value |
|-----------|-------|
| Type | BMI270 + BMM150 |
| Accelerometer Range | +/- 16g |
| Accelerometer Noise | 0.2 mg/rtHz |
| Gyroscope Range | +/- 2000 deg/s |
| Gyroscope Noise | 0.015 deg/s/rtHz |
| Update Rate | 200 Hz |
| Mount Location | Pelvis center |

#### Force/Torque Sensors

| Location | Fx, Fy Range (N) | Fz Range (N) | Tx, Ty Range (Nm) | Tz Range (Nm) |
|----------|------------------|--------------|-------------------|---------------|
| Left Foot | +/- 500 | 0-1000 | +/- 50 | +/- 30 |
| Right Foot | +/- 500 | 0-1000 | +/- 50 | +/- 30 |
| Left Wrist | +/- 200 | 0-400 | +/- 20 | +/- 10 |
| Right Wrist | +/- 200 | 0-400 | +/- 20 | +/- 10 |

### A.1.6 Communication Interface

#### CAN Bus Configuration

| Parameter | Value |
|-----------|-------|
| CAN Rate | 1 Mbps |
| Message Rate | 200 Hz |
| Joint Command | 0x141-0x14F |
| Joint Feedback | 0x141-0x14F |
| Control Mode | Position/Torque |

#### Ethernet Interface

| Parameter | Value |
|-----------|-------|
| Speed | 1 Gbps |
| IP Address | 192.168.1.120 (default) |
| Control Port | 8080 |
| Data Port | 8081 |
| Protocol | TCP/UDP |

### A.1.7 Software Interface

The H1 SDK provides high-level access to robot capabilities:

```python
# Example H1 Python interface
from unitree_h1 import H1Robot

# Initialize robot
robot = H1Robot()

# Get robot state
state = robot.get_state()
print(f"Joint positions: {state.joint_positions}")
print(f"Joint velocities: {state.joint_velocities}")
print(f"IMU orientation: {state.imu_orientation}")

# Send joint commands
commands = {
    "left_knee": 0.5,  # radians
    "right_knee": 0.5,
}
robot.send_joint_commands(commands, mode="position")

# Get foot positions
left_foot = robot.get_link_position("left_foot_link")
right_foot = robot.get_link_position("right_foot_link")
```

## A.2 Unitree G1 Humanoid Robot

The G1 is a smaller, more lightweight humanoid robot optimized for research and education. It provides a cost-effective platform for algorithm development.

### A.2.1 General Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Height | 0.70 m | Standing height |
| Total Mass | 35 kg | Including battery |
| DOF | 23 | Including hands |
| Max Walking Speed | 1.5 m/s | Level ground |
| Operating Time | ~3 hours | Continuous walking |
| Battery | 10 Ah, 25.2V | Li-ion |
| Control Rate | 200 Hz | Default |

### A.2.2 Joint Specifications

| Joint | Range (deg) | Velocity (deg/s) | Torque (Nm) |
|-------|-------------|------------------|------------|
| Hip Roll | -30 to 30 | 150 | 80 |
| Hip Pitch | -90 to 90 | 150 | 80 |
| Knee Pitch | -120 to 0 | 240 | 60 |
| Ankle Pitch | -45 to 45 | 180 | 40 |
| Waist Yaw | -90 to 90 | 90 | 50 |
| Shoulder Pitch | -180 to 180 | 180 | 40 |
| Elbow Pitch | -150 to 0 | 240 | 30 |
| Wrist (2 DOF) | -90 to 90 | 300 | 15 |

## A.3 Unitree Go2 Quadruped

While not humanoid, the Go2 quadruped provides valuable insights into legged locomotion and serves as a comparison platform for bipedal systems.

### A.3.1 General Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Length | 0.70 m | Body only |
| Standing Height | 0.40 m | Default |
| Total Mass | 12 kg | Including battery |
| DOF | 12 | 3 per leg |
| Max Speed | 3.5 m/s | Running |
| Operating Time | ~2 hours | Continuous walking |
| Terrain Capability | Rough terrain | Obstacle clearance 15cm |
| Control Rate | 500 Hz | High-performance |

### A.3.2 Leg Specifications

| Parameter | Value |
|-----------|-------|
| Hip Length | 0.10 m |
| Upper Leg Length | 0.20 m |
| Lower Leg Length | 0.25 m |
| Foot Radius | 0.04 m |
| Max Joint Torque | 45 Nm |
| Joint Velocity | 25 rad/s |

## A.4 Sensor Comparison

### A.4.1 IMU Specifications Comparison

| Parameter | H1 | G1 | Go2 |
|-----------|-----|-----|------|
| Model | BMI270 | BMI160 | BMI270 |
| Accel Range | +/- 16g | +/- 8g | +/- 16g |
| Accel Noise | 0.2 mg | 0.4 mg | 0.2 mg |
| Gyro Range | +/- 2000 | +/- 2000 | +/- 2000 |
| Gyro Noise | 0.015 dps | 0.030 dps | 0.015 dps |
| Update Rate | 200 Hz | 100 Hz | 500 Hz |

### A.4.2 Foot Force Sensor Comparison

| Parameter | H1 | Go2 |
|-----------|-----|------|
| Fz Range | 1000 N | 400 N |
| Fx, Fy Range | 500 N | 200 N |
| Torque Range | 50 Nm | 20 Nm |
| Resolution | 0.1 N | 0.1 N |
| Bandwidth | 100 Hz | 100 Hz |

## A.5 Dynamic Parameters

### A.5.1 H1 Inertia Approximations

| Link | Ixx (kg m^2) | Iyy (kg m^2) | Izz (kg m^2) |
|------|--------------|--------------|--------------|
| Pelvis | 0.15 | 0.25 | 0.20 |
| Torso | 0.30 | 0.20 | 0.35 |
| Upper Leg | 0.03 | 0.08 | 0.08 |
| Lower Leg | 0.02 | 0.05 | 0.05 |
| Foot | 0.005 | 0.008 | 0.008 |
| Upper Arm | 0.01 | 0.02 | 0.02 |
| Forearm | 0.005 | 0.01 | 0.01 |

### A.5.2 H1 COM Offsets

| Link | X (m) | Y (m) | Z (m) |
|------|-------|-------|-------|
| Pelvis | 0.00 | 0.00 | 0.02 |
| Torso | 0.00 | 0.00 | 0.15 |
| Upper Leg | 0.00 | 0.00 | -0.20 |
| Lower Leg | 0.00 | 0.00 | -0.20 |
| Foot | 0.05 | 0.00 | -0.02 |
| Upper Arm | 0.00 | -0.15 | 0.00 |
| Forearm | 0.00 | -0.10 | 0.00 |

## A.6 ROS 2 Integration

### A.6.1 Topic Names

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| /joint_states | JointState | 200 Hz | Joint positions, velocities, efforts |
| /imu/data | Imu | 200 Hz | Raw IMU data |
| /ft/left_foot | WrenchStamped | 100 Hz | Left foot force/torque |
| /ft/right_foot | WrenchStamped | 100 Hz | Right foot force/torque |
| /odom | Odometry | 100 Hz | Base odometry |
| /cmd_vel | Twist | 50 Hz | Velocity commands |

### A.6.2 Launch File Example

```python
# unitree_h1_bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unitree_h1_bringup',
            executable='h1_node',
            name='h1_robot',
            parameters=[{
                'use_sim': False,
                'control_rate': 200.0,
            }],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': 'xacro ...'
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
    ])
```

### A.6.3 URDF/XACRO Parameters

```xml
<!-- H1 robot description parameters -->
<xacro:property name="hip_lateral_offset" value="0.12" />
<xacro:property name="hip_to_knee" value="0.45" />
<xacro:property name="knee_to_ankle" value="0.45" />
<xacro:property name="ankle_to_ground" value="0.10" />
<xacro:property name="pelvis_height" value="0.95" />
<xacro:property name="total_mass" value="47.0" />
```

## A.7 Control Parameters

### A.7.1 Default Walking Parameters

| Parameter | H1 Value | G1 Value | Description |
|-----------|----------|----------|-------------|
| step_length | 0.25 m | 0.18 m | Forward step size |
| step_width | 0.12 m | 0.10 m | Lateral foot separation |
| step_height | 0.08 m | 0.06 m | Swing foot clearance |
| step_duration | 0.4 s | 0.35 s | Time per step |
| double_support_ratio | 0.2 | 0.25 | Double support fraction |
| com_height | 0.85 m | 0.60 m | CoM during walking |
| zmp_margin | 0.02 m | 0.015 m | Stability margin |

### A.7.2 Joint Control Gains

| Joint Type | Kp | Kd | Feedforward |
|------------|----|----|-------------|
| Hip | 500 | 50 | gravity_compensation |
| Knee | 600 | 60 | gravity_compensation |
| Ankle | 400 | 40 | gravity_compensation |
| Upper Body | 300 | 30 | 0 |
| Wrist | 200 | 20 | 0 |

## A.8 Troubleshooting

### A.8.1 Common Issues

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Joint not responding | CAN bus error | Check cable connections |
| Excessive jitter | Control gain too high | Reduce Kp/Kd |
| Robot falls forward | COM too far forward | Increase ankle pitch range |
| Motor overheating | Current limit exceeded | Reduce torque commands |
| IMU drift | Calibration needed | Run IMU calibration |
| Communication timeout | Network latency | Increase timeout threshold |

### A.8.2 Maintenance Schedule

| Interval | Task |
|----------|------|
| Daily | Check battery level, inspect cables |
| Weekly | Clean joints, check for wear |
| Monthly | Tighten bolts, replace grease |
| Quarterly | Full inspection, replace filters |
| Annually | Motor bearing replacement |

## A.9 Safety Guidelines

### A.9.1 Operating Precautions

1. **Emergency Stop**: Always have emergency stop accessible
2. **Weight Support**: Ensure robot cannot fall on people
3. **Power Limits**: Never exceed rated torque limits
4. **Environment**: Clear area of obstacles and bystanders
5. **Supervised Operation**: Never leave robot running unattended

### A.9.2 Emergency Procedures

| Situation | Action |
|-----------|--------|
| Robot falls | Cut power immediately |
| Joint malfunction | Stop all commands, cut power |
| Smoke/fire | Cut power, evacuate |
| Software freeze | Emergency stop, hard reset |

---

:::tip Pro Tip

When developing control algorithms, always start with simulation using accurate URDF models. The H1 SDK includes a simulation environment that mirrors hardware behavior closely.

:::

:::note Important

Joint limits are hardware enforced. Attempting to command joints beyond their range will result in protective shutdown. Always verify commands before sending.

:::

:::warning Caution

The H1 can move at speeds up to 2 m/s. Ensure the operating area is clear of obstacles and bystanders before any motion.

:::

## References

- Unitree Robotics: https://www.unitree.com/
- H1 SDK Documentation: https://sdk.unitree.com/
- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Humanoid Robot Control Literature: https://arxiv.org/list/cs.RO/recent

---

**Appendix A** | [Chapter 5: Humanoid Robot Development](part-5-humanoid/humanoid-robot-development) | [Appendix C: Community Resources](appendix/C-community-resources)
