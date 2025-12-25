---
title: "Weeks 6-7: Simulation Fundamentals"
sidebar_position: 4
---

# Part 3: Simulation - Weeks 6-7 Overview

This section provides a structured learning plan for mastering robot simulation using Gazebo and Unity. The two-week curriculum builds practical skills in simulation setup, robot description, sensor modeling, and visualization.

## Week 6: Gazebo Simulation Fundamentals

### Learning Objectives

By the end of Week 6, you will be able to:
- Install and configure Gazebo Harmonic for humanoid robot simulation
- Create and modify simulation worlds with appropriate physics settings
- Understand the Gazebo architecture (server, client, plugins)
- Launch complete simulation environments from ROS 2 launch files
- Configure physics engines for stable humanoid walking simulation
- Debug common simulation issues and performance problems

### Core Topics

#### 1. Gazebo Architecture and Installation

Gazebo uses a client-server architecture where the simulation physics runs independently from visualization. Understanding this separation is essential for efficient development and debugging.

The simulation server runs the physics engine, handles collision detection, and updates sensor plugins. The client provides the graphical interface for visualization and interaction. Multiple clients can connect to a single server, enabling collaborative debugging and monitoring.

Installation on Ubuntu 22.04 requires adding the OSRF repository and installing the ros-humble-gazebo-ros-pkgs package. This provides ROS 2 integration through Gazebo plugins that bridge simulation data with ROS topics.

#### 2. World Configuration

Simulation worlds define the environment where robots operate. Key elements include:
- **Physics configuration**: Gravity, timestep size, solver settings
- **Ground plane**: Friction properties for walking simulation
- **Lighting**: Directional, ambient, and spot lights for perception
- **Environmental models**: Obstacles, terrain, objects for testing

The physics timestep determines simulation accuracy. Humanoid walking requires 1ms timesteps for stable contact resolution, translating to 1000 Hz physics update rate. Larger timesteps reduce computational load but can cause instability in contact-rich behaviors.

#### 3. Gazebo-ROS 2 Integration

ROS 2 integration with Gazebo uses plugins that publish sensor data to ROS topics and subscribe to command topics. The robot_state_publisher node consumes URDF descriptions and provides transform data, while joint_state_publisher_gazebo maintains synchronization between simulation and ROS joint state representations.

### Key Concepts to Master

- **Simulation timestep**: The discrete time unit for physics updates, typically 1ms
- **Real-time factor**: Ratio of simulation speed to wall-clock time
- **Physics engine**: DART, ODE, or Bullet for constraint solving
- **Plugin system**: Extending Gazebo functionality with custom code
- **Topic remapping**: Connecting ROS nodes to Gazebo plugins

### Practice Exercises

1. **Exercise 1: Basic World Setup** (2 hours)
   - Create a new Gazebo world with a ground plane
   - Configure gravity and lighting
   - Add a simple box obstacle
   - Launch from ROS 2 and verify visualization

2. **Exercise 2: Physics Configuration** (2 hours)
   - Experiment with different timestep sizes
   - Compare DART, ODE, and Bullet physics engines
   - Tune contact parameters for stable foot-ground interaction
   - Measure real-time factor on your hardware

3. **Exercise 3: Robot Spawning** (3 hours)
   - Load a humanoid URDF into Gazebo
   - Verify link visualization and collision geometry
   - Check joint limits and dynamics parameters
   - Publish joint states and verify ROS topics

4. **Exercise 4: Sensor Visualization** (2 hours)
   - Add a camera plugin to the robot head
   - View camera output in RViz
   - Add LIDAR and visualize scan data
   - Configure IMU and verify orientation output

### Estimated Time Commitment

| Activity | Hours |
|----------|-------|
| Reading and tutorials | 6 |
| Exercise 1: Basic World | 2 |
| Exercise 2: Physics | 2 |
| Exercise 3: Robot Spawning | 3 |
| Exercise 4: Sensors | 2 |
| Troubleshooting and review | 3 |
| **Total** | **18 hours** |

## Week 7: URDF/SDF and Advanced Simulation

### Learning Objectives

By the end of Week 7, you will be able to:
- Create complete URDF/XACRO descriptions for humanoid robots
- Configure advanced joint types and transmission interfaces
- Implement sensor plugins with realistic noise models
- Build SDF worlds with nested models and custom plugins
- Integrate simulation with control systems using ros2_control
- Optimize simulation performance for complex humanoid models

### Core Topics

#### 1. URDF/XACRO Robot Description

The Unified Robot Description Format provides XML-based specification of robot structure. Links define rigid bodies with inertial, visual, and collision properties. Joints define connections between links with type, axis, and limits.

XACRO extends URDF with macros, variables, and file inclusion. This enables parameterized robot descriptions where one file can generate multiple robot configurations by changing parameter values.

For humanoid robots, common joint configurations include:
- **Revolute**: Single-axis rotation with limits (hip, knee, elbow)
- **Continuous**: Unlimited rotation (neck yaw, waist yaw)
- **Fixed**: Rigid connection (sensor mounts, end effectors)

#### 2. Transmission Interfaces

Transmissions connect actuators to joints and enable integration with ros2_control. Each transmission specifies the joint, actuator, and mechanical reduction. This abstraction allows controller code to remain independent of specific actuator hardware.

```xml
<transmission name="left_hip_yaw_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_yaw">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_yaw_motor">
    <mechanicalReduction>100.0</mechanicalReduction>
  </actuator>
</transmission>
```

#### 3. Sensor Plugin Configuration

Simulated sensors must match physical sensor characteristics to ensure algorithm validity. Each sensor plugin supports configurable noise parameters, bias, scale factors, and update rates.

**IMU Noise Model**:
- Accelerometer noise: Random walk plus bias instability
- Gyroscope noise: Rate random walk plus angle random walk
- Update rate: Should match physical sensor (typically 200-1000 Hz)

**Camera Noise Model**:
- Gaussian noise added to pixel values
- Lens distortion (radial and tangential)
- Exposure variation and motion blur

**LIDAR Noise Model**:
- Range noise proportional to distance
- Angular resolution limits
- Multiple echo handling for transparent surfaces

#### 4. Performance Optimization

Complex humanoid simulations require optimization for smooth real-time operation:

- **Multi-threaded physics**: Configure physics to use multiple CPU cores
- **Simplified collision geometry**: Use primitive shapes for collision even when visual geometry is complex
- **Contact filtering**: Disable collision between links that never contact
- **Sensor update rates**: Reduce rates for non-critical sensors
- **Visual quality reduction**: Lower graphics settings to prioritize physics

### Key Concepts to Master

- **URDF/XACRO**: Robot description with parameterization
- **Transmission**: Actuator-joint mapping for control systems
- **Sensor plugins**: Realistic sensor simulation with noise
- **ros2_control**: Hardware abstraction for simulation and real robots
- **Performance tuning**: Balancing fidelity and speed

### Practice Exercises

1. **Exercise 1: Humanoid URDF Creation** (4 hours)
   - Define pelvis, torso, and head links
   - Create leg joint chain (hip, knee, ankle)
   - Create arm joint chain (shoulder, elbow, wrist)
   - Add neck joints for head movement
   - Configure inertial parameters for each link

2. **Exercise 2: Transmission Configuration** (2 hours)
   - Add transmissions for all actuated joints
   - Configure effort interfaces for walking control
   - Test with ros2_control controller_manager
   - Verify joint command receiving in simulation

3. **Exercise 3: Sensor Plugin Setup** (3 hours)
   - Configure IMU with realistic noise parameters
   - Add camera plugin with distortion correction
   - Set up LIDAR with appropriate resolution
   - Verify sensor data matches expected characteristics

4. **Exercise 4: Control Integration** (3 hours)
   - Create ros2_control controller configuration
   - Implement simple joint position controller
   - Test in simulation with joint command interface
   - Add joint state broadcaster for monitoring

5. **Exercise 5: Performance Optimization** (2 hours)
   - Profile simulation performance
   - Simplify collision geometry where possible
   - Configure multi-threaded physics
   - Measure improvement in real-time factor

### Estimated Time Commitment

| Activity | Hours |
|----------|-------|
| Reading and tutorials | 5 |
| Exercise 1: URDF Creation | 4 |
| Exercise 2: Transmissions | 2 |
| Exercise 3: Sensor Plugins | 3 |
| Exercise 4: Control Integration | 3 |
| Exercise 5: Optimization | 2 |
| Troubleshooting and review | 3 |
| **Total** | **22 hours** |

## Hardware Requirements

For smooth simulation with humanoid robots:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4-core @ 2.5 GHz | 8-core @ 4.0 GHz |
| RAM | 8 GB | 16+ GB |
| GPU | Intel HD 4000 | NVIDIA GTX 1060+ |
| Storage | 20 GB free | 50 GB free SSD |
| Network | 1 Gbps | Not required for local |

### Minimum vs Recommended

The minimum configuration supports basic simulation with reduced physics rates and simplified collision geometry. The recommended configuration provides smooth real-time simulation with full collision geometry and high visualization quality.

## Troubleshooting Common Issues

### Simulation Runs Slowly

1. Check real-time factor in Gazebo window
2. Reduce physics timestep (at cost of accuracy)
3. Simplify collision geometry
4. Disable unnecessary sensors
5. Close Gazebo GUI, use only RViz

### Robot Falls Through Floor

1. Verify ground plane collision geometry
2. Check friction parameters
3. Increase contact solver iterations
4. Enable adaptive timestepping
5. Verify link inertial parameters are non-zero

### Joints Do Not Move

1. Check joint effort limits are non-zero
2. Verify transmission configuration
3. Ensure controller is running and publishing
4. Check for joint limit violations
5. Verify ros2_control is loaded correctly

### Sensor Data Not Publishing

1. Check sensor plugin is loaded (Gazebo terminal output)
2. Verify topic names match subscriptions
3. Check frame_id matches link names
4. Ensure sensor update rate is reasonable
5. Verify ROS domain ID matches between nodes

## Deliverables

At the end of Week 7, complete the following deliverables:

1. **Complete Humanoid URDF**: Full robot description with 25+ joints
2. **Sensor Configuration**: IMU, camera, and LIDAR plugins with noise
3. **Control Integration**: ros2_control with joint controller
4. **Launch File**: Complete simulation startup with all components
5. **Documentation**: Brief report on performance measurements and tuning

## Assessment Criteria

| Criterion | Weight | Description |
|-----------|--------|-------------|
| URDF Completeness | 25% | All joints, links, transmissions defined |
| Physics Stability | 25% | Robot stands and moves without instability |
| Sensor Functionality | 20% | All sensors publish valid data |
| Control Integration | 20% | Controllers interface correctly with simulation |
| Code Quality | 10% | Clean, documented, organized code |

## Next Steps

After completing Weeks 6-7, continue with:
- **Week 8-9**: Isaac Sim for GPU-accelerated simulation
- **Week 10-11**: Perception and AI integration
- **Week 12**: Final project combining all skills

## Additional Resources

### Documentation
- [Gazebo Documentation](https://gazebosim.org/docs)
- [ROS 2 Control Documentation](https://control.ros.org/)
- [URDF Documentation](http://wiki.ros.org/urdf)

### Tutorials
- [Gazebo Tutorials](https://gazebosim.org/tutorials)
- [ROS 2 Control Tutorials](https://control.ros.org/humble/doc/tutorials/controller_manager.html)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

### Community
- [Gazebo Answers](https://answers.gazebosim.org/)
- [ROS Discourse](https://discourse.ros.org/)
- [GitHub Discussions](https://github.com/gazebosim/gz-sim/discussions)

---

:::tip Pro Tip
Keep your URDF modular using XACRO macros. This allows you to create reusable component definitions for legs, arms, and other body parts, then compose complete robots by including and instantiating these macros.
:::

:::note Important
Always test your URDF with `check_urdf` before loading in simulation. This catches syntax errors and common configuration issues early in the development process.
:::

:::warning Caution
Physics timestep too large causes instability in contact-rich behaviors like walking. Start with 1ms and only increase if performance is unacceptable, accepting reduced accuracy.
:::

**Part 2: ROS 2 Fundamentals** | [Weeks 3-5 Overview](part-2-ros2/02a-week-3-5-overview) | [Chapter 3: Gazebo and Unity Simulation](part-3-simulation/gazebo-unity-simulation) | [Part 4: NVIDIA Isaac](part-4-isaac/nvidia-isaac-platform)
