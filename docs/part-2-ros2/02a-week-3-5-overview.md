---
title: "Weeks 3-5 Overview: ROS 2 Fundamentals"
sidebar_position: 6
---

# Weeks 3-5 Overview: ROS 2 Fundamentals

This three-week module provides comprehensive coverage of ROS 2 fundamentals for humanoid robotics development. The module progresses from core architecture concepts through package development to advanced launch file and parameter management techniques.

:::note
This overview is part of **Part 2: ROS 2 Fundamentals** of the Physical AI & Humanoid Robotics textbook. The complete textbook structure is available in the sidebar navigation.
:::

---

## Week 3: ROS 2 Architecture and Core Concepts

### Learning Objectives

By the end of Week 3, you will be able to:

1. **Explain the ROS 2 architecture** and distinguish it from ROS 1, including the DDS middleware layer
2. **Understand Quality of Service (QoS) policies** and their impact on robot communication
3. **Implement ROS 2 nodes** using rclpy with proper initialization and lifecycle management
4. **Design publish-subscribe systems** using topics for sensor data streaming
5. **Create service clients and servers** for synchronous robot operations

### Key Concepts

#### 3.1 ROS 2 Architecture Overview

ROS 2 represents a fundamental redesign from ROS 1, built on industry-standard DDS middleware:

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│         (Humanoid robot nodes and algorithms)               │
├─────────────────────────────────────────────────────────────┤
│                      ROS 2 Client Library                    │
│                   (rclpy - Python interface)                 │
├─────────────────────────────────────────────────────────────┤
│                     ROS 2 Middleware                         │
│              (DDS implementation - rmw layer)               │
├─────────────────────────────────────────────────────────────┤
│                   DDS Vendor Implementation                  │
│              (Fast-DDS, CycloneDDS, RTI Connext)            │
├─────────────────────────────────────────────────────────────┤
│                     Transport Layer                          │
│                   (UDP, TCP, shared memory)                 │
└─────────────────────────────────────────────────────────────┘
```

**Key architectural differences from ROS 1:**

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Communication | Custom master/node model | DDS-based peer-to-peer |
| Real-time | Requires Orocos integration | Native real-time support |
| Security | No built-in security | DDS Security Framework |
| Lifecycle | Custom scripts | Managed node lifecycle |
| Quality of Service | Best-effort only | Configurable QoS policies |

#### 3.2 Quality of Service (QoS) Policies

QoS policies enable fine-tuned control over communication behavior:

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Sensor data QoS - best effort, volatile
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)

# Control command QoS - reliable, transient local
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# State data QoS - reliable with history
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

#### 3.3 Node Implementation Structure

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HumanoidNode(Node):
    """Base class for humanoid robot nodes."""

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # Create timer for control loop
        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Create publishers/subscribers
        self.publisher = self.create_publisher(
            SomeMessage,
            '~/topic_name',
            10
        )

        self.subscription = self.create_subscription(
            SomeMessage,
            '~/input_topic',
            self.callback_handler,
            10
        )

        self.get_logger().info(f"Node '{self.get_name()}' initialized")

    def control_loop(self):
        """Main control loop."""
        # Override in subclasses
        pass

    def callback_handler(self, msg):
        """Handle incoming messages."""
        # Process message
        pass

def main():
    rclpy.init()
    node = HumanoidNode("my_node")
    rclpy.spin(node)
    rclpy.shutdown()
```

### Estimated Time Commitment

| Activity | Time |
|----------|------|
| Reading (ROS 2 architecture) | 4 hours |
| Understanding DDS and QoS | 3 hours |
| Node implementation exercises | 4 hours |
| Topic/service communication lab | 3 hours |
| Discussion and reflection | 1 hour |
| **Total** | **15 hours** |

### Practice Exercises

1. **Exercise 3.1**: Create a ROS 2 workspace and verify the installation with `ros2 doctor`.

2. **Exercise 3.2**: Implement a node that publishes simulated IMU data at 200 Hz using best-effort QoS.

3. **Exercise 3.3**: Create a service server that returns the current robot mode (disabled, ready, running).

4. **Exercise 3.4**: Implement a subscriber that filters incoming joint state messages and calculates total joint movement.

5. **Exercise 3.5**: Experiment with different QoS profiles and observe how they affect message delivery.

### Discussion Questions

- How does the DDS-based architecture improve upon ROS 1's master-node model?
- When would you choose best-effort vs. reliable QoS for sensor data?
- Why is lifecycle management important for production robot systems?
- What are the trade-offs between using topics vs. services for different operations?

---

## Week 4: Building ROS 2 Packages

### Learning Objectives

By the end of Week 4, you will be able to:

1. **Create well-structured ROS 2 packages** following best practices for humanoid robotics
2. **Configure package dependencies** and metadata in package.xml and setup.py
3. **Implement executable entry points** for ROS 2 nodes
4. **Organize modules, configs, and launch files** within a package
5. **Build and install packages** using colcon

### Key Concepts

#### 4.1 Package Structure

```
my_humanoid_package/
  package.xml          # Package metadata and dependencies
  setup.py             # Build and installation configuration
  setup.cfg            # Build tool configuration
  resource/            # Resource marker file
  my_package/
    __init__.py        # Package initialization
    nodes/
      __init__.py
      balance_controller.py
      trajectory_executor.py
    modules/
      __init__.py
      kinematics.py
      dynamics.py
    config/
      control_params.yaml
      joint_limits.yaml
    launch/
      bringup.launch.py
    tests/
      test_kinematics.py
```

#### 4.2 Package Configuration

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>1.0.0</version>
  <description>Control software for humanoid robot</description>

  <maintainer email="developer@humanoid.robot">
    Humanoid Robotics Team
  </maintainer>

  <license>MIT</license>

  <!-- Dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>control_msgs</exec_depend>

  <!-- Build dependencies -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- Test dependencies -->
  <test_depend>pytest</test_depend>
  <test_depend>launch_testing</test_depend>
</package>
```

```python
# setup.py
from setuptools import setup
import glob
import os

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['numpy', 'scipy'],
    zip_safe=True,
    author='Humanoid Robotics Team',
    description='Control software for humanoid robot platforms',
    entry_points={
        'console_scripts': [
            'balance_controller = humanoid_control.nodes.balance_controller:main',
            'trajectory_executor = humanoid_control.nodes.trajectory_executor:main',
            'state_publisher = humanoid_control.nodes.state_publisher:main',
        ],
    },
)
```

#### 4.3 Node Entry Point Pattern

```python
#!/usr/bin/env python3
"""
Balance Controller Node Entry Point

This is the main entry point for the balance controller node.
It handles ROS 2 initialization and graceful shutdown.
"""

import sys
import signal
import rclpy
from humanoid_control.nodes.balance_controller import BalanceControllerNode


def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    """Main entry point for the balance controller."""
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize ROS
    rclpy.init(args=args)

    try:
        # Create and spin the node
        node = BalanceControllerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node error: {e}")
        sys.exit(1)
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Estimated Time Commitment

| Activity | Time |
|----------|------|
| Reading (package development) | 3 hours |
| Creating package structure | 2 hours |
| Configuring dependencies | 2 hours |
| Building and testing packages | 4 hours |
| Organizing modules and configs | 2 hours |
| **Total** | **13 hours** |

### Practice Exercises

1. **Exercise 4.1**: Create a new ROS 2 package named `humanoid_sensors` with proper dependencies.

2. **Exercise 4.2**: Implement three nodes (publisher, subscriber, service) within the package.

3. **Exercise 4.3**: Create a YAML configuration file for sensor parameters.

4. **Exercise 4.4**: Build the package using colcon and verify all entry points work.

5. **Exercise 4.5**: Add unit tests for the Python modules.

### Discussion Questions

- Why is package organization important for team collaboration?
- What are the benefits of separating configuration from code?
- How should you handle dependencies between packages?
- What testing strategies are appropriate for ROS 2 packages?

---

## Week 5: Advanced Topics - Launch Files and Parameters

### Learning Objectives

By the end of Week 5, you will be able to:

1. **Create Python-based launch files** for complex robot system orchestration
2. **Declare and use launch arguments** for configurable system startup
3. **Manage parameters** using YAML files and dynamic reconfiguration
4. **Implement conditional node launching** based on simulation vs. hardware
5. **Apply best practices** for parameter management in production systems

### Key Concepts

#### 5.1 Launch File Architecture

```python
#!/usr/bin/env python3
"""
Humanoid Robot System Launch File

This launch file starts all components needed for humanoid robot
operation including sensors, controllers, and state estimation.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate the launch description for the humanoid robot system."""

    # Declare launch arguments
    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='Robot model identifier'
    )

    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    control_mode = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Joint control mode'
    )

    # Package share directories
    pkg_share = FindPackageShare('humanoid_control')
    pkg_description = FindPackageShare('humanoid_description')

    # Configuration files
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'control_params.yaml'
    ])

    # URDF/XACRO file
    urdf_file = PathJoinSubstitution([
        pkg_description, 'urdf', 'humanoid.urdf.xacro'
    ])

    # Create the launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(robot_model)
    ld.add_action(use_sim)
    ld.add_action(control_mode)

    # Robot state publisher (URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file,
            'use_tf_static': True,
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher)

    # IMU publisher node
    imu_publisher = Node(
        package='humanoid_control',
        executable='imu_publisher',
        name='imu_publisher',
        parameters=[
            params_file,
            {'frame_id': 'imu_link'},
            {'publish_rate': 200.0}
        ],
        output='screen'
    )
    ld.add_action(imu_publisher)

    # Balance controller (only on real hardware)
    balance_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            params_file,
            {'control_mode': control_mode},
        ],
        condition=UnlessCondition(use_sim),
        output='screen'
    )
    ld.add_action(balance_controller)

    # Simulation controller (only in simulation)
    sim_controller = Node(
        package='humanoid_control',
        executable='sim_controller',
        name='sim_controller',
        parameters=[params_file],
        condition=IfCondition(use_sim),
        output='screen'
    )
    ld.add_action(sim_controller)

    return ld
```

#### 5.2 Parameter Management

```yaml
# control_params.yaml
# Humanoid Robot Control Parameters

/**:
  ros__parameters:
    # Control loop parameters
    control_rate: 200.0  # Hz
    control_mode: "position"

    # Balance controller parameters
    balance_controller:
      kp: [100.0, 100.0, 50.0]
      kd: [10.0, 10.0, 5.0]
      ki: [1.0, 1.0, 0.5]
      com_height: 0.9  # meters
      foot_separation: 0.15  # meters
      max_tilt_angle: 0.1  # radians

    # Joint limits
    joint_limits:
      left_hip_yaw:
        min_position: -1.5
        max_position: 1.5
        max_velocity: 2.0
        max_effort: 100.0
      # ... additional joints

    # Sensor configuration
    sensors:
      imu:
        frame_id: "imu_link"
        publish_rate: 200.0
        accel_range: 16.0  # g
        gyro_range: 2000.0  # deg/s
```

```python
# Accessing parameters in a node
import rclpy
from rclpy.node import Node

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # Declare parameters
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('robot_name', 'humanoid')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])

        # Get parameter values
        self.control_rate = self.get_parameter('control_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.joint_names = self.get_parameter('joint_names').value

        self.get_logger().info(f"Control rate: {self.control_rate} Hz")
```

### Estimated Time Commitment

| Activity | Time |
|----------|------|
| Reading (launch files) | 3 hours |
| Creating launch files | 4 hours |
| Parameter management exercises | 3 hours |
| Conditional launch patterns | 2 hours |
| Testing launch configurations | 3 hours |
| **Total** | **15 hours** |

### Practice Exercises

1. **Exercise 5.1**: Create a launch file that starts joint state publisher, IMU publisher, and balance controller.

2. **Exercise 5.2**: Add configurable parameters for control gains and joint limits.

3. **Exercise 5.3**: Implement a launch argument to switch between simulation and hardware modes.

4. **Exercise 5.4**: Create a YAML configuration file with all robot parameters.

5. **Exercise 5.5**: Test the complete system launch in simulation (see Appendix B).

### Discussion Questions

- Why are Python-based launch files more flexible than XML launch files?
- How do launch arguments improve system configurability?
- What are the best practices for parameter versioning and migration?
- How should simulation parameters differ from hardware parameters?

---

## Code Examples Summary

### Week 3: Node and Communication Patterns

```python
# Basic node structure
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(MsgType, '~/topic', 10)
        self.subscription = self.create_subscription(MsgType, '~/input', self.callback, 10)

    def callback(self, msg):
        # Process message
        pass
```

### Week 4: Package Entry Points

```python
# setup.py entry point
entry_points={
    'console_scripts': [
        'my_node = my_package.nodes.my_node:main',
    ],
}
```

### Week 5: Launch File Structure

```python
from launch_ros.actions import Node

ld = LaunchDescription()
ld.add_action(Node(package='pkg', executable='node', parameters=['config.yaml']))
return ld
```

---

## Additional Resources

### Recommended Reading

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 2 Design Articles**: https://design.ros2.org/
- **DDS Specification**: https://www.omg.org/omg-dds-portal/
- "Programming Robots with ROS" - Morgan Quigley et al.

### Online Resources

- ROS 2 Tutorial Series: https://docs.ros.org/en/humble/Tutorials.html
- GitHub ROS 2 Examples: https://github.com/ros2/examples
- ROS Discourse: https://discourse.ros.org/

### Hardware References

For hands-on ROS 2 development:

| Platform | Purpose | Notes |
|----------|---------|-------|
| NVIDIA Jetson Orin | Edge computing | Native ROS 2 support |
| Intel NUC | Development | Good for simulation |
| Raspberry Pi 5 | Lightweight nodes | Requires ROS 2 rolling |
| AMD64/x86_64 | Standard development | Full ROS 2 compatibility |

### Simulation Reference

For testing ROS 2 code without hardware, see **Appendix B** which covers:
- Gazebo simulation environment setup
- Installing simulation packages
- Running simulated humanoid robot
- Verifying sensor data and control outputs

---

## Weeks 3-5 Progress Checklist

Use this checklist to track your progress through the three-week module:

### Week 3: ROS 2 Architecture

- [ ] Understand the ROS 2 layered architecture
- [ ] Explain DDS and its role in ROS 2
- [ ] Configure QoS profiles for different use cases
- [ ] Implement ROS 2 nodes with rclpy
- [ ] Create publishers, subscribers, and services
- [ ] Complete all practice exercises

### Week 4: Package Development

- [ ] Create well-structured ROS 2 packages
- [ ] Configure package.xml dependencies
- [ ] Write setup.py with entry points
- [ ] Organize modules, configs, and launch files
- [ ] Build packages using colcon
- [ ] Complete all practice exercises

### Week 5: Launch Files and Parameters

- [ ] Create Python-based launch files
- [ ] Declare and use launch arguments
- [ ] Manage parameters with YAML files
- [ ] Implement conditional node launching
- [ ] Apply parameter best practices
- [ ] Complete all practice exercises

---

## Transition to Next Section

After completing Weeks 3-5, you will be ready to explore **Actuators and Movement Systems** in Part 2, Chapter 3, where you will learn about:

- Motor types and control principles
- Joint design and kinematics
- Movement control programming
- Balance and locomotion algorithms

The ROS 2 knowledge from this module will enable you to implement and test these concepts on humanoid robot platforms.

### Quick Preview: Part 3 (Simulation)

Part 3 covers **Simulation and Testing**, where you will apply ROS 2 concepts in Gazebo simulation environments. Topics include:

- Setting up Gazebo simulation environments
- Creating robot models and plugins
- Simulating sensor data and actuator responses
- Testing control algorithms safely
- Bridging simulation and reality

### Quick Preview: Part 4 (Isaac Sim)

Part 4 introduces **NVIDIA Isaac Sim** for high-fidelity simulation:

- GPU-accelerated physics simulation
- Realistic sensor simulation (cameras, LIDAR)
- Reinforcement learning workflows
- Sim-to-real transfer strategies

---

**Part 2: ROS 2 Fundamentals** | [Chapter 2: ROS 2 Fundamentals](part-2-ros2/ros2-fundamentals) | [Part 3: Simulation](part-3-simulation/gazebo-unity-simulation)
