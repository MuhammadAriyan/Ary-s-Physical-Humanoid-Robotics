---
title: "ROS 2 Fundamentals"
sidebar_position: 2
---

# Chapter 2: ROS 2 Fundamentals

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the ROS 2 architecture and its communication paradigms
- Implement ROS 2 nodes, publishers, subscribers, services, and actions using Python (rclpy)
- Create and configure ROS 2 packages for humanoid robot applications
- Design launch files and manage parameters for robot systems
- Apply best practices for building robust ROS 2-based humanoid robot software

## 2.1 Introduction to ROS 2

The Robot Operating System 2 (ROS 2) is the second generation of ROS, redesigned from the ground up to meet the demands of production robotics systems. While ROS 1 revolutionized robotics research by providing a common framework for robot software development, ROS 2 addresses critical limitations including real-time requirements, safety certifications, and production deployment.

### The Evolution from ROS 1 to ROS 2

ROS 1 emerged from the Stanford AI Lab and Willow Garage in the late 2000s, becoming the de facto standard for robotics research worldwide. However, as robots moved from laboratory environments into real-world applications, several limitations became apparent:

| Aspect | ROS 1 | ROS 2 |
|--------|-------|-------|
| Real-time Support | Limited (via Orocos RTE) | Native real-time support |
| Security | No built-in security | DDS security framework |
| Lifecycle Management | Custom scripts | Managed node lifecycle |
| Communication Reliability | Best-effort | Configurable QoS policies |
| Multi-robot Support | Complex setup | Native support |
| Deployment | Research-focused | Production-ready |

ROS 2 adopts the Data Distribution Service (DDS) as its underlying communication middleware, providing industry-grade reliability and real-time performance. DDS is used in aerospace, defense, and autonomous vehicle systems where reliability is paramount.

### ROS 2 Architecture Overview

The ROS 2 architecture consists of multiple layers, each building upon the foundations below:

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│         (Robot-specific nodes and algorithms)                │
├─────────────────────────────────────────────────────────────┤
│                      ROS 2 Client Library                    │
│           (rclpy, rclcpp, rcljava, rclnodejs)               │
├─────────────────────────────────────────────────────────────┤
│                     ROS 2 Middleware                         │
│              (DDS implementation - rmw layer)               │
├─────────────────────────────────────────────────────────────┤
│                   DDS Vendor Implementation                  │
│        (Fast-DDS, CycloneDDS, RTI Connext, etc.)            │
├─────────────────────────────────────────────────────────────┤
│                     Transport Layer                          │
│                    (UDP, TCP, shared memory)                 │
└─────────────────────────────────────────────────────────────┘
```

The ROS 2 middleware interface (RMW) abstracts DDS implementations, allowing users to choose between different vendors without changing their application code. For humanoid robotics development, Fast-DDS is commonly used due to its open-source license and good performance characteristics.

### Key Concepts in ROS 2

ROS 2 introduces several fundamental concepts that form the building blocks of any robot application:

**Workspaces**: A workspace is a directory containing ROS 2 packages, where you build and install your software. The typical structure includes a `src` directory for source code, `install` for built artifacts, and `log` for build and runtime logs.

**Packages**: The fundamental unit of ROS 2 software organization. A package contains everything needed for a specific robot functionality: source code, configuration files, launch files, and metadata. Packages can contain nodes, libraries, tools, and documentation.

**Build System**: ROS 2 uses ament as its build system, typically invoked through colcon build tools. The build system handles dependency resolution, compilation, and installation of packages.

**Distribution**: A distribution is a versioned set of ROS 2 packages that are tested together. Current LTS distributions include Humble Hawksbill (Ubuntu 22.04) and Iron Irwini (Ubuntu 23.10). For humanoid robotics, Humble is recommended due to its long-term support and extensive documentation.

```bash
# Setting up a ROS 2 workspace
source /opt/ros/humble/setup.bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
colcon build
source install/setup.bash
```

### DDS and Quality of Service

One of the most powerful features of ROS 2 is its Quality of Service (QoS) policies, inherited from DDS. QoS policies allow developers to tune communication behavior for different use cases:

| Policy | Description | Use Case |
|--------|-------------|----------|
| Reliability | Reliable (retransmits) vs. Best Effort | Best effort for sensor streams |
| Durability | Persistent data for late joiners | Configuration sharing |
| Deadline | Expected publication rate | Real-time guarantees |
| Liveliness | Automated node health checking | Safety-critical systems |
| History | Keep last N samples | Sensor data buffering |

For humanoid robot sensor streams, a typical configuration uses:
- **Sensors**: Best effort reliability, keep last sample, deadline matching sample rate
- **Control Commands**: Reliable delivery, keep last sample, low latency
- **State Estimation**: Reliable, durability with transient-local, history depth of 10

## 2.2 Nodes, Topics, Services, and Actions

ROS 2 provides four primary communication mechanisms, each suited to different patterns of interaction between robot components.

### ROS 2 Nodes

A ROS 2 node is a single-purpose executable that performs a specific task. In humanoid robotics, you might have separate nodes for:

- IMU data acquisition and filtering
- Camera image processing
- Joint trajectory generation
- Balance control
- Path planning
- Speech synthesis

Nodes should be designed as single-responsibility components that communicate with other nodes through well-defined interfaces. This modular architecture enables independent development, testing, and replacement of system components.

```python
#!/usr/bin/env python3
"""
Basic ROS 2 Node Structure

This example demonstrates the fundamental structure of a ROS 2 node
using rclpy, the Python client library for ROS 2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped


class HumanoidNode(Node):
    """
    Base class for humanoid robot nodes.

    This node demonstrates proper initialization, shutdown handling,
    and lifecycle management for ROS 2 nodes.
    """

    def __init__(self, node_name: str, namespace: str = None):
        """
        Initialize the humanoid robot node.

        Args:
            node_name: Unique identifier for this node
            namespace: Optional namespace for topic scoping
        """
        # Initialize the node with a timer for periodic operations
        super().__init__(node_name, namespace=namespace)

        # Create a timer for the main control loop
        self.timer_period = 0.01  # 100 Hz control rate
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Track node state
        self.is_initialized = False
        self._initialize_state()

        self.get_logger().info(f"Node '{self.get_name()}' initialized")

    def _initialize_state(self):
        """Initialize internal state variables."""
        self.current_state = "idle"
        self.last_update_time = self.get_clock().now()
        self.cycle_count = 0

    def control_loop(self):
        """
        Main control loop executed at the timer rate.

        This method should be overridden by subclasses to implement
        specific node functionality.
        """
        self.cycle_count += 1
        # Placeholder for actual control logic
        pass

    def cleanup(self):
        """
        Cleanup resources before node shutdown.

        Override this method to release allocated resources,
        save state, or perform final operations.
        """
        self.get_logger().info(f"Shutting down node '{self.get_name()}'")
        self.is_initialized = False


def main(args=None):
    """Entry point for the ROS 2 node."""
    rclpy.init(args=args)

    try:
        node = HumanoidNode("humanoid_base_node")

        # Spin the node to process callbacks
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Topics: Publisher-Subscriber Communication

Topics implement a publish-subscribe communication pattern where publishers send messages without knowing who receives them, and subscribers receive messages without knowing who sent them. This decoupled architecture is ideal for sensor data streaming and broadcast communications.

For humanoid robots, typical topic-based communications include:

- **Sensor Streams**: IMU data, camera images, LIDAR scans published at high frequency
- **System State**: Joint positions, velocities, efforts published for monitoring
- **Command Broadcasts**: Trajectory commands, mode switches, emergency stops

```python
#!/usr/bin/env python3
"""
IMU Publisher Node

This node demonstrates publisher-subscriber patterns for streaming
sensor data in a humanoid robot system.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUCalibration:
    """Calibration parameters for an IMU sensor."""
    accel_scale: np.ndarray = None
    accel_offset: np.ndarray = None
    gyro_scale: np.ndarray = None
    gyro_offset: np.ndarray = None

    def __post_init__(self):
        if self.accel_scale is None:
            self.accel_scale = np.eye(3)
        if self.gyro_scale is None:
            self.gyro_scale = np.eye(3)


class IMUPublisherNode(Node):
    """
    Node for publishing IMU data from a humanoid robot sensor.

    This node:
    - Interfaces with IMU hardware (simulated in this example)
    - Applies calibration corrections
    - Publishes calibrated IMU data on a topic

    Topics:
        ~/imu_raw: Raw IMU data (calibration input)
        ~/imu: Calibrated IMU data (published)
    """

    def __init__(self):
        super().__init__('imu_publisher_node')

        # Declare parameters with defaults
        self.declare_parameter('sensor_frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 200.0)  # Hz
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('calibration_file', '')

        # Get parameter values
        self.frame_id = self.get_parameter('sensor_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value

        # Initialize calibration
        self.calibration = IMUCalibration()

        # Create publishers
        self.imu_pub = self.create_publisher(
            Imu,
            '~/imu',
            10  # QoS depth for sensor data
        )

        # Create timer for periodic publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        # Simulated sensor state
        self._sim_time = 0.0

        self.get_logger().info(f"IMU Publisher initialized at {self.publish_rate} Hz")

    def _apply_calibration(self, raw_accel: np.ndarray,
                           raw_gyro: np.ndarray) -> tuple:
        """
        Apply calibration corrections to raw sensor readings.

        Args:
            raw_accel: Raw accelerometer reading [ax, ay, az]
            raw_gyro: Raw gyroscope reading [gx, gy, gz]

        Returns:
            Tuple of (corrected_accel, corrected_gyro)
        """
        if not self.use_calibration:
            return raw_accel, raw_gyro

        # Apply scale and offset corrections
        corrected_accel = self.calibration.accel_scale @ (raw_accel - self.calibration.accel_offset)
        corrected_gyro = self.calibration.gyro_scale @ (raw_gyro - self.calibration.gyro_offset)

        return corrected_accel, corrected_gyro

    def _simulate_imu_reading(self) -> tuple:
        """
        Simulate IMU reading for demonstration.

        In a real implementation, this would read from actual hardware.
        """
        self._sim_time += 1.0 / self.publish_rate

        # Simulate static acceleration + gravity + small noise
        gravity = np.array([0.0, 0.0, 9.81])
        noise_level = 0.01
        accel = gravity + np.random.randn(3) * noise_level

        # Simulate angular velocity (slight tilt oscillation)
        omega = 0.1 * np.sin(self._sim_time * 0.5)
        gyro = np.array([omega, omega * 0.5, 0.0]) + np.random.randn(3) * 0.001

        return accel, gyro

    def publish_imu_data(self):
        """Read and publish calibrated IMU data."""
        # Get raw reading from hardware
        raw_accel, raw_gyro = self._simulate_imu_reading()

        # Apply calibration
        accel, gyro = self._apply_calibration(raw_accel, raw_gyro)

        # Create IMU message
        msg = Imu()

        # Set header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Set orientation (would come from sensor fusion in real system)
        msg.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

        # Set linear acceleration
        msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Set angular velocity
        msg.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
        msg.angular_velocity_covariance = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 1e-5]

        # Publish with QoS for sensors (best effort)
        self.imu_pub.publish(msg)


class IMUSubscriberNode(Node):
    """
    Node for subscribing to and processing IMU data.

    This node demonstrates the subscriber side of the publish-subscribe
    pattern, including message filtering and processing.
    """

    def __init__(self, imu_topic: str = '~/imu'):
        super().__init__('imu_subscriber_node')

        # Create subscription with callback
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10  # QoS depth
        )

        # Maintain message history for processing
        self.latest_imu: Optional[Imu] = None
        self.imu_history = []
        self.max_history = 100

        # Moving average filters
        self.accel_filtered = np.zeros(3)
        self.gyro_filtered = np.zeros(3)
        self.alpha = 0.1  # Smoothing factor

        self.get_logger().info(f"Subscribed to {imu_topic}")

    def imu_callback(self, msg: Imu):
        """
        Process incoming IMU messages.

        Args:
            msg: Incoming IMU message
        """
        self.latest_imu = msg

        # Apply low-pass filtering
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        self.accel_filtered = self.alpha * accel + (1 - self.alpha) * self.accel_filtered
        self.gyro_filtered = self.alpha * gyro + (1 - self.alpha) self.gyro_filtered

        # Store in history
        self.imu_history.append(msg)
        if len(self.imu_history) > self.max_history:
            self.imu_history.pop(0)

    def get_filtered_readings(self) -> tuple:
        """Get filtered IMU readings."""
        return self.accel_filtered.copy(), self.gyro_filtered.copy()


def main(args=None):
    """Run the IMU publisher node."""
    rclpy.init(args=args)

    try:
        node = IMUPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
```

### Services: Request-Reply Communication

Services implement a synchronous request-reply pattern where a client sends a request and waits for a response. This pattern is appropriate for:

- Getting the current robot configuration
- Executing one-time commands (calibration, homing)
- Querying system status
- Running diagnostic routines

Unlike topics, services are blocking operations and should be used sparingly for time-sensitive robot operations.

```python
#!/usr/bin/env python3
"""
Service Server and Client Examples

This module demonstrates ROS 2 service communication for
configuration queries and command execution.
"""

from enum import Enum
from typing import Any, Dict, List, Optional
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger, Trigger
from diagnostic_msgs.srv import AddDiagnostics
from trajectory_msgs.msg import JointTrajectoryPoint


class RobotMode(Enum):
    """Robot operational modes."""
    DISABLED = "disabled"
    CALIBRATION = "calibration"
    HOMING = "homing"
    READY = "ready"
    RUNNING = "running"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class JointConfiguration:
    """Configuration for a single joint."""
    name: str
    position: float
    velocity: float
    effort: float
    max_velocity: float
    max_effort: float
    is_enabled: bool


class RobotStateService(Node):
    """
    Service server for robot state management.

    Provides services for:
    - Getting current robot mode
    - Enabling/disabling the robot
    - Executing homing sequences
    """

    def __init__(self):
        super().__init__('robot_state_service')

        # Robot state
        self._current_mode = RobotMode.DISABLED
        self._joints: Dict[str, JointConfiguration] = {}
        self._is_enabled = False

        # Create services
        self.get_mode_srv = self.create_service(
            Trigger,
            '~/get_mode',
            self.get_mode_callback
        )

        self.enable_srv = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback
        )

        self.homing_srv = self.create_service(
            Trigger,
            '~/home',
            self.homing_callback
        )

        self.get_joints_srv = self.create_service(
            Trigger,
            '~/get_joints',
            self.get_joints_callback
        )

        self.get_logger().info("Robot State Service initialized")

    def get_mode_callback(self, request, response):
        """
        Handle mode query request.

        Returns the current operational mode of the robot.
        """
        response.success = True
        response.message = self._current_mode.value
        self.get_logger().debug(f"Mode query: {response.message}")
        return response

    def enable_callback(self, request: SetBool.Request,
                        response: SetBool.Response):
        """
        Handle enable/disable request.

        Args:
            request: SetBool request with data=True to enable

        Returns:
            Response indicating success or failure
        """
        if request.data:
            # Enable the robot
            if self._validate_system():
                self._is_enabled = True
                self._current_mode = RobotMode.READY
                response.success = True
                response.message = "Robot enabled and ready"
                self.get_logger().info("Robot enabled")
            else:
                response.success = False
                response.message = "Cannot enable: system validation failed"
        else:
            # Disable the robot
            self._is_enabled = False
            self._current_mode = RobotMode.DISABLED
            response.success = True
            response.message = "Robot disabled"
            self.get_logger().info("Robot disabled")

        return response

    def homing_callback(self, request: Trigger.Request,
                        response: Trigger.Response):
        """
        Handle homing request.

        Executes a homing sequence to find zero positions for all joints.
        """
        if self._current_mode == RobotMode.RUNNING:
            response.success = False
            response.message = "Cannot home while robot is running"
            return response

        self._current_mode = RobotMode.HOMING
        response.success = True
        response.message = "Homing sequence initiated"

        # In a real implementation, this would:
        # 1. Move each joint to its home position
        # 2. Detect mechanical stops
        # 3. Set zero position
        # 4. Verify calibration

        self.get_logger().info("Starting homing sequence")
        return response

    def get_joints_callback(self, request: Trigger.Request,
                            response: Trigger.Response):
        """
        Handle joint state query request.

        Returns the current state of all configured joints.
        """
        response.success = True
        joint_list = []
        for name, config in self._joints.items():
            joint_list.append(f"{name}: pos={config.position:.3f}")
        response.message = ", ".join(joint_list)
        return response

    def _validate_system(self) -> bool:
        """Validate system is ready to be enabled."""
        # Check all joints are configured
        if not self._joints:
            return False
        # Check all joints have valid calibration
        # Check safety systems are operational
        return True

    def add_joint(self, config: JointConfiguration):
        """Add a joint to the robot configuration."""
        self._joints[config.name] = config


class RobotStateClient(Node):
    """
    Service client for interacting with robot state service.

    Demonstrates how to make service calls from a ROS 2 node.
    """

    def __init__(self):
        super().__init__('robot_state_client')

        # Create service clients
        self.get_mode_client = self.create_client(
            Trigger,
            '~/get_mode'
        )

        self.enable_client = self.create_client(
            SetBool,
            '~/enable'
        )

        self.wait_for_services()

    def wait_for_services(self):
        """Wait for services to become available."""
        while not self.get_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for robot state service...")

    def get_mode(self) -> Optional[str]:
        """Query current robot mode."""
        request = Trigger.Request()
        future = self.get_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().message if future.result().success else None
        return None

    def enable_robot(self, enable: bool = True) -> bool:
        """
        Enable or disable the robot.

        Args:
            enable: True to enable, False to disable

        Returns:
            True if operation succeeded
        """
        request = SetBool.Request()
        request.data = enable

        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        return False
```

### Actions: Long-Running Tasks

Actions are designed for long-running tasks that require feedback and can be preempted. They are built on topics and services internally but provide a unified interface for:

- Trajectory execution (move arm to position)
- Navigation goals (go to location)
- Complex procedures (pick and place operations)
- Any task that should be cancellable with progress updates

Actions are ideal for humanoid robot motion control where trajectories may take several seconds and need to be interruptible.

```python
#!/usr/bin/env python3
"""
ROS 2 Action Server and Client for Trajectory Execution

Actions are ideal for long-running robot movements that require
feedback and can be preempted.
"""

from enum import IntEnum
from time import sleep
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TrajectoryActionServer(Node):
    """
    Action server for executing joint trajectories.

    This server receives trajectory goals and executes them while
    providing periodic feedback on progress.
    """

    def __init__(self):
        super().__init__('trajectory_action_server')

        # Create action server
        self._action_server = ActionServer(
            FollowJointTrajectory,
            '~/follow_joint_trajectory',
            self.execute_callback
        )

        # Track active trajectories
        self._active_trajectory = None
        self._is_cancelled = False

        self.get_logger().info("Trajectory Action Server initialized")

    def execute_callback(self, goal_handle):
        """
        Execute a trajectory goal.

        This method is called when a new trajectory goal is received.
        It executes the trajectory while providing feedback and
        handling cancellation requests.
        """
        self.get_logger().info("Received trajectory goal")

        goal = goal_handle.request.trajectory
        self._is_cancelled = False

        # Extract trajectory information
        joint_names = goal.joint_names
        points = goal.points

        if not points:
            goal_handle.succeed()
            return

        # Calculate total duration
        total_duration = points[-1].time_from_start.sec + \
                        points[-1].time_from_start.nanosec / 1e9

        # Execute trajectory with feedback
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        start_time = self.get_clock().now()

        for i, point in enumerate(points):
            # Check for cancellation
            if self._is_cancelled:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory cancelled"
                goal_handle.aborted(result)
                self._active_trajectory = None
                return

            # Calculate current progress
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9

            # Publish feedback
            feedback.header.stamp = current_time.to_msg()
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

            # Wait for trajectory timing
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            sleep(max(0, target_time - elapsed))

            self.get_logger().debug(f"Trajectory point {i+1}/{len(points)}")

        # Trajectory complete
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory executed successfully"
        goal_handle.succeed(result)
        self._active_trajectory = None

        return result

    def cancel_active_trajectory(self):
        """Cancel the currently executing trajectory."""
        self._is_cancelled = True


class TrajectoryActionClient(Node):
    """
    Action client for sending trajectory goals.

    Demonstrates how to send trajectories and handle feedback.
    """

    def __init__(self):
        super().__init__('trajectory_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '~/follow_joint_trajectory'
        )

        self._goal_handle = None

        self.get_logger().info("Trajectory Action Client initialized")

    def send_trajectory(self, joint_names: list,
                        positions: list, duration: float) -> bool:
        """
        Send a trajectory goal to the action server.

        Args:
            joint_names: List of joint names
            positions: Target positions for each joint
            duration: Time to reach target in seconds

        Returns:
            True if goal was accepted
        """
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(duration),
                                          nanosec=int((duration % 1) * 1e9))
        trajectory.points = [point]

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal
        self.get_logger().info("Sending trajectory goal")
        self._goal_handle = self._action_client.send_goal(
            goal,
            feedback_callback=self.feedback_callback
        )

        return self._goal_handle is not None

    def feedback_callback(self, feedback):
        """Handle feedback from the action server."""
        self.get_logger().debug(f"Feedback: desired time {feedback.feedback.desired.time_from_start}")

    def cancel_goal(self):
        """Cancel the current goal if active."""
        if self._goal_handle:
            self._goal_handle.cancel_goal()
            self._goal_handle = None
```

## 2.3 Building ROS 2 Packages with Python

Creating a well-structured ROS 2 package is essential for maintaining humanoid robot software. This section covers package organization, configuration, and best practices.

### Package Structure

A ROS 2 package for humanoid robotics should follow this structure:

```
humanoid_control/
  package.xml          # Package metadata and dependencies
  setup.py             # Build and installation configuration
  setup.cfg            # Build tool configuration
  humanoid_control/
    __init__.py        # Package initialization
    nodes/             # Executable node scripts
      balance_controller.py
      trajectory_executor.py
    modules/           # Reusable Python modules
      kinematics.py
      dynamics.py
      filters.py
    config/            # Configuration files
      control_params.yaml
      joint_limits.yaml
    launch/            # Launch files
      bringup.launch.py
    tests/             # Unit and integration tests
      test_kinematics.py
```

### Package Configuration Files

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

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>control_msgs</exec_depend>
  <exec_depend>builtin_interfaces</exec_depend>

  <test_depend>pytest</test_depend>
  <test_depend>launch_testing</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>
</package>
```

```python
# setup.py
from setuptools import setup
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
    maintainer='Humanoid Robotics Team',
    keywords=['robotics', 'humanoid', 'control'],
    classifiers=[
        'Environment :: Robots',
        'Intended Audience :: Developers',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
    ],
    description='Control software for humanoid robot platforms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_controller = humanoid_control.nodes.balance_controller:main',
            'trajectory_executor = humanoid_control.nodes.trajectory_executor:main',
        ],
    },
)
```

### Entry Point Scripts

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 2.4 Launch Files and Parameter Management

Launch files enable complex system startup, coordinating the initialization of multiple nodes with appropriate parameters. ROS 2 uses Python-based launch files for flexibility.

### Python Launch Files

```python
#!/usr/bin/env python3
"""
Humanoid Robot System Launch File

This launch file starts all components needed for humanoid robot
operation including sensors, controllers, and state estimation.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate the launch description for the humanoid robot system.

    This function defines all nodes, parameters, and remappings
    needed to bring up the complete robot system.
    """

    # Declare launch arguments
    robot_model = LaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='Robot model identifier'
    )

    use_sim = LaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation instead of real hardware'
    )

    control_mode = LaunchArgument(
        'control_mode',
        default_value='position',
        choices=['position', 'velocity', 'effort', 'impedance'],
        description='Joint control mode'
    )

    # Package share directories
    pkg_share = FindPackageShare('humanoid_control')
    pkg_bringup = FindPackageShare('humanoid_bringup')
    pkg_description = FindPackageShare('humanoid_description')

    # Configuration files
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'control_params.yaml'
    ])

    joint_limits_file = PathJoinSubstitution([
        pkg_share, 'config', 'joint_limits.yaml'
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

    # Balance controller node
    balance_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            params_file,
            joint_limits_file,
            {'control_mode': control_mode},
            {'use_simulation': use_sim}
        ],
        output='screen'
    )
    ld.add_action(balance_controller)

    # Trajectory executor
    trajectory_executor = Node(
        package='humanoid_control',
        executable='trajectory_executor',
        name='trajectory_executor',
        parameters=[
            params_file,
            joint_limits_file
        ],
        output='screen'
    )
    ld.add_action(trajectory_executor)

    # Joint state publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_smallest_joint_limits': True
        }],
        condition=UnlessCondition(use_sim)
    )
    ld.add_action(joint_state_publisher)

    # RViz visualization
    rviz_config = PathJoinSubstitution([
        pkg_bringup, 'rviz', 'humanoid.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    ld.add_action(rviz_node)

    return ld
```

### Parameter Management with YAML

```yaml
# control_params.yaml
# Humanoid Robot Control Parameters
# See Appendix B for simulation setup details

/**:
  ros__parameters:
    # Node configuration
    use_sim_time: false

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
      recovery_gain: 2.0

    # Joint limits (applied from joint_limits.yaml)
    joint_limits:
      left_hip_yaw:
        min_position: -1.5
        max_position: 1.5
        max_velocity: 2.0
        max_effort: 100.0
      right_hip_yaw:
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
      force_torque:
        left_foot:
          frame_id: "left_foot_link"
          sample_rate: 100.0
        right_foot:
          frame_id: "right_foot_link"
          sample_rate: 100.0

    # Logging configuration
    logging:
      level: "INFO"
      file: "/var/log/humanoid/control.log"
```

## 2.5 Complete Publisher-Subscriber Example

The following example demonstrates a complete working system with publishers and subscribers for a humanoid robot application.

```python
#!/usr/bin/env python3
"""
Complete Humanoid Robot State Publisher-Subscriber System

This module demonstrates a complete ROS 2 system for humanoid robot
state management including joint state publishing, IMU data streaming,
and state subscription for monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import publisher_callback, transition_callback
from rclpy.lifecycle import CallbackReturn
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import WrenchStamped, TransformStamped
import numpy as np
from dataclasses import dataclass
from typing import Dict, List, Optional
import transforms3d as t3d


@dataclass
class JointStateData:
    """Container for joint state information."""
    position: float
    velocity: float
    effort: float
    timestamp: float


class HumanoidStatePublisher(Node):
    """
    Lifecycle node for publishing humanoid robot state.

    This node manages the complete state publishing system including:
    - Joint states (position, velocity, effort)
    - IMU data (linear acceleration, angular velocity)
    - End-effector wrenches (force/torque)
    - Base transform (pose in world frame)
    """

    def __init__(self):
        super().__init__('humanoid_state_publisher')

        # Declare parameters
        self.declare_parameter('robot_name', 'humanoid')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('joint_names', [
            'left_hip_yaw', 'left_hip_pitch', 'left_hip_roll',
            'left_knee_pitch', 'left_ankle_pitch', 'left_ankle_roll',
            'right_hip_yaw', 'right_hip_pitch', 'right_hip_roll',
            'right_knee_pitch', 'right_ankle_pitch', 'right_ankle_roll',
            'waist_yaw', 'waist_pitch', 'waist_roll',
            'left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
            'left_elbow_pitch', 'left_wrist_roll', 'left_wrist_pitch',
            'right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
            'right_elbow_pitch', 'right_wrist_roll', 'right_wrist_pitch',
            'neck_pitch', 'neck_yaw'
        ])

        self.robot_name = self.get_parameter('robot_name').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.joint_names = self.get_parameter('joint_names').value

        # Initialize joint state
        self.joint_states: Dict[str, JointStateData] = {}
        for name in self.joint_names:
            self.joint_states[name] = JointStateData(0.0, 0.0, 0.0, 0.0)

        # Create publishers with QoS profiles
        qos_sensor = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        qos_state = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Joint state publisher
        self.joint_state_pub = self.create_publisher(
            JointState,
            '~/joint_states',
            qos_state
        )

        # IMU publisher
        self.imu_pub = self.create_publisher(
            Imu,
            '~/imu/data',
            qos_sensor
        )

        # Base transform publisher
        self.tf_pub = self.create_publisher(
            TransformStamped,
            '~/base_transform',
            qos_state
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )

        # Simulated state
        self.sim_time = 0.0
        self.base_position = np.array([0.0, 0.0, 0.88])  # Standing height

        self.get_logger().info(f"State publisher initialized with {len(self.joint_names)} joints")

    def publish_state(self):
        """Publish all state information."""
        self.sim_time += 1.0 / self.publish_rate

        # Update simulated state
        self._update_simulated_state()

        # Publish joint states
        self._publish_joint_states()

        # Publish IMU data
        self._publish_imu_data()

        # Publish base transform
        self._publish_base_transform()

    def _update_simulated_state(self):
        """Update simulated joint states."""
        for name, state in self.joint_states.items():
            # Add small oscillation for demonstration
            state.timestamp = self.sim_time
            if 'knee' in name or 'elbow' in name:
                # Simulate slight knee/elbow flexing
                state.position = 0.02 * np.sin(self.sim_time * 2.0)
            else:
                # Small oscillations
                state.position = 0.005 * np.sin(self.sim_time)

    def _publish_joint_states(self):
        """Publish joint state message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.name = self.joint_names
        msg.position = [self.joint_states[n].position for n in self.joint_names]
        msg.velocity = [self.joint_states[n].velocity for n in self.joint_names]
        msg.effort = [self.joint_states[n].effort for n in self.joint_names]

        self.joint_state_pub.publish(msg)

    def _publish_imu_data(self):
        """Publish IMU data."""
        from geometry_msgs.msg import Vector3, Quaternion

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulated IMU data (gravity + small motion)
        msg.linear_acceleration = Vector3(
            x=0.05 * np.sin(self.sim_time),
            y=0.03 * np.cos(self.sim_time),
            z=9.81
        )

        msg.angular_velocity = Vector3(
            x=0.01 * np.sin(self.sim_time),
            y=0.01 * np.cos(self.sim_time),
            z=0.005 * np.sin(self.sim_time * 0.5)
        )

        # Orientation from simple tilt estimate
        roll = 0.02 * np.sin(self.sim_time)
        pitch = 0.02 * np.cos(self.sim_time)
        quat = t3d.euler.euler2quat(roll, pitch, 0.0)
        msg.orientation = Quaternion(
            w=quat[0], x=quat[1], y=quat[2], z=quat[3]
        )

        self.imu_pub.publish(msg)

    def _publish_base_transform(self):
        """Publish base link transform."""
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'

        # Simulated base motion (standing still with sway)
        sway_x = 0.01 * np.sin(self.sim_time * 0.5)
        sway_y = 0.008 * np.cos(self.sim_time * 0.7)

        msg.transform.translation.x = sway_x
        msg.transform.translation.y = sway_y
        msg.transform.translation.z = self.base_position[2]

        # Small base orientation change
        roll = 0.01 * np.sin(self.sim_time * 0.3)
        pitch = 0.01 * np.cos(self.sim_time * 0.3)
        quat = t3d.euler.euler2quat(roll, pitch, 0.0)
        msg.transform.rotation = Quaternion(
            w=quat[0], x=quat[1], y=quat[2], z=quat[3]
        )

        self.tf_pub.publish(msg)


class HumanoidStateSubscriber(Node):
    """
    Node for subscribing to and processing humanoid robot state.

    This node demonstrates subscriber patterns for state monitoring
    and provides utilities for state analysis.
    """

    def __init__(self):
        super().__init__('humanoid_state_subscriber')

        # Subscription QoS
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Create subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '~/joint_states',
            self.joint_state_callback,
            qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '~/imu/data',
            self.imu_callback,
            qos
        )

        # State storage
        self.latest_joint_state: Optional[JointState] = None
        self.latest_imu: Optional[Imu] = None
        self.joint_history: Dict[str, List[float]] = {n: [] for n in range(100)}

        # Computed metrics
        self.total_joint_positions = 0.0
        self.num_joints = 28

        self.get_logger().info("State subscriber initialized")

    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state message."""
        self.latest_joint_state = msg

        # Calculate total joint movement
        self.total_joint_positions = sum(abs(p) for p in msg.position)

        # Store in history
        for i, name in enumerate(msg.name):
            if name not in self.joint_history:
                self.joint_history[name] = []
            self.joint_history[name].append(msg.position[i])

            # Keep history limited
            if len(self.joint_history[name]) > 100:
                self.joint_history[name].pop(0)

    def imu_callback(self, msg: Imu):
        """Process incoming IMU message."""
        self.latest_imu = msg

        # Could implement gravity rejection, orientation estimation, etc.

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """Get the current position of a specific joint."""
        if self.latest_joint_state is None:
            return None

        for i, name in enumerate(self.latest_joint_state.name):
            if name == joint_name:
                return self.latest_joint_state.position[i]
        return None

    def get_all_joint_positions(self) -> Dict[str, float]:
        """Get all joint positions as a dictionary."""
        if self.latest_joint_state is None:
            return {}

        return dict(zip(
            self.latest_joint_state.name,
            self.latest_joint_state.position
        ))

    def is_robot_moving(self, threshold: float = 0.001) -> bool:
        """Check if the robot is currently moving."""
        if self.latest_joint_state is None:
            return False

        velocities = self.latest_joint_state.velocity
        return any(abs(v) > threshold for v in velocities)


def main(args=None):
    """Run the state publisher node."""
    rclpy.init(args=args)

    try:
        # Create and spin the publisher
        publisher = HumanoidStatePublisher()
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Hardware Requirements Reference

For running ROS 2 humanoid robot software, the following specifications are recommended:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4-core @ 2.0 GHz | 8-core @ 3.5 GHz |
| RAM | 8 GB | 32 GB DDR4 |
| Storage | 64 GB SSD | 256 GB NVMe SSD |
| GPU | Integrated | NVIDIA RTX 3060+ |
| Network | 1 Gbps Ethernet | 2.5 Gbps or WiFi 6 |
| Power | 65W TDP | 125W TDP |

For simulation environments, see Appendix B which covers Gazebo simulation setup for testing ROS 2 code without physical hardware.

## Chapter Summary

This chapter covered the fundamental concepts of ROS 2 that are essential for humanoid robot software development:

1. **ROS 2 Architecture**: The layered architecture from application code through the ROS client library to DDS middleware provides a robust foundation for production robotics systems.

2. **Communication Paradigms**: Topics provide efficient one-to-many data streaming, services enable synchronous request-reply operations, and actions support long-running tasks with feedback.

3. **Package Development**: Well-structured packages with proper dependencies, entry points, and configuration files are essential for maintainable humanoid robot software.

4. **Launch Files**: Python-based launch files enable complex system orchestration with parameter management and conditional node launching.

5. **Parameter Management**: YAML-based configuration files provide clean separation between code and configuration, enabling easy tuning without code changes.

### Key Concepts

- **Nodes**: Single-purpose executables that form the building blocks of ROS 2 systems
- **Topics**: Publish-subscribe communication for streaming data
- **Services**: Request-reply communication for synchronous operations
- **Actions**: Long-running tasks with feedback and preemption support
- **QoS Policies**: Configurable reliability and performance characteristics
- **Lifecycle Management**: Controlled node startup and shutdown procedures

### Key Terminology

- **DDS (Data Distribution Service)**: Underlying communication middleware providing reliable data distribution
- **QoS (Quality of Service)**: Policies controlling communication behavior
- **Workspace**: Directory containing ROS 2 packages for building
- **Package**: Fundamental software organization unit in ROS 2
- **Colcon**: Build tool for ROS 2 packages
- **RMW**: ROS Middleware Interface abstracting DDS implementations

### Further Reading

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Design Articles: https://design.ros2.org/
- DDS Specification: https://www.omg.org/omg-dds-portal/
- "Programming Robots with ROS" - Morgan Quigley et al.

### Next Chapter

Chapter 3 explores **Actuators and Movement Systems**, examining the mechanical and control aspects of robot actuation. You will learn about motor types, joint design, kinematics, and the control algorithms that enable humanoid robots to move.

### Transition to Simulation

After completing Part 2, Part 3 covers **Simulation and Testing** where you will apply ROS 2 concepts in Gazebo simulation environments. This allows testing control algorithms without risking physical hardware damage.

---

**Part 2: ROS 2 Fundamentals** | [Weeks 3-5 Overview](part-2-ros2/02a-week-3-5-overview) | [Part 3: Simulation](part-3-simulation/gazebo-unity-simulation)
