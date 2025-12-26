---
title: "ROS 2 بنیادیات"
sidebar_position: 2
---

# فصل 2: ROS 2 بنیادیات

## سیکھنے کے مقاصد

اس فصل کے اختتام تک، آپ یہ کر سکیں گے:
- ROS 2 کی فن تعمیر اور اس کے coomunication paradigms کو سمجھنا
- ROS 2 nodes، publishers، subscribers، services، اور actions کو Python (rclpy) استعمال کرتے ہوئے implement کرنا
- Humanoid robot applications کے لیے ROS 2 packages بنانا اور configure کرنا
- Robot systems کے لیے launch files design کرنا اور parameters manage کرنا
- Robust ROS 2-based humanoid robot software بنانے کے بہترین طریقوں کو apply کرنا

## 2.1 ROS 2 کا تعارف

Robot Operating System 2 (ROS 2) ROS کی دوسری نسل ہے، جسے production robotics systems کے مطالبات کو پورا کرنے کے لیے zero سے دوبارہ ڈیزائن کیا گیا ہے۔ جب کہ ROS 1 نے robot software development کے لیے ایک عام فریم ورک فراہم کر کے robotics research میں انقلاب برپا کیا، ROS 2 اہم حدود کو دور کرتا ہے جن میں real-time requirements، safety certifications، اور production deployment شامل ہیں۔

### ROS 1 سے ROS 2 تک کا ارتقاء

ROS 1 Stanford AI Lab اور Willow Garage سے late 2000s میں نمودار ہوا، اور دنیا بھر میں robotics research کے لیے معیاری بن گیا۔ تاہم، جیسے جیسے robots laboratory environments سے real-world applications میں منتقل ہوئے، کچھ حدود واضح ہو گئیں:

| پہلو | ROS 1 | ROS 2 |
|--------|-------|-------|
| Real-time Support | محدود (Orocos RTE کے ذریعے) | Native real-time support |
| سیکیورٹی | کوئی built-in security نہیں | DDS security framework |
| Lifecycle Management | Custom scripts | Managed node lifecycle |
| Communication Reliability | Best-effort | Configurable QoS policies |
| Multi-robot Support | پیچیدہ setup | Native support |
| Deployment | Research-focused | Production-ready |

ROS 2 Data Distribution Service (DDS) کو اپنی بنیادی communication middleware کے طور پر اختیار کرتا ہے، جو industry-grade reliability اور real-time performance فراہم کرتا ہے۔ DDS aerospace، defense، اور autonomous vehicle systems میں استعمال ہوتا ہے جہاں reliability بہت اہم ہے۔

### ROS 2 آرکیٹیکچر کا جائزہ

ROS 2 architecture میں متعدد layers ہیں، ہر ایک اپنے نیچے کی بنیادوں پر تعمیر ہے:

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

ROS 2 middleware interface (RMW) DDS implementations کو خلاصہ کرتا ہے، جو صارفین کو اپنے application code کو تبدیل کیے بغیر مختلف vendors کے درمیان انتخاب کرنے کی اجازت دیتا ہے۔ Humanoid robotics development کے لیے، Fast-DDS عام طور پر استعمال ہوتا ہے کیونکہ اس کا open-source license اور اچھی performance ہے۔

### ROS 2 میں اہم تصورات

ROS 2 کچھ fundamental concepts متعارف کراتا ہے جو کسی بھی robot application کی بنیاد بنتے ہیں:

**Workspaces**: ایک workspace ایسی directory ہے جس میں ROS 2 packages ہوں، جہاں آپ اپنے software کو build اور install کرتے ہیں۔ عام ڈھانچے میں source code کے لیے `src` directory، built artifacts کے لیے `install`، اور build اور runtime logs کے لیے `log` directory شامل ہے۔

**Packages**: ROS 2 software organization کی بنیادی unit۔ ایک package میں specific robot functionality کے لیے سب کچھ ہوتا ہے: source code، configuration files، launch files، اور metadata۔ Packages میں nodes، libraries، tools، اور documentation ہو سکتے ہیں۔

**Build System**: ROS 2 اپنی build system کے طور پر ament استعمال کرتا ہے، عام طور پر colcon build tools کے ذریعے۔ Build system dependency resolution، compilation، اور packages کی installation سنبھالتا ہے۔

**Distribution**: ایک distribution ROS 2 packages کا ایک versioned مجموعہ ہے جو ایک ساتھ test ہوتے ہیں۔ موجودہ LTS distributions میں Humble Hawksbill (Ubuntu 22.04) اور Iron Irwini (Ubuntu 23.10) شامل ہیں۔ Humanoid robotics کے لیے، Humble recommended ہے کیونکہ اس کی long-term support اور extensive documentation ہے۔

```bash
# ROS 2 workspace سیٹ اپ کرنا
source /opt/ros/humble/setup.bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
colcon build
source install/setup.bash
```

### DDS اور Quality of Service

ROS 2 کی سب سے طاقتور features میں سے ایک اس کی Quality of Service (QoS) policies ہے، جو DDS سے موروثی ہے۔ QoS policies developers کو مختلف use cases کے لیے communication behavior کو tune کرنے کی اجازت دیتے ہیں:

| Policy | تفصیل | Use Case |
|--------|-------------|----------|
| Reliability | Reliable (retransmits) یا Best Effort | Sensor streams کے لیے best effort |
| Durability | Late joiners کے لیے persistent data | Configuration sharing |
| Deadline | متوقع publication rate | Real-time guarantees |
| Liveliness | Automated node health checking | Safety-critical systems |
| History | آخری N samples رکھنا | Sensor data buffering |

Humanoid robot sensor streams کے لیے، ایک عام configuration یہ استعمال کرتا ہے:
- **Sensors**: Best effort reliability، keep last sample، deadline matching sample rate
- **Control Commands**: Reliable delivery، keep last sample، low latency
- **State Estimation**: Reliable، durability with transient-local، history depth of 10

## 2.2 Nodes، Topics، Services، اور Actions

ROS 2 چار primary communication mechanisms فراہم کرتا ہے، ہر ایک robot components کے بیچ interaction کے مختلف patterns کے لیے موزوں ہے۔

### ROS 2 Nodes (نوڈز)

ایک ROS 2 node ایک single-purpose executable ہے جو کوئی specific task perform کرتا ہے۔ Humanoid robotics میں، آپ کے پاس الگ nodes ہو سکتے ہیں:

- IMU data acquisition اور filtering کے لیے
- Camera image processing کے لیے
- Joint trajectory generation کے لیے
- Balance control کے لیے
- Path planning کے لیے
- Speech synthesis کے لیے

Nodes کو single-responsibility components کے طور پر design کیا جانا چاہیے جو دوسرے nodes کے ساتھ well-defined interfaces کے ذریعے communicate کریں۔ یہ modular architecture system کے اجزاء کی independent development، testing، اور replacement کو enable کرتا ہے۔

```python
#!/usr/bin/env python3
"""
ROS 2 Node کی بنیادی ڈھانچہ

یہ مثال ROS 2 node کی بنیادی structure کو demonstrate کرتی ہے
جو rclpy، ROS 2 کی Python client library استعمال کرتے ہوئے۔
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import WrenchStamped


class HumanoidNode(Node):
    """
    Humanoid robot nodes کی بنیادی class۔

    یہ node proper initialization، shutdown handling،
    اور ROS 2 nodes کے لیے lifecycle management کو demonstrate کرتا ہے۔
    """

    def __init__(self, node_name: str, namespace: str = None):
        """
        Humanoid robot node کو initialize کرنا۔

        Args:
            node_name: اس node کے لیے منفرد identifier
            namespace: Topics کے scoping کے لیے optional namespace
        """
        # Node کو timer کے ساتھ initialize کریں جو periodic operations کے لیے ہو
        super().__init__(node_name, namespace=namespace)

        # Main control loop کے لیے timer بنائیں
        self.timer_period = 0.01  # 100 Hz control rate
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # Node state کو track کریں
        self.is_initialized = False
        self._initialize_state()

        self.get_logger().info(f"Node '{self.get_name()}' initialized")

    def _initialize_state(self):
        """Internal state variables کو initialize کریں۔"""
        self.current_state = "idle"
        self.last_update_time = self.get_clock().now()
        self.cycle_count = 0

    def control_loop(self):
        """
        Timer rate پر execute ہونے والا main control loop۔

        یہ method subclasses کے ذریعے override کیا جانا چاہیے تاکہ
        specific node functionality کو implement کیا جا سکے۔
        """
        self.cycle_count += 1
        # Actual control logic کے لیے placeholder
        pass

    def cleanup(self):
        """
        Node shutdown سے پہلے resources کو cleanup کریں۔

        یہ method override کیا جا سکتا ہے تاکہ allocated resources کو release کیا جا سکے،
        state کو save کیا جا سکے، یا final operations کو perform کیا جا سکے۔
        """
        self.get_logger().info(f"Shutting down node '{self.get_name()}'")
        self.is_initialized = False


def main(args=None):
    """ROS 2 node کے لیے entry point۔"""
    rclpy.init(args=args)

    try:
        node = HumanoidNode("humanoid_base_node")

        # Node کو spin کریں تاکہ callbacks کو process کیا جا سکے
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup اور shutdown
        if 'node' in locals():
            node.cleanup()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### Topics: Publisher-Subscriber Communication (موضوعات: ناشر-رکن رابطہ)

Topics publish-subscribe communication pattern کو implement کرتے ہیں جہاں publishers messages بھیجتے ہیں بغیر جانے کہ انہیں کون receive کرتا ہے، اور subscribers messages receive کرتے ہیں بغیر جانے کہ انہیں کون بھیجتا ہے۔ یہ decoupled architecture sensor data streaming اور broadcast communications کے لیے بہترین ہے۔

Humanoid robots کے لیے، عام topic-based communications میں شامل ہیں:

- **Sensor Streams**: IMU data، camera images، LIDAR scans high frequency پر published
- **System State**: Joint positions، velocities، efforts monitoring کے لیے published
- **Command Broadcasts**: Trajectory commands، mode switches، emergency stops

```python
#!/usr/bin/env python3
"""
IMU Publisher Node

یہ node humanoid robot system میں streaming sensor data کے لیے
publisher-subscriber patterns کو demonstrate کرتا ہے۔
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
    """IMU sensor کے لیے calibration parameters۔"""
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
    Humanoid robot sensor سے IMU data کو publish کرنے والا Node۔

    یہ node:
    - IMU hardware کے ساتھ interface کرتا ہے (اس مثال میں simulated ہے)
    - Calibration corrections کو apply کرتا ہے
    - ایک topic پر calibrated IMU data publish کرتا ہے

    Topics:
        ~/imu_raw: Raw IMU data (calibration input)
        ~/imu: Calibrated IMU data (published)
    """

    def __init__(self):
        super().__init__('imu_publisher_node')

        # Default values کے ساتھ parameters کو declare کریں
        self.declare_parameter('sensor_frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 200.0)  # Hz
        self.declare_parameter('use_calibration', True)
        self.declare_parameter('calibration_file', '')

        # Parameter values کو حاصل کریں
        self.frame_id = self.get_parameter('sensor_frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_calibration = self.get_parameter('use_calibration').value

        # Calibration کو initialize کریں
        self.calibration = IMUCalibration()

        # Publishers بنائیں
        self.imu_pub = self.create_publisher(
            Imu,
            '~/imu',
            10  # QoS depth for sensor data
        )

        # Periodic publishing کے لیے timer بنائیں
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_imu_data)

        # Simulated sensor state
        self._sim_time = 0.0

        self.get_logger().info(f"IMU Publisher initialized at {self.publish_rate} Hz")

    def _apply_calibration(self, raw_accel: np.ndarray,
                           raw_gyro: np.ndarray) -> tuple:
        """
        Raw sensor readings کو calibration corrections apply کریں۔

        Args:
            raw_accel: Raw accelerometer reading [ax, ay, az]
            raw_gyro: Raw gyroscope reading [gx, gy, gz]

        Returns:
            Tuple of (corrected_accel, corrected_gyro)
        """
        if not self.use_calibration:
            return raw_accel, raw_gyro

        # Scale اور offset corrections کو apply کریں
        corrected_accel = self.calibration.accel_scale @ (raw_accel - self.calibration.accel_offset)
        corrected_gyro = self.calibration.gyro_scale @ (raw_gyro - self.calibration.gyro_offset)

        return corrected_accel, corrected_gyro

    def _simulate_imu_reading(self) -> tuple:
        """
        Demonstration کے لیے IMU reading کو simulate کریں۔

        ایک real implementation میں، یہ actual hardware سے پڑھے گا۔
        """
        self._sim_time += 1.0 / self.publish_rate

        # Static acceleration + gravity + small noise کو simulate کریں
        gravity = np.array([0.0, 0.0, 9.81])
        noise_level = 0.01
        accel = gravity + np.random.randn(3) * noise_level

        # Angular velocity کو simulate کریں (slight tilt oscillation)
        omega = 0.1 * np.sin(self._sim_time * 0.5)
        gyro = np.array([omega, omega * 0.5, 0.0]) + np.random.randn(3) * 0.001

        return accel, gyro

    def publish_imu_data(self):
        """Raw data کو read اور calibrated IMU data کو publish کریں۔"""
        # Hardware سے raw reading حاصل کریں
        raw_accel, raw_gyro = self._simulate_imu_reading()

        # Calibration کو apply کریں
        accel, gyro = self._apply_calibration(raw_accel, raw_gyro)

        # IMU message بنائیں
        msg = Imu()

        # Header set کریں
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation set کریں (real system میں sensor fusion سے آئے گا)
        msg.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        msg.orientation_covariance = [1e-6, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 1e-6]

        # Linear acceleration set کریں
        msg.linear_acceleration = Vector3(x=accel[0], y=accel[1], z=accel[2])
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        # Angular velocity set کریں
        msg.angular_velocity = Vector3(x=gyro[0], y=gyro[1], z=gyro[2])
        msg.angular_velocity_covariance = [1e-5, 0.0, 0.0, 0.0, 1e-5, 0.0, 0.0, 0.0, 1e-5]

        # Sensors کے لیے QoS کے ساتھ publish کریں (best effort)
        self.imu_pub.publish(msg)


class IMUSubscriberNode(Node):
    """
    IMU data کو subscribe اور process کرنے والا Node۔

    یہ node publish-subscribe pattern کے subscriber side کو demonstrate کرتا ہے،
    بشمول message filtering اور processing۔
    """

    def __init__(self, imu_topic: str = '~/imu'):
        super().__init__('imu_subscriber_node')

        # Callback کے ساتھ subscription بنائیں
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10  # QoS depth
        )

        # Processing کے لیے message history کو maintain کریں
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
        Incoming IMU messages کو process کریں۔

        Args:
            msg: Incoming IMU message
        """
        self.latest_imu = msg

        # Low-pass filtering کو apply کریں
        accel = np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        gyro = np.array([msg.angular_velocity.x,
                         msg.angular_velocity.y,
                         msg.angular_velocity.z])

        self.accel_filtered = self.alpha * accel + (1 - self.alpha) * self.accel_filtered
        self.gyro_filtered = self.alpha * gyro + (1 - self.alpha) * self.gyro_filtered

        # History میں store کریں
        self.imu_history.append(msg)
        if len(self.imu_history) > self.max_history:
            self.imu_history.pop(0)

    def get_filtered_readings(self) -> tuple:
        """Filtered IMU readings حاصل کریں۔"""
        return self.accel_filtered.copy(), self.gyro_filtered.copy()


def main(args=None):
    """IMU publisher node کو run کریں۔"""
    rclpy.init(args=args)

    try:
        node = IMUPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
```

### Services: Request-Reply Communication (خدمات: درخواست-جواب رابطہ)

Services ایک synchronous request-reply pattern کو implement کرتی ہیں جہاں ایک client ایک request بھیجتا ہے اور response کا انتظار کرتا ہے۔ یہ pattern یہ کے لیے مناسب ہے:

- موجودہ robot configuration حاصل کرنا
- One-time commands کو execute کرنا (calibration، homing)
- System status کو query کرنا
- Diagnostic routines کو چلانا

Topics کے برعکس، services blocking operations ہیں اور robot کی time-sensitive operations کے لیے کم استعمال کیے جانے چاہیے۔

```python
#!/usr/bin/env python3
"""
Service Server اور Client کی مثالیں

یہ module ROS 2 service communication کو demonstrate کرتا ہے
configuration queries اور command execution کے لیے۔
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
    """Robot کی operational modes۔"""
    DISABLED = "disabled"
    CALIBRATION = "calibration"
    HOMING = "homing"
    READY = "ready"
    RUNNING = "running"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class JointConfiguration:
    """ایک single joint کے لیے configuration۔"""
    name: str
    position: float
    velocity: float
    effort: float
    max_velocity: float
    max_effort: float
    is_enabled: bool


class RobotStateService(Node):
    """
    Robot state management کے لیے service server۔

    یہ services فراہم کرتا ہے:
    - موجودہ robot mode حاصل کرنا
    - Robot کو enable/disable کرنا
    - Homing sequences کو execute کرنا
    """

    def __init__(self):
        super().__init__('robot_state_service')

        # Robot state
        self._current_mode = RobotMode.DISABLED
        self._joints: Dict[str, JointConfiguration] = {}
        self._is_enabled = False

        # Services بنائیں
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
        Mode query request کو handle کریں۔

        Robot کی موجودہ operational mode کو return کرتا ہے۔
        """
        response.success = True
        response.message = self._current_mode.value
        self.get_logger().debug(f"Mode query: {response.message}")
        return response

    def enable_callback(self, request: SetBool.Request,
                        response: SetBool.Response):
        """
        Enable/disable request کو handle کریں۔

        Args:
            request: SetBool request جس میں data=True ہو تو enable کریں

        Returns:
            Success یا failure کو indicate کرتے ہوئے Response
        """
        if request.data:
            # Robot کو enable کریں
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
            # Robot کو disable کریں
            self._is_enabled = False
            self._current_mode = RobotMode.DISABLED
            response.success = True
            response.message = "Robot disabled"
            self.get_logger().info("Robot disabled")

        return response

    def homing_callback(self, request: Trigger.Request,
                        response: Trigger.Response):
        """
        Homing request کو handle کریں۔

        تمام joints کے لیے zero positions تلاش کرنے کے لیے homing sequence کو execute کرتا ہے۔
        """
        if self._current_mode == RobotMode.RUNNING:
            response.success = False
            response.message = "Cannot home while robot is running"
            return response

        self._current_mode = RobotMode.HOMING
        response.success = True
        response.message = "Homing sequence initiated"

        # ایک real implementation میں، یہ:
        # 1. ہر joint کو اس کی home position پر منتقل کریں
        # 2. Mechanical stops کو detect کریں
        # 3. Zero position کو set کریں
        # 4. Calibration کو verify کریں

        self.get_logger().info("Starting homing sequence")
        return response

    def get_joints_callback(self, request: Trigger.Request,
                            response: Trigger.Response):
        """
        Joint state query request کو handle کریں۔

        تمام configured joints کی موجودہ state کو return کرتا ہے۔
        """
        response.success = True
        joint_list = []
        for name, config in self._joints.items():
            joint_list.append(f"{name}: pos={config.position:.3f}")
        response.message = ", ".join(joint_list)
        return response

    def _validate_system(self) -> bool:
        """Validate کریں کہ system enable کرنے کے لیے تیار ہے۔"""
        # Check کریں کہ تمام joints configured ہیں
        if not self._joints:
            return False
        # تمام joints کی valid calibration کو check کریں
        # Safety systems کو operational ہونے کو check کریں
        return True

    def add_joint(self, config: JointConfiguration):
        """Robot configuration میں ایک joint شامل کریں۔"""
        self._joints[config.name] = config


class RobotStateClient(Node):
    """
    Robot state service کے ساتھ interact کرنے کے لیے service client۔

    یہ demonstrate کرتا ہے کہ ایک ROS 2 node سے service calls کو کیسے بنایا جائے۔
    """

    def __init__(self):
        super().__init__('robot_state_client')

        # Service clients بنائیں
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
        """Services کے دستیاب ہونے کا انتظار کریں۔"""
        while not self.get_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for robot state service...")

    def get_mode(self) -> Optional[str]:
        """موجودہ robot mode کو query کریں۔"""
        request = Trigger.Request()
        future = self.get_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().message if future.result().success else None
        return None

    def enable_robot(self, enable: bool = True) -> bool:
        """
        Robot کو enable یا disable کریں۔

        Args:
            enable: True to enable، False to disable

        Returns:
            True اگر operation کامیاب رہا
        """
        request = SetBool.Request()
        request.data = enable

        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().success
        return False
```

### Actions: Long-Running Tasks (کارنامے: طویل مدتی کام)

Actions طویل مدتی کام کے لیے ڈیزائن کیے گئے ہیں جن کو feedback کی ضرورت ہے اور انہیں preempt کیا جا سکتا ہے۔ وہ topics اور services کے اوپر internally بنے ہیں لیکن یہ unified interface فراہم کرتے ہیں:

- Trajectory execution (arm کو position پر منتقل کرنا)
- Navigation goals (کسی جگہ جانا)
- Complex procedures (pick اور place operations)
- کوئی بھی task جو cancellable ہو progress updates کے ساتھ

Actions humanoid robot motion control کے لیے بہترین ہیں جہاں trajectories کچھ سیکنڈ لے سکتی ہیں اور قابل وقفہ ہونی چاہیے۔

```python
#!/usr/bin/env python3
"""
Trajectory Execution کے لیے ROS 2 Action Server اور Client

Actions طویل مدتی robot movements کے لیے بہترین ہیں جن کو
feedback کی ضرورت ہے اور preempt کیا جا سکتا ہے۔
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
    Joint trajectories کو execute کرنے کے لیے action server۔

    یہ server trajectory goals کو receive کرتا ہے اور انہیں execute کرتا ہے
    جبکہ progress پر periodic feedback فراہم کرتا ہے۔
    """

    def __init__(self):
        super().__init__('trajectory_action_server')

        # Action server بنائیں
        self._action_server = ActionServer(
            FollowJointTrajectory,
            '~/follow_joint_trajectory',
            self.execute_callback
        )

        # Active trajectories کو track کریں
        self._active_trajectory = None
        self._is_cancelled = False

        self.get_logger().info("Trajectory Action Server initialized")

    def execute_callback(self, goal_handle):
        """
        ایک trajectory goal کو execute کریں۔

        یہ method تب call ہوتا ہے جب نیا trajectory goal receive ہو۔
        یہ trajectory کو execute کرتا ہے جبکہ feedback فراہم کرتا ہے اور
        cancellation requests کو handle کرتا ہے۔
        """
        self.get_logger().info("Received trajectory goal")

        goal = goal_handle.request.trajectory
        self._is_cancelled = False

        # Trajectory information کو extract کریں
        joint_names = goal.joint_names
        points = goal.points

        if not points:
            goal_handle.succeed()
            return

        # Total duration کا calculate کریں
        total_duration = points[-1].time_from_start.sec + \
                        points[-1].time_from_start.nanosec / 1e9

        # Feedback کے ساتھ trajectory کو execute کریں
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        start_time = self.get_clock().now()

        for i, point in enumerate(points):
            # Cancellation کو check کریں
            if self._is_cancelled:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory cancelled"
                goal_handle.aborted(result)
                self._active_trajectory = None
                return

            # موجودہ progress کا calculate کریں
            current_time = self.get_clock().now()
            elapsed = (current_time - start_time).nanoseconds / 1e9

            # Feedback کو publish کریں
            feedback.header.stamp = current_time.to_msg()
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

            # Trajectory timing کے لیے انتظار کریں
            target_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            sleep(max(0, target_time - elapsed))

            self.get_logger().debug(f"Trajectory point {i+1}/{len(points)}")

        # Trajectory مکمل
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory executed successfully"
        goal_handle.succeed(result)
        self._active_trajectory = None

        return result

    def cancel_active_trajectory(self):
        """فی الوقت executing trajectory کو cancel کریں۔"""
        self._is_cancelled = True


class TrajectoryActionClient(Node):
    """
    Trajectory goals بھیجنے کے لیے action client۔

    یہ demonstrate کرتا ہے کہ trajectories کو کیسے بھیجا جائے اور feedback کو handle کیا جائے۔
    """

    def __init__(self):
        super().__init__('trajectory_action_client')

        # Action client بنائیں
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
        Action server کو ایک trajectory goal بھیجیں۔

        Args:
            joint_names: Joint names کی list
            positions: ہر joint کے لیے target positions
            duration: Target تک پہنچنے کا وقت سیکنڈ میں

        Returns:
            True اگر goal کو accept کیا گیا
        """
        # Trajectory message بنائیں
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names

        # Trajectory point بنائیں
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(sec=int(duration),
                                          nanosec=int((duration % 1) * 1e9))
        trajectory.points = [point]

        # Goal بنائیں
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Goal بھیجیں
        self.get_logger().info("Sending trajectory goal")
        self._goal_handle = self._action_client.send_goal(
            goal,
            feedback_callback=self.feedback_callback
        )

        return self._goal_handle is not None

    def feedback_callback(self, feedback):
        """Action server سے feedback کو handle کریں۔"""
        self.get_logger().debug(f"Feedback: desired time {feedback.feedback.desired.time_from_start}")

    def cancel_goal(self):
        """اگر active ہو تو موجودہ goal کو cancel کریں۔"""
        if self._goal_handle:
            self._goal_handle.cancel_goal()
            self._goal_handle = None
```

## 2.3 Python کے ساتھ ROS 2 Packages بنانا

ایک well-structured ROS 2 package بنانا humanoid robot software کو maintain کرنے کے لیے ضروری ہے۔ یہ section package organization، configuration، اور best practices کو cover کرتا ہے۔

### Package کی ڈھانچہ

Humanoid robotics کے لیے ایک ROS 2 package کو یہ ڈھانچہ follow کرنا چاہیے:

```
humanoid_control/
  package.xml          # Package metadata اور dependencies
  setup.py             # Build اور installation configuration
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
    tests/             # Unit اور integration tests
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
  <description>Humanoid robot کے لیے Control software</description>

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
    description='Humanoid robot platforms کے لیے Control software',
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

یہ balance controller node کے لیے main entry point ہے۔
یہ ROS 2 initialization اور graceful shutdown کو handle کرتا ہے۔
"""

import sys
import signal
import rclpy
from humanoid_control.nodes.balance_controller import BalanceControllerNode


def signal_handler(signum, frame):
    """Shutdown signals کو gracefully handle کریں۔"""
    rclpy.shutdown()


def main(args=None):
    """Balance controller کے لیے main entry point۔"""
    # Signal handlers کو register کریں
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ROS کو initialize کریں
    rclpy.init(args=args)

    try:
        # Node کو بنائیں اور spin کریں
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

## 2.4 Launch Files اور Parameter Management

Launch files متعدد nodes کی initialization کو coordinate کرتے ہوئے complex system startup کو enable کرتی ہیں appropriate parameters کے ساتھ۔ ROS 2 flexibility کے لیے Python-based launch files استعمال کرتا ہے۔

### Python Launch Files

```python
#!/usr/bin/env python3
"""
Humanoid Robot System Launch File

یہ launch file humanoid robot operation کے لیے ضروری تمام components
کو شروع کرتا ہے بشمول sensors، controllers، اور state estimation۔
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
    Humanoid robot system کے لیے launch description generate کریں۔

    یہ function تمام nodes، parameters، اور remappings کو define کرتا ہے
    جو مکمل robot system کو bring up کرنے کے لیے ضروری ہیں۔
    """

    # Launch arguments کو declare کریں
    robot_model = LaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='Robot model identifier'
    )

    use_sim = LaunchArgument(
        'use_sim',
        default_value='false',
        description='Real hardware کے بجائے simulation استعمال کریں'
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

    # Launch description بنائیں
    ld = LaunchDescription()

    # Arguments شامل کریں
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

    # Joint state publisher (visualization کے لیے)
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

### YAML کے ساتھ Parameter Management

```yaml
# control_params.yaml
# Humanoid Robot Control Parameters
# Appendix B میں simulation setup کی تفصیلات دیکھیں

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

    # Joint limits (joint_limits.yaml سے apply کیے جاتے ہیں)
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
      # ... اضافی joints

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

## 2.5 مکمل Publisher-Subscriber کی مثال

درج ذیل مثال humanoid robot application کے لیے publishers اور subscribers کے ساتھ ایک مکمل working system کو demonstrate کرتی ہے۔

```python
#!/usr/bin/env python3
"""
مکمل Humanoid Robot State Publisher-Subscriber System

یہ module humanoid robot state management کے لیے ایک مکمل ROS 2 system کو demonstrate کرتا ہے
بشمول joint state publishing، IMU data streaming،
اور state subscription for monitoring کے۔
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
    """Joint state information کے لیے container۔"""
    position: float
    velocity: float
    effort: float
    timestamp: float


class HumanoidStatePublisher(Node):
    """
    Humanoid robot state کو publish کرنے والا Lifecycle node۔

    یہ node مکمل state publishing system کو manage کرتا ہے بشمول:
    - Joint states (position، velocity، effort)
    - IMU data (linear acceleration، angular velocity)
    - End-effector wrenches (force/torque)
    - Base transform (pose in world frame)
    """

    def __init__(self):
        super().__init__('humanoid_state_publisher')

        # Parameters کو declare کریں
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

        # Joint state کو initialize کریں
        self.joint_states: Dict[str, JointStateData] = {}
        for name in self.joint_names:
            self.joint_states[name] = JointStateData(0.0, 0.0, 0.0, 0.0)

        # QoS profiles کے ساتھ publishers بنائیں
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

        # Periodic publishing کے لیے timer
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )

        # Simulated state
        self.sim_time = 0.0
        self.base_position = np.array([0.0, 0.0, 0.88])  # Standing height

        self.get_logger().info(f"State publisher initialized with {len(self.joint_names)} joints")

    def publish_state(self):
        """تمام state information کو publish کریں۔"""
        self.sim_time += 1.0 / self.publish_rate

        # Simulated state کو update کریں
        self._update_simulated_state()

        # Joint states کو publish کریں
        self._publish_joint_states()

        # IMU data کو publish کریں
        self._publish_imu_data()

        # Base transform کو publish کریں
        self._publish_base_transform()

    def _update_simulated_state(self):
        """Simulated joint states کو update کریں۔"""
        for name, state in self.joint_states.items():
            # Demonstration کے لیے چھوٹی oscillation شامل کریں
            state.timestamp = self.sim_time
            if 'knee' in name or 'elbow' in name:
                # Slight knee/elbow flexing کو simulate کریں
                state.position = 0.02 * np.sin(self.sim_time * 2.0)
            else:
                # چھوٹی oscillations
                state.position = 0.005 * np.sin(self.sim_time)

    def _publish_joint_states(self):
        """Joint state message کو publish کریں۔"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.name = self.joint_names
        msg.position = [self.joint_states[n].position for n in self.joint_names]
        msg.velocity = [self.joint_states[n].velocity for n in self.joint_names]
        msg.effort = [self.joint_states[n].effort for n in self.joint_names]

        self.joint_state_pub.publish(msg)

    def _publish_imu_data(self):
        """IMU data کو publish کریں۔"""
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

        # سادہ tilt estimate سے orientation
        roll = 0.02 * np.sin(self.sim_time)
        pitch = 0.02 * np.cos(self.sim_time)
        quat = t3d.euler.euler2quat(roll, pitch, 0.0)
        msg.orientation = Quaternion(
            w=quat[0], x=quat[1], y=quat[2], z=quat[3]
        )

        self.imu_pub.publish(msg)

    def _publish_base_transform(self):
        """Base link transform کو publish کریں۔"""
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

        # چھوٹی base orientation change
        roll = 0.01 * np.sin(self.sim_time * 0.3)
        pitch = 0.01 * np.cos(self.sim_time * 0.3)
        quat = t3d.euler.euler2quat(roll, pitch, 0.0)
        msg.transform.rotation = Quaternion(
            w=quat[0], x=quat[1], y=quat[2], z=quat[3]
        )

        self.tf_pub.publish(msg)


class HumanoidStateSubscriber(Node):
    """
    Humanoid robot state کو subscribe اور process کرنے والا Node۔

    یہ node state monitoring کے لیے subscriber patterns کو demonstrate کرتا ہے
    اور state analysis کے لیے utilities فراہم کرتا ہے۔
    """

    def __init__(self):
        super().__init__('humanoid_state_subscriber')

        # Subscription QoS
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=10
        )

        # Subscriptions بنائیں
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
        """Incoming joint state message کو process کریں۔"""
        self.latest_joint_state = msg

        # کل joint movement کا calculate کریں
        self.total_joint_positions = sum(abs(p) for p in msg.position)

        # History میں store کریں
        for i, name in enumerate(msg.name):
            if name not in self.joint_history:
                self.joint_history[name] = []
            self.joint_history[name].append(msg.position[i])

            # History کو محدود رکھیں
            if len(self.joint_history[name]) > 100:
                self.joint_history[name].pop(0)

    def imu_callback(self, msg: Imu):
        """Incoming IMU message کو process کریں۔"""
        self.latest_imu = msg

        # gravity rejection، orientation estimation، وغیرہ implement کر سکتے ہیں۔

    def get_joint_position(self, joint_name: str) -> Optional[float]:
        """کسی specific joint کی موجودہ position حاصل کریں۔"""
        if self.latest_joint_state is None:
            return None

        for i, name in enumerate(self.latest_joint_state.name):
            if name == joint_name:
                return self.latest_joint_state.position[i]
        return None

    def get_all_joint_positions(self) -> Dict[str, float]:
        """تمام joint positions کو dictionary کے طور پر حاصل کریں۔"""
        if self.latest_joint_state is None:
            return {}

        return dict(zip(
            self.latest_joint_state.name,
            self.latest_joint_state.position
        ))

    def is_robot_moving(self, threshold: float = 0.001) -> bool:
        """Check کریں کہ robot موجودہ میں move ہو رہا ہے۔"""
        if self.latest_joint_state is None:
            return False

        velocities = self.latest_joint_state.velocity
        return any(abs(v) > threshold for v in velocities)


def main(args=None):
    """State publisher node کو run کریں۔"""
    rclpy.init(args=args)

    try:
        # Publisher بنائیں اور spin کریں
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

ROS 2 humanoid robot software کو run کرنے کے لیے، درج ذیل specifications recommended ہیں:

| جزو | Minimum | تجویز کردہ |
|-----------|---------|-------------|
| CPU | 4-core @ 2.0 GHz | 8-core @ 3.5 GHz |
| RAM | 8 GB | 32 GB DDR4 |
| Storage | 64 GB SSD | 256 GB NVMe SSD |
| GPU | Integrated | NVIDIA RTX 3060+ |
| Network | 1 Gbps Ethernet | 2.5 Gbps یا WiFi 6 |
| Power | 65W TDP | 125W TDP |

Simulation environments کے لیے، Appendix B دیکھیں جو Gazebo simulation setup کو cover کرتا ہے ROS 2 code کو physical hardware کو نقصان پہنچائے بغیر test کرنے کے لیے۔

## فصل کا خلاصہ

اس فصل نے ROS 2 کے بنیادی تصورات کو cover کیا جو humanoid robot software development کے لیے ضروری ہیں:

1. **ROS 2 Architecture**: Application code سے لے کر ROS client library تک DDS middleware تک کی layered architecture production robotics systems کے لیے مضبوط بنیاد فراہم کرتی ہے۔

2. **Communication Paradigms**: Topics ایک-سے-متعدد efficient data streaming فراہم کرتے ہیں، services synchronous request-reply operations کو enable کرتی ہیں، اور actions long-running tasks کو feedback کے ساتھ support کرتے ہیں۔

3. **Package Development**: Proper dependencies، entry points، اور configuration files کے ساتھ well-structured packages humanoid robot software کے لیے ضروری ہیں۔

4. **Launch Files**: Python-based launch files parameter management اور conditional node launching کے ساتھ complex system orchestration کو enable کرتی ہیں۔

5. **Parameter Management**: YAML-based configuration files code اور configuration کے درمیان clean separation فراہم کرتے ہیں، code changes کے بغیر آسان tuning کو enable کرتے ہوئے۔

### اہم تصورات

- **Nodes**: Single-purpose executables جو ROS 2 systems کی building blocks ہیں
- **Topics**: Data streaming کے لیے publish-subscribe communication
- **Services**: Synchronous operations کے لیے request-reply communication
- **Actions**: Feedback اور preemption support کے ساتھ long-running tasks
- **QoS Policies**: Communication behavior کو control کرنے والی configurable reliability اور performance characteristics
- **Lifecycle Management**: Controlled node startup اور shutdown procedures

### اہم اصطلاحیں

- **DDS (Data Distribution Service)**: Reliable data distribution فراہم کرنے والی underlying communication middleware
- **QoS (Quality of Service)**: Communication behavior کو control کرنے والی policies
- **Workspace**: Building کے لیے ROS 2 packages والی directory
- **Package**: ROS 2 میں software organization کی بنیادی unit
- **Colcon**: ROS 2 packages کے لیے build tool
- **RMW**: DDS implementations کو خلاصہ کرنے والا ROS Middleware Interface

### مزید پڑھنے کے لیے

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Design Articles: https://design.ros2.org/
- DDS Specification: https://www.omg.org/omg-dds-portal/
- "Programming Robots with ROS" - Morgan Quigley et al.

### اگلی فصل

فصل 3 **Actuators اور Movement Systems** کو explore کرتی ہے، robot actuation کے mechanical اور control aspects کو examine کرتے ہوئے۔ آپ motor types، joint design، kinematics، اور control algorithms کے بارے میں سیکھیں گے جو humanoid robots کو move کرنے کے قابل بناتے ہیں۔

### Simulation کی طرف منتقل ہونا

Part 2 کے بعد، Part 3 **Simulation اور Testing** کو cover کرتا ہے جہاں آپ Gazebo simulation environments میں ROS 2 concepts کو apply کریں گے۔ یہ physical hardware کو نقصان پہنچائے بغیر control algorithms کو test کرنے کی اجازت دیتا ہے۔

---

**Part 2: ROS 2 Fundamentals** | [Weeks 3-5 Overview](part-2-ros2/02a-week-3-5-overview) | [Part 3: Simulation](part-3-simulation/gazebo-unity-simulation)
