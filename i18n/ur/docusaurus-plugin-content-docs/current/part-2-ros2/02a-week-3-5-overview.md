---
title: "هفته 3-5 کا جائزہ: ROS 2 بنیادی اصول"
sidebar_position: 6
---

# هفته 3-5 کا جائزہ: ROS 2 بنیادی اصول

یہ تین ہفتوں کا ماڈیول ہیومینوئڈ روبوٹکس کی ترقی کے لیے ROS 2 بنیادی اصول کا جامع احاطہ فراہم کرتا ہے۔ یہ ماڈیول مرکزی فن تعمیر کے تصورات سے لے کر پیکیج کی ترقی اور اعلیٰ سطحی لانچ فائل اور پیرامیٹر مینجمنٹ تکنیکوں تک پیش کرتا ہے۔

:::note
یہ جائزہ **حصہ 2: ROS 2 بنیادی اصول** کا حصہ ہے۔ مکمل کتاب کی ساخت سائڈبار نیویگیشن میں دستیاب ہے۔
:::

---

## هفته 3: ROS 2 فن تعمیر اور مرکزی تصورات

### سیکھنے کے مقاصد

هفته 3 کے اختتام پر، آپ کر سکیں گے:

1. **ROS 2 فن تعمیر کی وضاحت کریں** اور DDS مڈل ویئر لیئر سمیت ROS 1 سے اس کی تمیز کریں
2. **Service Quality (QoS) پالیسیوں کو سمجھیں** اور روبوٹ کمیونیکیشن پر ان کے اثرات کو سمجھیں
3. **rclpy کا استعمال کرتے ہوئے ROS 2 نوڈس نافذ کریں** صحیح ابتدائی اور لائف سائیکل مینجمنٹ کے ساتھ
4. **سینسر ڈیٹا سٹریمنگ کے لیے ٹاپکس کا استعمال کرتے ہوئے پبلش-سبسکرائیب سسٹم ڈیزائن کریں**
5. **متماثل روبوٹ آپریشنز کے لیے سروس کلائنٹس اور سرورز بنائیں**

### مرکزی تصورات

#### 3.1 ROS 2 فن تعمیر کا جائزہ

ROS 2 صنعتی معیار DDS مڈل ویئر پر بنا ایک بنیادی دوبارہ ڈیزائن ہے:

```
┌─────────────────────────────────────────────────────────────┐
│                     ایپلیکیشن لیئر                           │
│         (ہیومینوئڈ روبوٹ نوڈس اور الگورتھم)                 │
├─────────────────────────────────────────────────────────────┤
│                    ROS 2 کلائنٹ لائبریری                      │
│                  (rclpy - Python انٹرفیس)                    │
├─────────────────────────────────────────────────────────────┤
│                     ROS 2 مڈل ویئر                           │
│            (DDS نفاذ - rmw لیئر)                            │
├─────────────────────────────────────────────────────────────┤
│                  DDS وینڈر نفاذ                             │
│          (Fast-DDS, CycloneDDS, RTI Connext)               │
├─────────────────────────────────────────────────────────────┤
│                    ٹرانسپورٹ لیئر                            │
│                  (UDP, TCP, شیئرڈ میموری)                   │
└─────────────────────────────────────────────────────────────┘
```

**ROS 1 سے مرکزی فن تعمیری اختلافات:**

| خصوصیت | ROS 1 | ROS 2 |
|---------|-------|-------|
| کمیونیشن | کسٹم ماسٹر/نوڈ ماڈل | DDS پر مبنی پیئر-ٹو-پیئر |
| ریل ٹائم | Orocos انٹیگریشن درکار | قدرتی ریل ٹائم سپورٹ |
| سیکورٹی | کوئی بلٹ ان سیکورٹی نہیں | DDS سیکورٹی فریم ورک |
| لائف سائیکل | کسٹم سکرپٹس | مینیجڈ نوڈ لائف سائیکل |
| کوالٹی آف سروس | صرف بیسٹ ایفورٹ | کانفیگریبل QoS پالیسیاں |

#### 3.2 کوالٹی آف سروس (QoS) پالیسیاں

QoS پالیسیاں کمیونیشن سلوک پر باریک کنٹرول کو فعال کرتی ہیں:

```python
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# سینسر ڈیٹا QoS - بیسٹ ایفورٹ، وولیٹائل
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1
)

# کنٹرول کمانڈ QoS - قابل اعتماد، ٹرانزینٹ لوکل
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# ڈیٹا QoS - قابل اعتماد تاریخ کے ساتھ
state_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

<!--
کوڈ کا مقصد: مختلف قسم کے ڈیٹا کے لیے QoS پروفائلز کی تعریف کرنا
- سینسر ڈیٹا کے لیے تیز لیکن غیر قابل اعتماد QoS
- کنٹرول کمانڈز کے لیے قابل اعتماد اور پرسنٹ QoS
- روبوٹ اسٹیٹ ڈیٹا کے لیے متوازن QoS
-->

#### 3.3 نوڈ نفاذ کی ساخت

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HumanoidNode(Node):
    """ہیومینوئڈ روبوٹ نوڈس کی بنیادی کلاس۔"""

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # کنٹرول لوپ کے لیے ٹائمر بنائیں
        self.timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        # پبلشرز/سبسکرائبرز بنائیں
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
        """مرکزی کنٹرول لوپ۔"""
        # ذیلی کلاسز میں اوور رائیڈ کریں
        pass

    def callback_handler(self, msg):
        """آنے والے پیغامات کو ہینڈل کریں۔"""
        # پیغام پراسیس کریں
        pass

def main():
    rclpy.init()
    node = HumanoidNode("my_node")
    rclpy.spin(node)
    rclpy.shutdown()
```

<!--
کوڈ کا مقصد: ایک بنیادی ROS 2 نوڈ کی ساخت جو درج ذیل کرتا ہے:
- ہیومینوئڈ روبوٹ کے لیے ایک بنیادی نوڈ کلاس بنانا
- 100 Hz پر کنٹرول لوپ چلانا
- پبلشر اور سبسکرائبر بنانا
- لاگنگ اور ابتدائی/اختتام کا انتظام کرنا
-->

### اندازہ شدہ وقت کا عہدہ

| سرگرمی | وقت |
|----------|------|
| پڑھنا (ROS 2 فن تعمیر) | 4 گھنٹے |
| DDS اور QoS سمجھنا | 3 گھنٹے |
| نوڈ نفاذ مشقیں | 4 گھنٹے |
| ٹاپک/سروس کمیونیشن لیب | 3 گھنٹے |
| بحث اور غور وفکر | 1 گھنٹہ |
| **کل** | **15 گھنٹے** |

### مشقی تمارین

1. **مشق 3.1**: ایک ROS 2 ورک اسپیس بنائیں اور `ros2 doctor` کے ساتھ انسٹالیشن کی تصدیق کریں۔

2. **مشق 3.2**: ایک نوڈ نافذ کریں جو بیسٹ ایفورٹ QoS کا استعمال کرتے ہوئے 200 Hz پر نقل شدہ IMU ڈیٹا شائع کرے۔

3. **مشق 3.3**: ایک سروس سرور بنائیں جو موجودہ روبوٹ موڈ (disabled, ready, running) لوٹ کرے۔

4. **مشق 3.4**: ایک سبسکرائبر نافذ کریں جو آنے والے joint state پیغامات کو فلٹر کرے اور کل joint movement کا حساب لگائے۔

5. **مشق 3.5**: مختلف QoS پروفائلز کے ساتھ تجربہ کریں اور دیکھیں کہ وہ میسج ڈیلیوری کو کیسے متاثر کرتی ہیں۔

### بحث کے سوالات

- DDS پر مبنی فن تعمیر ROS 1 کے ماسٹر-نوڈ ماڈل پر کیسے بہتری لایا ہے؟
- سینسر ڈیٹا کے لیے بیسٹ ایفورٹ بمقابلہ قابل اعتماد QoS کب انتخاب کریں گے؟
- پروڈکشن روبوٹ سسٹمز کے لیے لائف سائیکل مینجمنٹ کیوں اہم ہے؟
- مختلف آپریشنز کے لیے ٹاپکس بمقابلہ سروسز کے درمیان تجارتی نکات کیا ہیں؟

---

## هفته 4: ROS 2 پیکیجز بنانا

### سیکھنے کے مقاصد

هفته 4 کے اختتام پر، آپ کر سکیں گے:

1. **ہیومینوئڈ روبوٹکس کے لیے بہترین طریقوں کی پیروی کرتے ہوئے اچھی ساخت والے ROS 2 پیکیجز بنائیں**
2. **package.xml اور setup.py میں پیکیج کی انحصارات اور میٹا ڈیٹا کانفیگر کریں**
3. **ROS 2 نوڈس کے لیے قابل نفاذ انٹری پوائنٹس نافذ کریں**
4. **ایک پیکیج کے اندر ماڈیولز، کانفگز، اور لانچ فائلز کی تنظیم کریں**
5. **colcon کا استعمال کرتے ہوئے پیکیجز بنائیں اور انسٹال کریں**

### مرکزی تصورات

#### 4.1 پیکیج کی ساخت

```
my_humanoid_package/
  package.xml          # پیکیج میٹا ڈیٹا اور انحصارات
  setup.py             # بنانے اور انسٹالیشن کانفیگریشن
  setup.cfg            # بنانے والے ٹول کانفیگریشن
  resource/            # ریسورس مارکر فائل
  my_package/
    __init__.py        # پیکیج ابتدائی
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

#### 4.2 پیکیج کانفیگریشن

```xml
<!-- package.xml -->
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>1.0.0</version>
  <description>ہیومینوئڈ روبوٹ کے لیے کنٹرول سافٹ ویئر</description>

  <maintainer email="developer@humanoid.robot">
    ہیومینوئڈ روبوٹکس ٹیم
  </maintainer>

  <license>MIT</license>

  <!-- انحصارات -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>control_msgs</exec_depend>

  <!-- بنانے کی انحصارات -->
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- ٹیسٹ انحصارات -->
  <test_depend>pytest</test_depend>
  <test_depend>launch_testing</test_depend>
</package>
```

<!--
کوڈ کا مقصد: ROS 2 پیکیج کی میٹا ڈیٹا اور انحصارات کی تعریف کرنا
- پیکیج کا نام، ورژن اور تفصیل بتانا
- روز/نوڈس اور سینسر میسجز جیسے انحصارات شامل کرنا
- ٹیسٹنگ کے لیے ضروری ٹولز کی فہرست دینا
-->

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
    author='ہیومینوئڈ روبوٹکس ٹیم',
    description='ہیومینوئڈ روبوٹ پلیٹ فارم کے لیے کنٹرول سافٹ ویئر',
    entry_points={
        'console_scripts': [
            'balance_controller = humanoid_control.nodes.balance_controller:main',
            'trajectory_executor = humanoid_control.nodes.trajectory_executor:main',
            'state_publisher = humanoid_control.nodes.state_publisher:main',
        ],
    },
)
```

<!--
کوڈ کا مقصد: پیکیج کی بنائیت اور انسٹالیشن کی سیٹنگ کرنا
- پیکیج کی بنیادی معلومات (نام، ورژن، تفصیل) بتانا
- کانفگریشن اور لانچ فائلز کو انسٹالیشن میں شامل کرنا
- کمانڈ لائن سے چلائے جانے والے نوڈس کے انٹری پوائنٹس متعین کرنا
-->

#### 4.3 نوڈ انٹری پوائنٹ پیٹرن

```python
#!/usr/bin/env python3
"""
بیلنس کنٹرولر نوڈ انٹری پوائنٹ

یہ بیلنس کنٹرولر نوڈ کا مرکزی انٹری پوائنٹ ہے۔
یہ ROS 2 کی ابتدا اور خوبصورت بندش کو سنبھالتا ہے۔
"""

import sys
import signal
import rclpy
from humanoid_control.nodes.balance_controller import BalanceControllerNode


def signal_handler(signum, frame):
    """بندش کے سگنلز کو خوبصورتی سے سنبھالیں۔"""
    rclpy.shutdown()
    sys.exit(0)


def main(args=None):
    """بیلنس کنٹرولر کے لیے مرکزی انٹری پوائنٹ۔"""
    # سگنل ہینڈلر رجسٹر کریں
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ROS کو شروع کریں
    rclpy.init(args=args)

    try:
        # نوڈ بنائیں اور اسپن کریں
        node = BalanceControllerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node error: {e}")
        sys.exit(1)
    finally:
        # صفائی کریں
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

<!--
کوڈ کا مقصد: ROS 2 نوڈ کے لیے ایک پکے ہوئے انٹری پوائنٹ کی تخلیق
- سگنل ہینڈلنگ (SIGINT, SIGTERM) کو لاگو کرنا
- ROS 2 کی صحیح ابتدا اور بندش کو یقینی بنانا
- غلطیوں کو خوبصورتی سے سنبھالنا اور وسائل کو صاف کرنا
-->

### اندازہ شدہ وقت کا عہدہ

| سرگرمی | وقت |
|----------|------|
| پڑھنا (پیکیج ترقی) | 3 گھنٹے |
| پیکیج کی ساخت بنانا | 2 گھنٹے |
| انحصارات کانفیگر کرنا | 2 گھنٹے |
| پیکیجز بنانا اور ٹیسٹ کرنا | 4 گھنٹے |
| ماڈیولز اور کانفگز کی تنظیم | 2 گھنٹے |
| **کل** | **13 گھنٹے** |

### مشقی تمارین

1. **مشق 4.1**: صحیح انحصارات کے ساتھ `humanoid_sensors` نامی ایک نیا ROS 2 پیکیج بنائیں۔

2. **مشق 4.2**: پیکیج کے اندر تین نوڈس (پبلشر، سبسکرائبر، سروس) نافذ کریں۔

3. **مشق 4.3**: سینسر پیرامیٹرز کے لیے ایک YAML کانفیگریشن فائل بنائیں۔

4. **مشق 4.4**: colcon کا استعمال کرتے ہوئے پیکیج بنائیں اور تمام انٹری پوائنٹس کی تصدیق کریں۔

5. **مشق 4.5**: Python ماڈیولز کے لیے یونٹ ٹیسٹس شامل کریں۔

### بحث کے سوالات

- ٹیم کے تعاون کے لیے پیکیج کی تنظیم کیوں اہم ہے؟
- کانفیگریشن کو کوڈ سے الگ کرنے کے کیا فوائد ہیں؟
- پیکیجز کے درمیان انحصار کو کیسے سنبھالنا چاہیے؟
- ROS 2 پیکیجز کے لیے کون سی ٹیسٹنگ حکمت عملی مناسب ہے؟

---

## هفہ 5: اعلیٰ موضوعات - لانچ فائلز اور پیرامیٹرز

### سیکھنے کے مقاصد

هفہ 5 کے اختتام پر، آپ کر سکیں گے:

1. **م Complex روبوٹ سسٹم آرکسٹریشن کے لیے Python پر مبنی لانچ فائلز بنائیں**
2. **قابل کانفیگریشن سسٹم اسٹارٹپ کے لیے لانچ آرگومینٹس کا اعلان اور استعمال کریں**
3. **YAML فائلز اور ڈینامک دوبارہ کانفیگریشن کا استعمال کرتے ہوئے پیرامیٹرز کا انتظام کریں**
4. **سیمولیشن بمقابلہ ہارڈویئر کی بنیاد پر مشروط نوڈ لانچنگ نافذ کریں**
5. **پروڈکشن سسٹمز میں پیرامیٹر مینجمنٹ کے لیے بہترین طریقے لاگو کریں**

### مرکزی تصورات

#### 5.1 لانچ فائل کا فن تعمیر

```python
#!/usr/bin/env python3
"""
ہیومینوئڈ روبوٹ سسٹم لانچ فائل

یہ لانچ فائل ہیومینوئڈ روبوٹ کے آپریشن کے لیے درکار تمام کمپوننٹس شروع کرتا ہے
جیسے کہ سینسرز، کنٹرولرز، اور اسٹیٹ ایسٹیمیشن۔
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
    """ہیومینوئڈ روبوٹ سسٹم کے لیے لانچ تفصیل تیار کریں۔"""

    # لانچ آرگومینٹس کا اعلان کریں
    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='روبوٹ ماڈل کی شناخت کنندہ'
    )

    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='اصل ہارڈویئر کے بجائے سیمولیشن استعمال کریں'
    )

    control_mode = DeclareLaunchArgument(
        'control_mode',
        default_value='position',
        description='Joint کنٹرول موڈ'
    )

    # پیکیج شیئر ڈائریکٹریز
    pkg_share = FindPackageShare('humanoid_control')
    pkg_description = FindPackageShare('humanoid_description')

    # کانفیگریشن فائلز
    params_file = PathJoinSubstitution([
        pkg_share, 'config', 'control_params.yaml'
    ])

    # URDF/XACRO فائل
    urdf_file = PathJoinSubstitution([
        pkg_description, 'urdf', 'humanoid.urdf.xacro'
    ])

    # لانچ تفصیل بنائیں
    ld = LaunchDescription()

    # آرگومینٹس شامل کریں
    ld.add_action(robot_model)
    ld.add_action(use_sim)
    ld.add_action(control_mode)

    # روبوٹ اسٹیٹ پبلشر (URDF)
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

    # IMU پبلشر نوڈ
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

    # بیلنس کنٹرولر (صرف اصل ہارڈویئر پر)
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

    # سیمولیشن کنٹرولر (صرف سیمولیشن میں)
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

<!--
کوڈ کا مقصد: ہیومینوئڈ روبوٹ سسٹم کے لیے ایک مکمل لانچ فائل کی تخلیق
- متعدد لانچ آرگومینٹس (روبوٹ ماڈل، سیمولیشن موڈ، کنٹرول موڈ) کا اعلان کرنا
- سینسرز اور کنٹرولرز کے لیے نوڈس کی شروعات کرنا
- سیمولیشن اور ہارڈویئر کے درمیان مشروط سوئچنگ لاگو کرنا
- URDF/XACRO روبوٹ ماڈل کی لوڈنگ کا انتظام کرنا
-->

#### 5.2 پیرامیٹر مینجمنٹ

```yaml
# control_params.yaml
# ہیومینوئڈ روبوٹ کنٹرول پیرامیٹرز

/**:
  ros__parameters:
    # کنٹرول لوپ پیرامیٹرز
    control_rate: 200.0  # Hz
    control_mode: "position"

    # بیلنس کنٹرولر پیرامیٹرز
    balance_controller:
      kp: [100.0, 100.0, 50.0]
      kd: [10.0, 10.0, 5.0]
      ki: [1.0, 1.0, 0.5]
      com_height: 0.9  # میٹر
      foot_separation: 0.15  # میٹر
      max_tilt_angle: 0.1  # ریڈیئن

    # Joint حدود
    joint_limits:
      left_hip_yaw:
        min_position: -1.5
        max_position: 1.5
        max_velocity: 2.0
        max_effort: 100.0
      # ... مزید joints

    # سینسر کانفیگریشن
    sensors:
      imu:
        frame_id: "imu_link"
        publish_rate: 200.0
        accel_range: 16.0  # g
        gyro_range: 2000.0  # deg/s
```

<!--
کوڈ کا مقصد: ہیومینوئڈ روبوٹ کے لیے ایک مرکزی کانفیگریشن فائل
- کنٹرول لوپ کی رفتار اور موڈ کی تعریف کرنا
- PID گینز اور بیلنس کنٹرول پیرامیٹرز کی وضاحت کرنا
- ہر joint کی پوزیشن، رفتار اور ایفورٹ حدود بتانا
- IMU سینسر کی کانفیگریشن (فریم ID، رفتار، رینج) دینا
-->

```python
# نوڈ میں پیرامیٹرس تک رسائی
import rclpy
from rclpy.node import Node

class ParameterizedNode(Node):
    def __init__(self):
        super().__init__('parameterized_node')

        # پیرامیٹرس کا اعلان کریں
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('robot_name', 'humanoid')
        self.declare_parameter('joint_names', ['joint1', 'joint2'])

        # پیرامیٹر ویلیوز حاصل کریں
        self.control_rate = self.get_parameter('control_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.joint_names = self.get_parameter('joint_names').value

        self.get_logger().info(f"Control rate: {self.control_rate} Hz")
```

<!--
کوڈ کا مقصد: ROS 2 نوڈ میں پیرامیٹر ڈیکلریشن اور رسائی کا نمونہ
- پیرامیٹرس کا اعلان کرنا اور ڈیفالٹ ویلیوز دینا
- پیرامیٹر ویلیوز کو موثر طریقے سے حاصل کرنا
- لاگنگ کے ذریعے پیرامیٹر ویلیوز کی تصدیق کرنا
-->

### اندازہ شدہ وقت کا عہدہ

| سرگرمی | وقت |
|----------|------|
| پڑھنا (لانچ فائلز) | 3 گھنٹے |
| لانچ فائلز بنانا | 4 گھنٹے |
| پیرامیٹر مینجمنٹ مشقیں | 3 گھنٹے |
| مشروط لانچ پیٹرنز | 2 گھنٹے |
| لانچ کانفیگریشنز کی ٹیسٹنگ | 3 گھنٹے |
| **کل** | **15 گھنٹے** |

### مشقی تمارین

1. **مشق 5.1**: ایک لانچ فائل بنائیں جو joint state پبلشر، IMU پبلشر، اور بیلنس کنٹرولر شروع کرے۔

2. **مشق 5.2**: کنٹرول گینز اور joint حدود کے لیے قابل کانفیگریشن پیرامیٹرز شامل کریں۔

3. **مشق 5.3**: سیمولیشن اور ہارڈویئر موڈس کے درمیان سوئچ کرنے کے لیے ایک لانچ آرگومینٹ نافذ کریں۔

4. **مشق 5.4**: تمام روبوٹ پیرامیٹرس کے ساتھ ایک YAML کانفیگریشن فائل بنائیں۔

5. **مشق 5.5**: سیمولیشن میں مکمل سسٹم لانچ کی ٹیسٹ کریں (ضمیمہ B دیکھیں)۔

### بحث کے سوالات

- XML لانچ فائلز کی نسبت Python پر مبنی لانچ فائلز زیادہ لچکدار کیوں ہیں؟
- لانچ آرگومنٹس سسٹم کی کانفیگریبلٹی کو کیسے بہتر بناتے ہیں؟
- پیرامیٹر ورزننگ اور مائیگریشن کے لیے بہترین طریقے کیا ہیں؟
- سیمولیشن پیرامیٹرس ہارڈویئر پیرامیٹرس سے کیسے مختلف ہونے چاہیئے؟

---

## کوڈ مثالوں کا خلاصہ

### هفته 3: نوڈ اور کمیونیشن پیٹرنز

```python
# بنیادی نوڈ کی ساخت
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(MsgType, '~/topic', 10)
        self.subscription = self.create_subscription(MsgType, '~/input', self.callback, 10)

    def callback(self, msg):
        # پیغام پراسیس کریں
        pass
```

<!--
کوڈ کا مقصد: ایک بنیادی ROS 2 نوڈ کی مختصر ساخت
- پبلشر اور سبسکرائبر دونوں بنانا
- کال بیک فنکشن کے ذریعے پیغامات کی پراسیس کرنا
-->

### هفته 4: پیکیج انٹری پوائنٹس

```python
# setup.py انٹری پوائنٹ
entry_points={
    'console_scripts': [
        'my_node = my_package.nodes.my_node:main',
    ],
}
```

<!--
کوڈ کا مقصد: ROS 2 پیکیج میں کمانڈ لائن انٹری پوائنٹ کی تعریف
- کسٹم کمانڈ نام اور متعلقہ Python ماڈیول/فنکشن کی میپنگ
-->

### هفته 5: لانچ فائل کی ساخت

```python
from launch_ros.actions import Node

ld = LaunchDescription()
ld.add_action(Node(package='pkg', executable='node', parameters=['config.yaml']))
return ld
```

<!--
کوڈ کا مقصد: ROS 2 لانچ فائل کا ایک مختصر نمونہ
- ایک نوڈ کو لانچ ایکشن کے ذریعے شامل کرنا
- پیرامیٹر فائل کی پاسنگ
-->

---

## اضافی وسائل

### تجویز کردہ پڑھنا

- **ROS 2 دستاویزات**: https://docs.ros.org/en/humble/
- **ROS 2 ڈیزین مضامین**: https://design.ros2.org/
- **DDS تفصیلات**: https://www.omg.org/omg-dds-portal/
- "Programming Robots with ROS" - Morgan Quigley et al.

### آن لائن وسائل

- ROS 2 ٹیوٹوریل سیریز: https://docs.ros.org/en/humble/Tutorials.html
- GitHub ROS 2 مثالیں: https://github.com/ros2/examples
- ROS ڈسکورس: https://discourse.ros.org/

### ہارڈویئر حوالاجات

Hands-on ROS 2 ترقی کے لیے:

| پلیٹ فارم | مقصد | نوٹس |
|----------|---------|-------|
| NVIDIA Jetson Orin | ایج کمپیوٹنگ | قدرتی ROS 2 سپورٹ |
| Intel NUC | ترقی | سیمولیشن کے لیے اچھا |
| Raspberry Pi 5 | ہلکے وزن نوڈز | ROS 2 rolling درکار |
| AMD64/x86_64 | معیاری ترقی | مکمل ROS 2 مماثلت |

### سیمولیشن حوالہ

ہارڈویئر کے بغیر ROS 2 کوڈ کی ٹیسٹنگ کے لیے، **ضمیمہ B** دیکھیں جو درج ذیل کا احاطہ کرتا ہے:
- Gazebo سیمولیشن ماحول کی سیٹ اپ
- سیمولیشن پیکیجز کی انسٹالیشن
- نقل شدہ ہیومینوئڈ روبوٹ چلانا
- سینسر ڈیٹا اور کنٹرول آؤٹ پٹس کی تصدیق

---

## هفته 3-5 کی پیش رفت چیک لسٹ

تین ہفتوں کے ماڈیول کے ذریعے اپنی پیش رفت کو ٹریک کرنے کے لیے اس چیک لسٹ کا استعمال کریں:

### هفته 3: ROS 2 فن تعمیر

- [ ] ROS 2 پرت والا فن تعمیر سمجھیں
- [ ] DDS اور اس کے کردار کی وضاحت کریں
- [ ] مختلف کیسوں کے لیے QoS پروفائلز کانفیگر کریں
- [ ] rclpy کے ساتھ ROS 2 نوڈس نافذ کریں
- [ ] پبلشرز، سبسکرائبرز، اور سروسز بنائیں
- [ ] تمام مشقی تمارین مکمل کریں

### هفته 4: پیکیج ترقی

- [ ] اچھی ساخت والے ROS 2 پیکیجز بنائیں
- [ ] package.xml انحصارات کانفیگر کریں
- [ ] entry_points کے ساتھ setup.py لکھیں
- [ ] ماڈیولز، کانفگز، اور لانچ فائلز کی تنظیم کریں
- [ ] colcon کا استعمال کرتے ہوئے پیکیجز بنائیں
- [ ] تمام مشقی تمارین مکمل کریں

### هفته 5: لانچ فائلز اور پیرامیٹرز

- [ ] Python پر مبنی لانچ فائلز بنائیں
- [ ] لانچ آرگومینٹس کا اعلان اور استعمال کریں
- [ ] YAML فائلز کے ساتھ پیرامیٹرز کا انتظام کریں
- [ ] مشروط نوڈ لانچنگ نافذ کریں
- [ ] پیرامیٹر بہترین طریقے لاگو کریں
- [ ] تمام مشقی تمارین مکمل کریں

---

## اگلے حصے میں منتقلی

هفته 3-5 مکمل کرنے کے بعد، آپ **حصہ 2، باب 3: ایکچویٹرز اور موومنٹ سسٹمز** میں جانے کے لیے تیار ہوں گے جہاں آپ سیکھیں گے:

- موٹر کی اقسام اور کنٹرول اصول
- Joint ڈیزائن اور کائنیمیٹکس
- موومنٹ کنٹرول پروگرامنگ
- بیلنس اور لوکوموشن الگورتھم

اس ماڈیول سے ROS 2 کا علم آپ کو ہیومینوئڈ روبوٹ پلیٹ فارمز پر ان تصورات کو نافذ اور ٹیسٹ کرنے کے قابل بنائے گا۔

### فوری پیش منظر: حصہ 3 (سیمولیشن)

حصہ 3 **سیمولیشن اور ٹیسٹنگ** کا احاطہ کرتا ہے، جہاں آپ ROS 2 کے تصورات کو Gazebo سیمولیشن ماحول میں لاگو کریں گے۔ مضامین میں شامل ہیں:

- Gazebo سیمولیشن ماحول کی سیٹ اپ
- روبوٹ ماڈلز اور پلگ انز بنانا
- سینسر ڈیٹا اور ایکچویٹر رسپانس کی سیمولیشن
- کنٹرول الگورتھم کی محفوظ ٹیسٹنگ
- سیمولیشن اور حقیقت کا پل بنانا

### فوری پیش منظر: حصہ 4 (Isaac Sim)

حصہ 4 **NVIDIA Isaac Sim** کو اعلیٰ وفاداری والی سیمولیشن کے لیے متعارف کراتا ہے:

- GPU-تیز رفتار فزکس سیمولیشن
- حقیقت پسند سینسر سیمولیشن (کیمرے، LIDAR)
- رینفورسمنٹ لرننگ ورکفلوز
- سیم-ٹو-ریل ٹرانسفر حکمت عملی

---

**حصہ 2: ROS 2 بنیادی اصول** | [باب 2: ROS 2 بنیادی اصول](part-2-ros2/ros2-fundamentals) | [حصہ 3: سیمولیشن](part-3-simulation/gazebo-unity-simulation)
