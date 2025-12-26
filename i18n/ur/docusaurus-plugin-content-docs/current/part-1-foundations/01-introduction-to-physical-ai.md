---
title: "فزیکل اے آئی کا تعارف"
sidebar_position: 1
---

# Chapter 1: فزیکل اے آئی کا تعارف (Introduction to Physical AI)

## سیکھنے کے مقاصد (Learning Objectives)

اس باب کے آخر میں، آپ یہ کر سکیں گے:
- فزیکل اے آئی کی تعریف کریں اور اسے روایتی ڈیجیٹل اے آئی سے ممیز کریں
- جسمانی ذہانت (Embodied Intelligence) کے تصور کو سمجھیں اور یہ جانیں کہ جسمانی مظہر کیوں اہم ہے
- ہیومینوئڈ روبوٹکس کے میدان میں اہم کھلاڑیوں کی شناخت کریں
- فزیکل اے آئی ایپلیکیشنز کے لیے سینسر سسٹم کو کانفیگر کریں
- ROS 2 اور Python کا استعمال کرتے ہوئے بنیادی سینسر کانفیگریشن لاگو کریں

## 1.1 فزیکل اے آئی کیا ہے؟ (What is Physical AI?)

فزیکل اے آئی (Physical AI) مصنوعی ذہانت میں ایک پیراڈائم شفٹ ہے جو پوری طرح سے کمپیوٹیشنل سسٹمز سے ایجنٹوں کی طرف ہے جو براہ راست فزیکل دنیا کے ساتھ تعامل کرتے ہیں۔ روایتی AI جو ڈیجیٹل تجرید میں کام کرتا ہے، اس کے برعکس، فزیکل اے آئی روبوٹک پلیٹ فارمز کے ذریعے حقیقی ماحول میں پر셉شن، ریزننگ، اور ایکشن کو ملا تا ہے۔

### جسمانی ذہانت کی تعریف (Embodied Intelligence Defined)

جسمانی ذہانت یہ اصول ہے کہ ایک سسٹم اور اس کے فزیکل ماحول کے درمیان تعامل سے ذہانت پیدا ہوتی ہے۔ یہ خالص سافٹویئر AI کے "برین ان اے جار" ماڈل سے شدید مختلف ہے۔ ایک فزیکل اے آئی سسٹم:

- **پیerceives (محسوس کرتا ہے)**: سینسرز کے ذریعے دنیا کو (ویژن، ٹچ، پروپرائوسیپشن)
- **Reasons (سوچتا ہے)**: AI ماڈلز کا استعمال کرتے ہوئے حالت، اہداف، اور کارروائیوں کے بارے میں
- **Acts (کارروائی کرتا ہے)**: ایکچویٹرز کے ذریعے جو فزیکل ماحول میں ترمیم کرتے ہیں
- **Learns (سیکھتا ہے)**: فزیکل تعاملات اور نتائج سے

پر셉شن اور ایکشن کے درمیان یہ بند لوپ رشتہ ایسی صلاحیتیں پیدا کرتا ہے جو خالص ڈیجیٹل سسٹمز حاصل نہیں کر سکتے۔ ایک ہیومینوئڈ روبوٹ کو چیز تھامتے ہوئے توازن برقرار رکھنا ہوگا، یہ پیش گوئی کرنی ہوگی کہ اشیاء قوتوں کا کیسے جواب دیں گی، اور ریل ٹائم فیڈ بیک کی بنیاد پر اپنی حرکات کو ایڈجسٹ کرنا ہوگا۔

### فزیکل اے آئی کا سپیکٹرم (The Physical AI Spectrum)

فزیکل اے آئی سسٹمز کی ایک وسیع رینج پر محیط ہے، فکسڈ انڈسٹریل آرمز سے لے کر جنرل پرپز کے لیے ڈیزائن کیے گئے ہیومینوئڈ روبوٹس تک:

| System Type | Embodiment | Intelligence Level | Primary Domain |
|-------------|------------|-------------------|----------------|
| Industrial Robots | Fixed/gantry | Pre-programmed | Manufacturing (تیاری) |
| Mobile Robots | Wheeled/track | Navigation-focused | Logistics (لاجسٹکس) |
| Manipulation Arms | Articulated | Task-specific | Assembly, surgery |
| Humanoids | Bipedal anthropomorphic | General-purpose | Healthcare, domestic |

ہیومینوئڈ روبوٹس فزیکل اے آئی کے لیے سب سے مشکل پلیٹ فارم ہیں کیونکہ دوپائیدار حرکت کی پیچیدگی اور ہیومین-ڈیزائن ماحول میں چلاؤ کے توقعات کی وجہ سے۔

## 1.2 ڈیجیٹل اے آئی سے فزیکل اے آئی تک (From Digital AI to Physical AI)

ڈیجیٹل اے آئی سے فزیکل اے آئی میں منتقلی میں سسٹمز کی ڈیزائن، تربیت، اور تعیناتی میں بنیادی تبدیلیاں شامل ہیں۔

### اہم اختلافات (Key Differences)

| Aspect | Digital AI | Physical AI |
|--------|-----------|-------------|
| Environment | Bounded data space | Unstructured real world |
| Feedback Latency | Batch processing | Real-time closed loop |
| Failure Modes | Incorrect outputs | Physical damage, safety hazards |
| Generalization | Domain-specific | Must handle open-world scenarios |
| Computation | Cloud/ڈیٹا سینٹر | Edge processing required |

### جسمانی مظہر کیوں اہم ہے (Why Embodiment Matters)

"کپ" کیا ہے اسے سمجھنے کی چیلنج پر غور کریں۔ خالص ڈیجیٹل AI جو تصاویر پر تربیت یافتہ ہے، سیکھتا ہے کہ کپس ہینڈلز اور خاص شکلیں رکھتے ہیں۔ ایک جسمانی روبوٹ سیکھتا ہے کہ کپس کو کئی طریقوں سے پکڑا جا سکتا ہے، ان کا وزن ہے، وہ مائع رکھ سکتے ہیں، اور پانی گرنے سے بچنے کے لیے احتیاط سے ہینڈل کرنا ہوگا۔ فزیکل تعامل وہ بنیاد فراہم کرتا ہے جو صرف ویژنل تربیت اکیلے نہیں دے سکتی۔

Sim2Real گیپ — سیمولیشن اور حقیقت کے درمیان فرق — فزیکل اے آئی میں سب سے بڑی چیلنجوں میں سے ایک ہے۔ سیمولیشن میں تربیت یافتہ ماڈلز اکثر فزیکل روبوٹس پر تعینات ہونے پر ناکام ہو جاتے ہیں کیونکہ:

- سینسر شور اور کیلبریشن اختلافات
- ایکچویٹر حدود اور تاخیر
- غیر ماڈلڈ فزیکل مظاہر (فriction، compliance، wear)
- ماحولیاتی تغیرات (روشنی، سطحوں، اشیاء)

## 1.3 ہیومینوئڈ روبوٹکس کا منظر (Humanoid Robotics Landscape)

ہیومینوئڈ روبوٹکس کا میدان نہایت تیزی سے ترقی کر گیا ہے، کئی اہم کھلاڑیوں نے ممکن کی حدوں کو آگے بڑھایا ہے۔

### موجودہ اہم کھلاڑی (Current Major Players)

**Boston Dynamics** نے Atlas کے ساتھ ڈائنامک لوکوموشن میں خود کو لیڈر کے طور پر قائم کیا ہے، پارکور، بیک فلپس، اور خودمختار ٹہنے جیسی صلاحیتیں ظاہر کرتے ہوئے۔ ان کا ہائیڈرولک ایکچویشن سسٹم استثنائی پاور ڈینسٹی فراہم کرتا ہے، اگرچہ پیچیدگی اور دیکھ ریکھ کی قیمت پر۔ Atlas ماڈل پریڈکٹو کنٹرول اور لرننگ پر مبنی اپروچز کے ایک مجموعے کا استعمال کرتا ہے۔

**Tesla Optimus** ایک مختلف اپروچ کی نمائندگی کرتا ہے — مینوفیکچرنگ ایفیشنسی اور آخر کار صارفین کی قیمت پر۔ $20,000 سے کم کی تخمینی ہدف لاگت کے ساتھ، Optimus کا مقصد ہیومینوئڈ روبوٹس کو بڑے پیمانے پر تجارتی طور پر قابل بنانا ہے۔ Tesla اپنی AI اور مینوفیکچرنگ مہارت کا فائدہ اٹھاتے ہوئے ایک عمودی طور پر مربوط حل بناتا ہے۔

**Agility Robotics** Digit کے ساتھ عملی تعیناتی پر توجہ دیتا ہے، ایک روبوٹ جو لاجسٹک ٹاسکس کے لیے ڈیزائن کیا گیا ہے۔ ان کا اپروچ زیادہ سے زیادہ کارکردگی کے مقابلے میں پائیداری اور دیکھ ریکھ میں آسانی پر زور دیتا ہے۔ Digit کو حقیقی گودام ماحول میں تعینات کیا گیا ہے، قیمتی آپریشنل ڈیٹا اکٹھا کرتے ہوئے۔

**Figure AI** نے جنرل پرپز ہیومینوئڈ روبوٹس تیار کرنے کے لیے اہم فنڈنگ حاصل کی ہے۔ ان کا اپروچ تیز تکرار اور تازہ ترین AI پیشرفتوں کے انضمام پر زور دیتا ہے، ابتدائی تجارتی تعیناتی کے لیے BMW کے ساتھ شراکت داری کرتے ہوئے۔

### تکنیکی اپروچز کا موازنہ (Technical Approaches Comparison)

| Platform | Actuation | Control Philosophy | Development Focus |
|----------|-----------|-------------------|-------------------|
| Atlas (Boston Dynamics) | Hydraulic | Model-predictive + learning | Research, demonstrations |
| Optimus (Tesla) | Electric (custom) | End-to-end learning | Commercial manufacturing |
| Digit (Agility) | Electric series elastic | Reactive control | Logistics, durability |
| Figure 01 | Electric | Hybrid AI/control | General-purpose AI |

### مزید دیکھیں (See Also)

- **Part 2**: ROS 2 Fundamentals covers robot operating system concepts
- **Part 3**: Simulation covers Gazebo and Unity tools
- **Appendix A**: [Hardware Specifications](appendix/hardware-specifications) provides detailed actuator specs

## 1.4 سینسر سسٹمز (Sensor Systems)

سینسر سسٹمز فزیکل اے آئی کی پرسبپٹوئل بنیاد بناتے ہیں۔ درست سینسنگ کے بغیر، سب سے زیادہ پیچیدہ AI بھی اچھے فیصلے نہیں کر سکتا۔

### ویژوئل پرسبپشن سسٹمز (Visual Perception Systems)

کیمرے کم لاگت اور پاور کھپت پر نسبتاً ماحول کی معلومات فراہم کرتے ہیں۔ جدید ہیومینوئڈ روبوٹس عام طور پر استعمال کرتے ہیں:

- **RGB Cameras**: ویژوئل ریکگنیشن ٹاسکس کے لیے معیاری رنگ امیجنگ
- **Depth Cameras**: 3D معلومات کے لیے ایکٹو سٹیریو (Intel RealSense) یا سٹرکچرڈ لائٹ (Microsoft Kinect)
- **Event Cameras**: بائیو-انسبائرڈ سینسر جو مائیکرو سیکنڈ لیٹنسی کے ساتھ پکسل-لیول تبدیلیاں پتہ لگاتے ہیں

### Inertial Measurement Units (IMUs)

IMUs ایکسلرومیٹر اور جائروسکوپ کو ملا کر روبوٹ کی پوزیشن اور ایکسلریشن ماپتے ہیں۔ ہیومینوئڈ روبوٹس کے لیے، IMUs لوکوموشن کے دوران توازن کے تخمینے، پریشانیوں اور گرنے کا پتہ لگانے، اور بہتر حالت کے تخمینے کے لیے دیگر سینسرز کے ساتھ فیوژن کے لیے اہم ہیں۔

ہیومینوئڈ IMUs کی عام تفصیلات:
- ایکسلرومیٹر رینج: ±16g
- جائروسکوپ رینج: ±2000 deg/s
- سیمپل ریٹ: 100-1000 Hz
- شور کثافت: < 0.01 deg/s/√Hz

### Force/Torque Sensors

Force/torque sensors روبوٹ کے پاؤں اور ہاتھوں پر تعامل قوتوں کو ماپتے ہیں:

- **Foot sensors**: گراؤنڈ کانٹیکٹ قوتوں کا پتہ لگا کر توازن کنٹرول کو ممکن بناتے ہیں
- **Wrist sensors**: گراسپ قوتوں کا پتہ لگا کر درست ٹہنے کو ممکن بناتے ہیں
- **Tactile sensors**: نازک ٹاسکس کے لیے تفصیلی کانٹیکٹ معلومات فراہم کرتے ہیں

### LIDAR Systems

اگرچہ ہیومینوئڈ روبوٹس پر وزن کی پابندیوں کی وجہ سے کم عام، LIDAR درست فاصلہ پیمائش فراہم کرتا ہے جو مفید ہے:

- لمبی رینج میں رکاوٹ کا پتہ لگانے کے لیے
- ماحول کے درست 3D نقشے بنانے کے لیے
- GPS-رہتے ماحول میں نیویگیشن کے لیے

## 1.5 سینسر کانفیگریشن کی مثال (Sensor Configuration Example)

مندرجہ ذیل مثال دکھاتی ہے کہ ہیومینوئڈ روبوٹ پلیٹ فارم کے لیے ROS 2 اور Python کا استعمال کرتے ہوئے سینسر سسٹمز کو کانفیگر کیسے کریں۔

<!-- یہ کوڈ ہیومینوئڈ روبوٹ کے لیے سینسر کانفیگریشن ماڈیول ہے جو ROS 2 اور Python کا استعمال کرتا ہے۔ یہ سینسر کی ترتیب اور ابتدا فراہم کرتا ہے۔ -->
```python
#!/usr/bin/env python3
"""
Humanoid Robot Sensor Configuration Module

This module provides sensor configuration and initialization
for Physical AI applications using ROS 2 and Python.
"""

from dataclasses import dataclass
from typing import List, Dict, Optional
import numpy as np
from enum import Enum


class SensorType(Enum):
    """Enumeration of supported sensor types."""
    CAMERA_RGB = "camera_rgb"
    CAMERA_DEPTH = "camera_depth"
    IMU = "imu"
    FORCE_TORQUE = "force_torque"
    LIDAR = "lidar"
    TACTILE = "tactile"


@dataclass
class SensorConfig:
    """Configuration for a single sensor."""
    name: str
    sensor_type: SensorType
    topic: str
    frame_id: str
    sample_rate: float
    noise_std: float
    calibration_file: Optional[str] = None

    def validate(self) -> bool:
        """Validate sensor configuration."""
        if self.sample_rate <= 0:
            raise ValueError(f"Sample rate must be positive: {self.sample_rate}")
        if self.noise_std < 0:
            raise ValueError(f"Noise std must be non-negative: {self.noise_std}")
        return True


class SensorSuite:
    """
    Manages sensor configuration and initialization for humanoid robots.

    Attributes:
        sensors: Dictionary of configured sensors by name
        calibration_data: Loaded calibration parameters
        sample_rates: Current sample rates for all sensors
    """

    def __init__(self):
        self.sensors: Dict[str, SensorConfig] = {}
        self.calibration_data: Dict[str, np.ndarray] = {}
        self.sample_rates: Dict[str, float] = {}
        self._is_initialized = False

    def add_sensor(self, config: SensorConfig) -> bool:
        """
        Add a sensor to the configuration.

        Args:
            config: Sensor configuration parameters

        Returns:
            True if sensor was added successfully
        """
        config.validate()
        self.sensors[config.name] = config
        self.sample_rates[config.name] = config.sample_rate
        return True

    def configure_imu(self, name: str, frame_id: str,
                      sample_rate: float = 200.0) -> SensorConfig:
        """
        Configure an IMU sensor with standard parameters.

        Args:
            name: Unique sensor identifier
            frame_id: ROS frame for sensor data
            sample_rate: Target sampling rate in Hz

        Returns:
            Configured IMU sensor
        """
        config = SensorConfig(
            name=name,
            sensor_type=SensorType.IMU,
            topic=f"/{name}/imu",
            frame_id=frame_id,
            sample_rate=sample_rate,
            noise_std=0.01,
            calibration_file=f"/config/calibration/{name}_calibration.yaml"
        )
        self.add_sensor(config)
        return config

    def configure_force_torque(self, name: str, frame_id: str,
                                location: str = "wrist") -> SensorConfig:
        """
        Configure a force/torque sensor for manipulation or balance.

        Args:
            name: Unique sensor identifier
            frame_id: ROS frame for sensor data
            location: Sensor mounting location (wrist, foot)

        Returns:
            Configured force/torque sensor
        """
        # Sample rates vary by location - higher for manipulation
        sample_rates = {"wrist": 100.0, "foot": 50.0}
        rate = sample_rates.get(location, 100.0)

        config = SensorConfig(
            name=name,
            sensor_type=SensorType.FORCE_TORQUE,
            topic=f"/{name}/wrench",
            frame_id=frame_id,
            sample_rate=rate,
            noise_std=0.5,  # Newton-meters
            calibration_file=f"/config/calibration/{name}_calibration.yaml"
        )
        self.add_sensor(config)
        return config

    def get_sensor_topics(self) -> List[str]:
        """Get list of all sensor topics for ROS 2 configuration."""
        return [s.topic for s in self.sensors.values()]

    def validate_system(self) -> Dict[str, bool]:
        """
        Validate sensor system configuration.

        Returns:
            Dictionary mapping sensor names to validation status
        """
        results = {}
        for name, sensor in self.sensors.items():
            try:
                sensor.validate()
                results[name] = True
            except ValueError as e:
                results[name] = False
                print(f"Sensor {name} validation failed: {e}")
        return results

    def initialize(self) -> bool:
        """
        Initialize all sensors in the suite.

        Returns:
            True if all sensors initialized successfully
        """
        if not self.validate_system():
            raise RuntimeError("Sensor validation failed")

        # Load calibration data for each sensor
        for name, sensor in self.sensors.items():
            if sensor.calibration_file:
                self._load_calibration(name, sensor.calibration_file)

        self._is_initialized = True
        return self._is_initialized

    def _load_calibration(self, sensor_name: str,
                          calibration_file: str) -> None:
        """
        Load calibration parameters from file.

        In a real implementation, this would parse YAML/XML files
        and apply transformations to raw sensor data.
        """
        # Placeholder for calibration loading
        self.calibration_data[sensor_name] = np.eye(6)


def create_humanoid_sensor_suite() -> SensorSuite:
    """
    Create a complete sensor suite for a humanoid robot.

    Returns:
        Configured SensorSuite ready for initialization
    """
    suite = SensorSuite()

    # Head-mounted cameras
    suite.configure_imu("head_imu", "head_link", sample_rate=200.0)

    # Body IMU for balance estimation
    suite.configure_imu("torso_imu", "torso_link", sample_rate=200.0)

    # Force/torque sensors at wrists
    suite.configure_force_torque("left_wrist_ft", "left_wrist_link", "wrist")
    suite.configure_force_torque("right_wrist_ft", "right_wrist_link", "wrist")

    # Force/torque sensors at feet
    suite.configure_force_torque("left_foot_ft", "left_foot_link", "foot")
    suite.configure_force_torque("right_foot_ft", "right_foot_link", "foot")

    return suite


if __name__ == "__main__":
    # Example usage
    suite = create_humanoid_sensor_suite()

    print("Humanoid Sensor Suite Configuration:")
    print("=" * 50)
    for name, sensor in suite.sensors.items():
        print(f"\nSensor: {name}")
        print(f"  Type: {sensor.sensor_type.value}")
        print(f"  Topic: {sensor.topic}")
        print(f"  Frame: {sensor.frame_id}")
        print(f"  Sample Rate: {sensor.sample_rate} Hz")

    print("\n" + "=" * 50)
    print("Sensor Topics for ROS 2:")
    for topic in suite.get_sensor_topics():
        print(f"  {topic}")
```
<!-- کوڈ کا اختتام - یہ ماڈل ہیومینوئڈ روبوٹ کے لیے ایک مکمل سینسر سٹیٹ بناتا ہے جس میں IMU، فورس/ٹارک سینسر شامل ہیں۔ -->

### YAML کانفیگریشن فائل (YAML Configuration File)

```yaml
# /config/sensors/humanoid_sensors.yaml
# Humanoid Robot Sensor Configuration

sensors:
  head_imu:
    type: imu
    frame_id: head_link
    topic: /sensors/head/imu
    sample_rate: 200.0
    accelerometer:
      range: 16.0  # g
      noise_density: 0.0002  # g/√Hz
    gyroscope:
      range: 2000.0  # deg/s
      noise_density: 0.01  # deg/s/√Hz

  torso_imu:
    type: imu
    frame_id: torso_link
    topic: /sensors/torso/imu
    sample_rate: 200.0
    accelerometer:
      range: 16.0
      noise_density: 0.0002
    gyroscope:
      range: 2000.0
      noise_density: 0.01

  left_wrist_ft:
    type: force_torque
    frame_id: left_wrist_link
    topic: /sensors/left_wrist/wrench
    sample_rate: 100.0
    ranges:
      fx: 100.0  # N
      fy: 100.0
      fz: 200.0
      tx: 10.0   # Nm
      ty: 10.0
      tz: 10.0

# Sensor fusion configuration
sensor_fusion:
  imu_samples: 10
  gravity_vector: [0.0, 0.0, 9.81]
  fusion_gain: 0.01
```

### ہارڈویئر کی ضروریات کا حوالہ (Hardware Requirements Reference)

سینسر کانفیگریشن کے لیے، مندرجہ ذیل کم از کم ہارڈویئر تفصیلات تجویز کی جاتی ہیں (مکمل تفصیلات کے لیے Appendix A دیکھیں):

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4-core @ 2.0 GHz | 8-core @ 3.0 GHz |
| RAM | 8 GB | 16 GB |
| GPU | None (CPU processing) | NVIDIA Jetson or equivalent |
| Storage | 32 GB SSD | 128 GB NVMe SSD |
| Power | 50W | 100W (with GPU) |

## باب کا خلاصہ (Chapter Summary)

فزیکل اے آئی (Physical AI) مصنوعی ذہانت کا روبوٹک مظہر کے ساتھ انٹیگریشن ہے، جو سسٹمز کو فزیکل دنیا میں محسوس کرنے، سوچنے، اور کارروائی کرنے کے قابل بناتا ہے۔ ڈیجیٹل اے آئی سے فزیکل اے آئی میں منتقلی میں منفرد چیلنج شامل ہیں جیسے ریل ٹائم پروسیسنگ کی ضروریات، حفاظتی تحفظات، اور مضبوط پرسبپشن سسٹمز کی ضرورت۔

ہیومینوئڈ روبوٹس کو ان کی پیچیدہ حرکت کی ضروریات اور ہیومین-ڈیزائن ماحول میں چلاؤ کی توقعات کی وجہ سے خاص چیلنجوں کا سامنا کرنا پڑتا ہے۔ اس کھیلے کے اہم کھلاڑی مختلف تکنیکی اپروچز پر کام کر رہے ہیں، ہائیڈرولک ایکچویشن کے ساتھ ماڈل پریڈکٹو کنٹرول سے لے کر الیکٹرک ایکچویشن کے ساتھ لرننگ پر مبنی اپروچز تک۔

سینسر سسٹمز فزیکل اے آئی کی بنیاد بناتے ہیں، جو فیصلہ سازی کے لیے درکار پرسبپٹوئل ڈیٹا فراہم کرتے ہیں۔ ایک جامع سینسر سٹیٹ میں ویژوئل سسٹمز، inertial measurement units، force/torque سینسرز، اور ممکنہ طور پر لمبی رینج پرسبپشن کے لیے LIDAR شامل ہے۔

### اہم تصورات (Key Concepts)

1. **Physical AI**: AI systems that interact directly with the physical world through robotic embodiment
2. **Embodied Intelligence**: Intelligence emerging from the interaction between a system and its physical environment
3. **Sim2Real Gap**: The challenge of transferring models trained in simulation to real-world deployment
4. **Sensor Fusion**: Combining data from multiple sensor types for improved state estimation

### اہم اصطلاحات (Key Terminology)

- **Embodiment**: ایک AI سسٹم کا روبوٹک پلیٹ فارم کے ذریعے فزیکل مثال
- **Proprioception**: جسم کے حصوں کی نسبتی پوزیشن اور ان کی حرکت کا احساس
- **Actuation**: وہ میکانزم جس کے ذریعے روبوٹ کے جوڑیں حرکت کرتے ہیں
- **Sensor Fusion**: متعدد سینسرز سے ڈیٹا کو ملا کر عمل

### مزید پڑھیں (Further Reading)

- **Springer Handbook of Robotics** - Siciliano & Khatib (Eds.)
- **Physical Intelligence**: The next frontier in embodied AI research
- ROS 2 Documentation: Robot Operating System 2 fundamentals

### اگلا باب (Next Chapter)

باب 2 گہرائی میں **سینسر سسٹمز اور پرسبپشن** کی جانچ کرتا ہے، ملٹی-موڈل سینسنگ آرکیٹیکچر، کمپیوٹر ویژن تکنیکوں، اور سینسر فیوژن الگورتھم کی جانچ کرتا ہے جو ماحول کی سمجھ کو ممکن بناتے ہیں۔

---

**Part 1: Foundations** | [Week 1-2 Overview](part-1-foundations/01a-week-1-2-overview) | [Part 2: ROS 2 Fundamentals](part-2-ros2/ros2-fundamentals)
