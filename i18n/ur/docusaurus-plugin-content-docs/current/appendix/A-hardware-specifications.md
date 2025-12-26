---
id: hardware-specifications
title: "Appendix A: Hardware Specifications"
sidebar_position: 70
description: "Physical AI اور humanoid robotics کے لیے جامع ہارڈویئر کی تفصیلات، بشمول ترقیاتی ورک اسٹیشنز، ایج کمپیوٹنگ پلیٹ فارمز، روبوٹ پلیٹ فارمز، اور سینسر کے اختیارات۔"
---

# Appendix A: Hardware Specifications (ہارڈویئر کی تفصیلات)

یہ ایپینڈکس physical AI systems کی ترقی اور تعیناتی کے لیے تفصیلی ہارڈویئر کی تفصیلات فراہم کرتا ہے، جس کا خاص focus humanoid robotics پر ہے۔ یہ تفصیلات تین ہارڈویئر tiers میں منظم ہیں، جو آپ کو اپنے بجٹ، سیکھنے کے اہداف، اور پراجیکٹ کی ضروریات کے مطابق مناسب تشکیل کا انتخاب کرنے کی اجازت دیتی ہیں۔

## A.1 Development Workstation (RTX GPU)

Development workstation ترقی، سیمولیشن، اور سافٹویئر ڈیولپمنٹ کے لیے بنیادی ماحول ہے۔ یہ tier NVIDIA RTX GPUs والے systems پر focus کرتا ہے جو CUDA-accelerated workloads چلانے کے قابل ہیں، reinforcement learning، computer vision، اور neural network training کے لیے۔

### A.1.1 Minimum Specifications (RTX 4070 Ti)

یہ کم سے کم تشکیل سیکھنے اور چھوٹے پیمانے کی ترقیاتی پراجیکٹس کے لیے کافی کارکردگی فراہم کرتا ہے۔ یہ setup بنیادی reinforcement learning training، inference testing، اور simulation workloads سنبھال سکتا ہے۔

| Component | Specification | Notes |
|-----------|---------------|-------|
| **CPU** | AMD Ryzen 7 7800X3D or Intel Core i7-13700K | 8+ cores parallel compilation کے لیے سفارش کیا جاتا ہے |
| **GPU** | NVIDIA RTX 4070 Ti (12GB VRAM) | CUDA compute capability 8.9+ |
| **RAM** | 32GB DDR5-5600 | بڑے ماڈلز کے لیے 64GB سفارش کیا جاتا ہے |
| **Storage** | 1TB NVMe SSD (PCIe 4.0) | OS اور پراجیکٹس کے لیے بنیادی ڈرائیو |
| **Secondary Storage** | 2TB NVMe SSD | Dataset storage اور model checkpoints |
| **Motherboard** | ATX with PCIe 4.0 x16 slot | کافی power delivery یقینی بنائیں |
| **Power Supply** | 850W 80+ Gold | کیبل مینجمنٹ کے لیے modular سفارش کیا جاتا ہے |
| **OS** | Ubuntu 22.04 LTS | بنیادی development environment |

### A.1.2 Recommended Specifications (RTX 4090)

یہ سفارشی تشکیل بڑے ماڈلز کی training، متعدد simultaneous experiments چلانے، اور high fidelity والے پیچیدہ simulation environments سنبھالنے کے لیے جگہ فراہم کرتا ہے۔

| Component | Specification | Notes |
|-----------|---------------|-------|
| **CPU** | AMD Ryzen 9 7950X or Intel Core i9-13900K | 16+ cores parallel workloads کے لیے |
| **GPU** | NVIDIA RTX 4090 (24GB VRAM) | 13B تک کے ماڈلز کو سنبھالنے کے قابل |
| **RAM** | 64GB DDR5-5600 (بڑے ماڈلز کے لیے 128GB) | قابل信赖 کے لیے ECC سفارش کیا جاتا ہے |
| **Primary Storage** | 2TB NVMe SSD (PCIe 4.0 or 5.0) | بڑے datasets کے لیے تیز لوڈ ٹائمز |
| **Secondary Storage** | 4TB NVMe SSD | طویل المدتی ماڈل اور dataset archive |
| **Power Supply** | 1200W 80+ Platinum | RTX 4090 کی high power draw |
| **Cooling** | 360mm AIO liquid cooler | مستقل بھاری loads کے لیے ضروری |
| **OS** | Ubuntu 22.04 LTS or Ubuntu 24.04 LTS | استحکام کے لیے LTS versions |

:::tip GPU VRAM Considerations
جب robotics کے لیے diffusion models یا بڑے language models train کر رہے ہوں، تو VRAM اکثر limiting factor ہوتا ہے۔ RTX 4090 کا 24GB VRAM RTX 4070 Ti سے تقریباً 2x بڑے batch sizes کو accommodate کر سکتا ہے، جو compute-intensive policies کے لیے training time کو خاص طور پر کم کرتا ہے۔
:::

### A.1.3 Software Environment Setup

مندرجہ ذیل تشکیلی فائلیں Ubuntu 22.04 LTS پر physical AI development کے لیے قابل再現 development environment قائم کرتی ہیں۔

```bash
#!/bin/bash
# Development Environment Setup Script
# Run as: sudo bash setup-dev-env.sh

# یہ سکرپٹ Ubuntu پر physical AI ڈیولپمنٹ کے لیے ماحول تیار کرتا ہے
# اس میں NVIDIA ڈرائیور، CUDA، Docker، اور ROS 2 شامل ہیں

# Update system
apt update && apt upgrade -y

# Install NVIDIA driver (if not pre-installed)
ubuntu-drivers autoinstall
nvidia-smi  # Verify installation

# Install CUDA Toolkit 12.3
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt update
apt install -y cuda-toolkit-12-3

# Configure environment variables
cat >> /etc/environment << 'EOF'
CUDA_HOME=/usr/local/cuda
LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
PATH=$CUDA_HOME/bin:$PATH
EOF

# Install Docker with NVIDIA container toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sed 's#deb https#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https#g' > /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt update
apt install -y nvidia-container-toolkit
systemctl restart docker

# Install ROS 2 Humble
locale  # Check locale settings
apt install -y locales
locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
apt install -y software-properties-common
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release -echo $VERSION_CODENAME) main" > /etc/apt/sources.list.d/ros2.list
apt update
apt install -y ros-humble-desktop-full

# Initialize rosdep
rosdep init
rosdep update
```

## A.2 Edge Computing (Jetson Orin)

Edge computing platforms trained models کو براہ راست robotic hardware پر deploy کرنے کے قابل بناتے ہیں، جو network latency کے بغیر real-time inference capabilities فراہم کرتے ہیں۔ NVIDIA کا Jetson Orin series mobile robotics applications کے لیے compute performance اور power efficiency کا ایک بہترین توازن پیش کرتا ہے۔

### A.2.1 Jetson Orin Nano Super Specifications

Jetson Orin Nano Super robotics میں edge AI development کے لیے optimal entry point ہے، جو battery-powered robots کے لیے موزو power-efficient form factor میں substantial neural network inference capabilities پیش کرتا ہے۔

| Specification | Details |
|---------------|---------|
| **AI Performance** | 67 TOPS (INT8) / 34 TFLOPS (FP16) |
| **GPU** | 1024-core NVIDIA Ampere architecture |
| **CPU** | 6-core ARM Cortex-A78AE (2.0GHz) |
| **Memory** | 8GB LPDDR5 (102 GB/s) |
| **Storage** | MicroSD card slot (UFS support) |
| **Power** | 7W - 25W configurable |
| **Connectivity** | Gigabit Ethernet, M.2 Key E (Wi-Fi), M.2 Key M (NVMe) |
| **Camera Interfaces** | 2x MIPI CSI-2 (15-pin) |
| **GPIO** | 40-pin expansion header |
| **Dimensions** | 69.6mm x 45mm (Pico-ITX form factor) |
| **OS** | Ubuntu 20.04 (JetPack 6.0) / Ubuntu 22.04 (JetPack 6.1) |

:::note JetPack Compatibility
JetPack 6.1 Ubuntu 22.04 کے ساتھ سرکاری support اور بہتر kernel upstreaming پیش کرتا ہے۔ ROS 2 Humble packages کی تازہ ترین versions کی ضرورت والے پراجیکٹس کے لیے، JetPack 6.1 کو JetPack 6.0 پر ترجیح دی جائے۔
:::

### A.2.2 Jetson Orin NX Specifications

Orin NX more demanding inference workloads کے لیے ایک اہم performance increase فراہم کرتا ہے، جو متعدد neural networks کو simultaneously چلانے یا higher-resolution sensor inputs کو سنبھالنے کے لیے موزو ہے۔

| Specification | Details |
|---------------|---------|
| **AI Performance** | 100 TOPS (INT8) / 50 TFLOPS (FP16) |
| **GPU** | 1536-core NVIDIA Ampere architecture |
| **CPU** | 8-core ARM Cortex-A78AE (2.0GHz) |
| **Memory** | 16GB LPDDR5 (102 GB/s) |
| **Power** | 10W - 60W configurable |
| **Camera Interfaces** | Up to 6x MIPI CSI-2 lanes |
| **Display** | HDMI 2.1, DP 1.2 (+ via Type-C) |
| **Storage** | M.2 Key M (NVMe) |
| **Form Factor** | Module (100mm x 87mm reference carrier) |

### A.2.3 Jetson AGX Orin Specifications

AGX Orin سبسے demanding robotic applications کے لیے workstation-class performance فراہم کرتا ہے، بشمول full-body humanoid control اور multi-modal perception systems جو simultaneously چلتے ہیں۔

| Specification | Details |
|---------------|---------|
| **AI Performance** | 275 TOPS (INT8) / 140 TFLOPS (FP16) |
| **GPU** | 2048-core NVIDIA Ampere architecture |
| **CPU** | 12-core ARM Cortex-A78AE (2.2GHz) with 4MB L2 + 6MB L3 |
| **Memory** | 64GB LPDDR5 (204 GB/s) |
| **Storage** | 64GB eMMC 5.1 + M.2 Key M (NVMe) |
| **Power** | 15W - 60W (default mode) / up to 60W (MAX power mode) |
| **Camera Interfaces** | Up to 16x MIPI CSI-2 lanes (12 Gbps per lane) |
| **Connectivity** | 10GbE, USB 3.2, CAN-FD, SPI, I2C, UART |
| **OS** | Ubuntu 20.04 / 22.04 with JetPack |
| **Form Factor** | Developer Kit (110mm x 110mm) |

### A.2.4 Jetson Family Comparison

| Feature | Orin Nano Super | Orin NX 16GB | AGX Orin 64GB |
|---------|-----------------|--------------|---------------|
| **AI Performance** | 67 TOPS | 100 TOPS | 275 TOPS |
| **GPU Cores** | 1024 | 1536 | 2048 |
| **CPU Cores** | 6 | 8 | 12 |
| **RAM** | 8GB | 16GB | 64GB |
| **Power Range** | 7-25W | 10-60W | 15-60W |
| **Price (USD)** | ~$249 | ~$399 | ~$999 |
| **Best For** | Learning, simple tasks | Intermediate projects | Production deployment |

:::warning Power Delivery for AGX Orin
AGX Orin developer kit کو 19V power supply کی ضرورت ہے جو کم از کم 60W deliver کرنے کے قابل ہو۔ جب custom robot designs میں integrate کر رہے ہوں، تو یقینی بنائیں کہ آپ کا power delivery board stable 19V required current پر provide کر سکتا ہے۔ Power instability silent model failures اور data corruption کا سبب بن سکتا ہے۔
:::

### A.2.5 Jetson Configuration for Robotics

مندرجہ ذیل ROS 2 parameter file robotic applications کے ساتھ optimal performance کے لیے Jetson-specific hardware parameters تشکیل کرتا ہے۔

```yaml
# config/jetson_params.yaml
# ROS 2 parameter file for Jetson Orin hardware configuration
# یہ فائل Jetson Orin ہارڈویئر کی تشکیل کرتی ہے جو robotics applications کے لیے optimize ہے

/**:
  ros__parameters:
    # Hardware Performance Settings
    performance:
      power_mode: "maxn"  # maxn, 15w, 30w, default
      gpu_clock: "max"    # max, nominal
      cpu_governor: "performance"

    # Sensor Configuration
    sensors:
      camera:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        codec: "raw"

      lidar:
        enabled: true
        topic: "/lidar/points"
        frame_id: "lidar_link"

      imu:
        enabled: true
        topic: "/imu/data"
        frame_id: "imu_link"

    # Model Inference Settings
    inference:
      device: "cuda"
      num_threads: 4
      enable_profiling: false
      cache_dir: "/var/lib/robot/models"

    # Networking
    network:
      ros_domain_id: 0
      multicast_interface: "l4tbr0"

    # Logging
    logging:
      level: "info"
      directory: "/var/log/robot"
```

## A.3 Robot Platforms

مباحثہ: Appropriate robot platform کا انتخاب physical AI اور humanoid robotics سیکھنے کے لیے crucial ہے۔ یہ section educational quadrupeds سے لے کر full-size humanoids تک سفارش کردہ platforms کا احاطہ کرتا ہے۔

### A.3.1 Unitree H1 Humanoid Specifications

Unitree H1 research اور development کے لیے available سبسے capable اور accessible full-size humanoid robots میں سے ایک ہے۔ اس کا humanoid form factor human-centric environments کے لیے design کیے گئے policies کی testing کو ممکن بناتا ہے۔

| Specification | Details |
|---------------|---------|
| **Height** | ~180 cm (71 inches) |
| **Weight** | ~47 kg (104 lbs) |
| **Degrees of Freedom** | 17 (7 per leg, 2 per arm, 1 waist, 1 head) |
| **Leg DOF** | 7 per leg (hip x3, knee x1, ankle x3) |
| **Arm DOF** | 2 per arm (shoulder x1, elbow x1) |
| **Actuator Type** | Unitree self-developed high-torque motors |
| **Max Walking Speed** | ~1.5 m/s |
| **Battery Capacity** | 15,000 mAh (720Wh) |
| **Runtime** | ~2-4 hours (depending on activity) |
| **Sensors** | Depth camera, LiDAR (optional), IMU, force/torque sensors |
| **Compute** | Jetson AGX Orin compatible (sold separately) |
| **Communication** | Ethernet, Wi-Fi, CAN bus |
| **OS** | Ubuntu, ROS 2 supported |

:::tip Learning Path with H1
جو لوگ humanoid robotics میں نئے ہیں، ان کے لیے custom policies train کرنے کی کوشش سے پہلے Unitree کے MotionDB سے pre-trained locomotion policies سے شروع کرنا چاہیے۔ Robot کی built-in safety features initial experimentation کے دوران falls سے protection فراہم کرتی ہیں۔
:::

### A.3.2 Unitree Go2 Quadruped Specifications

Go2 quadruped legged robotics سیکھنے کے لیے ایک excellent entry point ہے، جس سے پہلے humanoids کی طرف بڑھا جائے۔ اس کا چار leg design dynamic locomotion میں meaningful challenges پیش کرتے ہوئے experimentation کے لیے stability فراہم کرتا ہے۔

| Specification | Details |
|---------------|---------|
| **Height** | ~40 cm (16 inches) |
| **Weight** | ~12 kg (26 lbs) |
| **Degrees of Freedom** | 12 (3 per leg) |
| **Actuator Type** | Unitree Unitree Go2 motors |
| **Max Running Speed** | ~3.5 m/s (12.6 km/h) |
| **Battery Capacity** | 15,000 mAh |
| **Runtime** | ~1-2 hours |
| **Sensors** | Depth camera, LiDAR (pro version), IMU, ultrasonic sensors |
| **Compute** | Jetson Orin compatible (optional expansion) |
| **Communication** | Wi-Fi, UART, CAN |
| **OS** | Ubuntu, ROS 2 supported |

### A.3.3 Comparison of Robot Platforms

| Feature | Unitree Go2 | Unitree H1 | Boston Dynamics Atlas |
|---------|-------------|------------|----------------------|
| **Type** | Quadruped | Bipedal Humanoid | Bipedal Humanoid |
| **DOF** | 12 | 17 | 28+ |
| **Height** | 40 cm | 180 cm | 150 cm |
| **Weight** | 12 kg | 47 kg | 89 kg |
| **Payload** | 3 kg | 3 kg (per hand) | Unknown |
| **Price (USD)** | ~$2,500 | ~$90,000+ | Not sold |
| **ROS 2 Support** | Full | Full | Limited |
| **Open SDK** | Yes | Yes | No |
| **Best For** | Learning | Research | Advanced research |

:::warning Safety Considerations for Humanoids
H1 جیسے bipedal humanoids کے ساتھ کام کرنے کے لیے safety پر careful attention درکار ہے۔ ہمیشہ padded environment میں operate کریں، fall zone کو hard objects سے صاف رکھیں، اور ہر سیشن سے پہلے emergency stop procedures کو test کریں۔ Falls robot اور surrounding equipment دونوں کو نقصان پہنچا سکتی ہیں۔
:::

### A.3.4 Alternative Robot Platforms

مختلف budget constraints یا research requirements والے organizations کے لیے، کچھ alternative platforms worth consideration ہیں:

| Platform | Type | Key Strengths | Considerations |
|----------|------|---------------|----------------|
| **Boston Dynamics Spot** | Quadruped | Mature platform, excellent mobility | Closed ecosystem |
| **Agility Robotics Digit** | Humanoid | Designed for real-world deployment | Limited availability |
| **Robotis OP3** | Humanoid (miniature) | Educational, affordable | Limited payload |
| **RoboTitan H1** | Humanoid | Lower cost alternative | Less documentation |
| **Shadow Hand** | Manipulator | Excellent dextrous manipulation | No mobility |

## A.4 Sensor Options

مباحثہ: Perception physical AI systems کے لیے fundamental ہے۔ یہ section robots کو اپنا environment perceive کرنے کے قابل بنانے کے لیے sensor options کی تفصیل دیتا ہے، بنیادی RGB-D cameras سے لے کر high-resolution LIDAR systems تک۔

### A.4.1 LIDAR Options

Light Detection and Ranging (LIDAR) sensors navigation، obstacle avoidance، اور 3D environment reconstruction کے لیے essential precise distance measurements فراہم کرتے ہیں۔

#### RPLIDAR Series

| Model | Channels | Range | Accuracy | Refresh Rate | Price (USD) |
|-------|----------|-------|----------|--------------|-------------|
| A1 | Single | 12m | &lt;1% | 10 Hz | ~$100 |
| A2 | Single | 18m | &lt;1% | 10-15 Hz | ~$200 |
| A3 | Single | 25m | &lt;2% | 20 Hz | ~$400 |
| S1 | Multi-channel | 40m | &lt;2% | 20 Hz | ~$800 |

RPLIDAR series from SLAMTEC educational applications کے لیے excellent value پیش کرتا ہے۔ A1 اور A2 models indoor environments اور Go2 جیسے چھوٹے robots کے لیے خاص طور پر موزو ہیں۔

#### Ouster ES Series

| Model | Channels | Range | Resolution | FOV | Price (USD) |
|-------|----------|-------|------------|-----|-------------|
| ES-2 | 32 | 100m | 1024x128 | 90° x 360° | ~$12,000 |
| OS0 | 128 | 100m | 2048x256 | 90° x 360° | ~$32,000 |
| OS1 | 64 | 120m | 1024x64 | 360° x 360° | ~$16,000 |
| OS2 | 128 | 200m | 2048x128 | 360° x 360° | ~$45,000 |

Ouster sensors advanced perception tasks کے لیے high-resolution point clouds فراہم کرتے ہیں۔ OS1-64 اکثر robotics applications کے لیے resolution اور range کا excellent balance پیش کرتا ہے۔

#### Velodyne Puck Series

| Model | Channels | Range | Resolution | Price (USD) |
|-------|----------|-------|------------|-------------|
| VLP-16 | 16 | 100m | 300,000 pts/sec | ~$4,000 |
| VLS-128 | 128 | 200m | 2.4M pts/sec | ~$75,000 |

VLP-16 (Puck) research applications کے لیے ایک مقبول choice ہے کیونکہ اس کا cost اور performance میں balance ہے۔

:::tip LIDAR Selection Guidelines
indoor mobile robots کے لیے، RPLIDAR A2 جیسے single-channel LIDARs بھی کافی capability فراہم کرتے ہیں اور cost کا ایک fraction پر۔ outdoor navigation یا high-speed vehicles کے لیے، longer range والے multi-channel sensors essential بن جاتے ہیں۔ refresh rate کا انتخاب کرتے وقت robot کی maximum speed کو consider کریں۔
:::

### A.4.2 Camera Options

RGB-D cameras color imagery کو depth information کے ساتھ combine کرتے ہیں، جو simultaneous object detection، segmentation، اور 3D reconstruction کو ممکن بناتا ہے۔

#### Intel RealSense Series

| Model | Depth Tech | RGB Resolution | Depth Range | Price (USD) |
|-------|------------|----------------|-------------|-------------|
| D405 | Stereo | 2MP | 0.2-3m | ~$149 |
| D455 | Stereo | 2MP | 0.4-6m | ~$449 |
| D461 | Stereo | 2MP | 0.2-10m | ~$299 |
| L515 | LiDAR | 2MP | 0.2-9m | ~$349 |
| D435i | Stereo | 2MP | 0.2-10m | ~$199 |

RealSense D455 humanoid applications کے لیے best balance فراہم کرتا ہے، full-body perception کے لیے sufficient range اور visual-inertial odometry کے لیے integrated IMU کے ساتھ۔

#### Orbbec Series

| Model | Technology | RGB Resolution | Depth Range | Price (USD) |
|-------|------------|----------------|-------------|-------------|
| Astra | Structured Light | 1MP | 0.4-8m | ~$100 |
| Femto | Structured Light | 1MP | 0.2-1.5m | ~$200 |
| Gemini 2 | Stereo | 2MP | 0.2-10m | ~$400 |

Orbbec cameras RealSense کے لیے cost-effective alternatives فراہم کرتی ہیں، خاص طور پر educational deployments جہاں multiple cameras کی ضرورت ہوتی ہے۔

#### Allied Vision GigE Cameras

high-end perception systems کے لیے جو precise color reproduction اور high resolution require کرتے ہیں، industrial GigE cameras superior image quality اور flexibility پیش کرتے ہیں۔

| Model | Resolution | Frame Rate | Interface | Price (USD) |
|-------|------------|------------|-----------|-------------|
| Alvium 1800 C-2050 | 5MP | 60 fps | GigE | ~$500 |
| Alvium 1800 C-5000 | 20MP | 15 fps | GigE | ~$2,000 |

### A.4.3 IMU Options

Inertial Measurement Units accelerometers اور gyroscopes combine کر کے motion اور orientation data فراہم کرتے ہیں جو state estimation اور sensor fusion کے لیے essential ہیں۔

| Model | Accelerometer | Gyroscope | Drift Rate | Interface | Price (USD) |
|-------|---------------|-----------|------------|-----------|-------------|
| BMI270 | 16-bit | 16-bit | 2°/s | SPI, I2C | ~$15 |
| ICM20948 | 16-bit | 16-bit | 5°/s | SPI, I2C | ~$10 |
| VN-100 | 16-bit | 16-bit | 0.5°/s | UART, SPI | ~$400 |
| VectorNav VN-100 | 16-bit | 16-bit | 0.2°/s | UART, SPI, USB | ~$800 |
| Analog Devices ADIS16470 | 16-bit | 16-bit | 0.1°/s | SPI | ~$500 |

Jetson platform کے ساتھ integration کے لیے، BMI270 اور ICM20948 pre-built ROS 2 driver support کے ساتھ excellent value فراہم کرتے ہیں۔ جب high accuracy required ہو تو VN-100 navigation tasks کے لیے recommended ہے۔

### A.4.4 Sensor Integration Configuration

مندرجہ ذیل YAML configuration file humanoid robot پر complete perception system کے لیے sensor integration demonstrate کرتا ہے۔

```yaml
# config/sensors.yaml
# Complete sensor configuration for humanoid robot perception
# یہ فائل humanoid robot کے لیے مکمل sensor configuration فراہم کرتی ہے

perception:
  lidar:
    driver: "ouster_os1"
    params:
      sensor_hostname: "os1-xxxxxxxx.local"
      udp_dest: "192.168.1.100"
      lidar_port: 7502
      imu_port: 7503
      lidar_mode: "1024x10"  # 1024 horizontal, 10 Hz
      udp_profile: "legacy"
    frame_id: "lidar_link"
    topic: "/perception/lidar/points"

  cameras:
    left_hand:
      driver: "realsense_d455"
      params:
        serial_number: "xxxxxxxx"
        depth_width: 848
        depth_height: 480
        depth_fps: 30
        color_width: 1280
        color_height: 720
        color_fps: 30
      frame_id: "left_hand_camera_link"
      topic: "/perception/left_hand/color"

    right_hand:
      driver: "realsense_d455"
      params:
        serial_number: "xxxxxxxx"
        depth_width: 848
        depth_height: 480
        depth_fps: 30
        color_width: 1280
        color_height: 720
        color_fps: 30
      frame_id: "right_hand_camera_link"
      topic: "/perception/right_hand/color"

    head:
      driver: "realsense_d455"
      params:
        serial_number: "xxxxxxxx"
        depth_width: 1280
        depth_height: 720
        depth_fps: 15
        color_width: 1920
        color_height: 1080
        color_fps: 30
      frame_id: "head_camera_link"
      topic: "/perception/head/color"

  imu:
    main:
      driver: "vn_100"
      params:
        port: "/dev/ttyUSB0"
        baudrate: 921600
        frame_id: "imu_link"
      topic: "/perception/imu/data"

  sensor_fusion:
    enabled: true
    method: "robot_localization"
    params:
      frequency: 100
      publish_tf: true
      world_frame: "odom"
      base_link_frame: "base_link"
      odom_frame: "odom"
```

## A.5 Hardware Configuration Examples

یہ section تینوں development tiers میں ہارڈویئر set up کرنے کے لیے مکمل configuration files فراہم کرتا ہے۔

### A.5.1 Tier 1: Development Workstation Configuration

```yaml
# config/workstation.yaml
# Hardware configuration for development workstation (Tier 1)
# یہ فائل Tier 1 development workstation کی ہارڈویئر تشکیل فراہم کرتی ہے

workstation:
  identification:
    name: "dev-ws-01"
    role: "development"
    tier: 1

  hardware:
    cpu:
      model: "AMD Ryzen 9 7950X"
      cores: 16
      threads: 32
      base_clock: "4.5 GHz"

    gpu:
      model: "NVIDIA RTX 4090"
      vram_gb: 24
      cuda_cores: 16384
      tensor_cores: 512

    memory:
      capacity_gb: 64
      type: "DDR5"
      speed_mhz: 5600
      channels: 4

    storage:
      primary:
        type: "NVMe PCIe 4.0"
        capacity_tb: 2
        read_mbps: 7000
        write_mbps: 5000
      secondary:
        type: "NVMe PCIe 4.0"
        capacity_tb: 4
        read_mbps: 7000
        write_mbps: 5000

  software:
    os: "Ubuntu 22.04 LTS"
    kernel: "6.5.0-generic"
    cuda_version: "12.3"
    cudnn_version: "8.9.7"
    tensorrt_version: "8.6.1"
    ros2_distribution: "humble"
    python_version: "3.10"

  performance_limits:
    cpu_power_limit: "200W"
    gpu_power_limit: "450W"
    memory_tdp: "normal"

  network:
    ethernet:
      interface: "eno1"
      speed_gbps: 10
    wireless:
      interface: "wlan0"
      standard: "Wi-Fi 6E"
```

### A.5.2 Tier 2: Edge Computing Configuration

```yaml
# config/jetson_orin_nano.yaml
# Hardware configuration for Jetson Orin Nano edge device (Tier 2)
# یہ فائل Tier 2 Jetson Orin Nano edge device کی ہارڈویئر تشکیل فراہم کرتی ہے

edge_device:
  identification:
    name: "edge-orin-nano-01"
    role: "edge_inference"
    tier: 2

  hardware:
    platform: "NVIDIA Jetson Orin Nano Super"
    sku: "p3767-0005"

    gpu:
      architecture: "Ampere"
      cores: 1024
      tensor_cores: 32
      max_frequency_mhz: 920

    cpu:
      cores: 6
      architecture: "ARM Cortex-A78AE"
      max_frequency_mhz: 2000

    memory:
      capacity_gb: 8
      type: "LPDDR5"
      bandwidth_gb_s: 102

    storage:
      type: "microSD UHS-I"
      capacity_gb: 256
      interface: "SDIO"

    expansion:
      m2_key_e: "Wi-Fi/BT module"
      m2_key_m: "NVMe SSD (optional)"

  power:
    modes:
      mode_10w:
        name: "10W"
        cpu_cores: 4
        gpu_freq_mhz: 510
        cpu_freq_mhz: 1200
      mode_15w:
        name: "15W"
        cpu_cores: 6
        gpu_freq_mhz: 628
        cpu_freq_mhz: 1470
      mode_25w:
        name: "25W (max)"
        cpu_cores: 6
        gpu_freq_mhz: 920
        cpu_freq_mhz: 2000

  software:
    os: "Ubuntu 22.04"
    jetpack_version: "6.1"
    cuda_version: "12.2"
    tensorrt_version: "8.6.1"
    l4t_version: "36.3"
    ros2_distribution: "humble"

  inference_config:
    default_precision: "int8"
    max_batch_size: 4
    workspace_size_mb: 512
    gpu_id: 0
```

### A.5.3 Tier 3: Full Robot Configuration

```yaml
# config/humanoid_full.yaml
# Complete hardware configuration for humanoid robot (Tier 3)
# یہ فائل Tier 3 humanoid robot کی مکمل ہارڈویئر تشکیل فراہم کرتی ہے

robot_system:
  identification:
    name: "humanoid-h1-01"
    model: "Unitree H1"
    tier: 3

  mechanical:
    height_mm: 1800
    weight_kg: 47
    dof_total: 17

  actuation:
    left_leg:
      joints: ["left_hip_yaw", "left_hip_roll", "left_hip_pitch",
               "left_knee_pitch", "left_ankle_roll", "left_ankle_pitch"]
      motors: 7
      max_torque_nm: 360

    right_leg:
      joints: ["right_hip_yaw", "right_hip_roll", "right_hip_pitch",
               "right_knee_pitch", "right_ankle_roll", "right_ankle_pitch"]
      motors: 7
      max_torque_nm: 360

    torso:
      joints: ["waist_yaw"]
      motors: 1

    arms:
      left_arm: ["left_shoulder_pitch", "left_elbow_pitch"]
      right_arm: ["right_shoulder_pitch", "right_elbow_pitch"]

    head:
      joints: ["head_yaw"]

  sensors:
    perception:
      lidar:
        model: "RPLIDAR A2"
        range_m: 18
        refresh_hz: 10
        topic: "/lidar/scan"

      depth_camera:
        model: "Intel RealSense D455"
        topic: "/camera/depth"
        color_topic: "/camera/color"
        frame_id: "head_camera_link"

      imu:
        model: "Embedded IMU"
        topic: "/imu/data"
        frequency_hz: 200

    proprioception:
      joint_sensors: true
      torque_sensors: true
      foot_force_sensors: true

  compute:
    primary:
      model: "NVIDIA Jetson AGX Orin"
      memory_gb: 64
      storage_gb: 64
      power_mode: "60W"

  power:
    battery:
      capacity_wh: 720
      voltage_v: 24
      cells: 6
      chemistry: "LiPo"

    runtime_minutes:
      idle: 240
      walking: 120
      manipulation: 180

  communication:
    interfaces:
      - type: "Ethernet"
        speed_gbps: 1
      - type: "Wi-Fi"
        standard: "802.11ac"
      - type: "CAN Bus"
        bitrate_bps: 1000000

  safety:
    emergency_stop: true
    fall_protection: true
    overcurrent_protection: true
    temperature_monitoring: true
```

## A.6 Troubleshooting Common Hardware Issues

یہ section physical AI development کے دوران common ہارڈویئر problems اور ان کے solutions کو address کرتا ہے۔

### A.6.1 GPU Driver Issues

```bash
# Check NVIDIA driver installation
# یہ کمانڈ NVIDIA ڈرائیور کی installation کو verify کرتا ہے
nvidia-smi

# If nvidia-smi fails, reinstall drivers
# اگر nvidia-smi fail ہو جائے تو ڈرائیور دوبارہ install کریں
sudo apt update
sudo apt install --reinstall nvidia-dkms-535
sudo reboot

# Verify CUDA installation
# CUDA installation کو verify کریں
nvcc --version

# Check GPU utilization during inference
# inference کے دوران GPU utilization check کریں
nvidia-smi -l 1
```

### A.6.2 Jetson Boot and Power Issues

```bash
# Force recovery mode boot (for flashing)
# Recovery mode boot کو force کریں (flashing کے لیے)
# Connect USB-C while holding recovery button, then:
sudo ./flash.sh jetson-orin-nano-super mmcblk0p1

# Monitor power consumption
# Power consumption monitor کریں
sudo tegrastats

# Check thermal status
# Thermal status check کریں
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# Set to maximum performance mode
# Maximum performance mode پر set کریں
sudo nvpmodel -m 0
sudo jetson_clocks
```

### A.6.3 Sensor Connectivity

```bash
# List connected USB devices
# Connected USB devices کی list دیکھیں
lsusb

# Check RealSense devices
# RealSense devices check کریں
rs-enumerate-devices

# View LIDAR data
# LIDAR data دیکھیں
# For RPLIDAR:
ros2 launch rplidar_ros rplidar.launch.py

# For Ouster:
ros2 launch ouster_ros ouster.launch.py sensor_hostname:=<sensor-ip>
```

:::warning Hardware Safety Precautions
humanoid robots اور high-power computing equipment کے ساتھ کام کرتے ہوئے ہمیشہ ان safety guidelines کو follow کریں:
- robot joints سے ہاتھ اور loose clothing کو دور رکھیں
- power on کرنے سے پہلے emergency stop accessible ہونے کو یقینی بنائیں
- electronics handle کرتے وقت proper ESD protection use کریں
- computing hardware کے لیے adequate ventilation maintain کریں
- humanoid robots کو people کے نزدیک safety barriers کے بغیر operate نہ کریں
- battery handling اور charging کے لیے manufacturer guidelines follow کریں
:::

## A.7 Recommended Suppliers and Resources

| Category | Supplier | Website | Notes |
|----------|----------|---------|-------|
| Workstation Components | NVIDIA Partner Network | nvidia.com/partners | Pre-configured AI workstations |
| Jetson Products | NVIDIA | developer.nvidia.com/embedded | Official developer kits |
| Humanoid Robots | Unitree | en.unitree.com | H1, Go2 direct sales |
| LIDAR | Ouster | ouster.com | High-resolution sensors |
| LIDAR | SLAMTEC | slamtec.com | RPLIDAR series |
| Depth Cameras | Intel RealSense | intel.com/realsense | D400 series |
| Depth Cameras | Orbbec | orbbec3d.com | Astra, Gemini series |
| IMUs | VectorNav | vectornav.com | VN-100, VN-200 |
| Research Platforms | ROBOTIS | en.robotis.com | Educational robots |

---

**Revision History**

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-12-25 | Initial release |

---

*یہ ایپینڈکس physical AI development کے لیے foundational ہارڈویئر knowledge فراہم کرتا ہے۔ تفصیلات manufacturer documentation پر مبنی ہیں اور purchase decisions سے پہلے verify کی جانی چاہیے۔*
