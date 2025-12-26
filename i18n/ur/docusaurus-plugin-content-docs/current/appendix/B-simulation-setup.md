---
title: "ضمیمہ B: سمیولیشن سیٹ اپ گائیڈ"
sidebar_position: 71
description: "Gazebo، NVIDIA Isaac Sim، Unity، اور کنٹینر پر مبنی سمیولیشن ماحول کے لیے جامع سیٹ اپ گائیڈ"
---

# ضمیمہ B: سمیولیشن سیٹ اپ گائیڈ

یہ ایپینڈکس اس کتاب میں استعمال ہونے والے primary simulation tools کے لیے تفصیلی سیٹ اپ instructions فراہم کرتا ہے۔ proper simulation environment configuration physical hardware پر deploy کرنے سے پہلے humanoid robotics algorithms تیار اور test کرنے کے لیے essential ہے۔

:::note Prerequisites
کسی بھی simulation setup شروع کرنے سے پہلے، یقینی بنائیں کہ آپ کا system Chapter 2 میں outline کیے گئے minimum hardware requirements کو meet کرتا ہے۔ GPU-intensive simulations کے لیے CUDA support والا dedicated graphics card strongly recommended ہے۔
:::

## B.1 Gazebo Simulation Setup

Gazebo ایک open-source robotics simulator ہے جو indoor اور outdoor environments کی accurate simulation فراہم کرتا ہے۔ یہ ROS (Robot Operating System) کے ساتھ seamlessly integrate ہوتا ہے اور robotics research اور development کا de facto standard ہے۔

### B.1.1 Installation on Ubuntu 22.04

Ubuntu 22.04 LTS (Jammy Jellyfish) ROS 2 Humble کے ساتھ Gazebo simulation کے لیے recommended operating system ہے۔ مندرجہ ذیل steps ایک complete installation outline کرتے ہیں:

```bash
# Update package lists
# پیکیج lists کو update کریں
sudo apt update
sudo apt upgrade -y

# Install required dependencies
# درکار dependencies install کریں
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    ca-certificates \
    wget \
    software-properties-common \
    apt-transport-https \
    git

# Add ROS 2 repository
# ROS 2 repository add کریں
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble desktop installation
# ROS 2 Humble desktop installation install کریں
sudo apt update
sudo apt install -y ros-humble-desktop

# Initialize rosdep (dependency manager for ROS)
# rosdep initialize کریں (ROS کے لیے dependency manager)
sudo rosdep init
rosdep update

# Install Gazebo simulation packages
# Gazebo simulation packages install کریں
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros2-multicontroller \
    ros-humble-gazebo-plugins \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-controllers

# Install additional useful tools
# اضافی useful tools install کریں
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-argcomplete \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-robot-steering

# Set up environment variables (add to ~/.bashrc for persistence)
# Environment variables set up کریں (persistence کے لیے ~/.bashrc میں add کریں)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models" >> ~/.bashrc

# Reload bash configuration
# bash configuration reload کریں
source ~/.bashrc
```

### B.1.2 Verifying Your Installation

installation کے بعد، verify کریں کہ سب components correctly install ہوئے ہیں:

```bash
# Check ROS 2 installation
# ROS 2 installation check کریں
ros2 doctor --report

# Verify Gazebo loads correctly
# Gazebo correctly load ہونے کو verify کریں
gazebo --version

# Test ROS-Gazebo integration
# ROS-Gazebo integration test کریں
ros2 launch gazebo_ros gazebo.launch.py world:=/usr/share/gazebo-11/worlds/empty.world
```

:::tip Common Verification Steps
اگر `ros2 doctor` missing dependencies report کرے، تو `rosdep update` run کریں اور terminal session restart کریں۔ Gazebo-specific issues کے لیے، یقینی بنائیں کہ `GAZEBO_MODEL_PATH` اور `GAZEBO_PLUGIN_PATH` environment variables correctly set ہیں۔
:::

### B.1.3 Creating a Humanoid Simulation Workspace

humanoid robotics simulation کے لیے ایک dedicated workspace set up کریں:

```bash
# Create workspace directory
# workspace directory create کریں
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Initialize workspace
# workspace initialize کریں
colcon build --symlink-install

# Source the workspace
# workspace source کریں
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### B.1.4 Troubleshooting Common Issues

مندرجہ ذیل table frequently encountered Gazebo simulation problems کو address کرتی ہے:

| Issue | Cause | Solution |
|-------|-------|----------|
| Black screen in Gazebo | GPU/driver incompatibility | Install NVIDIA drivers: `sudo ubuntu-drivers install nvidia` |
| Models not loading | Missing model files | Check `GAZEBO_MODEL_PATH` environment variable |
| Joint controllers not responding | Controller manager not started | Ensure `controller_manager` is launched |
| Physics unstable | Default physics settings | Adjust `sim_time` and physics parameters in URDF |
| Slow simulation | Insufficient resources | Reduce GUI elements, use headless mode |

persistent physics issues کے لیے، ایک custom physics configuration file create کریں:

```xml
<!-- ~/.gazebo/physics.config -->
<!-- یہ فайл Gazebo physics configuration فراہم کرتی ہے -->
<!-- یہ XML physics solver کی settings define کرتا ہے جو simulation کی accuracy اور speed کو affect کرتی ہیں -->
<sdf version="1.6">
  <world name="default">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <gravity>0 0 -9.81</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.0</sor>
        </solver>
      </ode>
    </physics>
  </world>
</sdf>
```

## B.2 NVIDIA Isaac Sim Setup

NVIDIA Isaac Sim NVIDIA Omniverse پر build کیا گیا ایک powerful robotics simulation platform ہے۔ یہ photorealistic rendering، physics simulation، اور ROS/ROS 2 کے ساتھ seamless integration فراہم کرتا ہے جو high-fidelity humanoid robotics development کے لیے ہے۔

### B.2.1 System Requirements

installation سے پہلے یقینی بنائیں کہ آپ کا system ان requirements کو meet کرتا ہے:

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | Intel Core i7-12700 | Intel Core i9-13900K |
| RAM | 16 GB | 64 GB |
| GPU | NVIDIA RTX 3060 (8GB) | NVIDIA RTX 4090 (24GB) |
| CUDA | 12.0 | 12.2+ |
| Storage | 50 GB SSD | 100+ GB NVMe SSD |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS |

### B.2.2 Installation Steps

```bash
# Install NVIDIA driver (if not already installed)
# NVIDIA driver install کریں (اگر پہلے سے installed نہیں ہے)
sudo ubuntu-drivers install nvidia-driver-535
sudo reboot

# Verify CUDA installation
# CUDA installation verify کریں
nvidia-smi
nvcc --version

# Install Python dependencies
# Python dependencies install کریں
sudo apt install -y python3-pip python3-venv

# Create Isaac Sim directory
# Isaac Sim directory create کریں
mkdir -p ~/isaac-sim
cd ~/isaac-sim

# Download Isaac Sim from NVIDIA NGC
# NVIDIA NGC سے Isaac Sim download کریں
# Visit https://developer.nvidia.com/isaac-sim to download the latest release
# Extract the downloaded file
tar -xf Isaac-Sim-Release-4.0.0.tar.xz

# Install Python dependencies in isolated environment
# isolated environment میں Python dependencies install کریں
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r ./exts/omni.isaac.sim/requirements.txt

# Set up environment variables
# Environment variables set up کریں
echo "export ISAAC_SIM_PATH=$HOME/isaac-sim" >> ~/.bashrc
echo "export OMNI_KIT_BUNDLE_PATH=$ISAAC_SIM_PATH/bundle" >> ~/.bashrc
echo "export CARB_APP_PATH=$ISAAC_SIM_PATH/bundle" >> ~/.bashrc
echo "source $ISAAC_SIM_PATH/setup_boost_env.sh" >> ~/.bashrc

source ~/.bashrc
```

### B.2.3 Licensing Information

Isaac Sim کو NVIDIA account کی ضرورت ہے اور یہ مندرجہ ذیل options کے ذریعے licensing accept کرتا ہے:

| License Type | Cost | Features |
|--------------|------|----------|
| Evaluation | Free (30 days) | Full feature access |
| Academic | Free (with .edu email) | Full features for research |
| Commercial | Subscription-based | Full commercial features |

[NVIDIA NGC](https://ngc.nvidia.com/) پر register کریں اور اپنے downloaded Isaac Sim package کے ساتھ provide کی گئی licensing instructions follow کریں۔

### B.2.4 Getting Started Workflow

humanoid simulation کے لیے ایک basic Isaac Sim script create کریں:

```python
#!/usr/bin/env python3
"""
Basic Isaac Sim humanoid simulation setup
Save as: ~/isaac-sim/workspace/humanoid_basic.py
یہ سکرپٹ humanoid simulation کے لیے Isaac Sim initialize کرتا ہے
ground plane اور lighting add کرتا ہے
"""

import omni.isaac.core
import omni.isaac.nucleus as nucleus
from pxr import Usd, UsdGeom, Gf, Sdf

# Initialize simulation
# سیمولیشن initialize کریں
omni.isaac.core.set_default_backend("omni.isaac.core")

# Create a new stage
# نیا stage create کریں
stage = omni.usd.get_context().get_stage()

# Add ground plane
# ground plane add کریں
ground_path = "/World/GroundPlane"
UsdGeom.Plane.Define(stage, ground_path)
ground_prim = stage.GetPrimAtPath(ground_path)

# Add lighting
# lighting add کریں
light_path = "/World/DistantLight"
distant_light = UsdLux.DistantLight.Define(stage, light_path)
distant_light.CreateIntensityAttr(1000)
distant_light.CreateAngleAttr(0.53)

# Configure physics
# physics configure کریں
scene_path = "/World"
scene = UsdPhysics.Scene.Define(stage, scene_path)
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

print("Isaac Sim humanoid workspace initialized successfully")
# یہ پرنٹ confirm کرتا ہے کہ workspace successfully initialize ہوا
```

Isaac Sim کو اپنے custom script کے ساتھ launch کریں:

```bash
cd ~/isaac-sim
./isaac-sim.sh --no-window --script ./workspace/humanoid_basic.py
```

### B.2.5 ROS 2 Integration with Isaac Sim

Isaac Sim سے ROS 2 communication enable کریں:

```bash
# Install ROS 2 bridge
# ROS 2 bridge install کریں
cd ~/isaac-sim
./isaac-sim.sh --exts ext/omni.isaac.ros2_bridge

# Create a ROS 2 enabled launch script
# ROS 2 enabled launch script create کریں
cat > ~/isaac-sim/workspace/ros2_humanoid.launch.py << 'EOF'
"""ROS 2 enabled humanoid simulation launch script"""
# یہ سکرپٹ ROS 2 enabled humanoid simulation کے لیے launch configuration فراہم کرتا ہے
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='isaac_sim_container',
        namespace='/isaac_sim',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                name='robot_state_publisher',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': '/path/to/robot.urdf'}]
            ),
            ComposableNode(
                name='joint_state_publisher',
                package='joint_state_publisher',
                executable='joint_state_publisher'
            )
        ],
        output='screen'
    )
    return launch.LaunchDescription([container])

if __name__ == '__main__':
    generate_launch_description()
EOF
```

## B.3 Unity for Robotics

Unity اپنے high-fidelity physics engine اور extensive asset ecosystem کے ساتھ robotics simulation کے لیے ایک flexible environment فراہم کرتا ہے۔ Unity Robotics Hub robotics simulation اور ROS integration کے لیے specialized packages offer کرتا ہے۔

### B.3.1 Unity Installation

```bash
# Install Unity Hub
# Unity Hub install کریں
wget -q https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.deb
sudo dpkg -i UnityHubSetup.deb
sudo apt-get install -f -y

# Install Unity Editor (2022.3 LTS recommended for robotics)
# Unity Editor install کریں (robotics کے لیے 2022.3 LTS recommended)
# Open Unity Hub and install Unity Editor 2022.3.20f1 or newer
# Select the following modules during installation:
# - Linux Build Support
# - Windows Build Support
# - Android Build Support
# - iOS Build Support
```

### B.3.2 ROS-Unity Integration Setup

ROS integration کے ساتھ ایک نیا Unity project create کریں:

```bash
# Create project directory
# project directory create کریں
mkdir -p ~/unity-ros-projects
cd ~/unity-ros-projects

# Clone Unity Robotics Hub packages
# Unity Robotics Hub packages clone کریں
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub

# Note: Import packages through Unity Package Manager
# Window > Package Manager > Add package from git URL
# Add: https://github.com/Unity-Technologies/ros2-message-publisher.git
# Add: https://github.com/Unity-Technologies/ros2-service-publisher.git
```

### B.3.3 Basic Unity Robotics Project Setup

Unity میں ایک simple humanoid robot prefab create کریں:

```csharp
// HumanoidRobotController.cs
// یہ سکرپٹ Unity میں humanoid robot کے لیے controller فراہم کرتا ہے
// ROS connection establish کرتا ہے اور joint movements کو control کرتا ہے

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using sensor_msgs.msg;
using geometry_msgs.msg;

public class HumanoidRobotController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public ArticulationBody[] leftLegJoints;
    public ArticulationBody[] rightLegJoints;
    public ArticulationBody[] leftArmJoints;
    public ArticulationBody[] rightArmJoints;
    public ArticulationBody torsoJoint;

    [Header("ROS Settings")]
    public string cmdVelTopic = "/cmd_vel";
    public string jointStateTopic = "/joint_states";

    private ROSConnection ros;
    private Vector3 targetVelocity;
    private float targetYaw;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<geometry_msgs.msg.Twist>(cmdVelTopic);
        ros.Subscribe<geometry_msgs.msg.Twist>(cmdVelTopic, ReceiveVelocityCommand);

        InitializeJoints();
    }

    void InitializeJoints()
    {
        // Configure joint drive properties
        // Joint drive properties configure کریں
        foreach (var joint in leftLegJoints)
        {
            joint.xDrive = new ArticulationDrive
            {
                stiffness = 1000f,
                damping = 100f,
                forceLimit = 500f
            };
        }
    }

    void ReceiveVelocityCommand(Twist msg)
    {
        targetVelocity = msg.linear.From<FLU>();
        targetYaw = (float)msg.angular.z;
    }

    void FixedUpdate()
    {
        // Apply velocity commands to joints
        // Joints پر velocity commands apply کریں
        for (int i = 0; i < leftLegJoints.Length; i++)
        {
            float targetPosition = Mathf.Clamp(
                targetVelocity.z * i * 0.1f,
                leftLegJoints[i].xDrive.lowerLimit,
                leftLegJoints[i].xDrive.upperLimit
            );
            leftLegJoints[i].SetTarget(TargetIndex.X, targetPosition);
            rightLegJoints[i].SetTarget(TargetIndex.X, -targetPosition);
        }
    }
}
```

### B.3.4 Unity-ROS 2 Communication Setup

Unity میں ROS 2 communication configure کریں:

```bash
# Build ROS 2 message types for Unity
# Unity کے لیے ROS 2 message types build کریں
cd ~/unity-ros-projects
mkdir -p msg_gen
cd msg_gen

# Generate C# message definitions
# C# message definitions generate کریں
ros2 run ros2_message_gen generate_messages \
    --input-pkg your_robot_description \
    --output-dir ./src/GeneratedMessages

# Import generated messages into Unity project
# Generated messages کو Unity project میں import کریں
# Copy GeneratedMessages to Assets/RosMessages/
```

## B.4 Container and Virtual Machine Setup

Containerization different machines پر reproducible simulation environments ensure کرتا ہے اور simulation workflows کے deployment کو facilitate کرتا ہے۔

### B.4.1 Docker Setup for Simulation

consistent Gazebo simulation کے لیے ایک Docker environment create کریں:

```dockerfile
# Dockerfile.gazebo-sim
# یہ Dockerfile Gazebo simulation کے لیے containerized environment فراہم کرتا ہے
# ROS 2 Humble اور Gazebo install کرتا ہے

FROM osrf/ros:humble-desktop-full

# Install Gazebo and dependencies
# Gazebo اور dependencies install کریں
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo11-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up non-interactive installation
# Non-interactive installation set up کریں
ENV DEBIAN_FRONTEND=noninteractive

# Create user for simulation
# Simulation کے لیے user create کریں
RUN useradd -m -s /bin/bash simuser
USER simuser
WORKDIR /home/simuser

# Copy workspace files
# Workspace files copy کریں
COPY --chown=simuser:simuser . /home/simuser/catkin_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    cd /home/simuser/catkin_ws && \
    colcon build --symlink-install"

# Set entry point
# Entry point set کریں
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /home/simuser/catkin_ws/install/setup.sh && exec \"$@\""]

# Build and run
# Build اور run کریں
# docker build -f Dockerfile.gazebo-sim -t gazebo-sim .
# docker run -it --rm \
#     --privileged \
#     --gpus all \
#     -e DISPLAY=$DISPLAY \
#     -v /tmp/.X11-unix:/tmp/.X11-unix \
#     gazebo-sim
```

### B.4.2 Docker Compose for Multi-Container Simulation

```yaml
# docker-compose.yml
# یہ فائل multi-container simulation environment کی تشکیل فراہم کرتی ہے
# Gazebo اور ROS bridge containers کو define کرتی ہے

version: '3.8'

services:
  gazebo-sim:
    build:
      context: .
      dockerfile: Dockerfile.gazebo-sim
    container_name: humanoid-sim
    privileged: true
    gpus: all
    environment:
      - DISPLAY=${DISPLAY}
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./workspace:/home/simuser/workspace:rw
    networks:
      - sim-network
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]

  ros-bridge:
    image: osrf/ros:humble-ros-base
    container_name: ros-bridge
    command: >
      ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    networks:
      - sim-network
    depends_on:
      - gazebo-sim

networks:
  sim-network:
    driver: bridge
```

### B.4.3 NVIDIA Isaac Sim Container

Isaac Sim کو Docker container میں run کریں:

```dockerfile
# Dockerfile.isaac-sim
# یہ Dockerfile Isaac Sim کے لیے containerized environment فراہم کرتا ہے
# CUDA-enabled container پر build ہے

FROM nvidia/cuda:12.2-devel-ubuntu22.04

# Install dependencies
# Dependencies install کریں
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libnvidia-gl-535 \
    && rm -rf /var/lib/apt/lists/*

# Set up Isaac Sim environment
# Isaac Sim environment set up کریں
ENV ISAAC_SIM_PATH=/isaac-sim
ENV OMNI_KIT_BUNDLE_PATH=/isaac-sim/bundle
ENV CARB_APP_PATH=/isaac-sim/bundle

# Copy Isaac Sim files (mount volume in production)
# Isaac Sim files copy کریں (production میں volume mount کریں)
COPY Isaac-Sim-4.0.0 /isaac-sim

WORKDIR /isaac-sim
CMD ["./isaac-sim.sh"]
```

### B.4.4 Virtual Machine Recommendations

complete system isolation کی ضرورت والے users کے لیے، مندرجہ ذیل VM configurations recommended ہیں:

| Use Case | Provider | Recommended Config |
|----------|----------|-------------------|
| Development | VirtualBox | 8 CPU, 32GB RAM, 100GB disk |
| CI/CD | Vagrant + VirtualBox | 4 CPU, 16GB RAM, 60GB disk |
| Cloud | AWS EC2 (g4dn.xlarge) | GPU instance for rendering |
| High-Performance | VMware ESXi | Passthrough GPU recommended |

```bash
# Vagrantfile for simulation VM
# یہ Vagrantfile simulation VM کی تشکیل فراہم کرتا ہے
Vagrant.configure("2") do |config|
  config.vm.box = "generic/ubuntu2204"
  config.vm.provider "virtualbox" do |vb|
    vb.memory = "32768"
    vb.cpus = "8"
    vb.graphics_controller = "vmsvga"
    vb.video_memory = "128"
  end
  config.vm.provision "shell", inline: <<-SHELL
    apt-get update
    apt-get install -y ros-humble-desktop-full gazebo
    SHELL
end
```

## B.5 Performance Optimization

simulation performance optimization real-time humanoid robotics development کے لیے critical ہے۔ مندرجہ ذیل techniques simulation speed اور accuracy maximize کرتی ہیں۔

### B.5.1 GPU Optimization

accelerated physics اور rendering کے لیے GPU utilization maximize کریں:

```bash
# Verify GPU is available for simulation
# Simulation کے لیے GPU available ہونے کو verify کریں
nvidia-smi

# Check CUDA version compatibility
# CUDA version compatibility check کریں
nvcc --version

# Monitor GPU usage during simulation
# Simulation کے دوران GPU usage monitor کریں
watch -n 1 nvidia-smi
```

optimal GPU performance کے لیے Isaac Sim configure کریں:

```python
# Optimized Isaac Sim settings
# یہ سکرپٹ Isaac Sim کو optimal GPU performance کے لیے configure کرتا ہے
# RTX rendering enable کرتا ہے اور physics settings optimize کرتا ہے

import omni.kit.settings

# Enable RTX rendering
# RTX rendering enable کریں
omni.kit.settings.get_settings().set_bool("/rtx/enabled", True)
omni.kit.settings.get_settings().set_bool("/rtx/directLighting/enabled", True)

# Configure DLSS for performance
# Performance کے لیے DLSS configure کریں
omni.kit.settings.get_settings().set_bool("/rtx/dlsr/enabled", True)
omni.kit.settings.get_settings().set_int("/rtx/dlsr/width", 1280)
omni.kit.settings.get_settings().set_int("/rtx/dlsr/height", 720)

# Optimize physics settings
# Physics settings optimize کریں
omni.kit.settings.get_settings().set_int("/physics/maxSolverIterations", 64)
omni.kit.settings.get_settings().set_float("/physics/solverTolerance", 1e-4)
```

### B.5.2 Memory Management

large-scale humanoid simulations کو efficiently handle کریں:

```python
# Memory-optimized simulation script
# یہ سکرپٹ memory-optimized simulation فراہم کرتا ہے
# large-scale humanoid simulations کے لیے design ہے

import gc
import omni.usd
from pxr import Usd, UsdGeom

class OptimizedSimulation:
    def __init__(self, stage_url):
        self.stage = omni.usd.get_context().open_stage(stage_url)
        self.cache_size = 100  # Limit cached prims
        self.prim_cache = []

    def load_robot_safely(self, robot_usd_path):
        """Load robot with memory-conscious approach"""
        # Clear old cache
        # پرانی cache clear کریں
        self.clear_cache()
        gc.collect()

        # Load stage in parts to reduce peak memory
        # Stage کو parts میں load کریں تاکہ peak memory کم ہو
        stage = omni.usd.get_context().open_stage(robot_usd_path)

        # Process in batches
        # Batches میں process کریں
        batch_size = 50
        all_prims = [p for p in stage.Traverse()
                    if p.GetTypeName() in ["Xform", "Mesh"]]

        for i in range(0, len(all_prims), batch_size):
            batch = all_prims[i:i + batch_size]
            self.process_batch(batch)

        return stage

    def clear_cache(self):
        """Clear cached prims to free memory"""
        # Cached prims clear کریں تاکہ memory free ہو
        self.prim_cache.clear()
        gc.collect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.clear_cache()
        omni.usd.get_context().close_stage()
```

### B.5.3 Simulation Speed Optimization

ان techniques کے ساتھ simulation speed اور accuracy balance کریں:

```yaml
# Gazebo physics optimization (world.sdf)
# یہ فائل Gazebo physics optimization فراہم کرتی ہے
# Simulation speed اور accuracy کو optimize کرتی ہے
<sdf version="1.10">
  <world name="optimized_world">
    <physics name="fast_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>2.0</real_time_factor>
      <real_time_update_rate>120</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Simplified collision for humanoid -->
    <!-- Humanoid کے لیے simplified collision -->
    <model name="humanoid">
      <link name="torso">
        <collision>
          <geometry>
            <box>
              <size>0.3 0.4 0.5</size>
            </box>
          </geometry>
          <max_contacts>4</max_contacts>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

training scenarios کے لیے headless mode use کریں:

```bash
# Run Gazebo in headless mode
# Headless mode میں Gazebo run کریں
gzserver --headless worlds/humanoid.world

# Run Isaac Sim without UI
# UI کے بغیر Isaac Sim run کریں
./isaac-sim.sh --no-window --num-frames 1000

# Unity headless build
# Unity headless build
./Unity -batchmode -nographics -quit -projectPath /path/to/project
```

### B.5.4 Multi-Node Distributed Simulation

complex scenarios کے لیے simulation کو multiple machines پر scale کریں:

```bash
# Master node setup
# Master node setup
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=master.local

# Worker node setup
# Worker node setup
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=worker1.local

# Launch distributed Gazebo
# Distributed Gazebo launch کریں
# On master node:
roslaunch gazebo_ros multi_robot_name:=true world_name:=humanoid.world

# On worker nodes:
roslaunch gazebo_ros worker_node.launch
```

```python
# Distributed simulation manager
# یہ class distributed simulation کو manage کرتا ہے
# Multiple nodes پر humanoid components کو distribute کرتا ہے

class DistributedSimManager:
    def __init__(self, master_uri, worker_nodes):
        self.master_uri = master_uri
        self.worker_nodes = worker_nodes
        self.allocations = {}

    def allocate_humanoid_parts(self):
        """Distribute humanoid components across nodes"""
        # Humanoid components کو nodes پر distribute کریں
        parts = {
            "left_leg": "worker1",
            "right_leg": "worker2",
            "torso": "master",
            "left_arm": "worker3",
            "right_arm": "worker4",
            "head": "master"
        }

        for part, node in parts.items():
            self.allocations[part] = {
                "node": node,
                "status": "allocated"
            }

        return self.allocations

    def synchronize_simulation(self):
        """Ensure simulation state is synchronized"""
        # Simulation state synchronized ہونے کو ensure کریں
        import rospy
        from std_msgs.msg import Float64MultiArray

        pub = rospy.Publisher('/sync/timestamp', Float64MultiArray, queue_size=10)
        rate = rospy.Rate(100)  # 100 Hz sync

        while not rospy.is_shutdown():
            msg = Float64MultiArray()
            msg.data = [rospy.get_time()]
            pub.publish(msg)
            rate.sleep()
```

### B.5.5 Performance Monitoring Tools

comprehensive performance monitoring implement کریں:

```python
#!/usr/bin/env python3
"""
Simulation Performance Monitor
Usage: python perf_monitor.py
یہ سکرپٹ simulation performance monitor فراہم کرتا ہے
CPU، GPU، اور memory usage track کرتا ہے
"""

import time
import psutil
import GPUtil
from datetime import datetime

class SimulationMonitor:
    def __init__(self, log_interval=1.0):
        self.log_interval = log_interval
        self.metrics = {
            "cpu_percent": [],
            "memory_percent": [],
            "gpu_load": [],
            "gpu_memory": [],
            "timestamps": []
        }

    def get_cpu_usage(self):
        return psutil.cpu_percent(interval=None)

    def get_memory_usage(self):
        return psutil.virtual_memory().percent

    def get_gpu_usage(self):
        gpus = GPUtil.getGPUs()
        if gpus:
            return gpus[0].load * 100
        return 0.0

    def get_gpu_memory(self):
        gpus = GPUtil.getGPUs()
        if gpus:
            return (gpus[0].memoryUsed / gpus[0].memoryTotal) * 100
        return 0.0

    def collect_metrics(self):
        """Collect all performance metrics"""
        # سب performance metrics collect کریں
        timestamp = datetime.now().isoformat()
        self.metrics["timestamps"].append(timestamp)
        self.metrics["cpu_percent"].append(self.get_cpu_usage())
        self.metrics["memory_percent"].append(self.get_memory_usage())
        self.metrics["gpu_load"].append(self.get_gpu_usage())
        self.metrics["gpu_memory"].append(self.get_gpu_memory())

    def generate_report(self):
        """Generate performance report"""
        # Performance report generate کریں
        report = {
            "cpu_avg": sum(self.metrics["cpu_percent"]) / len(self.metrics["cpu_percent"]),
            "memory_avg": sum(self.metrics["memory_percent"]) / len(self.metrics["memory_percent"]),
            "gpu_load_avg": sum(self.metrics["gpu_load"]) / len(self.metrics["gpu_load"]),
            "gpu_memory_avg": sum(self.metrics["gpu_memory"]) / len(self.metrics["gpu_memory"]),
            "samples": len(self.metrics["timestamps"])
        }
        return report

    def print_status(self):
        """Print current system status"""
        # موجودہ system status print کریں
        print(f"[{datetime.now().strftime('%H:%M:%S')}] "
              f"CPU: {self.get_cpu_usage():.1f}% | "
              f"RAM: {self.get_memory_usage():.1f}% | "
              f"GPU: {self.get_gpu_usage():.1f}% | "
              f"VRAM: {self.get_gpu_memory():.1f}%")

if __name__ == "__main__":
    monitor = SimulationMonitor()
    try:
        while True:
            monitor.print_status()
            monitor.collect_metrics()
            time.sleep(monitor.log_interval)
    except KeyboardInterrupt:
        report = monitor.generate_report()
        print("\n=== Performance Report ===")
        for key, value in report.items():
            print(f"{key}: {value:.2f}")
```

## B.6 Summary

یہ ایپینڈکس humanoid robotics development میں use ہونے والے primary simulation platforms کے لیے comprehensive setup instructions فراہم کرتا ہے۔ key takeaways:

- **Gazebo** ROS 2 کے ساتھ best integration فراہم کرتا ہے اور algorithm development اور testing کے لیے ideal ہے
- **NVIDIA Isaac Sim** perception research کے لیے photorealistic rendering اور high-fidelity physics فراہم کرتا ہے
- **Unity** اپنے extensive asset ecosystem اور C# scripting کے ساتھ rapid prototyping enable کرتا ہے
- **Containerization** reproducibility ensure کرتا ہے اور different environments میں deployment simplify کرتا ہے
- **Performance optimization** real-time simulation کے لیے essential ہے، particularly learning-based approaches کے لیے

ان simulation environments کو properly configure کرنے کے ساتھ، آپ subsequent chapters میں describe کیے گئے humanoid robotics algorithms implement کرنے کے لیے proceed کر سکتے ہیں، physical hardware پر deployment سے پہلے انہیں simulation میں test کر کے۔

---

**Next Steps:**

- Verification commands run کر کر کے installations verify کریں
- Specific chapters کے لیے جن پر کام کرنے کا ارادہ ہے اپنا preferred simulation environment set up کریں
- System کے لیے baseline metrics establish کرنے کے لیے performance monitoring tools review کریں
