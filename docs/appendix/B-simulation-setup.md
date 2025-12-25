---
title: "Appendix B: Simulation Setup Guide"
sidebar_position: 71
description: Comprehensive setup guide for Gazebo, NVIDIA Isaac Sim, Unity, and container-based simulation environments
---

# Appendix B: Simulation Setup Guide

This appendix provides detailed setup instructions for the primary simulation tools used throughout this book. Proper simulation environment configuration is essential for developing and testing humanoid robotics algorithms before deploying to physical hardware.

:::note Prerequisites
Before beginning any simulation setup, ensure your system meets the minimum hardware requirements outlined in Chapter 2. For GPU-intensive simulations, a dedicated graphics card with CUDA support is strongly recommended.
:::

## B.1 Gazebo Simulation Setup

Gazebo is an open-source robotics simulator that provides accurate simulation of indoor and outdoor environments. It integrates seamlessly with ROS (Robot Operating System) and is the de facto standard for robotics research and development.

### B.1.1 Installation on Ubuntu 22.04

Ubuntu 22.04 LTS (Jammy Jellyfish) is the recommended operating system for Gazebo simulation with ROS 2 Humble. The following steps outline a complete installation:

```bash
# Update package lists
sudo apt update
sudo apt upgrade -y

# Install required dependencies
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
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble desktop installation
sudo apt update
sudo apt install -y ros-humble-desktop

# Initialize rosdep (dependency manager for ROS)
sudo rosdep init
rosdep update

# Install Gazebo simulation packages
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
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-argcomplete \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-robot-steering

# Set up environment variables (add to ~/.bashrc for persistence)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/share/gazebo-11/models" >> ~/.bashrc

# Reload bash configuration
source ~/.bashrc
```

### B.1.2 Verifying Your Installation

After installation, verify that all components are correctly installed:

```bash
# Check ROS 2 installation
ros2 doctor --report

# Verify Gazebo loads correctly
gazebo --version

# Test ROS-Gazebo integration
ros2 launch gazebo_ros gazebo.launch.py world:=/usr/share/gazebo-11/worlds/empty.world
```

:::tip Common Verification Steps
If `ros2 doctor` reports missing dependencies, run `rosdep update` and restart your terminal session. For Gazebo-specific issues, ensure the `GAZEBO_MODEL_PATH` and `GAZEBO_PLUGIN_PATH` environment variables are set correctly.
:::

### B.1.3 Creating a Humanoid Simulation Workspace

Set up a dedicated workspace for humanoid robotics simulation:

```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Initialize workspace
colcon build --symlink-install

# Source the workspace
echo "source ~/humanoid_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### B.1.4 Troubleshooting Common Issues

The following table addresses frequently encountered Gazebo simulation problems:

| Issue | Cause | Solution |
|-------|-------|----------|
| Black screen in Gazebo | GPU/driver incompatibility | Install NVIDIA drivers: `sudo ubuntu-drivers install nvidia` |
| Models not loading | Missing model files | Check `GAZEBO_MODEL_PATH` environment variable |
| Joint controllers not responding | Controller manager not started | Ensure `controller_manager` is launched |
| Physics unstable | Default physics settings | Adjust `sim_time` and physics parameters in URDF |
| Slow simulation | Insufficient resources | Reduce GUI elements, use headless mode |

For persistent physics issues, create a custom physics configuration file:

```xml
<!-- ~/.gazebo/physics.config -->
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

NVIDIA Isaac Sim is a powerful robotics simulation platform built on NVIDIA Omniverse. It offers photorealistic rendering, physics simulation, and seamless integration with ROS/ROS 2 for high-fidelity humanoid robotics development.

### B.2.1 System Requirements

Ensure your system meets these requirements before installation:

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
sudo ubuntu-drivers install nvidia-driver-535
sudo reboot

# Verify CUDA installation
nvidia-smi
nvcc --version

# Install Python dependencies
sudo apt install -y python3-pip python3-venv

# Create Isaac Sim directory
mkdir -p ~/isaac-sim
cd ~/isaac-sim

# Download Isaac Sim from NVIDIA NGC
# Visit https://developer.nvidia.com/isaac-sim to download the latest release
# Extract the downloaded file
tar -xf Isaac-Sim-Release-4.0.0.tar.xz

# Install Python dependencies in isolated environment
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r ./exts/omni.isaac.sim/requirements.txt

# Set up environment variables
echo "export ISAAC_SIM_PATH=$HOME/isaac-sim" >> ~/.bashrc
echo "export OMNI_KIT_BUNDLE_PATH=$ISAAC_SIM_PATH/bundle" >> ~/.bashrc
echo "export CARB_APP_PATH=$ISAAC_SIM_PATH/bundle" >> ~/.bashrc
echo "source $ISAAC_SIM_PATH/setup_boost_env.sh" >> ~/.bashrc

source ~/.bashrc
```

### B.2.3 Licensing Information

Isaac Sim requires an NVIDIA account and accepts licensing through the following options:

| License Type | Cost | Features |
|--------------|------|----------|
| Evaluation | Free (30 days) | Full feature access |
| Academic | Free (with .edu email) | Full features for research |
| Commercial | Subscription-based | Full commercial features |

Register at [NVIDIA NGC](https://ngc.nvidia.com/) and follow the licensing instructions provided with your downloaded Isaac Sim package.

### B.2.4 Getting Started Workflow

Create a basic Isaac Sim script for humanoid simulation:

```python
#!/usr/bin/env python3
"""
Basic Isaac Sim humanoid simulation setup
Save as: ~/isaac-sim/workspace/humanoid_basic.py
"""

import omni.isaac.core
import omni.isaac.nucleus as nucleus
from pxr import Usd, UsdGeom, Gf, Sdf

# Initialize simulation
omni.isaac.core.set_default_backend("omni.isaac.core")

# Create a new stage
stage = omni.usd.get_context().get_stage()

# Add ground plane
ground_path = "/World/GroundPlane"
UsdGeom.Plane.Define(stage, ground_path)
ground_prim = stage.GetPrimAtPath(ground_path)

# Add lighting
light_path = "/World/DistantLight"
distant_light = UsdLux.DistantLight.Define(stage, light_path)
distant_light.CreateIntensityAttr(1000)
distant_light.CreateAngleAttr(0.53)

# Configure physics
scene_path = "/World"
scene = UsdPhysics.Scene.Define(stage, scene_path)
scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
scene.CreateGravityMagnitudeAttr(9.81)

print("Isaac Sim humanoid workspace initialized successfully")
```

Launch Isaac Sim with your custom script:

```bash
cd ~/isaac-sim
./isaac-sim.sh --no-window --script ./workspace/humanoid_basic.py
```

### B.2.5 ROS 2 Integration with Isaac Sim

Enable ROS 2 communication from Isaac Sim:

```bash
# Install ROS 2 bridge
cd ~/isaac-sim
./isaac-sim.sh --exts ext/omni.isaac.ros2_bridge

# Create a ROS 2 enabled launch script
cat > ~/isaac-sim/workspace/ros2_humanoid.launch.py << 'EOF'
"""ROS 2 enabled humanoid simulation launch script"""
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

Unity provides a flexible environment for robotics simulation with its high-fidelity physics engine and extensive asset ecosystem. The Unity Robotics Hub offers specialized packages for robot simulation and ROS integration.

### B.3.1 Unity Installation

```bash
# Install Unity Hub
wget -q https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.deb
sudo dpkg -i UnityHubSetup.deb
sudo apt-get install -f -y

# Install Unity Editor (2022.3 LTS recommended for robotics)
# Open Unity Hub and install Unity Editor 2022.3.20f1 or newer
# Select the following modules during installation:
# - Linux Build Support
# - Windows Build Support
# - Android Build Support
# - iOS Build Support
```

### B.3.2 ROS-Unity Integration Setup

Create a new Unity project with ROS integration:

```bash
# Create project directory
mkdir -p ~/unity-ros-projects
cd ~/unity-ros-projects

# Clone Unity Robotics Hub packages
git clone https://github.com/Unity-Technologies/Unity-Robotics-Hub.git
cd Unity-Robotics-Hub

# Note: Import packages through Unity Package Manager
# Window > Package Manager > Add package from git URL
# Add: https://github.com/Unity-Technologies/ros2-message-publisher.git
# Add: https://github.com/Unity-Technologies/ros2-service-publisher.git
```

### B.3.3 Basic Unity Robotics Project Setup

Create a simple humanoid robot prefab in Unity:

```csharp
// HumanoidRobotController.cs
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

Configure ROS 2 communication in Unity:

```bash
# Build ROS 2 message types for Unity
cd ~/unity-ros-projects
mkdir -p msg_gen
cd msg_gen

# Generate C# message definitions
ros2 run ros2_message_gen generate_messages \
    --input-pkg your_robot_description \
    --output-dir ./src/GeneratedMessages

# Import generated messages into Unity project
# Copy GeneratedMessages to Assets/RosMessages/
```

## B.4 Container and Virtual Machine Setup

Containerization ensures reproducible simulation environments across different machines and facilitates deployment of simulation workflows.

### B.4.1 Docker Setup for Simulation

Create a Docker environment for consistent Gazebo simulation:

```dockerfile
# Dockerfile.gazebo-sim
FROM osrf/ros:humble-desktop-full

# Install Gazebo and dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo11-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Create user for simulation
RUN useradd -m -s /bin/bash simuser
USER simuser
WORKDIR /home/simuser

# Copy workspace files
COPY --chown=simuser:simuser . /home/simuser/catkin_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    cd /home/simuser/catkin_ws && \
    colcon build --symlink-install"

# Set entry point
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.sh && source /home/simuser/catkin_ws/install/setup.sh && exec \"$@\""]

# Build and run
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

Run Isaac Sim in a Docker container:

```dockerfile
# Dockerfile.isaac-sim
FROM nvidia/cuda:12.2-devel-ubuntu22.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libnvidia-gl-535 \
    && rm -rf /var/lib/apt/lists/*

# Set up Isaac Sim environment
ENV ISAAC_SIM_PATH=/isaac-sim
ENV OMNI_KIT_BUNDLE_PATH=/isaac-sim/bundle
ENV CARB_APP_PATH=/isaac-sim/bundle

# Copy Isaac Sim files (mount volume in production)
COPY Isaac-Sim-4.0.0 /isaac-sim

WORKDIR /isaac-sim
CMD ["./isaac-sim.sh"]
```

### B.4.4 Virtual Machine Recommendations

For users requiring complete system isolation, the following VM configurations are recommended:

| Use Case | Provider | Recommended Config |
|----------|----------|-------------------|
| Development | VirtualBox | 8 CPU, 32GB RAM, 100GB disk |
| CI/CD | Vagrant + VirtualBox | 4 CPU, 16GB RAM, 60GB disk |
| Cloud | AWS EC2 (g4dn.xlarge) | GPU instance for rendering |
| High-Performance | VMware ESXi | Passthrough GPU recommended |

```bash
# Vagrantfile for simulation VM
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

Optimizing simulation performance is critical for real-time humanoid robotics development. The following techniques maximize simulation speed and accuracy.

### B.5.1 GPU Optimization

Maximize GPU utilization for accelerated physics and rendering:

```bash
# Verify GPU is available for simulation
nvidia-smi

# Check CUDA version compatibility
nvcc --version

# Monitor GPU usage during simulation
watch -n 1 nvidia-smi
```

Configure Isaac Sim for optimal GPU performance:

```python
# Optimized Isaac Sim settings
import omni.kit.settings

# Enable RTX rendering
omni.kit.settings.get_settings().set_bool("/rtx/enabled", True)
omni.kit.settings.get_settings().set_bool("/rtx/directLighting/enabled", True)

# Configure DLSS for performance
omni.kit.settings.get_settings().set_bool("/rtx/dlsr/enabled", True)
omni.kit.settings.get_settings().set_int("/rtx/dlsr/width", 1280)
omni.kit.settings.get_settings().set_int("/rtx/dlsr/height", 720)

# Optimize physics settings
omni.kit.settings.get_settings().set_int("/physics/maxSolverIterations", 64)
omni.kit.settings.get_settings().set_float("/physics/solverTolerance", 1e-4)
```

### B.5.2 Memory Management

Handle large-scale humanoid simulations efficiently:

```python
# Memory-optimized simulation script
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
        self.clear_cache()
        gc.collect()

        # Load stage in parts to reduce peak memory
        stage = omni.usd.get_context().open_stage(robot_usd_path)

        # Process in batches
        batch_size = 50
        all_prims = [p for p in stage.Traverse()
                    if p.GetTypeName() in ["Xform", "Mesh"]]

        for i in range(0, len(all_prims), batch_size):
            batch = all_prims[i:i + batch_size]
            self.process_batch(batch)

        return stage

    def clear_cache(self):
        """Clear cached prims to free memory"""
        self.prim_cache.clear()
        gc.collect()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.clear_cache()
        omni.usd.get_context().close_stage()
```

### B.5.3 Simulation Speed Optimization

Balance simulation speed and accuracy with these techniques:

```yaml
# Gazebo physics optimization (world.sdf)
<sdf version="1.10">
  <world name="optimized_world">
    <physics name="fast_physics" type="dart">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>2.0</real_time_factor>
      <real_time_update_rate>120</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
    </physics>

    <!-- Simplified collision for humanoid -->
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

Use headless mode for training scenarios:

```bash
# Run Gazebo in headless mode
gzserver --headless worlds/humanoid.world

# Run Isaac Sim without UI
./isaac-sim.sh --no-window --num-frames 1000

# Unity headless build
./Unity -batchmode -nographics -quit -projectPath /path/to/project
```

### B.5.4 Multi-Node Distributed Simulation

Scale simulation across multiple machines for complex scenarios:

```bash
# Master node setup
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=master.local

# Worker node setup
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=worker1.local

# Launch distributed Gazebo
# On master node:
roslaunch gazebo_ros multi_robot_name:=true world_name:=humanoid.world

# On worker nodes:
roslaunch gazebo_ros worker_node.launch
```

```python
# Distributed simulation manager
class DistributedSimManager:
    def __init__(self, master_uri, worker_nodes):
        self.master_uri = master_uri
        self.worker_nodes = worker_nodes
        self.allocations = {}

    def allocate_humanoid_parts(self):
        """Distribute humanoid components across nodes"""
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

Implement comprehensive performance monitoring:

```python
#!/usr/bin/env python3
"""
Simulation Performance Monitor
Usage: python perf_monitor.py
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
        timestamp = datetime.now().isoformat()
        self.metrics["timestamps"].append(timestamp)
        self.metrics["cpu_percent"].append(self.get_cpu_usage())
        self.metrics["memory_percent"].append(self.get_memory_usage())
        self.metrics["gpu_load"].append(self.get_gpu_usage())
        self.metrics["gpu_memory"].append(self.get_gpu_memory())

    def generate_report(self):
        """Generate performance report"""
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

This appendix has provided comprehensive setup instructions for the primary simulation platforms used in humanoid robotics development. The key takeaways are:

- **Gazebo** offers the best integration with ROS 2 and is ideal for algorithm development and testing
- **NVIDIA Isaac Sim** provides photorealistic rendering and high-fidelity physics for perception research
- **Unity** enables rapid prototyping with its extensive asset ecosystem and C# scripting
- **Containerization** ensures reproducibility and simplifies deployment across different environments
- **Performance optimization** is essential for real-time simulation, particularly for learning-based approaches

With these simulation environments properly configured, you can proceed to implement the humanoid robotics algorithms described in the subsequent chapters, testing them in simulation before deployment to physical hardware.

---

**Next Steps:**

- Verify all installations by running the verification commands in each section
- Set up your preferred simulation environment for the specific chapters you plan to work through
- Review the performance monitoring tools to establish baseline metrics for your system
