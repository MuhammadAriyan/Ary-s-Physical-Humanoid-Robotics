---
title: "Gazebo and Unity Simulation"
sidebar_position: 3
---

# Chapter 3: Gazebo and Unity Simulation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the importance of robot simulation in development workflows
- Configure Gazebo simulation environments for humanoid robot testing
- Create URDF and SDF descriptions for robot models with sensors
- Configure physics simulation including joints, constraints, and collision properties
- Implement sensor simulation for cameras, LIDAR, and IMUs
- Use Unity for high-fidelity robot visualization and rendering
- Integrate simulation with ROS 2 for complete development workflows

## 3.1 Introduction to Robot Simulation

Robot simulation has become an indispensable tool in the development of humanoid robotics. Before diving into the technical aspects of simulation setup and configuration, it is essential to understand why simulation matters and how it fits into the overall development workflow for physical humanoid robots.

### Why Simulate?

The development of humanoid robots involves significant risks and costs that simulation helps mitigate. Physical testing on real hardware can result in equipment damage, personal injury, and substantial downtime while repairs are made. A humanoid robot with 30+ degrees of freedom, sophisticated sensors, and complex control systems represents a significant financial investment, often costing hundreds of thousands of dollars. Simulation allows developers to test algorithms, validate control strategies, and iterate on designs without risking this expensive hardware.

Beyond risk mitigation, simulation enables development scenarios that would be impractical or impossible with physical robots. Testing robot behavior in hazardous environments, extreme temperatures, or space conditions requires simulation. Reproducing edge cases and failure modes for validation and certification becomes straightforward in simulation. Researchers can explore robot learning through reinforcement learning, testing millions of iterations that would take years on physical hardware.

Simulation also democratizes robot development by reducing the physical infrastructure requirements. Teams without access to well-equipped robotics laboratories can still develop and test algorithms. This accessibility accelerates innovation and allows more researchers and developers to contribute to the field.

### Simulation Fidelity Considerations

Not all simulations are created equal, and choosing the appropriate level of fidelity depends on the development stage and objectives. Low-fidelity simulation prioritizes speed and scalability, suitable for algorithm development, integration testing, and rapid prototyping. High-fidelity simulation focuses on physical accuracy, sensor realism, and environmental detail, essential for final validation, perception algorithm development, and hardware-in-the-loop testing.

Gazebo strikes an effective balance between fidelity and performance, offering configurable physics engines, realistic sensor models, and efficient computation. For humanoid robotics, Gazebo has become the standard simulation environment due to its ROS integration, extensive sensor model library, and active community support. Unity complements Gazebo by providing superior rendering capabilities for visualization, marketing materials, and human-robot interaction studies.

### The Simulation Development Workflow

Effective simulation integration follows a staged approach that aligns with hardware readiness and development maturity. In the early stages, when algorithms are being developed and concepts validated, simulation provides the primary development environment. Hardware-in-the-loop testing begins once basic functionality is verified, using simulation alongside physical robot testing. As development matures, simulation serves increasingly for regression testing, edge case validation, and continuous integration.

This workflow requires maintaining consistency between simulation and physical robot implementations. Changes to sensor configurations, actuator characteristics, or physical parameters must propagate to both environments. Establishing clear interfaces and version control practices prevents drift between simulated and physical systems.

## 3.2 Gazebo Simulation Environment Setup

Gazebo's architecture separates the simulation server from visualization clients, allowing multiple users to observe and interact with the same simulation instance. Understanding this architecture helps in configuring efficient development environments and troubleshooting issues.

### Installation and Configuration

Gazebo installation on Ubuntu 22.04 follows a straightforward process through the osrfoundation repository:

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install gnupg
sudo wget https://packages.osrfoundation.org/gazebo/keys/packages.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'

# Install Gazebo Harmonic (current recommended version)
sudo apt-get update
sudo apt-get install gz-harmonic ros-humble-gazebo-ros-pkgs

# Verify installation
gz sim --version
```

The simulation requires careful resource allocation for smooth operation with humanoid models. The physics simulation step size, typically 1 millisecond for accurate dynamics, determines computational requirements. A simulation step requires calculating positions, velocities, and accelerations for all joints, resolving contacts, and updating sensor readings. Humanoid robots with 30+ joints, multiple contact points, and several sensors can strain single-core computation.

For complex humanoid simulations, consider configuring multi-threaded physics or distributed simulation. Gazebo supports running physics in a separate thread from rendering, reducing visualization impact on simulation accuracy. The following configuration demonstrates thread separation and resource allocation:

```xml
<!-- ~/.gz/sim/config/humanoid_simulation.config -->
<sdf version="1.10">
  <world name="humanoid_world">
    <physics name="multi-threaded-physics" default="true" type="omp">
      <engine name="dart">
        <max_step_size>0.001</max_step_size>
        <real_time_factor>1.0</real_time_factor>
        <real_time_update_rate>1000</real_time_update_rate>
        <num_threads>4</num_threads>
      </engine>
    </physics>
    <gui>
      <camera name="main_camera">
        <pose>2.0 2.0 1.5 0 0.3 2.5</pose>
      </camera>
    </gui>
  </world>
</sdf>
```

### Environment Configuration

Creating appropriate simulation environments involves configuring gravity, lighting, ground plane, and atmospheric conditions. Humanoid robots require careful ground contact modeling, as walking dynamics depend critically on friction and contact resolution.

```xml
<!-- humanoid_world.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="humanoid_simulation">
    <!-- Physics configuration for stable walking simulation -->
    <physics name="dart" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.81</gravity>
      <dart name="dart">
        <solver>
          <type>dantzig</type>
          <dtosolver_iterations>30</dtosolver_iterations>
          <sor_iterations>50</sor_iterations>
          <use_adaptive_time_stepping>true</use_adaptive_time_stepping>
        </solver>
      </dart>
    </physics>

    <!-- Ground plane with friction configuration for walking -->
    <model name="ground_plane">
      <pose>0 0 0 0 0 0</pose>
      <link name="ground">
        <collision name="ground_collision">
          <geometry>
            <plane>
              <size>20 20</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
                <fdir1>0 0 1</fdir1>
                <slip1>0.0</slip1>
                <slip2>0.0</slip2>
              </ode>
            </friction>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <contact_cfm>0.0</contact_cfm>
              <contact_erp>0.2</contact_erp>
            </contact>
          </surface>
        </collision>
        <visual name="ground_visual">
          <geometry>
            <plane>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Gray</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Lighting configuration for perception testing -->
    <light name="sun" type="directional">
      <pose>5 5 10 0 0.5 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>true</cast_shadows>
      <intensity>1.0</intensity>
      <direction>0.1 -0.1 -1</direction>
    </light>

    <!-- Include sensor models from Gazebo model database -->
    <include>
      <uri>model://camera</uri>
      <name>head_camera</uri>
    </include>
  </world>
</sdf>
```

## 3.3 URDF and SDF Robot Description Formats

Robot description formats provide the mathematical representation of robot structure for simulation, visualization, and motion planning. Understanding both URDF and SDF, their strengths, and appropriate use cases is essential for humanoid robot development.

### Unified Robot Description Format (URDF)

URDF uses XML to describe robot kinematics and visual properties. The format organizes robots as a tree of links connected by joints, where links represent rigid bodies and joints represent connections allowing relative motion.

```xml
<!-- humanoid.urdf.xacro -->
<?xml version="1.0" ?>
<robot name="atlas_humanoid" xmlns:xacro="http://wiki.ros.org/xacro">
  <!-- Include macros and constants -->
  <xacro:include filename="$(find humanoid_description)/urdf/constants.urdf.xacro" />
  <xacro:include filename="$(find humanoid_description)/urdf/materials.urdf.xacro" />

  <!-- Base link - world connection -->
  <link name="base_link">
    <inertial>
      <mass value="${base_mass}" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="${base_ixx}" ixy="${base_ixy}" ixz="${base_ixz}"
               iyy="${base_iyy}" iyz="${base_iyz}" izz="${base_izz}" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}" />
      </geometry>
      <material name="base_color" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="${base_width} ${base_depth} ${base_height}" />
      </geometry>
    </collision>
  </link>

  <!-- Pelvis link - central body reference -->
  <link name="pelvis_link">
    <inertial>
      <mass value="${pelvis_mass}" />
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <inertia ixx="${pelvis_ixx}" ixy="0" ixz="0"
               iyy="${pelvis_iyy}" iyz="0" izz="${pelvis_izz}" />
    </inertial>
    <visual>
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${pelvis_radius}" length="${pelvis_length}" />
      </geometry>
      <material name="pelvis_color" />
    </visual>
    <collision>
      <origin xyz="0 0 ${pelvis_z_offset}" rpy="0 0 0" />
      <geometry>
        <cylinder radius="${pelvis_radius}" length="${pelvis_length}" />
      </geometry>
    </collision>
  </link>

  <!-- Continuous joint for yaw rotation -->
  <joint name="base_to_pelvis" type="fixed">
    <origin xyz="0 0 ${pelvis_height}" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="pelvis_link" />
    <axis xyz="0 0 1" />
  </joint>

  <!-- Left hip yaw joint -->
  <joint name="left_hip_yaw" type="revolute">
    <origin xyz="0 ${hip_separation/2} 0" rpy="0 0 0" />
    <parent link="pelvis_link" />
    <child link="left_hip_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- Left hip link -->
  <link name="left_hip_link">
    <inertial>
      <mass value="2.5" />
      <origin xyz="0 0.05 -0.1" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.02" iyz="0.001" izz="0.01" />
    </inertial>
    <visual>
      <geometry>
        <box size="0.15 0.12 0.25" />
      </geometry>
      <material name="hip_color" />
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.12 0.25" />
      </geometry>
    </collision>
  </link>

  <!-- Left hip roll joint -->
  <joint name="left_hip_roll" type="revolute">
    <origin xyz="0 0 -0.15" rpy="0 0 0" />
    <parent link="left_hip_link" />
    <child link="left_upper_leg_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.5" upper="0.5" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- Left upper leg link -->
  <link name="left_upper_leg_link">
    <inertial>
      <mass value="5.0" />
      <origin xyz="0 0 -0.25" rpy="0 0 0" />
      <inertia ixx="0.05" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.05" />
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.08" length="0.50" />
      </geometry>
      <material name="leg_color" />
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.08" length="0.50" />
      </geometry>
    </collision>
  </link>

  <!-- Left knee joint -->
  <joint name="left_knee" type="revolute">
    <origin xyz="0 0 -0.50" rpy="0 0 0" />
    <parent link="left_upper_leg_link" />
    <child link="left_lower_leg_link" />
    <axis xyz="1 0 0" />
    <limit lower="-2.0" upper="0" effort="100" velocity="8.0" />
    <dynamics damping="0.3" friction="1.5" />
  </joint>

  <!-- Left lower leg link -->
  <link name="left_lower_leg_link">
    <inertial>
      <mass value="3.0" />
      <origin xyz="0 0 -0.25" rpy="0 0 0" />
      <inertia ixx="0.03" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.03" />
    </inertial>
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.50" />
      </geometry>
      <material name="leg_color" />
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.50" />
      </geometry>
    </collision>
  </link>

  <!-- Left ankle joint -->
  <joint name="left_ankle" type="revolute">
    <origin xyz="0 0 -0.50" rpy="0 0 0" />
    <parent link="left_lower_leg_link" />
    <child link="left_foot_link" />
    <axis xyz="1 0 0" />
    <limit lower="-0.5" upper="0.5" effort="80" velocity="6.0" />
    <dynamics damping="0.3" friction="1.5" />
  </joint>

  <!-- Left foot link -->
  <link name="left_foot_link">
    <inertial>
      <mass value="1.5" />
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.02" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.12 0.04" />
      </geometry>
      <material name="foot_color" />
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0" />
      <geometry>
        <box size="0.25 0.12 0.04" />
      </geometry>
    </collision>
  </link>

  <!-- Right leg mirrors left leg -->
  <joint name="right_hip_yaw" type="revolute">
    <origin xyz="0 ${-hip_separation/2} 0" rpy="0 0 0" />
    <parent link="pelvis_link" />
    <child link="right_hip_link" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="150" velocity="5.0" />
    <dynamics damping="0.5" friction="2.0" />
  </joint>

  <!-- Left shoulder joint -->
  <joint name="left_shoulder_pitch" type="revolute">
    <origin xyz="0 ${shoulder_x_offset} ${shoulder_z_offset}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="left_upper_arm_link" />
    <axis xyz="1 0 0" />
    <limit lower="-3.14" upper="3.14" effort="80" velocity="6.0" />
    <dynamics damping="0.3" friction="1.0" />
  </joint>

  <!-- IMU sensor link -->
  <link name="imu_link">
    <inertial>
      <mass value="0.05" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <inertia ixx="1e-5" ixy="0" ixz="0"
               iyy="1e-5" iyz="0" izz="1e-5" />
    </inertial>
  </link>

  <!-- IMU joint to pelvis -->
  <joint name="imu_joint" type="fixed">
    <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="imu_link" />
    <axis xyz="0 0 1" />
  </joint>

  <!-- Head link -->
  <link name="head_link">
    <inertial>
      <mass value="1.0" />
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <inertia ixx="0.01" ixy="0" ixz="0"
               iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.12" />
      </geometry>
      <material name="head_color" />
    </visual>
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0" />
      <geometry>
        <sphere radius="0.12" />
      </geometry>
    </collision>
  </link>

  <!-- Head joint -->
  <joint name="neck_joint" type="revolute">
    <origin xyz="0 0 ${torso_height/2}" rpy="0 0 0" />
    <parent link="torso_link" />
    <child link="head_link" />
    <axis xyz="0 1 0" />
    <limit lower="-1.0" upper="1.0" effort="20" velocity="3.0" />
    <dynamics damping="0.2" friction="0.5" />
  </joint>

  <!-- Transmissions for hardware interfaces -->
  <transmission name="left_hip_yaw_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_hip_yaw">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_hip_yaw_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="left_knee_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_knee">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_knee_motor">
      <mechanicalReduction>1.0</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

### Simulation Description Format (SDF)

SDF provides more comprehensive modeling capabilities than URDF, supporting nested models, scenes, plugins, and advanced physics. SDF is Gazebo's native format and should be used for complex simulation scenarios.

```xml
<!-- humanoid_complete.sdf -->
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="humanoid_simulation">
    <plugin name="ros2_interface" filename="libgazebo_ros2_control.so">
      <parameters>$(find humanoid_control)/config/controllers.yaml</parameters>
    </plugin>

    <model name="atlas_humanoid">
      <pose>0 0 1.0 0 0 0</pose>

      <!-- Joint state publisher plugin -->
      <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>joint_states:=joint_states</remapping>
        </ros>
        <publish_rate>100</publish_rate>
      </plugin>

      <!-- Link definitions with full inertial, visual, and collision -->
      <link name="pelvis">
        <pose>0 0 0.95 0 0 0</pose>
        <inertial>
          <mass>15.0</mass>
          <pose>0 0 0 0 0 0</pose>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.3</iyy>
            <iyz>0</iyz>
            <izz>0.5</izz>
          </inertia>
        </inertial>
        <visual name="pelvis_visual">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
        <collision name="pelvis_collision">
          <geometry>
            <cylinder>
              <radius>0.15</radius>
              <length>0.1</length>
            </cylinder>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>

      <!-- Left leg with full joint chain -->
      <link name="left_hip">
        <pose>0 0.12 0 0 0 0</pose>
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <iyy>0.05</iyy>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="left_hip_visual">
          <geometry>
            <box size="0.1 0.08 0.15</size>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/DarkGray</name>
            </script>
          </material>
        </visual>
      </link>

      <joint name="left_hip_yaw" type="revolute">
        <parent>pelvis</parent>
        <child>left_hip</child>
        <pose>0 0.12 0 0 0 0</pose>
        <axis>
          <xyz>0 0 1</xyz>
        </axis>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>150</effort>
          <velocity>5.0</velocity>
        </limit>
        <physics>
          <ode>
            <cfm>0.0</cfm>
            <erp>0.2</erp>
          </ode>
        </physics>
      </joint>

      <!-- Sensor plugins for the humanoid -->
      <link name="head_camera_link">
        <pose>0 0 0.25 0 -0.3 0</pose>
        <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>image_raw:=camera/image_raw</remapping>
            <remapping>camera_info:=camera/camera_info</remapping>
          </ros>
          <camera_name>head_camera</camera_name>
          <frame_name>head_camera_link</frame_name>
          <hack_baseline>0.07</hack_baseline>
        </plugin>
      </link>

      <!-- IMU sensor on pelvis -->
      <link name="imu_link">
        <pose>0 0 0.02 0 0 0</pose>
        <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>imu:=imu/data</remapping>
          </ros>
          <frame_name>imu_link</frame_name>
          <topic_name>/humanoid/imu/data</topic_name>
        </plugin>
      </link>

      <!-- Force/torque sensors at feet -->
      <link name="left_foot_link">
        <pose>0.05 0 -0.45 0 0 0</pose>
        <plugin name="ft_sensor" filename="libgazebo_ros_force_torque.so">
          <ros>
            <namespace>/humanoid</namespace>
            <remapping>force_torque:=left_foot/ft</remapping>
          </ros>
          <frame_name>left_foot_link</frame_name>
          <measure_direction>child_to_parent</measure_direction>
        </plugin>
      </link>
    </model>

    <!-- Environment elements -->
    <model name="obstacle_course">
      <pose>2.0 0 0 0 0 0</pose>
      <link name="box1">
        <pose>0 0 0.25 0 0 0</pose>
        <visual name="box1_visual">
          <geometry>
            <box size="0.5 0.5 0.5</size>
          </geometry>
        </visual>
        <collision name="box1_collision">
          <geometry>
            <box size="0.5 0.5 0.5</size>
          </geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## 3.4 Physics Simulation

Accurate physics simulation is fundamental to meaningful humanoid robot testing. This section covers joint dynamics, contact modeling, and constraint configuration for realistic simulation behavior.

### Joint Types and Configuration

Humanoid robots require various joint types to achieve their range of motion. Revolute joints provide single-axis rotation common in biological joints. Continuous joints allow unlimited rotation for neck and waist yaw. Prismatic joints enable linear motion for suspension systems. Fixed joints rigidly connect links that should not move relative to each other.

```yaml
# joint_config.yaml
# Joint configuration for humanoid robot simulation
# Reference: See Appendix B for physics engine configuration details

joints:
  # Lower body joints - high power for walking
  left_hip_yaw:
    type: revolute
    axis: [0, 0, 1]
    position:
      min: -1.57  # -90 degrees
      max: 1.57   # +90 degrees
    limits:
      effort: 150.0  # Nm
      velocity: 5.0   # rad/s
    dynamics:
      damping: 0.5    # Linear damping coefficient
      friction: 2.0   # Coulomb friction
    safety:
      k_position: 100.0
      k_velocity: 2.0

  left_hip_roll:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -0.5
      max: 0.5
    limits:
      effort: 150.0
      velocity: 5.0
    dynamics:
      damping: 0.5
      friction: 2.0

  left_knee:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -2.0  # Nearly straight to bent
      max: 0.0
    limits:
      effort: 100.0
      velocity: 8.0
    dynamics:
      damping: 0.3
      friction: 1.5
    # Stiffness for series elastic actuation
    stiffness: 500.0
    damping_isa: 10.0

  left_ankle_pitch:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -0.5
      max: 0.5
    limits:
      effort: 80.0
      velocity: 6.0
    dynamics:
      damping: 0.3
      friction: 1.5

  # Upper body joints - lower power but precision
  left_shoulder_pitch:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -3.14
      max: 3.14
    limits:
      effort: 80.0
      velocity: 6.0
    dynamics:
      damping: 0.3
      friction: 1.0

  left_elbow:
    type: revolute
    axis: [1, 0, 0]
    position:
      min: -2.5
      max: 0.0
    limits:
      effort: 50.0
      velocity: 8.0
    dynamics:
      damping: 0.2
      friction: 0.8

  neck_yaw:
    type: revolute
    axis: [0, 1, 0]
    position:
      min: -1.0
      max: 1.0
    limits:
      effort: 20.0
      velocity: 3.0
    dynamics:
      damping: 0.2
      friction: 0.5

# Contact parameters for foot-ground interaction
contact:
  foot:
    material: rubber
    friction:
      mu: 0.8        # Coefficient of friction
      mu2: 0.8       # Secondary friction direction
      slip1: 0.0     # Velocity-dependent slip
      slip2: 0.0
    contact:
      cfm: 0.0       # Constraint force mixing
      erp: 0.2       # Error reduction parameter
      max_contact_cfm: 1e-5
      max_contact_erp: 0.1
    bounce:
      restitution: 0.0  # No bouncing for walking
      threshold: 100.0

# ODE physics solver settings
physics:
  solver:
    type: dantzig  # Fast LCP solver
    iterations: 30    # LCP solver iterations
    sor: 1.0         # Successive over-relaxation
  contacts:
    max_contacts: 20  # Maximum contact points
    max_contacts_per_link: 5
```

### Contact and Friction Modeling

Realistic foot-ground interaction requires careful contact parameter tuning. The friction model determines whether the robot can walk without slipping, while restitution controls bounce behavior. For indoor environments, moderate friction with no restitution provides stable walking behavior.

```xml
<!-- contact_parameters.sdf -->
<sdf version="1.10">
  <world name="humanoid_simulation">
    <physics name="physics" type="dart">
      <dart>
        <collision_detector>bullet</collision_detector>
        <solver>
          <type>dantzig</type>
          <dtosolver_iterations>30</dtosolver_iterations>
          <sor_iterations>50</sor_iterations>
          <use_adaptive_time_stepping>true</use_adaptive_time_stepping>
        </solver>
      </dart>
    </physics>

    <!-- Custom friction model for humanoid walking -->
    <model name="custom_friction_floor">
      <link name="floor">
        <collision name="collision">
          <geometry>
            <plane>
              <size>50 50</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.9</mu>
                <mu2>0.9</mu2>
                <fdir1>1 0 0</fdir1>
                <slip1>0.01</slip1>
                <slip2>0.01</slip2>
              </ode>
              <torsional>
                <coefficient>0.1</coefficient>
                <patch_radius>0.1</patch_radius>
              </torsional>
            </friction>
            <bounce>
              <restitution_coefficient>0.0</restitution_coefficient>
              <threshold_velocity>0.5</threshold_velocity>
            </bounce>
            <contact>
              <collide_without_contact>0</collide_without_contact>
              <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
              <collide_bitmask>1</collide_bitmask>
              <contact_cfm>0.0</contact_cfm>
              <contact_erp>0.2</contact_erp>
            </contact>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

## 3.5 Sensor Simulation in Gazebo

Simulated sensors must produce data similar to physical sensors for algorithm development. This section covers camera, LIDAR, and IMU simulation with realistic noise models and configurations.

### Camera Simulation

Camera simulation includes lens distortion, exposure effects, and noise modeling. For humanoid applications, head-mounted cameras require proper pose tracking and synchronization with robot motion.

```xml
<!-- camera_sensor.sdf -->
<sdf version="1.10">
  <model name="head_camera">
    <link name="camera_link">
      <pose>0 0 0 0 -0.3 0</pose>

      <!-- Camera sensor plugin -->
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>image_raw:=camera/image_raw</remapping>
          <remapping>camera_info:=camera/camera_info</remapping>
          <remapping>depth_image:=camera/depth_image</remapping>
        </ros>

        <!-- Camera parameters matching physical sensor -->
        <camera_name>head_camera</camera_name>
        <frame_name>camera_link</frame_name>

        <!-- Image properties -->
        <image>
          <width>1280</width>
          <height>720</height>
          <format>R8G8B8</format>
        </image>

        <!-- Clip planes -->
        <clip>
          <near>0.1</near>
          <far>100.0</far>
        </clip>

        <!-- Lens distortion (Brown-Conrady model) -->
        <distortion>
          <k1>-0.2</k1>
          <k2>0.1</k2>
          <k3>0.0</k3>
          <p1>0.0</p1>
          <p2>0.0</p2>
          <center>0.5 0.5</center>
        </distortion>

        <!-- Camera intrinsics -->
        <intrinsics>
          <fx>800.0</fx>
          <fy>800.0</fy>
          <cx>640.0</cx>
          <cy>360.0</cy>
        </intrinsics>

        <!-- Noise model for realism -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </plugin>
    </link>
  </model>
</sdf>
```

### LIDAR Simulation

LIDAR sensors provide distance measurements essential for navigation and obstacle avoidance. Simulation must account for range limits, angular resolution, and measurement noise.

```xml
<!-- lidar_sensor.sdf -->
<sdf version="1.10">
  <model name="head_lidar">
    <link name="lidar_link">
      <pose>0 0 0.1 0 0 0</pose>

      <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>scan:=lidar/scan</remapping>
          <remapping>pointcloud:=lidar/points</remapping>
        </ros>

        <topic_name>/humanoid/lidar/scan</topic_name>
        <frame_name>lidar_link</frame_name>

        <!-- 2D LIDAR configuration -->
        <laser_scan>
          <sampling>360</sampling>
          <range>
            <min>0.1</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <angle>
            <min>-3.14159</min>
            <max>3.14159</max>
            <resolution>0.0087</resolution>  <!-- 0.5 degrees -->
          </angle>
        </laser_scan>

        <!-- Noise parameters -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>
        </noise>

        <!-- Multiple echo settings -->
        <num_echoes>2</num_echoes>
        <cadenary>true</cadenary>
      </plugin>
    </link>
  </model>
</sdf>
```

### IMU Simulation

IMU simulation must account for bias drift, scale factors, and cross-axis sensitivity. These imperfections are critical for algorithm development as they affect state estimation accuracy.

```xml
<!-- imu_sensor.sdf -->
<sdf version="1.10">
  <model name="pelvis_imu">
    <link name="imu_link">
      <pose>0 0 0.02 0 0 0</pose>

      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/humanoid</namespace>
          <remapping>imu:=imu/data</remapping>
        </ros>

        <frame_name>imu_link</frame_name>
        <topic_name>/humanoid/imu/data</topic_name>

        <!-- Accelerometer configuration -->
        <acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>  <!-- 2 mg RMS noise -->
            </noise>
            <bias>0.0</bias>
            <scale_factor>1.0</scale_factor>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.002</stddev>
            </noise>
            <bias>9.81</bias>  <!-- Gravity at rest -->
          </z>
        </acceleration>

        <!-- Gyroscope configuration -->
        <gyroscope>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>  <!-- 0.05 deg/s RMS -->
            </noise>
            <bias>0.001</bias>
            <scale_factor>1.0</scale_factor>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0005</stddev>
            </noise>
          </z>
        </gyroscope>

        <!-- Update rate -->
        <update_rate>200</update_rate>
      </plugin>
    </link>
  </model>
</sdf>
```

### Complete Sensor Integration

The following Python code demonstrates sensor integration with ROS 2:

```python
#!/usr/bin/env python3
"""
Humanoid Robot Sensor Integration for Simulation

This module provides sensor interfaces for reading simulated
sensor data in Gazebo and converting it to usable formats.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from geometry_msgs.msg import WrenchStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, List
import cv2
from cv_bridge import CvBridge


@dataclass
class SensorConfig:
    """Configuration for simulated sensors."""
    frame_id: str
    topic: str
    update_rate: float = 100.0
    noise_stddev: float = 0.0
    bias: float = 0.0


class HumanoidSensorInterface(Node):
    """
    Unified sensor interface for humanoid robot simulation.

    Provides standardized access to all simulated sensors including
    cameras, LIDAR, IMU, and force/torque sensors.
    """

    def __init__(self):
        super().__init__('humanoid_sensor_interface')

        # QoS profile for sensor data
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Camera configuration
        self.declare_parameter('camera.frame_id', 'head_camera_link')
        self.declare_parameter('camera.image_topic', '/humanoid/camera/image_raw')
        self.declare_parameter('camera.info_topic', '/humanoid/camera/camera_info')
        self.declare_parameter('camera.width', 1280)
        self.declare_parameter('camera.height', 720)

        # LIDAR configuration
        self.declare_parameter('lidar.frame_id', 'lidar_link')
        self.declare_parameter('lidar.topic', '/humanoid/lidar/scan')
        self.declare_parameter('lidar.min_range', 0.1)
        self.declare_parameter('lidar.max_range', 30.0)

        # IMU configuration
        self.declare_parameter('imu.frame_id', 'imu_link')
        self.declare_parameter('imu.topic', '/humanoid/imu/data')
        self.declare_parameter('imu.update_rate', 200.0)

        # Initialize CV bridge for image processing
        self.cv_bridge = CvBridge()

        # Sensor data storage
        self.latest_image: Optional[Image] = None
        self.latest_lidar: Optional[LaserScan] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_ft_left: Optional[WrenchStamped] = None
        self.latest_ft_right: Optional[WrenchStamped] = None

        # Initialize subscribers
        self._init_subscribers()

        self.get_logger().info("Humanoid Sensor Interface initialized")

    def _init_subscribers(self):
        """Initialize all sensor subscribers."""
        # Camera subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/humanoid/camera/image_raw',
            self.image_callback,
            self.sensor_qos
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/humanoid/camera/camera_info',
            self.info_callback,
            self.sensor_qos
        )

        # LIDAR subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/humanoid/lidar/scan',
            self.lidar_callback,
            self.sensor_qos
        )

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/humanoid/imu/data',
            self.imu_callback,
            self.sensor_qos
        )

        # Force/torque subscribers
        self.ft_left_sub = self.create_subscription(
            WrenchStamped,
            '/humanoid/left_foot/ft',
            self.ft_left_callback,
            self.sensor_qos
        )

        self.ft_right_sub = self.create_subscription(
            WrenchStamped,
            '/humanoid/right_foot/ft',
            self.ft_right_callback,
            self.sensor_qos
        )

    def image_callback(self, msg: Image):
        """Process incoming camera image."""
        self.latest_image = msg

    def info_callback(self, msg: CameraInfo):
        """Store camera calibration information."""
        self.camera_info = msg

    def lidar_callback(self, msg: LaserScan):
        """Process incoming LIDAR scan."""
        self.latest_lidar = msg

    def imu_callback(self, msg: Imu):
        """Process incoming IMU data."""
        self.latest_imu = msg

    def ft_left_callback(self, msg: WrenchStamped):
        """Process left foot force/torque data."""
        self.latest_ft_left = msg

    def ft_right_callback(self, msg: WrenchStamped):
        """Process right foot force/torque data."""
        self.latest_ft_right = msg

    def get_latest_image(self) -> Optional[np.ndarray]:
        """Get latest image as numpy array."""
        if self.latest_image is None:
            return None
        try:
            return self.cv_bridge.imgmsg_to_cv2(
                self.latest_image, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None

    def get_latest_scan(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Get latest LIDAR scan data.

        Returns:
            Tuple of (ranges, angles) arrays, or (None, None) if no data.
        """
        if self.latest_lidar is None:
            return None, None

        ranges = np.array(self.latest_lidar.ranges)
        angles = np.linspace(
            self.latest_lidar.angle_min,
            self.latest_lidar.angle_max,
            len(ranges)
        )

        return ranges, angles

    def get_latest_imu_reading(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Get latest IMU readings.

        Returns:
            Tuple of (linear_acceleration, angular_velocity) arrays.
        """
        if self.latest_imu is None:
            return None

        accel = np.array([
            self.latest_imu.linear_acceleration.x,
            self.latest_imu.linear_acceleration.y,
            self.latest_imu.linear_acceleration.z
        ])

        gyro = np.array([
            self.latest_imu.angular_velocity.x,
            self.latest_imu.angular_velocity.y,
            self.latest_imu.angular_velocity.z
        ])

        return accel, gyro

    def get_center_of_pressure(self) -> Tuple[float, float]:
        """
        Calculate center of pressure from foot force/torque sensors.

        Returns:
            Tuple of (cop_x, cop_y) in foot frame coordinates.
        """
        fx_l = fy_l = tz_l = 0.0
        fx_r = fy_r = tz_r = 0.0

        if self.latest_ft_left:
            fx_l = self.latest_ft_left.wrench.force.x
            fy_l = self.latest_ft_left.wrench.force.y
            tz_l = self.latest_ft_left.wrench.torque.z

        if self.latest_ft_right:
            fx_r = self.latest_ft_right.wrench.force.x
            fy_r = self.latest_ft_right.wrench.force.y
            tz_r = self.latest_ft_right.wrench.torque.z

        # Combined forces
        fx_total = fx_l + fx_r
        fz_total = fy_l + fy_r  # Y force corresponds to Z moment

        # Center of pressure calculation
        if abs(fx_total) > 1.0:  # Minimum force threshold
            cop_x = (tz_r - tz_l) / fx_total
        else:
            cop_x = 0.0

        if abs(fz_total) > 1.0:
            cop_y = (fx_r * 0.12 - fx_l * 0.12) / fz_total
        else:
            cop_y = 0.0

        return cop_x, cop_y


def main(args=None):
    """Run the sensor interface node."""
    rclpy.init(args=args)

    try:
        interface = HumanoidSensorInterface()
        rclpy.spin(interface)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 3.6 Unity for Robot Visualization

While Gazebo excels at physics simulation, Unity provides superior rendering capabilities for visualization, user interfaces, and photorealistic rendering. Integrating Unity with ROS 2 enables the best of both worlds: accurate physics and compelling visualization.

### ROS-Unity Integration Architecture

The integration between Unity and ROS 2 requires a communication bridge. The ROS TCP Endpoint package in Unity receives data from ROS topics, while the Unity-ROS bridge sends commands and receives state updates.

```csharp
// C# script for Unity - ROS 2 Connection Manager
using UnityEngine;
using ROS2;
using System.Collections.Generic;

public class ROSConnectionManager : MonoBehaviour
{
    // ROS 2 node and publisher handles
    private Node ros_node;
    private Publisher<JointStateMsg> joint_state_pub;
    private Publisher<OdometryMsg> odometry_pub;
    private Subscriber<JointTrajectoryMsg> trajectory_sub;

    // Joint state storage
    private float[] joint_positions;
    private float[] joint_velocities;
    private string[] joint_names;

    // Connection settings
    private string ros_domain = "0";
    private string ros_ip = "127.0.0.1";
    private int ros_port = 9090;

    void Start()
    {
        // Initialize ROS 2
        var options = new RCLdotnet().CreateNodeOptions(ros_domain);
        ros_node = new Node("unity_visualization", options);

        // Create publishers
        joint_state_pub = ros_node.CreatePublisher<JointStateMsg>(
            "/humanoid/joint_states"
        );
        odometry_pub = ros_node.CreatePublisher<OdometryMsg>(
            "/humanoid/odometry"
        );

        // Create subscriber for trajectory commands
        trajectory_sub = ros_node.CreateSubscriber<JointTrajectoryMsg>(
            "/humanoid/trajectory_command",
            TrajectoryCallback
        );

        // Initialize joint arrays
        InitializeJoints();

        Debug.Log("ROS-Unity connection established");
    }

    void InitializeJoints()
    {
        // Define all humanoid joint names
        joint_names = new string[] {
            "left_hip_yaw", "left_hip_roll", "left_hip_pitch",
            "left_knee", "left_ankle_pitch", "left_ankle_roll",
            "right_hip_yaw", "right_hip_roll", "right_hip_pitch",
            "right_knee", "right_ankle_pitch", "right_ankle_roll",
            "waist_yaw", "waist_pitch", "waist_roll",
            "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
            "left_elbow", "left_wrist_roll", "left_wrist_pitch",
            "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
            "right_elbow", "right_wrist_roll", "right_wrist_pitch",
            "neck_pitch", "neck_yaw"
        };

        joint_positions = new float[joint_names.Length];
        joint_velocities = new float[joint_names.Length];
    }

    void Update()
    {
        // Process ROS callbacks
        RCLdotnet.SpinOnce(ros_node, 0.01);

        // Read joint positions from Unity transforms
        UpdateJointPositions();

        // Publish joint states
        PublishJointStates();
    }

    void UpdateJointPositions()
    {
        // Read joint positions from Unity GameObjects
        for (int i = 0; i < joint_names.Length; i++)
        {
            var joint = GameObject.Find(joint_names[i]);
            if (joint != null)
            {
                joint_positions[i] = joint.transform.localRotation.eulerAngles.x;
                joint_velocities[i] = 0.0f;  // Would calculate from delta
            }
        }
    }

    void PublishJointStates()
    {
        var msg = new JointStateMsg();

        // Set header timestamp
        // Note: Would use actual ROS time in production
        msg.Header.Stamp.Sec = (int)Time.time;
        msg.Header.Stamp.Nanosec = (uint)((Time.time % 1.0) * 1e9);
        msg.Header.Frame_id = "world";

        msg.Name = joint_names;
        msg.Position = joint_positions;
        msg.Velocity = joint_velocities;
        msg.Effort = new double[joint_names.Length];  // Not available in Unity

        joint_state_pub.Publish(msg);
    }

    void TrajectoryCallback(JointTrajectoryMsg msg)
    {
        // Process incoming trajectory commands
        if (msg.Points.Length > 0)
        {
            var firstPoint = msg.Points[0];
            for (int i = 0; i < joint_names.Length && i < msg.JointNames.Length; i++)
            {
                int jointIndex = System.Array.IndexOf(joint_names, msg.JointNames[i]);
                if (jointIndex >= 0)
                {
                    // Apply position command to Unity joint
                    var joint = GameObject.Find(joint_names[jointIndex]);
                    if (joint != null && firstPoint.Positions.Length > i)
                    {
                        // Rotate joint to target position
                        var targetRot = Quaternion.Euler(
                            (float)(firstPoint.Positions[i] * Mathf.Rad2Deg),
                            0, 0
                        );
                        joint.transform.localRotation = targetRot;
                    }
                }
            }
        }
    }

    void OnDestroy()
    {
        // Cleanup ROS resources
        joint_state_pub.Dispose();
        odometry_pub.Dispose();
        trajectory_sub.Dispose();
        ros_node.Dispose();
    }
}
```

### Visualization Best Practices

Effective humanoid robot visualization requires attention to rendering quality, camera positioning, and animation smoothness. The following guidelines ensure professional-quality visualizations:

Camera setup should include multiple view angles: a third-person follow camera, an over-the-shoulder view for arm manipulation tasks, and a first-person view from the robot head. Smooth camera transitions using cinematographic techniques enhance the viewing experience.

Lighting should balance realism with clarity. HDRI environment lighting provides natural reflections and ambient illumination. Key, fill, and rim lighting setups emphasize robot form and silhouette. Dynamic lighting that responds to robot motion adds visual interest.

Material rendering should accurately represent robot materials: metallic finishes with appropriate roughness and clear coat, plastic components with subsurface scattering for translucency, and fabric or rubber materials with proper normal mapping.

## 3.7 Simulation Launch and Control

Complete simulation startup requires coordinating multiple components: the physics world, robot description, sensor plugins, and visualization tools. The following ROS 2 launch file provides a comprehensive example.

```python
#!/usr/bin/env python3
"""
Humanoid Robot Simulation Launch File

This launch file starts the complete simulation environment including
Gazebo, robot description, sensors, and visualization tools.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate the complete simulation launch description."""

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='atlas_v2',
        description='Robot model identifier'
    )

    world_file = DeclareLaunchArgument(
        'world',
        default_value='humanoid_lab.sdf',
        description='Gazebo world file'
    )

    gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Show Gazebo GUI'
    )

    rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Show RViz visualization'
    )

    # Package shares
    pkg_share = FindPackageShare('humanoid_gazebo')
    pkg_description = FindPackageShare('humanoid_description')
    pkg_control = FindPackageShare('humanoid_control')

    # Paths
    urdf_file = PathJoinSubstitution([
        pkg_description, 'urdf', 'humanoid.urdf.xacro'
    ])

    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', LaunchConfiguration('world')
    ])

    rviz_config = PathJoinSubstitution([
        pkg_control, 'rviz', 'humanoid_simulation.rviz'
    ])

    # Create launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(use_sim_time)
    ld.add_action(robot_model)
    ld.add_action(world_file)
    ld.add_action(gui)
    ld.add_action(rviz)

    # Gazebo server
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'physics': 'dart',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(gz_server)

    # Gazebo client
    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gzclient.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    ld.add_action(gz_client)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_file,
            'use_tf_static': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    ld.add_action(robot_state_publisher)

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'atlas_humanoid',
            '-file', urdf_file,
            '-robot_namespace', 'humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # Joint state publisher (Gazebo plugin)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen'
    )
    ld.add_action(joint_state_publisher)

    # IMU filter node
    imu_filter = Node(
        package='imu_tools',
        executable='imu_filter_node',
        name='imu_filter',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'world_frame': 'enu',
            'orientation_stddev': 0.1,
        }],
        remappings=[
            ('imu_in', '/humanoid/imu/data'),
            ('imu_out', '/humanoid/imu/filtered')
        ],
        output='screen'
    )
    ld.add_action(imu_filter)

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )
    ld.add_action(rviz_node)

    # Robot controller
    robot_controller = Node(
        package='humanoid_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[
            PathJoinSubstitution([pkg_control, 'config', 'control_params.yaml']),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )
    ld.add_action(robot_controller)

    return ld
```

## Chapter Summary

This chapter covered the essential aspects of robot simulation for humanoid robotics development:

1. **Simulation Importance**: Robot simulation enables safe algorithm development, cost-effective testing, and scenarios impossible with physical hardware.

2. **Gazebo Configuration**: Setting up simulation environments with appropriate physics engines, thread configurations, and resource allocation.

3. **Robot Description Formats**: URDF provides standardized robot description for kinematics and basic properties, while SDF offers more comprehensive modeling capabilities.

4. **Physics Simulation**: Joint dynamics, contact modeling, and friction configuration are critical for realistic humanoid behavior in simulation.

5. **Sensor Simulation**: Camera, LIDAR, and IMU simulation with realistic noise models prepares algorithms for physical deployment.

6. **Unity Visualization**: High-fidelity rendering complements physics simulation for visualization and communication purposes.

7. **Launch and Control**: Complete launch files coordinate all simulation components for reproducible development environments.

### Key Concepts

- **URDF/SDF**: Robot description formats providing kinematics, visual, and collision properties
- **Physics Engines**: DART, ODE, and Bullet provide different accuracy/performance tradeoffs
- **Sensor Plugins**: Gazebo plugins simulate sensor behavior with configurable noise
- **ROS Integration**: Seamless integration between ROS 2 and simulation environments
- **Visualization**: Unity provides photorealistic rendering complementary to Gazebo

### Hardware Requirements Reference

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| CPU | 4-core @ 2.5 GHz | 8-core @ 4.0 GHz |
| RAM | 8 GB | 32 GB DDR4 |
| GPU | Integrated | NVIDIA RTX 3060+ |
| Storage | 64 GB SSD | 256 GB NVMe SSD |
| Physics Rate | 500 Hz | 1000 Hz |

### Further Reading

- Gazebo Documentation: https://gazebosim.org/docs
- ROS 2 Control: https://control.ros.org/
- Unity Robotics Hub: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- DARwin-OP2 Documentation: https://github.com/ROBOTIS-GIT/ROBOTIS-Documents

### Next Chapter

Chapter 4 explores **Isaac Sim** for GPU-accelerated simulation, enabling high-performance simulation for training perception models and reinforcement learning with realistic rendering and physics.

---

**Part 2: ROS 2 Fundamentals** | [Weeks 3-5 Overview](part-2-ros2/02a-week-3-5-overview) | [Part 3: Simulation](part-3-simulation/gazebo-unity-simulation) | [Part 4: NVIDIA Isaac](part-4-isaac/nvidia-isaac-platform)
