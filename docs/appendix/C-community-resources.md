---
title: "Appendix C: Community Resources"
sidebar_position: 72
---

# Appendix C: Community Resources

This appendix provides a comprehensive collection of external resources, tools, and community links essential for physical AI and humanoid robotics development. These resources complement the concepts covered throughout this course and provide pathways for continued learning and professional development.

## C.1 ROS 2 Resources

The Robot Operating System 2 (ROS 2) is the foundational software framework for most modern robotics applications. The following resources will help you master ROS 2 and leverage its ecosystem effectively.

### C.1.1 Official Documentation

The official ROS 2 documentation serves as the authoritative reference for all aspects of the framework. The documentation is continuously updated and covers installation, concepts, tutorials, and API references for all LTS distributions.

| Resource | URL | Description |
|----------|-----|-------------|
| ROS 2 Documentation | https://docs.ros.org/en/humble/ | Official documentation for ROS 2 Humble Hawksbill (LTS) |
| ROS 2 Installation Guide | https://docs.ros.org/en/humble/Installation.html | Step-by-step installation instructions for all platforms |
| ROS 2 Concepts | https://docs.ros.org/en/humble/Concepts.html | Core concepts including nodes, topics, services, and actions |
| ROS 2 API Reference | https://docs.ros.org/en/humble/Concepts.html | Detailed API documentation for rclcpp and rclpy |
| ROS 2 Design Guide | https://docs.ros.org/en/humble/Design.html | Architectural decisions and design patterns |
| ROS 2 Quality Assurance | https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html | Client library comparison and quality standards |

:::tip Pro Tip

Always use the LTS distribution (currently Humble Hawksbill) for production applications. Non-LTS releases may have breaking changes and shorter support windows.

:::

### C.1.2 Tutorial Websites

Structured learning paths and hands-on tutorials are essential for mastering ROS 2. These platforms offer everything from beginner concepts to advanced robotics applications.

| Resource | URL | Focus Area |
|----------|-----|------------|
| ROS 2 Tutorials | https://docs.ros.org/en/humble/Tutorials.html | Official tutorials covering all core concepts |
| The Construct | https://www.theconstructsim.com/ | Interactive ROS 2 courses with simulation environments |
| ROS 2 Challenge | https://ros2challenge.github.io/ | Gamified learning with practical challenges |
| Robotics Backend | https://roboticsbackend.com/category/ros2/ | ROS 2 tutorials with Python focus |
| Learn ROS 2 | https://learn ROS2.com/ | Comprehensive video and written tutorials |

### C.1.3 Community Forums and Support

The ROS community is vibrant and supportive. These forums and channels are excellent for getting help, sharing knowledge, and staying updated on the latest developments.

| Resource | URL | Description |
|----------|-----|-------------|
| ROS Discourse | https://discourse.ros.org/ | Official community forum for discussions |
| ROS Answers | https://answers.ros.org/ | Q&A platform for troubleshooting help |
| ROS GitHub | https://github.com/ros/ | Source code, issue tracking, and contributions |
| ROS Discord | https://discord.gg/ros | Real-time chat community |
| r/ROS on Reddit | https://www.reddit.com/r/ROS/ | Community discussions and news |

:::note Important

When asking questions on ROS Answers or Discourse, always include your ROS distribution version, operating system, and a minimal reproducible example. This dramatically increases the likelihood of receiving helpful responses.

:::

### C.1.4 Essential ROS 2 Packages

The ROS ecosystem includes thousands of packages. The following are particularly relevant for humanoid robotics and physical AI applications.

| Package | URL | Purpose |
|---------|-----|---------|
| moveit2 | https://moveit.ros.org/ | Motion planning, manipulation, and collision checking |
| navigation2 | https://navigation.ros.org/ | Autonomous navigation and localization |
| ros2_control | https://control.ros.org/ | Hardware abstraction and control framework |
| ros2_controllers | https://github.com/ros-controls/ros2_controllers | Collection of robot controllers |
| gazebo_ros_pkgs | https://github.com/ros-simulation/gazebo_ros_pkgs | Gazebo simulator integration |
| robot_state_publisher | https://github.com/ros/robot_state_publisher | URDF-based state publishing |
| joint_state_publisher | https://github.com/ros/joint_state_publisher | Joint state visualization and control |
| xacro | https://github.com/ros/xacro | XML macro language for URDF |
| urdfdom | https://github.com/ros/urdfdom | URDF parsing and validation |

## C.2 NVIDIA Isaac Resources

NVIDIA Isaac provides a comprehensive platform for robotics simulation, AI training, and deployment. The following resources cover the full Isaac ecosystem.

### C.2.1 Official Documentation

NVIDIA provides extensive documentation for the Isaac platform, from basic setup to advanced AI training workflows.

| Resource | URL | Description |
|----------|-----|-------------|
| Isaac Sim Documentation | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/root.html | Main Isaac Sim documentation |
| Isaac Sim User Guide | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_guide.html | Getting started and core workflows |
| Isaac Gym Documentation | https://nvidia-omniverse.github.io/isaacgym-envs/ | Reinforcement learning environments |
| Isaac ROS | https://nvidia-isaac.github.io/ros2-nvidia/ | ROS 2 packages for Isaac |
| Isaac SDK | https://docs.nvidia.com/isaac-sdk/ | Isaac SDK documentation (legacy) |
| Isaac App Streaming | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/app_streaming.html | Cloud deployment options |

### C.2.2 Isaac Gym Resources

Isaac Gym provides high-performance GPU-accelerated reinforcement learning for robotics. These resources cover the gym framework and available environments.

| Resource | URL | Description |
|----------|-----|-------------|
| Isaac Gym Envs | https://github.com/NVIDIA-Omniverse/isaacgymenvs | Pre-built RL environments |
| Isaac Gym Documentation | https://nvidia-omniverse.github.io/isaacgym-envs/ | API reference and examples |
| Isaac Gym RL Examples | https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/tree/main/rsl_rl | RL training examples |
| Isaac Gym Preview Release | https://forums.developer.nvidia.com/c/isaac/isaac-gym/ | Community forum |

### C.2.3 Isaac Sim Tutorials

Isaac Sim tutorials cover everything from basic scene creation to complex robotics workflows. These tutorials are essential for learning to use the simulator effectively.

| Resource | URL | Topics Covered |
|----------|-----|----------------|
| Isaac Sim Samples | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/isaac_sim_samples.html | Pre-built simulation examples |
| Isaac Sim Python Scripting | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/python_snippets.html | Programmatic scene creation |
| Isaac Sim Tutorials | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_intro.html | Step-by-step tutorials |
| Isaac Orbit | https://github.com/NVIDIA-Omniverse/Isaac-SIM-Orbit | Interactive orbit tutorials |

### C.2.4 Example Projects and Repositories

Open-source examples demonstrate best practices and provide starting points for your own projects.

| Repository | URL | Description |
|------------|-----|-------------|
| Isaac ROS Navigation | https://github.com/NVIDIA-Omniverse/isaac_ros_navigation | ROS 2 navigation stack for Isaac |
| Isaac ROS Manipulation | https://github.com/NVIDIA-Omniverse/isaac_ros_manipulation | Manipulation pipeline examples |
| Isaac ROS Image Pipeline | https://github.com/NVIDIA-Omniverse/isaac_ros_image_pipeline | Camera processing stack |
| Isaac ROS Object Detection | https://github.com/NVIDIA-Omniverse/isaac_ros_object_detection | AI-based object detection |
| Isaac Orbit | https://github.com/NVIDIA-Omniverse/Isaac-SIM-Orbit | Interactive tutorials repository |

## C.3 Humanoid Robotics Resources

Humanoid robotics is a specialized field with unique challenges. The following resources cover research, open-source projects, and industry developments.

### C.3.1 Research Paper Repositories

Staying current with research is essential in this rapidly evolving field. These repositories provide access to the latest publications.

| Resource | URL | Description |
|----------|-----|-------------|
| arXiv Robotics | https://arxiv.org/list/cs.RO/recent | Latest robotics research papers |
| IEEE Xplore | https://ieeexplore.ieee.org/search/advanced | Peer-reviewed robotics publications |
| ACM Digital Library | https://dl.acm.org/journals/tor | ACM transactions on robotics |
| Google Scholar | https://scholar.google.com/scholar?q=humanoid+robotics | Comprehensive academic search |
| Papers With Code | https://paperswithcode.com/area/robotics | Papers with implementations |

### C.3.2 Key Research Labs

Leading research labs push the boundaries of humanoid robotics. Their publications and open-source contributions are invaluable resources.

| Lab | Institution | URL | Focus Areas |
|-----|-------------|-----|-------------|
| CSAIL | MIT | https://www.csail.mit.edu/ | Humanoid locomotion, manipulation |
| HRP Lab | AIST | https://www.dm.kyoto-u.ac.jp/hrp-ri-en/ | Humanoid platform development |
| JSK Lab | University of Tokyo | https://www.jsk.t.u-tokyo.ac.jp/ | Humanoid and legged robots |
| DLR | German Aerospace Center | https://www.dlr.de/rm/en/ | Space robotics, humanoid systems |
| IIT | Italian Institute of Technology | https://www.iit.it/ | Whole-body control, compliant actuation |
| Boston Dynamics | Boston Dynamics | https://www.bostondynamics.com/ | Atlas, Spot, commercial humanoids |
| Agility Robotics | Agility | https://agilityrobotics.com/ | Cassie, Digit bipedal robots |

### C.3.3 Open-Source Humanoid Projects

Several humanoid robot platforms have open-source software, enabling research and development without proprietary constraints.

| Project | Robot | URL | License |
|---------|-------|-----|---------|
| Open Humanoid | Open Humanoid | https://github.com/openhumanoid | BSD |
| ROS Humanoid | Multiple | https://github.com/ros-humanoid | Apache 2.0 |
| Humanoid Control | MuJoCo | https://github.com/ahundt/humanoid-control | MIT |
| Valkyrie | NASA | https://github.com/NASA-JSC-Robotics/valkyrie | NASA Open Source |
| Talos | PAL Robotics | https://github.com/pal-robotics/talos_robot | GPL |

## C.4 Simulation Resources

Simulation is critical for developing and testing robotics systems safely. These resources cover major simulators and related tools.

### C.4.1 Gazebo Resources

Gazebo is the most widely used open-source robotics simulator. Its resources cover installation, tutorials, and plugin development.

| Resource | URL | Description |
|----------|-----|-------------|
| Gazebo Sim | https://gazebosim.org/ | Main Gazebo website |
| Gazebo Documentation | https://gazebosim.org/docs | Official documentation |
| Gazebo Tutorials | https://gazebosim.org/tutorials | Step-by-step tutorials |
| Gazebo Models | https://gazebosim.org/models | Model database |
| Gazebo Answers | https://answers.gazebosim.org/ | Community Q&A |
| Gazebo Fuel | https://fuel.gazebosim.org/ | Model and world repository |

### C.4.2 Unity Robotics Resources

Unity provides a powerful game engine-based simulation platform with extensive robotics support.

| Resource | URL | Description |
|----------|-----|-------------|
| Unity Robotics Hub | https://unity-robotics.com/ | Main Unity robotics portal |
| Unity Robotics ROS | https://github.com/Unity-Technologies/Unity-Robotics-Hub | ROS integration packages |
| Unity ML-Agents | https://github.com/Unity-Technologies/ml-agents | Reinforcement learning toolkit |
| Unity Perception | https://github.com/Unity-Technologies/Unity-Perception | Synthetic data generation |

### C.4.3 URDF and XACRO Resources

The Unified Robot Description Format (URDF) is essential for robot modeling in ROS. These resources cover URDF creation and advanced modeling techniques.

| Resource | URL | Description |
|----------|-----|-------------|
| URDF Documentation | https://docs.ros.org/en/humble/Concepts/About-URDF.html | URDF concepts and syntax |
| URDF XML Reference | http://wiki.ros.org/urdf/XML | Complete XML element reference |
| XACRO Documentation | https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro.html | XACRO macro usage |
| URDF Viewer | https://github.com/ros/urdfdom | URDF validation tools |
| SolidWorks to URDF | https://wiki.ros.org/sw_urdf_exporter | CAD export plugin |

:::tip Pro Tip

Use XACRO for complex robot descriptions. It allows parameterization, macros, and includes, making URDF files maintainable and reusable across different robot configurations.

:::

### C.4.4 MuJoCo Resources

MuJoCo is a physics engine optimized for robotics and biomechanics, widely used for humanoid simulation.

| Resource | URL | Description |
|----------|-----|-------------|
| MuJoCo Documentation | https://mujoco.readthedocs.io/ | Official documentation |
| MuJoCo Python Binding | https://mujoco.readthedocs.io/en/stable/Python.html | mujoco_py and mujoco |
| OpenAI Gym MuJoCo | https://github.com/openai/gym/tree/master/gym/envs/mujoco | RL environments |
| DeepMind Control Suite | https://github.com/deepmind/dm_control | Physics-based control tasks |

## C.5 Learning Platforms

Formal and informal learning resources help build skills at every level. These platforms offer structured courses and self-paced learning.

### C.5.1 Online Courses

University-level courses provide comprehensive foundations in robotics, AI, and control theory.

| Course | Platform | URL | Level |
|--------|----------|-----|-------|
| Introduction to Robotics | Stanford | https://see.stanford.edu/course/cs223a | Intermediate |
| Underactuated Robotics | MIT OCW | https://ocw.mit.edu/6-832 | Advanced |
| Robot Learning | Stanford | https://web.stanford.edu/class/cs287/ | Advanced |
| Deep Learning for Robotics | NVIDIA | https://www.nvidia.com/en-us/learning/ | Intermediate |
| Modern Robotics | Northwestern | https://modernrobotics.northwestern.edu/ | Intermediate |
| Robotics: Aerial Robotics | Coursera | https://www.coursera.org/learn/robotics-flight | Beginner |

### C.5.2 YouTube Channels

Video content provides visual explanations and demonstrations of robotics concepts.

| Channel | URL | Content Focus |
|---------|-----|---------------|
| Robotics Back-End | https://www.youtube.com/@RoboticsBackend | ROS tutorials, robot projects |
| Articulated Robotics | https://www.youtube.com/@ArticulatedRobotics | ROS 2, hardware builds |
| The Construct | https://www.youtube.com/@TheConstructSim | Simulation tutorials |
| Boston Dynamics | https://www.youtube.com/@BostonDynamics | Research demos, product videos |
| NVIDIA Robotics | https://www.youtube.com/@NVIDIARobotics | Isaac Sim tutorials |
| 3Blue1Brown | https://www.youtube.com/@3blue1brown | Math foundations (neural networks, linear algebra) |

### C.5.3 Books and Publications

Comprehensive books provide in-depth coverage of robotics theory and practice.

| Title | Author | URL | Topics |
|-------|--------|-----|--------|
| Robotics: Modelling, Planning and Control | Siciliano et al. | https://link.springer.com/book/10.1007/978-1-84628-642-1 | Robot dynamics, control |
| Introduction to Robotics: Mechanics and Control | Craig | https://www.pearson.com/en-us/subject-catalog/p/introduction-to-robotics-mechanics-and-control/P270000374307 | Kinematics, dynamics |
| Modern Robotics: Mechanics, Planning, and Control | Lynch & Park | https://modernrobotics.org/ | Comprehensive coverage |
| Robot Modeling and Control | Spong et al. | https://www.wiley.com/en-us/Robot+Modeling+and+Control-p-9780471649908 | Modeling, control |
| Underactuated Robotics | Tedrake | https://underactuated.csail.mit.edu/ | Advanced control theory |
| Reinforcement Learning: An Introduction | Sutton & Barto | http://incompleteideas.net/book/the-book.html | RL foundations |

### C.5.4 Practice Platforms

Hands-on practice is essential for developing robotics skills. These platforms provide challenges and environments for skill development.

| Platform | URL | Description |
|----------|-----|-------------|
| ROS Challenge | https://roschallenge.github.io/ | ROS development challenges |
| OpenAI Gym | https://gymnasium.farama.org/ | RL environment suite |
| PyBullet | https://pybullet.org/wordpress/ | Physics simulation for RL |
| Roboschool | https://github.com/openai/roboschool | Open-source robot simulation |
| AI2 THOR | https://github.com/allenai/ai2thor | Indoor robot navigation |

## C.6 Hardware Vendors

Physical hardware is essential for real-world robotics development. These vendors provide robots, sensors, and components.

### C.6.1 Robot Manufacturers

Commercial humanoid and legged robots are available from several manufacturers for research and development.

| Manufacturer | Products | URL | Notes |
|--------------|----------|-----|-------|
| Unitree Robotics | H1, G1, Go2 | https://www.unitree.com/ | Quadrupeds and humanoids |
| Boston Dynamics | Atlas, Spot | https://www.bostondynamics.com/ | Advanced research platforms |
| Agility Robotics | Digit, Cassie | https://agilityrobotics.com/ | Bipedal delivery robots |
| PAL Robotics | Talos, REEM-C | https://www.pal-robotics.com/ | Research humanoids |
| Robotis | OP3, Darwin-OP | https://en.robotis.com/ | Educational humanoid kits |
| Fourier Intelligence | GR-1 | https://www.fourierintelligence.com/ | General-purpose humanoid |
| Sanctuary AI | Phoenix | https://www.sanctuary.ai/ | General-purpose humanoid |

### C.6.2 Sensor Vendors

Perception is critical for humanoid robots. These vendors provide cameras, LIDARs, IMUs, and force sensors.

| Vendor | Product Types | URL | Specialties |
|--------|---------------|-----|-------------|
| Intel RealSense | Depth cameras | https://www.intel.com/content/www/us/en/architecture-and-technology/depth.html | RGB-D, tracking |
| Stereolabs | Zed cameras | https://www.stereolabs.com/ | Stereo depth, AI |
| Ouster | LIDAR sensors | https://ouster.com/ | 3D LiDAR scanners |
| Hokuyo | LIDAR sensors | https://www.hokuyo-aut.jp/ | 2D/3D LIDAR |
| VectorNav | IMUs, VNAs | https://www.vectornav.com/ | High-performance IMUs |
| Analog Devices | IMUs | https://www.analog.com/en/products/mems.html | MEMS IMUs |
| ATI Industrial | Force/torque sensors | https://www.ati-ia.com/ | Industrial F/T sensors |
| Robotiq | Grippers, F/T sensors | https://robotiq.com/ | Manipulation sensors |

### C.6.3 Component Suppliers

Motors, actuators, and mechanical components are essential for custom robot builds.

| Vendor | Components | URL | Notes |
|--------|------------|-----|-------|
| Maxon Motor | DC motors, gearheads | https://www.maxongroup.com/ | Precision motors |
| Harmonic Drive | Strain wave gears | https://www.harmonicdrive.net/ | Zero-backlash gears |
| Wittenstein | Precision gears | https://www.wittenstein-us.com/ | High-torque gears |
| Pololu | Motors, controllers | https://www.pololu.com/ | Hobbyist components |
| ServoCity | Motion components | https://www.servocity.com/ | Modular actuation |
| Enful | Brushless motors | https://www.enful.com/ | Robot-specific motors |
| T-Motor | Motors, ESCs | https://www.tmotor.com/ | Drone and robot motors |

## C.7 Career and Research

Building a career in humanoid robotics requires networking, continuous learning, and professional development.

### C.7.1 Research Labs and Groups

Leading research groups provide opportunities for collaboration and career development.

| Lab | URL | Focus Areas |
|-----|-----|-------------|
| CSAIL Robotics | https://www.csail.mit.edu/research/robotics | AI, locomotion, manipulation |
| Stanford AI Lab | https://ai.stanford.edu/research/robotics/ | Learning, perception |
| CMU Robotics Institute | https://www.ri.cmu.edu/ | Comprehensive robotics research |
| ETH Zurich Autonomous Systems | https://asl.ethz.ch/ | Aerial, legged robots |
| Oxford Robotics Institute | https://ori.ox.ac.uk/ | Autonomous systems |
| TU Delft Robotics | https://www.tudelft.nl/en/3me/about-department/robotics-institute | Human-robot interaction |
| WPI Robotics | https://www.wpi.edu/robotics | Legged locomotion |

### C.7.2 Conferences

Conferences provide opportunities to present research, network, and learn about the latest developments.

| Conference | Typical Date | URL | Focus |
|------------|--------------|-----|-------|
| ICRA | May-June | https://www.ieee-icra.org/ | General robotics |
| IROS | September-October | https://www.iros.org/ | Intelligent robots |
| RSS | June-July | https://www.robotics-science.org/ | Robotics science |
| Humanoids | October-November | https://humanoids.org/ | Humanoid robots |
| CLAWAR | September | https://www.clawar.org/ | Climbing and walking |
| RoboCup | July | https://www.robocup.org/ | Robot competitions |
| CoRL | October-November | https://corl2024.org/ | Learning for robotics |

### C.7.3 Job Boards and Opportunities

Finding positions in robotics requires knowing where to look. These resources list robotics-specific opportunities.

| Resource | URL | Description |
|----------|-----|-------------|
| IEEE Robotics Jobs | https://jobs.ieee.org/jobs/keyword/robotics | IEEE job board |
| Robotics Jobs | https://www.roboticstojobs.com/ | Robotics-specific listings |
| LinkedIn Robotics | https://www.linkedin.com/jobs/robotics-jobs/ | Professional network |
| WeWorkInRobotics | https://www.weworkinrobotics.com/ | Robotics company directory |
| Robohub Jobs | https://robohub.org/jobs/ | Research and industry jobs |
| Crunchbase | https://www.crunchbase.com/discover/organization.investors | Startup funding research |

### C.7.4 Internship Programs

Internships provide valuable experience in humanoid robotics. Many companies and labs offer structured programs.

| Organization | Program | URL | Eligibility |
|--------------|---------|-----|-------------|
| Boston Dynamics | Internships | https://www.bostondynamics.com/careers | Students |
| NVIDIA | Robotics Internships | https://www.nvidia.com/en-us/careers/ | Various levels |
| iRobot | Internship Program | https://careers.irobot.com/ | Engineering students |
| NASA JPL | Robotics Internships | https://www.jpl.nasa.gov/education/internships/ | US students |
| Toyota Research Institute | TRI Interns | https://www.tri.global/careers/ | Graduate students |
| Bosch | Robotics Research | https://www.bosch.com/careers/ | Various levels |
| Willow Garage | Legacy (historical) | https://www.willowgarage.com/ | Pioneering lab |

## C.8 Additional Resources

### C.8.1 Data Sets and Benchmarks

Standardized datasets enable benchmarking and reproducibility in robotics research.

| Dataset | URL | Description |
|---------|-----|-------------|
| KITTI | http://www.cvlibs.net/datasets/kitti/ | Autonomous driving, computer vision |
| COCO | https://cocodataset.org/ | Object detection, segmentation |
| BDD100K | https://bdd-data.berkeley.edu/ | Diverse driving dataset |
| Amazon Robotics | https://amazon-robotics.github.io/ | Warehouse automation |
| PartNet | https://partnet.cs.stanford.edu/ | Object part hierarchy |
| ShapeNet | https://shapenet.org/ | 3D model repository |

### C.8.2 Software Development Tools

Development tools and utilities for robotics software engineering.

| Tool | URL | Purpose |
|------|-----|---------|
| Git | https://git-scm.com/ | Version control |
| Docker | https://www.docker.com/ | Containerization |
| VS Code | https://code.visualstudio.com/ | IDE with ROS extension |
| GitHub Codespaces | https://github.com/features/codespaces | Cloud development |
| GitLab CI | https://docs.gitlab.com/ci/ | Continuous integration |
| Colab | https://colab.research.google.com/ | Free GPU notebooks |

### C.8.3 ROS Distribution Timeline

Understanding ROS distribution lifecycle helps with project planning.

| Distribution | Release Date | End of Life | Status |
|--------------|--------------|-------------|--------|
| ROS 2 Foxy | June 2020 | December 2023 | End of life |
| ROS 2 Galactic | May 2021 | December 2022 | End of life |
| ROS 2 Humble | May 2022 | May 2027 | LTS |
| ROS 2 Iron | May 2023 | November 2024 | End of life |
| ROS 2 Jazzy | May 2024 | May 2029 | LTS |
| ROS 2 Kinetic (ROS 1) | May 2016 | April 2021 | End of life |
| ROS 2 Noetic (ROS 1) | May 2020 | May 2025 | End of life |

:::note Important

Always verify the support status of a ROS distribution before starting a production project. Long-term support (LTS) distributions like Humble and Jazzy receive five years of maintenance.

:::

---

## References

- ROS 2 Official Documentation: https://docs.ros.org/en/humble/
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- Unitree Robotics: https://www.unitree.com/
- arXiv Robotics Category: https://arxiv.org/list/cs.RO/recent
- Gazebo Simulator: https://gazebosim.org/

---

**Appendix C** | [Appendix A: Hardware Specifications](appendix/hardware-specifications) | [Appendix D: Assessment Rubrics](appendix/D-assessment-rubrics)
