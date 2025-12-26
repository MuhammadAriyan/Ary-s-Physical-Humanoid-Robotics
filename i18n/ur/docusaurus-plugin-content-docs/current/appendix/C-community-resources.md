---
title: "Appendix C: Community Resources"
sidebar_position: 72
---

# Appendix C: Community Resources (برادری کے ذرائع)

یہ ایپینڈکس physical AI اور humanoid robotics development کے لیے essential external resources، tools، اور community links کا جامع collection فراہم کرتا ہے۔ یہ resources اس کورس بھر میں cover کیے گئے concepts کو complement کرتے ہیں اور continued learning اور professional development کے لیے pathways فراہم کرتے ہیں۔

## C.1 ROS 2 Resources

Robot Operating System 2 (ROS 2) modern robotics applications کے لیے foundational software framework ہے۔ مندرجہ ذیل resources ROS 2 میں mastery حاصل کرنے اور اس کے ecosystem کو effectively leverage کرنے میں آپ کی مدد کریں گے۔

### C.1.1 Official Documentation

official ROS 2 documentation framework کے all aspects کے لیے authoritative reference کے طور پر کام کرتی ہے۔ documentation continuously updated ہے اور installation، concepts، tutorials، اور API references کو cover کرتی ہے۔

| Resource | URL | Description |
|----------|-----|-------------|
| ROS 2 Documentation | https://docs.ros.org/en/humble/ | ROS 2 Humble Hawksbill (LTS) کے لیے official documentation |
| ROS 2 Installation Guide | https://docs.ros.org/en/humble/Installation.html | All platforms کے لیے step-by-step installation instructions |
| ROS 2 Concepts | https://docs.ros.org/en/humble/Concepts.html | Core concepts including nodes, topics, services, and actions |
| ROS 2 API Reference | https://docs.ros.org/en/humble/Concepts.html | rclcpp اور rclpy کے لیے detailed API documentation |
| ROS 2 Design Guide | https://docs.ros.org/en/humble/Design.html | Architectural decisions اور design patterns |
| ROS 2 Quality Assurance | https://docs.ros.org/en/humble/Concepts/About-ROS-2-Client-Libraries.html | Client library comparison اور quality standards |

:::tip Pro Tip

production applications کے لیے ہمیشہ LTS distribution (فی الحال Humble Hawksbill) use کریں۔ Non-LTS releases میں breaking changes ہو سکتے ہیں اور shorter support windows ہیں۔

:::

### C.1.2 Tutorial Websites

structured learning paths اور hands-on tutorials ROS 2 میں mastery کے لیے essential ہیں۔ یہ platforms beginner concepts سے لے کر advanced robotics applications تک everything offer کرتے ہیں۔

| Resource | URL | Focus Area |
|----------|-----|------------|
| ROS 2 Tutorials | https://docs.ros.org/en/humble/Tutorials.html | Core concepts کو cover کرنے والے official tutorials |
| The Construct | https://www.theconstructsim.com/ | Simulation environments کے ساتھ interactive ROS 2 courses |
| ROS 2 Challenge | https://ros2challenge.github.io/ | Gamified learning with practical challenges |
| Robotics Backend | https://roboticsbackend.com/category/ros2/ | Python focus کے ساتھ ROS 2 tutorials |
| Learn ROS 2 | https://learn ROS2.com/ | Comprehensive video اور written tutorials |

### C.1.3 Community Forums and Support

ROS community vibrant اور supportive ہے۔ یہ forums اور channels help حاصل کرنے، knowledge share کرنے، اور latest developments پر updated رہنے کے لیے excellent ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| ROS Discourse | https://discourse.ros.org/ | Discussions کے لیے official community forum |
| ROS Answers | https://answers.ros.org/ | Troubleshooting help کے لیے Q&A platform |
| ROS GitHub | https://github.com/ros/ | Source code, issue tracking, اور contributions |
| ROS Discord | https://discord.gg/ros | Real-time chat community |
| r/ROS on Reddit | https://www.reddit.com/r/ROS/ | Community discussions اور news |

:::note Important

ROS Answers یا Discourse پر سوالات پوچھتے وقت ہمیشہ اپنا ROS distribution version، operating system، اور ایک minimal reproducible example include کریں۔ یہ helpful responses receive کرنے کی possibility کو dramatically increase کرتا ہے۔

:::

### C.1.4 Essential ROS 2 Packages

ROS ecosystem میں thousands of packages شامل ہیں۔ مندرجہ ذیل humanoid robotics اور physical AI applications کے لیے particularly relevant ہیں۔

| Package | URL | Purpose |
|---------|-----|---------|
| moveit2 | https://moveit.ros.org/ | Motion planning, manipulation, اور collision checking |
| navigation2 | https://navigation.ros.org/ | Autonomous navigation اور localization |
| ros2_control | https://control.ros.org/ | Hardware abstraction اور control framework |
| ros2_controllers | https://github.com/ros-controls/ros2_controllers | Robot controllers کا collection |
| gazebo_ros_pkgs | https://github.com/ros-simulation/gazebo_ros_pkgs | Gazebo simulator integration |
| robot_state_publisher | https://github.com/ros/robot_state_publisher | URDF-based state publishing |
| joint_state_publisher | https://github.com/ros/joint_state_publisher | Joint state visualization اور control |
| xacro | https://github.com/ros/xacro | URDF کے لیے XML macro language |
| urdfdom | https://github.com/ros/urdfdom | URDF parsing اور validation |

## C.2 NVIDIA Isaac Resources

NVIDIA Isaac robotics simulation، AI training، اور deployment کے لیے ایک comprehensive platform فراہم کرتا ہے۔ مندرجہ ذیل resources Isaac ecosystem کے ساتھ cover کرتے ہیں۔

### C.2.1 Official Documentation

NVIDIA Isaac platform کے لیے extensive documentation فراہم کرتا ہے، basic setup سے لے کر advanced AI training workflows تک۔

| Resource | URL | Description |
|----------|-----|-------------|
| Isaac Sim Documentation | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/root.html | Main Isaac Sim documentation |
| Isaac Sim User Guide | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_guide.html | Getting started اور core workflows |
| Isaac Gym Documentation | https://nvidia-omniverse.github.io/isaacgym-envs/ | Reinforcement learning environments |
| Isaac ROS | https://nvidia-isaac.github.io/ros2-nvidia/ | Isaac کے لیے ROS 2 packages |
| Isaac SDK | https://docs.nvidia.com/isaac-sdk/ | Isaac SDK documentation (legacy) |
| Isaac App Streaming | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/app_streaming.html | Cloud deployment options |

### C.2.2 Isaac Gym Resources

Isaac Gym robotics کے لیے high-performance GPU-accelerated reinforcement learning فراہم کرتا ہے۔ یہ resources gym framework اور available environments cover کرتے ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| Isaac Gym Envs | https://github.com/NVIDIA-Omniverse/isaacgymenvs | Pre-built RL environments |
| Isaac Gym Documentation | https://nvidia-omniverse.github.io/isaacgym-envs/ | API reference اور examples |
| Isaac Gym RL Examples | https://github.com/NVIDIA-Omniverse/IsaacGymEnvs/tree/main/rsl_rl | RL training examples |
| Isaac Gym Preview Release | https://forums.developer.nvidia.com/c/isaac/isaac-gym/ | Community forum |

### C.2.3 Isaac Sim Tutorials

Isaac Sim tutorials basic scene creation سے لے کر complex robotics workflows تک everything cover کرتی ہیں۔ یہ tutorials simulator کو effectively use کرنا سیکھنے کے لیے essential ہیں۔

| Resource | URL | Topics Covered |
|----------|-----|----------------|
| Isaac Sim Samples | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/isaac_sim_samples.html | Pre-built simulation examples |
| Isaac Sim Python Scripting | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/python_snippets.html | Programmatic scene creation |
| Isaac Sim Tutorials | https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/tutorial_intro.html | Step-by-step tutorials |
| Isaac Orbit | https://github.com/NVIDIA-Omniverse/Isaac-SIM-Orbit | Interactive orbit tutorials |

### C.2.4 Example Projects and Repositories

open-source examples best practices demonstrate کرتے ہیں اور آپ کے own projects کے لیے starting points فراہم کرتے ہیں۔

| Repository | URL | Description |
|------------|-----|-------------|
| Isaac ROS Navigation | https://github.com/NVIDIA-Omniverse/isaac_ros_navigation | Isaac کے لیے ROS 2 navigation stack |
| Isaac ROS Manipulation | https://github.com/NVIDIA-Omniverse/isaac_ros_manipulation | Manipulation pipeline examples |
| Isaac ROS Image Pipeline | https://github.com/NVIDIA-Omniverse/isaac_ros_image_pipeline | Camera processing stack |
| Isaac ROS Object Detection | https://github.com/NVIDIA-Omniverse/isaac_ros_object_detection | AI-based object detection |
| Isaac Orbit | https://github.com/NVIDIA-Omniverse/Isaac-SIM-Orbit | Interactive tutorials repository |

## C.3 Humanoid Robotics Resources

Humanoid robotics ایک specialized field ہے جس میں unique challenges ہیں۔ مندرجہ ذیل resources research، open-source projects، اور industry developments cover کرتے ہیں۔

### C.3.1 Research Paper Repositories

research کے ساتھ updated رہنا اس rapidly evolving field میں essential ہے۔ یہ repositories latest publications تک access فراہم کرتے ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| arXiv Robotics | https://arxiv.org/list/cs.RO/recent | Latest robotics research papers |
| IEEE Xplore | https://ieeexplore.ieee.org/search/advanced | Peer-reviewed robotics publications |
| ACM Digital Library | https://dl.acm.org/journals/tor | ACM transactions on robotics |
| Google Scholar | https://scholar.google.com/scholar?q=humanoid+robotics | Comprehensive academic search |
| Papers With Code | https://paperswithcode.com/area/robotics | Papers with implementations |

### C.3.2 Key Research Labs

leading research labs humanoid robotics کی boundaries push کرتی ہیں۔ ان کی publications اور open-source contributions invaluable resources ہیں۔

| Lab | Institution | URL | Focus Areas |
|-----|-------------|-----|-------------|
| CSAIL | MIT | https://www.csail.mit.edu/ | Humanoid locomotion, manipulation |
| HRP Lab | AIST | https://www.dm.kyoto-u.ac.jp/hrp-ri-en/ | Humanoid platform development |
| JSK Lab | University of Tokyo | https://www.jsk.t.u-tokyo.ac.jp/ | Humanoid اور legged robots |
| DLR | German Aerospace Center | https://www.dlr.de/rm/en/ | Space robotics, humanoid systems |
| IIT | Italian Institute of Technology | https://www.iit.it/ | Whole-body control, compliant actuation |
| Boston Dynamics | Boston Dynamics | https://www.bostondynamics.com/ | Atlas, Spot, commercial humanoids |
| Agility Robotics | Agility | https://agilityrobotics.com/ | Cassie, Digit bipedal robots |

### C.3.3 Open-Source Humanoid Projects

کئی humanoid robot platforms کے پاس open-source software ہے جو research اور development کو proprietary constraints کے بغیر enable کرتا ہے۔

| Project | Robot | URL | License |
|---------|-------|-----|---------|
| Open Humanoid | Open Humanoid | https://github.com/openhumanoid | BSD |
| ROS Humanoid | Multiple | https://github.com/ros-humanoid | Apache 2.0 |
| Humanoid Control | MuJoCo | https://github.com/ahundt/humanoid-control | MIT |
| Valkyrie | NASA | https://github.com/NASA-JSC-Robotics/valkyrie | NASA Open Source |
| Talos | PAL Robotics | https://github.com/pal-robotics/talos_robot | GPL |

## C.4 Simulation Resources

simulation robotics systems کو safely develop اور test کرنے کے لیے critical ہے۔ یہ resources major simulators اور related tools cover کرتے ہیں۔

### C.4.1 Gazebo Resources

Gazebo سبسے widely used open-source robotics simulator ہے۔ اس کے resources installation، tutorials، اور plugin development cover کرتے ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| Gazebo Sim | https://gazebosim.org/ | Main Gazebo website |
| Gazebo Documentation | https://gazebosim.org/docs | Official documentation |
| Gazebo Tutorials | https://gazebosim.org/tutorials | Step-by-step tutorials |
| Gazebo Models | https://gazebosim.org/models | Model database |
| Gazebo Answers | https://answers.gazebosim.org/ | Community Q&A |
| Gazebo Fuel | https://fuel.gazebosim.org/ | Model اور world repository |

### C.4.2 Unity Robotics Resources

Unity ایک powerful game engine-based simulation platform فراہم کرتا ہے جس میں extensive robotics support ہے۔

| Resource | URL | Description |
|----------|-----|-------------|
| Unity Robotics Hub | https://unity-robotics.com/ | Main Unity robotics portal |
| Unity Robotics ROS | https://github.com/Unity-Technologies/Unity-Robotics-Hub | ROS integration packages |
| Unity ML-Agents | https://github.com/Unity-Technologies/ml-agents | Reinforcement learning toolkit |
| Unity Perception | https://github.com/Unity-Technologies/Unity-Perception | Synthetic data generation |

### C.4.3 URDF and XACRO Resources

Unified Robot Description Format (URDF) ROS میں robot modeling کے لیے essential ہے۔ یہ resources URDF creation اور advanced modeling techniques cover کرتے ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| URDF Documentation | https://docs.ros.org/en/humble/Concepts/About-URDF.html | URDF concepts اور syntax |
| URDF XML Reference | http://wiki.ros.org/urdf/XML | Complete XML element reference |
| XACRO Documentation | https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/Using-Xacro.html | XACRO macro usage |
| URDF Viewer | https://github.com/ros/urdfdom | URDF validation tools |
| SolidWorks to URDF | https://wiki.ros.org/sw_urdf_exporter | CAD export plugin |

:::tip Pro Tip

complex robot descriptions کے لیے XACRO use کریں۔ یہ parameterization، macros، اور includes allow کرتا ہے، جو URDF files کو different robot configurations میں maintainable اور reusable بناتا ہے۔

:::

### C.4.4 MuJoCo Resources

MuJoCo robotics اور biomechanics کے لیے optimize کیا گیا ایک physics engine ہے، جو humanoid simulation کے لیے widely used ہے۔

| Resource | URL | Description |
|----------|-----|-------------|
| MuJoCo Documentation | https://mujoco.readthedocs.io/ | Official documentation |
| MuJoCo Python Binding | https://mujoco.readthedocs.io/en/stable/Python.html | mujoco_py اور mujoco |
| OpenAI Gym MuJoCo | https://github.com/openai/gym/tree/master/gym/envs/mujoco | RL environments |
| DeepMind Control Suite | https://github.com/deepmind/dm_control | Physics-based control tasks |

## C.5 Learning Platforms

formal اور informal learning resources ہر level پر skills build کرنے میں مدد کرتے ہیں۔ یہ platforms structured courses اور self-paced learning offer کرتے ہیں۔

### C.5.1 Online Courses

university-level courses robotics، AI، اور control theory میں comprehensive foundations فراہم کرتے ہیں۔

| Course | Platform | URL | Level |
|--------|----------|-----|-------|
| Introduction to Robotics | Stanford | https://see.stanford.edu/course/cs223a | Intermediate |
| Underactuated Robotics | MIT OCW | https://ocw.mit.edu/6-832 | Advanced |
| Robot Learning | Stanford | https://web.stanford.edu/class/cs287/ | Advanced |
| Deep Learning for Robotics | NVIDIA | https://www.nvidia.com/en-us/learning/ | Intermediate |
| Modern Robotics | Northwestern | https://modernrobotics.northwestern.edu/ | Intermediate |
| Robotics: Aerial Robotics | Coursera | https://www.coursera.org/learn/robotics-flight | Beginner |

### C.5.2 YouTube Channels

video content robotics concepts کی visual explanations اور demonstrations فراہم کرتا ہے۔

| Channel | URL | Content Focus |
|---------|-----|---------------|
| Robotics Back-End | https://www.youtube.com/@RoboticsBackend | ROS tutorials, robot projects |
| Articulated Robotics | https://www.youtube.com/@ArticulatedRobotics | ROS 2, hardware builds |
| The Construct | https://www.youtube.com/@TheConstructSim | Simulation tutorials |
| Boston Dynamics | https://www.youtube.com/@BostonDynamics | Research demos, product videos |
| NVIDIA Robotics | https://www.youtube.com/@NVIDIARobotics | Isaac Sim tutorials |
| 3Blue1Brown | https://www.youtube.com/@3blue1brown | Math foundations (neural networks, linear algebra) |

### C.5.3 Books and Publications

comprehensive books robotics theory اور practice میں in-depth coverage فراہم کرتی ہیں۔

| Title | Author | URL | Topics |
|-------|--------|-----|--------|
| Robotics: Modelling, Planning and Control | Siciliano et al. | https://link.springer.com/book/10.1007/978-1-84628-642-1 | Robot dynamics, control |
| Introduction to Robotics: Mechanics and Control | Craig | https://www.pearson.com/en-us/subject-catalog/p/introduction-to-robotics-mechanics-and-control/P270000374307 | Kinematics, dynamics |
| Modern Robotics: Mechanics, Planning, and Control | Lynch & Park | https://modernrobotics.org/ | Comprehensive coverage |
| Robot Modeling and Control | Spong et al. | https://www.wiley.com/en-us/Robot+Modeling+and+Control-p-9780471649908 | Modeling, control |
| Underactuated Robotics | Tedrake | https://underactuated.csail.mit.edu/ | Advanced control theory |
| Reinforcement Learning: An Introduction | Sutton & Barto | http://incompleteideas.net/book/the-book.html | RL foundations |

### C.5.4 Practice Platforms

hands-on practice robotics skills develop کرنے کے لیے essential ہے۔ یہ platforms challenges اور environments provide کرتے ہیں۔

| Platform | URL | Description |
|----------|-----|-------------|
| ROS Challenge | https://roschallenge.github.io/ | ROS development challenges |
| OpenAI Gym | https://gymnasium.farama.org/ | RL environment suite |
| PyBullet | https://pybullet.org/wordpress/ | Physics simulation for RL |
| Roboschool | https://github.com/openai/roboschool | Open-source robot simulation |
| AI2 THOR | https://github.com/allenai/ai2thor | Indoor robot navigation |

## C.6 Hardware Vendors

physical hardware real-world robotics development کے لیے essential ہے۔ یہ vendors robots، sensors، اور components provide کرتے ہیں۔

### C.6.1 Robot Manufacturers

commercial humanoid اور legged robots research اور development کے لیے کئی manufacturers سے available ہیں۔

| Manufacturer | Products | URL | Notes |
|--------------|----------|-----|-------|
| Unitree Robotics | H1, G1, Go2 | https://www.unitree.com/ | Quadrupeds اور humanoids |
| Boston Dynamics | Atlas, Spot | https://www.bostondynamics.com/ | Advanced research platforms |
| Agility Robotics | Digit, Cassie | https://agilityrobotics.com/ | Bipedal delivery robots |
| PAL Robotics | Talos, REEM-C | https://www.pal-robotics.com/ | Research humanoids |
| Robotis | OP3, Darwin-OP | https://en.robotis.com/ | Educational humanoid kits |
| Fourier Intelligence | GR-1 | https://www.fourierintelligence.com/ | General-purpose humanoid |
| Sanctuary AI | Phoenix | https://www.sanctuary.ai/ | General-purpose humanoid |

### C.6.2 Sensor Vendors

perception humanoid robots کے لیے critical ہے۔ یہ vendors cameras، LIDARs، IMUs، اور force sensors provide کرتے ہیں۔

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

motors، actuators، اور mechanical components custom robot builds کے لیے essential ہیں۔

| Vendor | Components | URL | Notes |
|--------|------------|-----|-------|
| Maxon Motor | DC motors, gearheads | https://www.maxongroup.com/ | Precision motors |
| Harmonic Drive | Strain wave gears | https://www.harmonicdrive.net/ | Zero-backlash gears |
| Wittenstein | Precision gears | https://www.wittenstein-us.com/ | High-torque gears |
| Pololu | Motors, controllers | https://www.pololu.com/ | Hobbyist components |
| ServoCity | Motion components | https://www.servocity.com/ | Modular actuation |
| Enful | Brushless motors | https://www.enful.com/ | Robot-specific motors |
| T-Motor | Motors, ESCs | https://www.tmotor.com/ | Drone اور robot motors |

## C.7 Career and Research

humanoid robotics میں career build کرنے کے لیے networking، continuous learning، اور professional development کی ضرورت ہوتی ہے۔

### C.7.1 Research Labs and Groups

leading research groups collaboration اور career development کے لیے opportunities provide کرتے ہیں۔

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

conferences research present کرنے، network کرنے، اور latest developments کے بارے میں جاننے کے opportunities provide کرتے ہیں۔

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

robotics میں positions find کرنے کے لیے جگہ جگہ look کرنے کی ضرورت ہوتی ہے۔ یہ resources robotics-specific opportunities list کرتے ہیں۔

| Resource | URL | Description |
|----------|-----|-------------|
| IEEE Robotics Jobs | https://jobs.ieee.org/jobs/keyword/robotics | IEEE job board |
| Robotics Jobs | https://www.roboticstojobs.com/ | Robotics-specific listings |
| LinkedIn Robotics | https://www.linkedin.com/jobs/robotics-jobs/ | Professional network |
| WeWorkInRobotics | https://www.weworkinrobotics.com/ | Robotics company directory |
| Robohub Jobs | https://robohub.org/jobs/ | Research اور industry jobs |
| Crunchbase | https://www.crunchbase.com/discover/organization.investors | Startup funding research |

### C.7.4 Internship Programs

internships humanoid robotics میں valuable experience provide کرتے ہیں۔ کئی companies اور labs structured programs offer کرتے ہیں۔

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

standardized datasets robotics research میں benchmarking اور reproducibility enable کرتی ہیں۔

| Dataset | URL | Description |
|---------|-----|-------------|
| KITTI | http://www.cvlibs.net/datasets/kitti/ | Autonomous driving, computer vision |
| COCO | https://cocodataset.org/ | Object detection, segmentation |
| BDD100K | https://bdd-data.berkeley.edu/ | Diverse driving dataset |
| Amazon Robotics | https://amazon-robotics.github.io/ | Warehouse automation |
| PartNet | https://partnet.cs.stanford.edu/ | Object part hierarchy |
| ShapeNet | https://shapenet.org/ | 3D model repository |

### C.8.2 Software Development Tools

robotics software engineering کے لیے development tools اور utilities۔

| Tool | URL | Purpose |
|------|-----|---------|
| Git | https://git-scm.com/ | Version control |
| Docker | https://www.docker.com/ | Containerization |
| VS Code | https://code.visualstudio.com/ | IDE with ROS extension |
| GitHub Codespaces | https://github.com/features/codespaces | Cloud development |
| GitLab CI | https://docs.gitlab.com/ci/ | Continuous integration |
| Colab | https://colab.research.google.com/ | Free GPU notebooks |

### C.8.3 ROS Distribution Timeline

ROS distribution lifecycle understand کرنے سے project planning میں مدد ملتی ہے۔

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

کسی production project شروع کرنے سے پہلے ہمیشہ ROS distribution کی support status verify کریں۔ Long-term support (LTS) distributions جیسے Humble اور Jazzy کو five years of maintenance ملتا ہے۔

:::

---

## References (حوالہ جات)

- ROS 2 Official Documentation: https://docs.ros.org/en/humble/
- NVIDIA Isaac Sim Documentation: https://docs.omniverse.nvidia.com/app_isaacsim/
- Unitree Robotics: https://www.unitree.com/
- arXiv Robotics Category: https://arxiv.org/list/cs.RO/recent
- Gazebo Simulator: https://gazebosim.org/

---

**Appendix C** | [Appendix A: Hardware Specifications](appendix/hardware-specifications) | [Appendix D: Assessment Rubrics](appendix/D-assessment-rubrics)
