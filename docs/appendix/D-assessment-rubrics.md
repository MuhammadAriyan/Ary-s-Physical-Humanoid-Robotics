---
title: "Appendix D: Assessment Rubrics"
sidebar_position: 73
---

# Appendix D: Assessment Rubrics

This appendix provides comprehensive assessment criteria and grading rubrics for all components of the Physical AI and Humanoid Robotics course. Instructors can use these rubrics to evaluate student work consistently and provide meaningful feedback across ROS 2 package development, Gazebo simulation, Isaac Perception pipelines, and capstone projects.

## D.1 ROS 2 Package Assessment

The ROS 2 package assessment evaluates students' ability to create production-quality robot software packages that follow ROS 2 best practices, implement proper node architecture, and demonstrate effective communication patterns. This assessment covers code organization, documentation, testing, and functional correctness.

### D.1.1 Package Structure and Organization

A well-structured ROS 2 package demonstrates professional software engineering practices and facilitates collaboration among developers. The following criteria evaluate package organization:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Directory Layout** | Follows ROS 2 ament_cmake/ament_python conventions perfectly with clear separation of nodes, tests, configs, and launch files | Minor deviations from conventions; overall structure is navigable | Inconsistent structure; some confusion between node types | No clear organization; files scattered randomly |
| **CMakeLists.txt/Package.xml** | Complete metadata with all dependencies, maintainers, licenses, and export configurations properly declared | Minor omissions in dependencies or metadata | Incomplete configuration; missing key dependencies | Broken or missing configuration files |
| **Modularity** | Highly modular design with clear single-responsibility nodes; minimal coupling between components | Reasonably modular with some opportunities for improvement | Modules exist but have excessive responsibilities | Monolithic code; no meaningful separation |
| **Naming Conventions** | Consistent, descriptive names for nodes, topics, services, and actions following ROS 2 style guide | Minor inconsistencies in naming | Inconsistent naming that causes confusion | Random or misleading names |

### D.1.2 Node Implementation Quality

Node implementation quality assesses the internal code quality, error handling, and adherence to ROS 2 patterns:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Callback Design** | Elegant callback chains with appropriate priorities; no blocking operations in callbacks; proper timer usage | Minor callback design issues; generally non-blocking | Some blocking callbacks or callback hell patterns | All work done in single callback; severe blocking |
| **Parameter Handling** | Comprehensive parameter declarations with type validation, dynamic reconfiguration, and clear documentation | Parameters declared but lacking dynamic updates | Parameters hardcoded or scattered | No parameterization; hardcoded values throughout |
| **Error Handling** | Graceful error handling with proper exception catching, error state machines, and recovery strategies | Error handling exists but recovery is limited | Errors caught but not handled appropriately | No error handling; crashes on exceptions |
| **Resource Management** | Proper lifecycle management with cleanup in destructors; no resource leaks; appropriate QoS profile usage | Minor resource management issues | Some leaks or improper cleanup | Severe resource leaks; memory warnings |

### D.1.3 Communication Patterns

Effective use of ROS 2 communication primitives is essential for building robust robotic systems:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Topic Publishing** | Optimized publish rates with appropriate QoS profiles; message coalescing for high-frequency data; proper latching for static data | Correct topics with appropriate QoS | Topics exist but QoS poorly chosen | Incorrect or missing publishers |
| **Subscription Handling** | Efficient message handling with appropriate buffer sizes; callback groups properly configured; message filtering at source | Correct subscriptions with proper buffers | Subscriptions exist but buffer issues | Messages dropped; no subscription handling |
| **Service Design** | Well-designed services with proper request/response patterns; idempotent operations; timeout handling | Correct services with basic timeout handling | Services exist but design suboptimal | Services not implemented or broken |
| **Action Implementation** | Complete action servers with proper goal handling, preempt support, and feedback loops; no blocking in action callbacks | Actions work but feedback incomplete | Actions exist but lack proper state machine | Actions not implemented or crash |

### D.1.4 Documentation and Testing

Professional software requires comprehensive documentation and testing:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Code Documentation** | Complete docstrings for all public APIs; internal comments explaining complex logic; README with examples and launch instructions | Documentation exists for main components | Minimal documentation; public API incomplete | No documentation |
| **Unit Test Coverage** | Greater than 90% coverage with meaningful tests; edge cases covered; continuous integration with automated testing | 70-90% coverage with reasonable tests | 40-70% coverage; basic functionality tested | Less than 40% coverage; no meaningful tests |
| **Integration Tests** | Complete integration tests for node interactions; simulation-based testing; performance benchmarks | Integration tests for critical paths | Basic integration tests | No integration tests |
| **Launch File Quality** | Comprehensive launch files with parameter overrides; argument handling; visualization tools configured | Launch files work for standard cases | Incomplete launch configurations | No launch files or non-functional |

### D.1.5 Example ROS 2 Package Deliverables

Student packages should include the following deliverables:

```
my_robot_package/
├── package.xml                    # Complete package metadata
├── setup.py / setup.cfg           # Build configuration
├── my_robot_package/
│   ├── __init__.py               # Package initialization
│   ├── nodes/
│   │   ├── data_publisher.py     # Main publisher node
│   │   ├── command_processor.py  # Service and action server
│   │   └── sensor_fuser.py       # Subscription and fusion node
│   ├── libraries/
│   │   ├── __init__.py
│   │   ├── kinematics.py         # Reusable kinematics library
│   │   └── filters.py            # Filter implementations
│   └── config/
│       ├── params.yaml           # Default parameters
│       └── calibration.yaml      # Calibration data
├── launch/
│   ├── bringup.launch.py         # Main launch file
│   └── visualization.launch.py   # RViz configuration
├── test/
│   ├── test_nodes.py             # Unit tests
│   └── test_integration.py       # Integration tests
├── resource/
│   └── meshes/                   # 3D meshes
├── urdf/
│   └── robot.urdf.xacro          # Robot description
└── README.md                     # Package documentation
```

## D.2 Gazebo Simulation Assessment

Gazebo simulation assessment evaluates students' ability to create physically accurate, performant, and well-designed simulation environments for humanoid robot testing and validation.

### D.2.1 World and Environment Design

The simulation world must accurately represent the robot's operating environment:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Physics Fidelity** | Accurate physics parameters matching real robot dynamics; properly tuned friction, damping, and mass properties | Physics generally accurate with minor tuning needed | Physics present but unrealistic for key interactions | Physics unrealistic or broken |
| **Terrain Modeling** | Varied terrain types with proper surface properties; accurate slope modeling; obstacle configurations matching real scenarios | Good terrain variety with mostly accurate properties | Basic terrain present but properties inconsistent | No meaningful terrain or flat plane only |
| **Lighting and Atmosphere** | Realistic lighting conditions; proper shadows; atmospheric effects for depth perception | Adequate lighting for visibility | Minimal lighting; poor visibility | No lighting or unusable conditions |
| **Sensor Simulation** | Camera, LiDAR, and IMU simulation with realistic noise models; proper frame transformations | Major sensors simulated with noise models | Basic sensor models present | Sensors missing or non-functional |

### D.2.2 Robot Model Quality

The robot model must accurately represent the physical hardware:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **URDF/XACRO Quality** | Clean, parameterized URDF with proper joint and link definitions; inertial parameters from CAD data | Valid URDF with minor organizational issues | URDF present but has errors or warnings | Broken URDF or missing model |
| **Collision Geometry** | Optimized collision geometry balancing accuracy and performance; proper friction coefficients | Collision geometry present and functional | Basic collision geometry | No collision geometry |
| **Visual Quality** | Professional appearance with appropriate materials; CAD-quality meshes; proper scaling and positioning | Acceptable visuals with minor issues | Basic visuals; scale or texture problems | Missing visuals or incorrect rendering |
| **Joint Configuration** | Proper joint limits, axis alignment, and control interfaces matching hardware | Mostly correct joints with minor issues | Some joint configuration errors | Broken joint configuration |

### D.2.3 Simulation Performance

Performance is critical for real-time simulation and algorithm development:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Real-Time Factor** | Consistent 1.0x or better real-time factor for full simulation; optimized for complex scenarios | Above 0.9x real-time factor; minor lag during complex scenes | 0.7-0.9x real-time factor; noticeable lag | Below 0.7x real-time factor; unusable |
| **Frame Rate** | Stable 60+ FPS with visualization; 30+ FPS for physics-heavy scenarios | 30-60 FPS with visualization | 15-30 FPS; choppy visualization | Below 15 FPS; unusable |
| **Resource Efficiency** | Efficient mesh usage; level-of-detail for distant objects; proper physics stepping | Generally efficient with minor optimizations possible | Some performance bottlenecks | Severe performance issues |
| **Memory Usage** | Stable memory consumption under long simulations; no leaks; proper cleanup | Minor memory growth over long sessions | Noticeable memory increase | Memory leaks or unbounded growth |

### D.2.4 Plugin and Controller Integration

Simulation plugins must properly interface with control systems:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Gazebo Plugins** | Complete plugin implementation for robot interfaces; accurate sensor simulation; proper state publishing | Plugins work but some advanced features missing | Basic plugins present | No plugins or non-functional |
| **Controller Integration** | Seamless ROS 2 control integration with ros2_control; proper hardware abstraction; trajectory execution | Controller integration mostly complete | Basic controller integration | No controller integration |
| **State Estimation** | Accurate state estimation from simulation; proper sensor fusion; realistic noise injection | State estimation mostly accurate | Basic state estimation | Broken state estimation |
| **Reset and Recovery** | Proper simulation reset handling; state persistence; recovery from failures | Reset works but state may be lost | Partial reset functionality | Cannot reset simulation |

### D.2.5 Simulation Quality Metrics

The following metrics should be measured and reported for each simulation deliverable:

| Metric | Target Value | Acceptable | Unacceptable |
|--------|--------------|------------|--------------|
| Real-time factor (empty world) | 1.0+ | 0.9-1.0 | < 0.9 |
| Real-time factor (populated) | 1.0+ | 0.8-1.0 | < 0.8 |
| Frame rate (visualization) | 60+ FPS | 30-60 FPS | < 30 FPS |
| Joint position error (static) | < 0.01 rad | 0.01-0.05 rad | > 0.05 rad |
| Joint position error (dynamic) | < 0.02 rad | 0.02-0.1 rad | > 0.1 rad |
| Odometry drift (per meter) | < 0.01 m | 0.01-0.05 m | > 0.05 m |
| LiDAR range accuracy | < 2% error | 2-5% error | > 5% error |
| Simulation startup time | < 10 seconds | 10-30 seconds | > 30 seconds |

## D.3 Isaac Perception Assessment

Isaac Perceptual assessment evaluates students' ability to implement computer vision and perception pipelines using NVIDIA Isaac for humanoid robot applications.

### D.3.1 Pipeline Architecture

Perception pipeline architecture determines system efficiency and maintainability:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Modular Design** | Clean separation between perception stages; reusable components; proper data flow architecture | Well-organized with minor coupling | Basic separation but tight coupling | Monolithic perception code |
| **Buffer Management** | Proper message buffering with configurable queue sizes; thread-safe data handling | Mostly correct with minor timing issues | Buffer issues under load | Messages dropped; race conditions |
| **GPU Utilization** | Efficient CUDA kernel usage; proper memory management; GPU-CPU data transfer optimization | GPU acceleration present but not fully optimized | Some GPU usage | Entirely CPU-based; no GPU offload |
| **Latency Management** | End-to-end latency under 100ms; proper timestamping; latency compensation techniques | Latency acceptable with minor variations | High latency but functional | Unacceptable latency |

### D.3.2 Object Detection and Tracking

Object detection and tracking are fundamental perception capabilities:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Detection Accuracy** | mAP > 0.85 on test set; precision > 0.90; recall > 0.80 | mAP 0.75-0.85; balanced precision/recall | mAP 0.60-0.75; significant precision/recall imbalance | mAP < 0.60; poor detection |
| **Real-Time Performance** | 30+ FPS with full detection pipeline | 20-30 FPS detection | 10-20 FPS detection | < 10 FPS; unusable |
| **Multi-Object Tracking** | MOTA > 0.80; proper ID preservation; occlusion handling | MOTA 0.65-0.80; minor ID switches | MOTA 0.50-0.65; frequent ID switches | MOTA < 0.50; no tracking |
| **Robustness** | Consistent performance across lighting, viewpoints, and occlusions | Minor degradation under challenging conditions | Significant degradation under some conditions | Only works under ideal conditions |

### D.3.3 Semantic Segmentation

Semantic understanding enables complex scene reasoning:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Segmentation Accuracy** | mIoU > 0.80; class-balanced performance | mIoU 0.70-0.80; some class imbalances | mIoU 0.55-0.70; significant class imbalances | mIoU < 0.55; poor segmentation |
| **Boundary Precision** | Clean boundaries with proper edge refinement; boundary IoU > 0.75 | Good boundaries with minor artifacts | Acceptable boundaries with noticeable artifacts | Poor boundaries; extensive artifacts |
| **Inference Speed** | 25+ FPS segmentation inference | 15-25 FPS inference | 8-15 FPS inference | < 8 FPS; unusable |
| **Class Coverage** | All relevant classes with balanced performance | All classes present with minor imbalances | Missing some classes or severe imbalances | Limited class coverage |

### D.3.4 Depth and 3D Perception

Three-dimensional perception is essential for humanoid manipulation and navigation:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Depth Accuracy** | < 2% relative error at 5m; < 1% at 2m | 2-4% error at 5m | 4-8% error at 5m | > 8% error; unusable |
| **3D Bounding Boxes** | BEV mAP > 0.80; proper orientation; accurate dimensions | BEV mAP 0.70-0.80 | BEV mAP 0.55-0.70 | BEV mAP < 0.55 |
| **Point Cloud Processing** | Efficient PCL processing; proper filtering and segmentation | Mostly correct with minor issues | Basic point cloud processing | Broken or missing point cloud |
| **Sensor Fusion** | Accurate multi-sensor calibration; proper temporal alignment; fusion improves over single sensor | Sensor fusion present but benefits limited | Basic fusion attempts | No sensor fusion |

### D.3.5 Isaac-Specific Implementation

Isaac-specific implementation demonstrates platform expertise:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Isaac SDK Usage** | Proper Isaac modules usage; optimized graph configuration; CUDA extensions where appropriate | Isaac modules used correctly | Basic Isaac usage | No Isaac usage |
| **DNN Inference** | Optimized DNN inference with TensorRT; proper precision selection (FP16/INT8); batching support | DNN inference functional but not optimized | Basic DNN inference | No DNN or broken inference |
| **VPI Integration** | Proper VPI algorithms for vision processing; efficient pipeline | VPI used correctly | Basic VPI usage | No VPI integration |
| **Performance Profiling** | Comprehensive profiling; bottleneck identification; optimization recommendations | Profiling done but analysis limited | Some profiling attempted | No profiling |

### D.3.6 Perception Performance Benchmarks

| Test Scenario | Metric | Target | Minimum |
|---------------|--------|--------|---------|
| Object detection (person) | mAP@0.5 | > 0.90 | 0.80 |
| Object detection (all classes) | mAP@0.5 | > 0.85 | 0.75 |
| Semantic segmentation | mIoU | > 0.80 | 0.70 |
| Depth estimation | Abs Rel Error | < 0.02 | 0.05 |
| 3D object detection (BEV) | mAP | > 0.80 | 0.70 |
| Multi-object tracking | MOTA | > 0.80 | 0.65 |
| End-to-end latency | P95 | < 100ms | 150ms |
| Pipeline throughput | FPS | > 30 | 20 |

## D.4 Capstone Project Assessment

The capstone project integrates all course concepts into a comprehensive robot system. Assessment evaluates technical depth, system integration, and professional deliverables.

### D.4.1 Project Scope and Objectives

Capstone projects must demonstrate appropriate scope and clear objectives:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Problem Definition** | Clear, well-motivated problem with significant technical challenges; addresses real humanoid robotics needs | Well-defined problem with technical depth | Problem defined but scope unclear | Vague or trivial problem |
| **Scope Appropriateness** | Challenging but achievable within constraints; meaningful integration of multiple course concepts | Appropriate scope with minor concerns | Scope slightly too large or small | Scope inappropriate |
| **Learning Objectives** | Explicit alignment with course learning objectives; clear skill demonstration | Most objectives addressed | Some objectives addressed | Limited alignment |
| **Innovation** | Novel approach or application; creative problem-solving; potential for publication or research contribution | Some innovative elements | Mostly derivative but competent | No innovation apparent |

### D.4.2 Technical Implementation

Technical implementation quality determines project success:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **ROS 2 Architecture** | Professional-grade package design; scalable node structure; robust communication patterns | Solid architecture with minor issues | Basic architecture present | Architecture problems |
| **Simulation Fidelity** | High-fidelity Gazebo simulation accurately representing real robot behavior | Good simulation with minor gaps | Basic simulation present | Poor or missing simulation |
| **Control Implementation** | Sophisticated control algorithms properly implemented; stability and performance guarantees | Correct control implementation | Basic control present | Broken or missing control |
| **Perception Integration** | Working perception pipeline integrated with control; real-time performance; robust to variations | Perception integrated with minor issues | Basic perception integration | Missing perception |

### D.4.3 System Integration

System integration demonstrates holistic engineering capability:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Module Integration** | Seamless integration between all system components; proper interfaces; data flow correctness | Good integration with minor issues | Some integration present | Poor integration |
| **Hardware-Software Bridge** | Proper abstraction between simulation and hardware; clean migration path | Abstraction mostly correct | Basic abstraction present | No clear abstraction |
| **Error Handling** | Comprehensive error detection and recovery; graceful degradation; fail-safe states | Good error handling with gaps | Basic error handling | No error handling |
| **System Testing** | Complete test coverage including integration and system tests; automated testing pipeline | Good test coverage | Some testing present | No testing |

### D.4.4 Documentation and Reproducibility

Professional projects require comprehensive documentation:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Technical Documentation** | Complete system documentation; API docs; architecture diagrams; deployment instructions | Good documentation with minor gaps | Basic documentation present | Incomplete or missing docs |
| **Code Documentation** | Comprehensive inline documentation; clear code organization; coding standards followed | Good code documentation | Basic code comments | No documentation |
| **Reproducibility** | Complete setup scripts; containerization; environment specification; step-by-step reproduction guide | Good reproducibility with minor issues | Basic setup instructions | Not reproducible |
| **User Guide** | Clear user guide with examples; troubleshooting section; FAQ | User guide mostly complete | Basic user guide | No user guide |

### D.4.5 Presentation Rubric

The final presentation demonstrates communication and understanding:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Problem Statement** | Clear motivation; contextual importance; stakeholder needs addressed | Clear problem statement | Problem stated but motivation weak | Problem unclear |
| **Technical Depth** | Deep technical understanding demonstrated; algorithm details explained; tradeoffs analyzed | Good technical depth | Surface-level technical coverage | Superficial treatment |
| **System Demo** | Impressive live or video demonstration; clear explanation of capabilities; handles questions gracefully | Good demo with minor issues | Demo present but problematic | No demo or failed demo |
| **Q&A Performance** | Confident, accurate responses; acknowledges limitations; suggests improvements | Good Q&A performance | Basic Q&A performance | Unable to answer questions |
| **Visual Aids** | Professional slides; clear diagrams; effective visualizations; appropriate level of detail | Good visual aids | Basic slides present | Poor or missing visuals |
| **Time Management** | Well-paced presentation; covers all material; respects time limits | Good time management | Some timing issues | Significant timing problems |

### D.4.6 Capstone Scoring Matrix

| Component | Weight | Excellent | Proficient | Developing | Beginning |
|-----------|--------|-----------|------------|------------|----------|
| Technical Implementation | 35% | 100-90 | 89-80 | 79-70 | 69-0 |
| System Integration | 20% | 100-90 | 89-80 | 79-70 | 69-0 |
| Documentation | 15% | 100-90 | 89-80 | 79-70 | 69-0 |
| Presentation | 15% | 100-90 | 89-80 | 79-70 | 69-0 |
| Innovation | 10% | 100-90 | 89-80 | 79-70 | 69-0 |
| Team Collaboration | 5% | 100-90 | 89-80 | 79-70 | 69-0 |

## D.5 General Grading Scale

The following grading scale applies to all assessments in the Physical AI and Humanoid Robotics course.

### D.5.1 Percentage to Letter Grade Conversion

| Percentage Range | Letter Grade | Grade Points | Description |
|------------------|--------------|--------------|-------------|
| 97-100% | A+ | 4.0 | Exceptional mastery; exceeds all expectations |
| 93-96% | A | 4.0 | Outstanding performance; demonstrates deep understanding |
| 90-92% | A- | 3.7 | Excellent work; minor areas for improvement |
| 87-89% | B+ | 3.3 | Very good work; solid understanding demonstrated |
| 83-86% | B | 3.0 | Good work; meets all requirements |
| 80-82% | B- | 2.7 | Above average; some minor deficiencies |
| 77-79% | C+ | 2.3 | Satisfactory; meets basic requirements |
| 73-76% | C | 2.0 | Adequate; meets minimum expectations |
| 70-72% | C- | 1.7 | Marginal; several areas need improvement |
| 67-69% | D+ | 1.3 | Poor; barely meets minimum requirements |
| 63-66% | D | 1.0 | Deficient; significant gaps in understanding |
| 60-62% | D- | 0.7 | Very deficient; barely passing |
| Below 60% | F | 0.0 | Failing; does not meet requirements |

### D.5.2 Assignment Type Weighting

The course grade is computed using the following component weights:

| Assignment Type | Weight | Components Included |
|-----------------|--------|---------------------|
| Weekly Labs | 25% | All weekly hands-on exercises and implementations |
| ROS 2 Packages | 20% | All package development assignments |
| Simulation Projects | 20% | Gazebo world and robot model projects |
| Perception Pipelines | 15% | Isaac perception implementations |
| Midterm Assessment | 10% | Theory and concepts examination |
| Capstone Project | 10% | Final integrated project |

### D.5.3 Grading Criteria by Component

#### ROS 2 Package Grading (100 points total)

| Category | Points | Assessment Basis |
|----------|--------|------------------|
| Package Structure | 15 | Directory layout, configuration files, naming conventions |
| Node Implementation | 25 | Code quality, error handling, parameter management |
| Communication Patterns | 20 | Topics, services, actions implemented correctly |
| Testing | 20 | Unit tests, integration tests, coverage |
| Documentation | 10 | README, inline comments, API documentation |
| Performance | 10 | Memory efficiency, CPU usage, real-time factors |

#### Simulation Project Grading (100 points total)

| Category | Points | Assessment Basis |
|----------|--------|------------------|
| World Design | 20 | Physics fidelity, terrain, environment variety |
| Robot Model | 25 | URDF quality, collision geometry, visual appearance |
| Performance | 20 | Real-time factor, frame rate, resource usage |
| Plugins | 15 | Controller integration, sensor simulation |
| Documentation | 10 | Setup instructions, parameter documentation |
| Creativity | 10 | Novel scenarios, additional features |

#### Perception Pipeline Grading (100 points total)

| Category | Points | Assessment Basis |
|----------|--------|------------------|
| Pipeline Architecture | 20 | Modularity, buffer management, GPU utilization |
| Detection/Tracking | 25 | Accuracy metrics, real-time performance |
| Segmentation | 20 | Accuracy, boundary quality, inference speed |
| 3D Perception | 15 | Depth accuracy, point cloud processing |
| Isaac Implementation | 10 | Proper SDK usage, TensorRT optimization |
| Documentation | 10 | Architecture docs, performance benchmarks |

### D.5.4 Late Work and Extensions

Late work is assessed according to the following policy:

| Submission Time | Penalty |
|-----------------|---------|
| On time | No penalty |
| 1-24 hours late | 10% deduction |
| 24-48 hours late | 20% deduction |
| 48-72 hours late | 35% deduction |
| More than 72 hours late | 50% maximum (no credit after 1 week) |

Extension requests must be submitted at least 48 hours before the deadline except in documented emergencies. Extension grants will not exceed 7 days and must be approved by the instructor.

### D.5.5 Academic Integrity

All assessments are subject to the following academic integrity guidelines:

- **Individual Work**: Weekly labs and perception implementations must be completed individually unless explicitly stated otherwise.
- **Group Work**: Capstone projects and designated team assignments may be collaborative.
- **Code Reuse**: Students may use open-source code with proper attribution and within license terms.
- **Citation Requirement**: Any borrowed code, algorithms, or concepts must be clearly cited.
- **AI Tools**: Use of AI coding assistants is permitted for learning but submissions must demonstrate individual understanding.

### D.5.6 Feedback and Revision

The course emphasizes iterative improvement through feedback:

- **Lab Submissions**: Feedback provided within 7 days; no revisions permitted.
- **Major Projects**: Feedback within 10 days; one optional revision within 14 days of original feedback (10% penalty).
- **Capstone**: Continuous feedback during development; final evaluation is terminal.
- **Grade Disputes**: Must be submitted in writing within 7 days of grade return.

---

:::note Instructor Notes

These rubrics are designed to be comprehensive yet flexible. Instructors should adapt criteria based on specific assignment requirements and student skill levels. When in doubt, assess whether work demonstrates understanding of core concepts and professional engineering practices.

:::

:::tip Best Practice

Consider using a rubric scorecard during grading to ensure consistent evaluation across students and assignments. Record specific evidence for each criterion to support feedback and grade disputes.

:::

## References

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Sim Documentation: https://gazebosim.org/docs
- NVIDIA Isaac Documentation: https://docs.nvidia.com/isaac/
- ROS 2 Quality Assurance Standards: https://github.com/ros2/ros2/wiki/QA-Level
- Humanoid Robot Control Literature: https://arxiv.org/list/cs.RO/recent

---

**Appendix D** | [Appendix A: Hardware Specifications](appendix/hardware-specifications) | [Part 1: Introduction](part-1-foundations/introduction-to-physical-ai)
