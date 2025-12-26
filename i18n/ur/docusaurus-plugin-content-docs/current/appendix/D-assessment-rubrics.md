---
title: "Appendix D: Assessment Rubrics"
sidebar_position: 73
---

# Appendix D: Assessment Rubrics (جانچ کے معیار)

یہ ایپینڈکس Physical AI اور Humanoid Robotics course کے all components کے لیے comprehensive assessment criteria اور grading rubrics فراہم کرتا ہے۔ instructors ان rubrics کو students کے work کو consistently evaluate کرنے اور ROS 2 package development، Gazebo simulation، Isaac Perception pipelines، اور capstone projects میں meaningful feedback provide کرنے کے لیے use کر سکتے ہیں۔

## D.1 ROS 2 Package Assessment

ROS 2 package assessment students کی ability کو evaluate کرتا ہے کہ وہ production-quality robot software packages create کریں جو ROS 2 best practices follow کرتے ہوں، proper node architecture implement کرتے ہوں، اور effective communication patterns demonstrate کرتے ہوں۔ یہ assessment code organization، documentation، testing، اور functional correctness cover کرتا ہے۔

### D.1.1 Package Structure and Organization

ایک well-structured ROS 2 package professional software engineering practices demonstrate کرتا ہے اور developers کے درمی collaboration facilitate کرتا ہے۔ مندرجہ ذیل criteria package organization evaluate کرتی ہیں:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Directory Layout** | ROS 2 ament_cmake/ament_python conventions کو perfectly follow کرتا ہے nodes، tests، configs، اور launch files کے clear separation کے ساتھ | Conventions سے minor deviations؛ overall structure navigable ہے | Inconsistent structure؛ node types کے درمی کچھ confusion | No clear organization؛ files randomly scattered |
| **CMakeLists.txt/Package.xml** | Complete metadata with all dependencies, maintainers, licenses, اور export configurations properly declared | Dependencies یا metadata میں minor omissions | Incomplete configuration؛ key dependencies missing | Broken or missing configuration files |
| **Modularity** | Highly modular design with clear single-responsibility nodes؛ components کے درمی minimal coupling | Reasonably modular with some opportunities for improvement | Modules exist but have excessive responsibilities | Monolithic code؛ no meaningful separation |
| **Naming Conventions** | Nodes، topics، services، اور actions کے لیے consistent, descriptive names ROS 2 style guide follow کرتے ہیں | Naming میزان minor inconsistencies | Inconsistent naming جو confusion cause کرتی ہے | Random or misleading names |

### D.1.2 Node Implementation Quality

Node implementation quality internal code quality، error handling، اور ROS 2 patterns کے adherence assess کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Callback Design** | Elegant callback chains with appropriate priorities؛ callbacks میں no blocking operations؛ proper timer usage | Minor callback design issues؛ generally non-blocking | Some blocking callbacks or callback hell patterns | All work done in single callback؛ severe blocking |
| **Parameter Handling** | Comprehensive parameter declarations with type validation, dynamic reconfiguration, اور clear documentation | Parameters declared but lacking dynamic updates | Parameters hardcoded or scattered | No parameterization؛ hardcoded values throughout |
| **Error Handling** | Graceful error handling with proper exception catching, error state machines, اور recovery strategies | Error handling exists but recovery is limited | Errors caught but not handled appropriately | No error handling؛ crashes on exceptions |
| **Resource Management** | Proper lifecycle management with cleanup in destructors؛ no resource leaks؛ appropriate QoS profile usage | Minor resource management issues | Some leaks or improper cleanup | Severe resource leaks؛ memory warnings |

### D.1.3 Communication Patterns

ROS 2 communication primitives کا effective use robust robotic systems build کرنے کے لیے essential ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Topic Publishing** | Optimized publish rates with appropriate QoS profiles؛ high-frequency data کے لیے message coalescing؛ static data کے لیے proper latching | Correct topics with appropriate QoS | Topics exist but QoS poorly chosen | Incorrect or missing publishers |
| **Subscription Handling** | Efficient message handling with appropriate buffer sizes؛ callback groups properly configured؛ source پر message filtering | Correct subscriptions with proper buffers | Subscriptions exist but buffer issues | Messages dropped؛ no subscription handling |
| **Service Design** | Well-designed services with proper request/response patterns؛ idempotent operations؛ timeout handling | Correct services with basic timeout handling | Services exist but design suboptimal | Services not implemented or broken |
| **Action Implementation** | Complete action servers with proper goal handling, preempt support, اور feedback loops؛ action callbacks میں no blocking | Actions work but feedback incomplete | Actions exist but lack proper state machine | Actions not implemented or crash |

### D.1.4 Documentation and Testing

Professional software کے لیے comprehensive documentation اور testing required ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Code Documentation** | All public APIs کے لیے complete docstrings؛ complex logic explain کرنے والے internal comments؛ examples اور launch instructions کے ساتھ README | Main components کے لیے documentation exists | Minimal documentation؛ public API incomplete | No documentation |
| **Unit Test Coverage** | 90% سے زیادہ coverage with meaningful tests؛ edge cases covered؛ continuous integration with automated testing | 70-90% coverage with reasonable tests | 40-70% coverage؛ basic functionality tested | Less than 40% coverage؛ no meaningful tests |
| **Integration Tests** | Node interactions کے لیے complete integration tests؛ simulation-based testing؛ performance benchmarks | Critical paths کے لیے integration tests | Basic integration tests | No integration tests |
| **Launch File Quality** | Parameter overrides کے ساتھ comprehensive launch files؛ argument handling؛ visualization tools configured | Standard cases کے لیے launch files work | Incomplete launch configurations | No launch files or non-functional |

### D.1.5 Example ROS 2 Package Deliverables

Student packages میں مندرجہ ذیل deliverables شامل ہونے چاہیں:

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

Gazebo simulation assessment students کی ability کو evaluate کرتا ہے کہ وہ humanoid robot testing اور validation کے لیے physically accurate، performant، اور well-designed simulation environments create کریں۔

### D.2.1 World and Environment Design

simulation world کو robot کے operating environment کو accurately represent کرنا چاہیے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Physics Fidelity** | Real robot dynamics سے match کرتے ہوئے accurate physics parameters؛ properly tuned friction, damping, اور mass properties | Physics generally accurate with minor tuning needed | Physics present but unrealistic for key interactions | Physics unrealistic or broken |
| **Terrain Modeling** | Varied terrain types with proper surface properties؛ accurate slope modeling؛ real scenarios سے match کرنے والے obstacle configurations | Good terrain variety with mostly accurate properties | Basic terrain present but properties inconsistent | No meaningful terrain or flat plane only |
| **Lighting and Atmosphere** | Realistic lighting conditions؛ proper shadows؛ depth perception کے لیے atmospheric effects | Adequate lighting for visibility | Minimal lighting؛ poor visibility | No lighting or unusable conditions |
| **Sensor Simulation** | Camera, LiDAR, اور IMU simulation with realistic noise models؛ proper frame transformations | Major sensors simulated with noise models | Basic sensor models present | Sensors missing or non-functional |

### D.2.2 Robot Model Quality

robot model کو physical hardware کو accurately represent کرنا چاہیے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **URDF/XACRO Quality** | Clean, parameterized URDF with proper joint and link definitions؛ CAD data سے inertial parameters | Valid URDF with minor organizational issues | URDF present but has errors or warnings | Broken URDF or missing model |
| **Collision Geometry** | Accuracy اور performance balance کرتے ہوئے optimized collision geometry؛ proper friction coefficients | Collision geometry present and functional | Basic collision geometry | No collision geometry |
| **Visual Quality** | Professional appearance with appropriate materials؛ CAD-quality meshes؛ proper scaling and positioning | Acceptable visuals with minor issues | Basic visuals؛ scale or texture problems | Missing visuals or incorrect rendering |
| **Joint Configuration** | Proper joint limits, axis alignment, اور control interfaces hardware سے match کرتے ہیں | Mostly correct joints with minor issues | Some joint configuration errors | Broken joint configuration |

### D.2.3 Simulation Performance

real-time simulation اور algorithm development کے لیے performance critical ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Real-Time Factor** | Full simulation کے لیے consistent 1.0x یا better real-time factor؛ complex scenarios کے لیے optimized | Above 0.9x real-time factor؛ complex scenes میں minor lag | 0.7-0.9x real-time factor؛ noticeable lag | Below 0.7x real-time factor؛ unusable |
| **Frame Rate** | Visualization کے ساتھ stable 60+ FPS؛ physics-heavy scenarios کے لیے 30+ FPS | 30-60 FPS with visualization | 15-30 FPS؛ choppy visualization | Below 15 FPS؛ unusable |
| **Resource Efficiency** | Efficient mesh usage؛ distant objects کے لیے level-of-detail؛ proper physics stepping | Generally efficient with minor optimizations possible | Some performance bottlenecks | Severe performance issues |
| **Memory Usage** | Long simulations کے دوران stable memory consumption؛ no leaks؛ proper cleanup | Minor memory growth over long sessions | Noticeable memory increase | Memory leaks or unbounded growth |

### D.2.4 Plugin and Controller Integration

Simulation plugins کو control systems کے ساتھ properly interface کرنا چاہیے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Gazebo Plugins** | Robot interfaces کے لیے complete plugin implementation؛ accurate sensor simulation؛ proper state publishing | Plugins work but some advanced features missing | Basic plugins present | No plugins or non-functional |
| **Controller Integration** | ros2_control کے ساتھ seamless ROS 2 control integration؛ proper hardware abstraction؛ trajectory execution | Controller integration mostly complete | Basic controller integration | No controller integration |
| **State Estimation** | Simulation سے accurate state estimation؛ proper sensor fusion؛ realistic noise injection | State estimation mostly accurate | Basic state estimation | Broken state estimation |
| **Reset and Recovery** | Proper simulation reset handling؛ state persistence؛ failures سے recovery | Reset works but state may be lost | Partial reset functionality | Cannot reset simulation |

### D.2.5 Simulation Quality Metrics

مندرجہ ذیل metrics کو measure کریں اور report کریں:

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

Isaac Perceptual assessment students کی ability کو evaluate کرتا ہے کہ وہ NVIDIA Isaac use کرتے ہوئے humanoid robot applications کے لیے computer vision اور perception pipelines implement کریں۔

### D.3.1 Pipeline Architecture

Perception pipeline architecture system efficiency اور maintainability determine کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Modular Design** | Perception stages کے درمی clean separation؛ reusable components؛ proper data flow architecture | Well-organized with minor coupling | Basic separation but tight coupling | Monolithic perception code |
| **Buffer Management** | Configurable queue sizes کے ساتھ proper message buffering؛ thread-safe data handling | Mostly correct with minor timing issues | Buffer issues under load | Messages dropped؛ race conditions |
| **GPU Utilization** | Efficient CUDA kernel usage؛ proper memory management؛ GPU-CPU data transfer optimization | GPU acceleration present but not fully optimized | Some GPU usage | Entirely CPU-based؛ no GPU offload |
| **Latency Management** | End-to-end latency under 100ms؛ proper timestamping؛ latency compensation techniques | Latency acceptable with minor variations | High latency but functional | Unacceptable latency |

### D.3.2 Object Detection and Tracking

Object detection اور tracking fundamental perception capabilities ہیں:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Detection Accuracy** | Test set پر mAP > 0.85؛ precision > 0.90؛ recall > 0.80 | mAP 0.75-0.85؛ balanced precision/recall | mAP 0.60-0.75؛ significant precision/recall imbalance | mAP < 0.60؛ poor detection |
| **Real-Time Performance** | Full detection pipeline کے ساتھ 30+ FPS | 20-30 FPS detection | 10-20 FPS detection | < 10 FPS؛ unusable |
| **Multi-Object Tracking** | MOTA > 0.80؛ proper ID preservation؛ occlusion handling | MOTA 0.65-0.80؛ minor ID switches | MOTA 0.50-0.65؛ frequent ID switches | MOTA < 0.50؛ no tracking |
| **Robustness** | Lighting, viewpoints, اور occlusions پر consistent performance | Minor degradation under challenging conditions | Some conditions میں significant degradation | Only works under ideal conditions |

### D.3.3 Semantic Segmentation

Semantic understanding complex scene reasoning enable کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Segmentation Accuracy** | mIoU > 0.80؛ class-balanced performance | mIoU 0.70-0.80؛ some class imbalances | mIoU 0.55-0.70؛ significant class imbalances | mIoU < 0.55؛ poor segmentation |
| **Boundary Precision** | Proper edge refinement کے ساتھ clean boundaries؛ boundary IoU > 0.75 | Good boundaries with minor artifacts | Acceptable boundaries with noticeable artifacts | Poor boundaries؛ extensive artifacts |
| **Inference Speed** | 25+ FPS segmentation inference | 15-25 FPS inference | 8-15 FPS inference | < 8 FPS؛ unusable |
| **Class Coverage** | All relevant classes with balanced performance | All classes present with minor imbalances | Missing some classes or severe imbalances | Limited class coverage |

### D.3.4 Depth and 3D Perception

Three-dimensional perception humanoid manipulation اور navigation کے لیے essential ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Depth Accuracy** | 5m پر < 2% relative error؛ 2m پر < 1% | 5m پر 2-4% error | 5m پر 4-8% error | > 8% error؛ unusable |
| **3D Bounding Boxes** | BEV mAP > 0.80؛ proper orientation؛ accurate dimensions | BEV mAP 0.70-0.80 | BEV mAP 0.55-0.70 | BEV mAP < 0.55 |
| **Point Cloud Processing** | Efficient PCL processing؛ proper filtering and segmentation | Mostly correct with minor issues | Basic point cloud processing | Broken or missing point cloud |
| **Sensor Fusion** | Accurate multi-sensor calibration؛ proper temporal alignment؛ single sensor سے better fusion | Sensor fusion present but benefits limited | Basic fusion attempts | No sensor fusion |

### D.3.5 Isaac-Specific Implementation

Isaac-specific implementation platform expertise demonstrate کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|------------------|
| **Isaac SDK Usage** | Proper Isaac modules usage؛ optimized graph configuration؛ appropriate پر CUDA extensions | Isaac modules used correctly | Basic Isaac usage | No Isaac usage |
| **DNN Inference** | TensorRT کے ساتھ optimized DNN inference؛ proper precision selection (FP16/INT8)؛ batching support | DNN inference functional but not optimized | Basic DNN inference | No DNN or broken inference |
| **VPI Integration** | Vision processing کے لیے proper VPI algorithms؛ efficient pipeline | VPI used correctly | Basic VPI usage | No VPI integration |
| **Performance Profiling** | Comprehensive profiling؛ bottleneck identification؛ optimization recommendations | Profiling done but analysis limited | Some profiling attempted | No profiling |

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

capstone project course concepts کو comprehensive robot system میں integrate کرتا ہے۔ Assessment technical depth، system integration، اور professional deliverables evaluate کرتی ہے۔

### D.4.1 Project Scope and Objectives

capstone projects کو appropriate scope اور clear objectives demonstrate کرنے چاہییں:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Problem Definition** | Clear, well-motivated problem with significant technical challenges؛ real humanoid robotics needs address کرتا ہے | Well-defined problem with technical depth | Problem defined but scope unclear | Vague or trivial problem |
| **Scope Appropriateness** | Challenging but achievable within constraints؛ multiple course concepts کا meaningful integration | Appropriate scope with minor concerns | Scope slightly too large or small | Scope inappropriate |
| **Learning Objectives** | Course learning objectives کے ساتھ explicit alignment؛ clear skill demonstration | Most objectives addressed | Some objectives addressed | Limited alignment |
| **Innovation** | Novel approach or application؛ creative problem-solving؛ publication or research contribution کی potential | Some innovative elements | Mostly derivative but competent | No innovation apparent |

### D.4.2 Technical Implementation

Technical implementation quality project success determine کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **ROS 2 Architecture** | Professional-grade package design؛ scalable node structure؛ robust communication patterns | Solid architecture with minor issues | Basic architecture present | Architecture problems |
| **Simulation Fidelity** | High-fidelity Gazebo simulation jo real robot behavior کو accurately represent کرتا ہے | Good simulation with minor gaps | Basic simulation present | Poor or missing simulation |
| **Control Implementation** | Sophisticated control algorithms properly implemented؛ stability اور performance guarantees | Correct control implementation | Basic control present | Broken or missing control |
| **Perception Integration** | Working perception pipeline control کے ساتھ integrated؛ real-time performance؛ variations کے لیے robust | Perception integrated with minor issues | Basic perception integration | Missing perception |

### D.4.3 System Integration

System integration holistic engineering capability demonstrate کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Module Integration** | All system components کے درمی seamless integration؛ proper interfaces؛ data flow correctness | Good integration with minor issues | Some integration present | Poor integration |
| **Hardware-Software Bridge** | Simulation اور hardware کے درمان proper abstraction؛ clean migration path | Abstraction mostly correct | Basic abstraction present | No clear abstraction |
| **Error Handling** | Comprehensive error detection اور recovery؛ graceful degradation؛ fail-safe states | Good error handling with gaps | Basic error handling | No error handling |
| **System Testing** | Complete test coverage including integration اور system tests؛ automated testing pipeline | Good test coverage | Some testing present | No testing |

### D.4.4 Documentation and Reproducibility

Professional projects کے لیے comprehensive documentation required ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Technical Documentation** | Complete system documentation؛ API docs؛ architecture diagrams؛ deployment instructions | Good documentation with minor gaps | Basic documentation present | Incomplete or missing docs |
| **Code Documentation** | Comprehensive inline documentation؛ clear code organization؛ coding standards followed | Good code documentation | Basic code comments | No documentation |
| **Reproducibility** | Complete setup scripts؛ containerization؛ environment specification؛ step-by-step reproduction guide | Good reproducibility with minor issues | Basic setup instructions | Not reproducible |
| **User Guide** | Examples کے ساتھ clear user guide؛ troubleshooting section؛ FAQ | User guide mostly complete | Basic user guide | No user guide |

### D.4.5 Presentation Rubric

final presentation communication اور understanding demonstrate کرتی ہے:

| Criterion | Excellent (100%) | Proficient (85%) | Developing (70%) | Beginning (50%) |
|-----------|------------------|------------------|------------------|-----------------|
| **Problem Statement** | Clear motivation؛ contextual importance؛ stakeholder needs addressed | Clear problem statement | Problem stated but motivation weak | Problem unclear |
| **Technical Depth** | Deep technical understanding demonstrated؛ algorithm details explained؛ tradeoffs analyzed | Good technical depth | Surface-level technical coverage | Superficial treatment |
| **System Demo** | Impressive live or video demonstration؛ capabilities کی clear explanation؛ questions gracefully handle کرتا ہے | Good demo with minor issues | Demo present but problematic | No demo or failed demo |
| **Q&A Performance** | Confident, accurate responses؛ limitations acknowledge کرتا ہے؛ improvements suggest کرتا ہے | Good Q&A performance | Basic Q&A performance | Unable to answer questions |
| **Visual Aids** | Professional slides؛ clear diagrams؛ effective visualizations؛ appropriate level of detail | Good visual aids | Basic slides present | Poor or missing visuals |
| **Time Management** | Well-paced presentation؛ all material cover کرتا ہے؛ time limits respect کرتا ہے | Good time management | Some timing issues | Significant timing problems |

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

مندرجہ ذیل grading scale Physical AI اور Humanoid Robotics course میں all assessments پر apply ہوتی ہے۔

### D.5.1 Percentage to Letter Grade Conversion

| Percentage Range | Letter Grade | Grade Points | Description |
|------------------|--------------|--------------|-------------|
| 97-100% | A+ | 4.0 | Exceptional mastery؛ all expectations سے زیادہ |
| 93-96% | A | 4.0 | Outstanding performance؛ deep understanding demonstrate کرتا ہے |
| 90-92% | A- | 3.7 | Excellent work؛ minor areas for improvement |
| 87-89% | B+ | 3.3 | Very good work؛ solid understanding demonstrated |
| 83-86% | B | 3.0 | Good work؛ all requirements meet کرتا ہے |
| 80-82% | B- | 2.7 | Above average؛ some minor deficiencies |
| 77-79% | C+ | 2.3 | Satisfactory؛ basic requirements meet کرتا ہے |
| 73-76% | C | 2.0 | Adequate؛ minimum expectations meet کرتا ہے |
| 70-72% | C- | 1.7 | Marginal؛ several areas need improvement |
| 67-69% | D+ | 1.3 | Poor؛ minimum requirements barely meet کرتا ہے |
| 63-66% | D | 1.0 | Deficient؛ significant gaps in understanding |
| 60-62% | D- | 0.7 | Very deficient؛ barely passing |
| Below 60% | F | 0.0 | Failing؛ requirements meet نہیں کرتا |

### D.5.2 Assignment Type Weighting

course grade کو مندرجہ ذیل component weights use کرتے ہوئے compute کیا جاتا ہے:

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

Late work کو مندرجہ ذیل policy کے مطابق assess کیا جاتا ہے:

| Submission Time | Penalty |
|-----------------|---------|
| On time | No penalty |
| 1-24 hours late | 10% deduction |
| 24-48 hours late | 20% deduction |
| 48-72 hours late | 35% deduction |
| More than 72 hours late | 50% maximum (1 week کے بعد no credit) |

Extension requests کو deadline سے کم از کم 48 hours پہلے submit کرنا چاہیے سوائے documented emergencies کے۔ Extension grants 7 days سے زیادہ نہیں ہوں گے اور instructor کی approval سے ہوں گے۔

### D.5.5 Academic Integrity

All assessments مندرجہ ذیل academic integrity guidelines کے subject ہیں:

- **Individual Work**: Weekly labs اور perception implementations کو individually complete کرنا چاہیے unless explicitly stated otherwise۔
- **Group Work**: Capstone projects اور designated team assignments collaborative ہو سکتی ہیں۔
- **Code Reuse**: Students open-source code proper attribution اور license terms کے within use کر سکتے ہیں۔
- **Citation Requirement**: Any borrowed code, algorithms, یا concepts کو clearly cite کرنا چاہیے۔
- **AI Tools**: AI coding assistants کا use learning کے لیے permitted ہے لیکن submissions کو individual understanding demonstrate کرنا چاہیے۔

### D.5.6 Feedback and Revision

course feedback کے ذریعے iterative improvement پر emphasis دیتی ہے:

- **Lab Submissions**: 7 days میں feedback provide کی جائے گی؛ no revisions permitted۔
- **Major Projects**: 10 days میں feedback؛ original feedback سے 14 days کے within ایک optional revision (10% penalty)۔
- **Capstone**: Development کے دوران continuous feedback؛ final evaluation terminal ہے۔
- **Grade Disputes**: Grade return سے 7 days کے within written میں submit کرنا چاہیے۔

---

:::note Instructor Notes

یہ rubrics comprehensive yet flexible ہیں۔ instructors کو specific assignment requirements اور student skill levels کے مطابق criteria adapt کرنا چاہی۔ جب doubt میں ہو، تو assess کریں کہ work core concepts اور professional engineering practices کی understanding demonstrate کرتا ہے یا نہیں۔

:::

:::tip Best Practice

students اور assignments میں consistent evaluation ensure کرنے کے لیے rubric scorecard use کرنے پر غور کریں۔ ہر criterion کے لیے specific evidence record کریں تاکہ feedback اور grade disputes support ہو سکیں۔

:::

## References (حوالہ جات)

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Gazebo Sim Documentation: https://gazebosim.org/docs
- NVIDIA Isaac Documentation: https://docs.nvidia.com/isaac/
- ROS 2 Quality Assurance Standards: https://github.com/ros2/ros2/wiki/QA-Level
- Humanoid Robot Control Literature: https://arxiv.org/list/cs.RO/recent

---

**Appendix D** | [Appendix A: Hardware Specifications](appendix/hardware-specifications) | [Part 1: Introduction](part-1-foundations/introduction-to-physical-ai)
