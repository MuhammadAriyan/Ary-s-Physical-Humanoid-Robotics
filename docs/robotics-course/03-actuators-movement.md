---
title: 03. Actuators and Movement
sidebar_position: 3
---

# 03. Actuators and Movement

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand different types of actuators and their characteristics
- Explain robot kinematics and degrees of freedom
- Analyze control systems for robot movement
- Design actuation systems for robotic applications

## Introduction to Actuators

**Actuators are the muscles of a robot** - they convert energy into physical motion to perform tasks and interact with the environment. Just as human muscles enable movement, actuators enable robots to move, grasp, walk, and manipulate objects.

### Actuator Classification

```
    ┌─────────────────────────────────────┐
    │            ACTUATORS                │
    │                                     │
    │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │
    │  │Electric │ │Hydraulic│ │Pneumatic│ │
    │  │         │ │         │ │         │ │
    │  │• Motors │ │• Cylinders│ │• Cylinders│ │
    │  │• Servos │ │• Pistons │ │• Diaphragms│ │
    │  │• Solenoids│ │• Pumps   │ │• Valves  │ │
    │  └─────────┘ └─────────┘ └─────────┘ │
    │                                     │
    │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │
    │  │Smart    │ │Shape    │ │Bio-     │ │
    │  │Materials│ │Memory   │ │hybrid   │ │
    │  │         │ │Alloys   │ │Actuators│ │
    │  └─────────┘ └─────────┘ └─────────┘ │
    └─────────────────────────────────────┘
```

## Electric Actuators

### 1. DC Motors
Convert electrical energy into rotational motion:

```
    ┌─────────────────┐
    │   +     -       │ ← Power terminals
    │                 │
    │    ┌───────┐    │
    │    │Rotor  │    │ ← Rotating part
    │    │       │    │
    │    └───────┘    │
    │         │       │
    │      ┌──┴──┐    │
    │      │Shaft│    │ ← Output
    │      └────┘    │
    └─────────────────┘
    
    Characteristics:
    • Speed proportional to voltage
    • Torque proportional to current
    • Simple control
    • Good efficiency
    
    Applications:
    • Wheels in mobile robots
    • Conveyor belts
    • Fans and pumps
```

### 2. Stepper Motors
Move in precise angular steps:

```
    Step 1    Step 2    Step 3    Step 4
      ●         ●         ●         ●
    ┌─┴─┐     ┌─┴─┐     ┌─┴─┐     ┌─┴─┐
    │   │ →   │   │ →   │   │ →   │   │
    └─┬─┘     └─┬─┘     └─┬─┘     └─┬─┘
      ●         ●         ●         ●
    
    Characteristics:
    • Precise position control
    • No feedback needed (open-loop)
    • High torque at low speed
    • Can hold position
    
    Applications:
    • 3D printers
    • CNC machines
    • Camera positioning
    • Robot joint control
```

### 3. Servo Motors
Provide precise angular position control:

```
    ┌─────────────────┐
    │   Position      │
    │   Feedback      │ ← Potentiometer
    │       │         │
    │       ▼         │
    │   ┌───────┐     │
    │   │Motor  │     │
    │   └───────┘     │
    │       │         │
    │   ┌───┴───┐     │
    │   │Gear   │     │ ← Reduction
    │   │Train  │     │
    │   └───┬───┘     │
    │       │         │
    │   ┌───┴───┐     │
    │   │Output │     │ ← Controlled position
    │   │Shaft  │     │
    │   └───────┘     │
    └─────────────────┘
    
    Control Signal:
    PWM (Pulse Width Modulation)
    1.5ms = Center position
    1.0ms = 0° position
    2.0ms = 180° position
    
    Applications:
    • Robot arms
    • RC vehicles
    • Camera gimbals
    • Steering systems
```

## Hydraulic Actuators

Use pressurized fluid for high-force applications:

```
    ┌─────────────────┐
    │   Fluid In      │ ← High pressure
    │       │         │
    │       ▼         │
    │   ┌───────┐     │
    │   │Piston │     │
    │   │  ↔    │     │ ← Linear motion
    │   └───────┘     │
    │       │         │
    │       ▼         │
    │   Fluid Out      │ ← Low pressure
    └─────────────────┘
    
    Characteristics:
    • Very high force capability
    • Precise control
    • Self-lubricating
    • Can handle heavy loads
    
    Applications:
    • Excavators
    • Industrial presses
    • Aircraft control surfaces
    • Heavy-duty robots
```

### Hydraulic System Components

```
    ┌─────────┐    ┌─────────┐    ┌─────────┐
    │ Pump    │ →  │ Valve   │ →  │Cylinder │
    │         │    │         │    │         │
    │ Creates │    │ Controls│    │ Provides │
    │ Pressure│    │ Flow    │    │ Motion  │
    └─────────┘    └─────────┘    └─────────┘
         ↑                              ↓
         └──────────────────────────────┘
                Return Line
```

## Pneumatic Actuators

Use compressed air for clean, fast actuation:

```
    ┌─────────────────┐
    │   Air In        │ ← Compressed air
    │       │         │
    │       ▼         │
    │   ┌───────┐     │
    │   │Diaphragm│   │
    │   │   ↑    │     │ ← Flexible membrane
    │   └───────┘     │
    │       │         │
    │       ▼         │
    │   Air Out       │ ← Exhaust
    └─────────────────┘
    
    Characteristics:
    • Fast response
    • Clean operation
    • Lightweight
    • Lower force than hydraulic
    
    Applications:
    • Factory automation
    • Medical devices
    • Food processing
    • Soft robotics
```

## Robot Arm Kinematics

### Degrees of Freedom (DOF)
Each independent movement a robot can make:

```
    6-DOF Robot Arm:
        Joint 1 (Base)
            │
            ▼
        ┌─────┐
        │  ●  │ ← Rotation
        └─────┘
            │
            ▼
        ┌─────┐ Joint 2
        │  ●  │ ← Shoulder
        └─────┘
            │
            ▼
        ┌─────┐ Joint 3
        │  ●  │ ← Elbow
        └─────┘
            │
            ▼
        ┌─────┐ Joint 4
        │  ●  │ ← Wrist pitch
        └─────┘
            │
            ▼
        ┌─────┐ Joint 5
        │  ●  │ ← Wrist roll
        └─────┘
            │
            ▼
        ┌─────┐ Joint 6
        │  ●  │ ← Wrist yaw
        └─────┘
            │
            ▼
        ┌─────┐
        │Tool │ ← End effector
        └─────┘
```

### Forward Kinematics
Calculate end-effector position from joint angles:

```
    Joint Angles → End-Effector Position
    
    θ1, θ2, θ3 → x, y, z, roll, pitch, yaw
    
    Transformation:
    T = T01 × T12 × T23 × ... × Tn-1,n
    
    Where Tij is transformation from joint i to joint j
```

### Inverse Kinematics
Calculate joint angles for desired position:

```
    Desired Position → Joint Angles
    
    x, y, z, roll, pitch, yaw → θ1, θ2, θ3
    
    Challenges:
    • Multiple solutions
    • No solution (out of reach)
    • Singularities
    • Computational complexity
```

## Workspace Analysis

### Reachable Workspace
All positions the robot can reach:

```
    Top View:
            ●
          ●   ●
        ●       ●
      ●           ●
    ●               ●
      ●           ●
        ●       ●
          ●   ●
            ●
    
    Side View:
        ●
       ● ●
      ●   ●
     ●     ●
    ●       ●
```

### Dexterity
Ability to achieve different orientations:

```
    High Dexterity:
    ●─●─●─●─●
    
    Low Dexterity:
    ●─────●
```

## End Effectors

### Gripper Types

#### Parallel Gripper
Two fingers move in parallel:

```
    ┌─────────────────┐
    │                 │
    │  ╱╲        ╱╲   │ ← Fingers
    │ ╱  ╲      ╱  ╲  │
    │╱    ╲    ╱    ╲ │
    │      ╲  ╱      │ │
    │       ╲╱       │ │
    └─────────────────┘
    
    Characteristics:
    • Precise gripping
    • Suitable for regular shapes
    • Simple control
```

#### Vacuum Gripper
Uses suction to lift objects:

```
    ┌─────────────────┐
    │   Vacuum        │
    │   ┌─────┐       │
    │   │  ●  │       │ ← Suction cup
    │   └─────┘       │
    │       │         │
    │       ▼         │
    │   Object        │
    └─────────────────┘
    
    Characteristics:
    • Good for flat surfaces
    • Fast operation
    • Gentle handling
```

#### Magnetic Gripper
Uses magnetic force:

```
    ┌─────────────────┐
    │   Electromagnet │
    │   ┌───────┐     │
    │   │ N   S │     │ ← Magnetic field
    │   └───────┘     │
    │       │         │
    │       ▼         │
    │   Metal Object  │
    └─────────────────┘
    
    Characteristics:
    • Only for ferromagnetic materials
    • Can be turned on/off
    • No mechanical contact
```

## Mobile Robot Locomotion

### Wheeled Robots
Most common for flat surfaces:

```
    Differential Drive:
    ┌─────────────────┐
    │                 │
    │    ┌─────┐      │
    │    │Body │      │
    │    └─────┘      │
    │   O         O   │ ← Independent wheels
    └─────────────────┘
    
    Movement:
    • Forward: Both wheels same speed
    • Turn: Different wheel speeds
    • Rotate: Opposite wheel speeds
```

### Tracked Robots
For rough terrain:

```
    ┌─────────────────┐
    │                 │
    │    ┌─────┐      │
    │    │Body │      │
    │    └─────┘      │
    │ ═════════════   │ ← Continuous tracks
    └─────────────────┘
    
    Advantages:
    • Good traction
    • Can handle rough terrain
    • Low ground pressure
```

### Legged Robots
For complex environments:

```
    Quadruped Robot:
    ┌─────┐
    │Body │
    └─────┘
     │ │ │ │
     ▼ ▼ ▼ ▼
    ┌─┐ ┌─┐ ┌─┐ ┌─┐
    │ │ │ │ │ │ │ │ ← Four legs
    └─┘ └─┘ └─┘ └─┘
    
    Gaits:
    • Walk: One leg at a time
    • Trot: Diagonal pairs
    • Pace: Same side pairs
    • Bound: Front/rear pairs
```

## Control Systems for Actuators

### Open-Loop Control
No feedback on position or speed:

```
    Command → [Motor Driver] → Actuator → Motion
    
    Example:
    "Move forward 2 seconds" → Motor runs for 2 seconds
    
    Disadvantages:
    • No position accuracy
    • No error correction
    • Affected by load changes
```

### Closed-Loop Control
Uses feedback for precise control:

```
    Command → [Controller] → Actuator → Motion
      ↑                                      │
      └──── [Feedback] ← [Sensor] ←─────────┘
    
    Example:
    "Move to position X" → Controller adjusts until reached
    
    Advantages:
    • Precise positioning
    • Error correction
    • Adapts to load changes
```

### PID Control
Most common control algorithm:

```
    Error(t) = Setpoint - Measured
    
    ┌─────────────────────────────────────────┐
    │                                         │
    │   Proportional  Integral   Derivative    │
    │      │            │          │          │
    │      ▼            ▼          ▼          │
    │   Kp·e(t)    Ki·∫e(t)dt  Kd·de(t)/dt   │
    │      │            │          │          │
    │      └─────┬──────┴───────┬──┘          │
    │            ▼              ▼            │
    │         ┌─────────────────┐            │
    │         │   Summation     │            │
    │         └─────────────────┘            │
    │                  │                     │
    │                  ▼                     │
    │            Control Output               │
    │                                         │
    └─────────────────────────────────────────┘
    
    Tuning:
    • Kp: Response speed
    • Ki: Steady-state error
    • Kd: Damping/overshoot
```

## Performance Metrics

### Force and Torque
```
    Force: Linear push/pull (Newtons)
    Torque: Rotational force (Newton-meters)
    
    Example:
    10N force = 1kg weight
    10Nm torque = 1kg at 1m radius
```

### Speed and Precision
```
    Speed: How fast the actuator moves
    Precision: How accurately it reaches target
    Repeatability: Consistency of movements
    
    Trade-offs:
    • High speed → Lower precision
    • High precision → Lower speed
```

### Power and Efficiency
```
    Power = Force × Velocity
    Efficiency = Output Power / Input Power
    
    Typical efficiencies:
    • Electric motors: 80-95%
    • Hydraulic systems: 60-80%
    • Pneumatic systems: 20-40%
```

## Design Considerations

### Load Requirements
```
    Static Load: Weight the actuator must hold
    Dynamic Load: Forces during movement
    Inertial Load: Resistance to acceleration
    
    Safety Factor: Design for 2-3× expected load
```

### Speed vs Torque
```
    High Speed, Low Torque:
    ┌─────┐
    │     │
    │     │
    └─────┘
    
    Low Speed, High Torque:
    ┌─────────┐
    │         │
    │         │
    └─────────┘
    
    Gear ratios can trade speed for torque
```

### Environmental Factors
```
    Temperature: Affects motor performance
    Humidity: Can cause corrosion
    Dust: Requires sealing
    Vibration: Needs robust mounting
```

## Applications and Examples

### Industrial Robotics
```
    6-DOF Articulated Arm:
    • Payload: 5-200kg
    • Reach: 0.5-3.5m
    • Repeatability: ±0.02mm
    • Applications: Welding, painting, assembly
```

### Service Robotics
```
    Humanoid Robot:
    • 25+ DOF
    • Height: 1.2-1.8m
    • Weight: 50-100kg
    • Applications: Assistance, entertainment
```

### Medical Robotics
```
    Surgical Robot:
    • 7 DOF per arm
    • Tremor filtering
    • Motion scaling
    • Applications: Minimally invasive surgery
```

## Chapter Summary

### Key Takeaways:
1. **Actuators convert energy to motion** - they are robot muscles
2. **Different actuator types** serve different purposes (electric, hydraulic, pneumatic)
3. **Kinematics describes robot motion** without considering forces
4. **Control systems ensure precise movement** through feedback
5. **End effectors enable interaction** with the environment

### Important Terms:
- **DOF**: Degree of Freedom - independent movement
- **Forward Kinematics**: Joint angles → End-effector position
- **Inverse Kinematics**: Desired position → Joint angles
- **PID Control**: Proportional-Integral-Derivative control
- **Workspace**: Reachable positions of robot

### Next Chapter Preview:
In the next chapter, we'll explore **Control Systems** in depth - how robots make decisions and execute complex behaviors through advanced control algorithms.

## Review Questions

1. What are the main types of actuators and their characteristics?
2. Explain the difference between open-loop and closed-loop control.
3. Describe forward and inverse kinematics with examples.
4. How does a servo motor differ from a stepper motor?
5. What factors should be considered when selecting actuators?
6. Explain PID control and the role of each term.

## Practical Exercise

**Robot Arm Design:**
Design a 3-DOF robot arm for picking and placing objects:
1. Choose appropriate actuators for each joint
2. Calculate the workspace
3. Design a suitable end effector
4. Specify the control system requirements

This exercise will help you apply actuator concepts to real robot design!