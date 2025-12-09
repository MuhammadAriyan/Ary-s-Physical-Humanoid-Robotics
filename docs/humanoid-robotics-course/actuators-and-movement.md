---
title: "Chapter 3: Actuators and Movement Systems"
sidebar_position: 3
---

# Chapter 3: Actuators and Movement Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the comprehensive actuator ecosystem in humanoid robotics
- Explain the principles of bipedal locomotion and balance control
- Analyze different actuation technologies for human-like movement
- Design integrated movement systems for humanoid applications
- Evaluate the trade-offs between hydraulic, electric, and advanced actuator types

## Introduction

Actuators and movement systems form the muscular foundation of humanoid robotics, enabling robots to walk, grasp, and interact with the physical world. Just as human muscles convert metabolic energy into mechanical work, robotic actuators transform electrical, hydraulic, or pneumatic energy into controlled motion. This chapter explores the diverse actuation technologies that power modern humanoid robots, from high-power hydraulic systems to advanced bio-inspired artificial muscles.

### Humanoid Actuator Classification

```
     ┌─────────────────────────────────────┐
     │        HUMANOID ACTUATORS           │
     │                                     │
     │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │
     │  │Electric │ │Hydraulic│ │Pneumatic│ │
     │  │         │ │         │ │         │ │
     │  │• Motors │ │• Cylinders│ │• Artificial│ │
     │  │• Servos │ │• Pistons │ │  Muscles │ │
     │  │• Linear  │ │• Pumps   │ │• Soft    │ │
     │  │  actuators│ │• Valves  │ │  actuators│ │
     │  └─────────┘ └─────────┘ └─────────┘ │
     │                                     │
     │  ┌─────────┐ ┌─────────┐ ┌─────────┐ │
     │  │Smart    │ │Shape    │ │Bio-     │ │
     │  │Materials│ │Memory   │ │hybrid   │ │
     │  │         │ │Alloys   │ │Systems  │ │
     │  │• EAP    │ │• SMA    │ │• Tendon │ │
     │  │• IPMC   │ │• Nitinol│ │  Drive  │ │
     │  │• DE     │ │• Ferro  │ │• Muscle │ │
     │  │         │ │  fluids │ │  Like  │ │
     │  └─────────┘ └─────────┘ └─────────┘ │
     └─────────────────────────────────────┘
```

## Electric Actuators for Humanoid Robots

### 1. Brushless DC Motors
```
Characteristics:
┌─────────────────┐
│ Power Density:  │
│ • 200-500 W/kg  │
│ • High efficiency│
│ • Low maintenance│
├─────────────────┤
│ Control:        │
│ • Precise position│
│ • Speed control │
│ • Torque control│
│ • High bandwidth│
├─────────────────┤
│ Integration:    │
│ • Compact design│
│ • Lightweight   │
│ • Quiet operation│
│ • Clean energy  │
└─────────────────┘

Applications:
• Joint motors (shoulder, elbow, hip, knee)
• Hand actuators
• Neck movement
• Fine manipulation
```

### 2. Harmonic Drives
```
Design:
┌─────────────────┐
│ Wave Generator  │ ← Elliptical cam
│ (Input shaft)   │    • High reduction
│                 │    • Zero backlash
└─────────┬───────┘    • Compact size
          │
          ▼
┌─────────▼───────┐
│ Flex Spline     │ ← Flexible outer gear
│ (Flexible gear) │    • Deforms to match
│                 │    • Transmits motion
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│ Circular Spline │ ← Rigid outer gear
│ (Fixed gear)    │    • Internal teeth
│                 │    • Stationary
└─────────────────┘

Benefits:
• 50:1 to 200:1 reduction ratios
• High torque capacity
• Compact and lightweight
• Zero backlash
• High efficiency (85-95%)
```

### 3. Series Elastic Actuators (SEA)
```
Architecture:
┌─────────────────┐
│   Motor         │ ← High-speed motor
│  (High speed,   │    • Low torque
│   low torque)   │    • Lightweight
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Gearbox       │ ← Reduction stage
│  (Speed reduction)│   • Increase torque
│                 │   • Reduce speed
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Spring        │ ← Elastic element
│  (Force sensing,│   • Compliance
│   energy storage)│   • Shock absorption
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Output        │ ← Joint output
│  (Controlled    │    • Force control
│   force/position)│   • Safe interaction
└─────────────────┘

Advantages:
• Force control capability
• Shock absorption
• Energy efficiency
• Safe human interaction
• Backdrivability
```

## Hydraulic Actuators

### 1. Hydraulic Cylinders
```
Design:
┌─────────────────┐
│   Piston        │ ← Moving element
│  (Force output) │    • High force
│                 │    • Linear motion
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Cylinder      │ ← Pressure chamber
│  (Fluid chamber) │    • Sealed system
│                 │    • High pressure
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Fluid         │ ← Power transmission
│  (Hydraulic oil)│    • Incompressible
│                 │    • Power transmission
└─────────────────┘

Specifications:
• Force: 1000-10000 N
• Stroke: 50-500 mm
• Pressure: 20-35 MPa
• Response time: 10-50 ms

Applications:
• Leg actuators (hip, knee, ankle)
• Arm joints (shoulder, elbow)
• Heavy lifting
• High-power tasks
```

### 2. Hydraulic Systems
```
Complete System:
┌─────────────────┐
│   Pump          │ ← Power source
│  (Pressure      │    • Electric motor
│   generation)   │    • Fluid circulation
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Valves        │ ← Control system
│  (Flow control,│    • Direction control
│   pressure,     │    • Flow regulation
│   safety)       │    • Proportional control
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Actuators     │ ← Motion generation
│  (Cylinders,    │    • Linear/rotary
│   motors)       │    • High force
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Reservoir     │ ← Fluid storage
│  (Fluid storage,│    • Cooling
│   cooling,      │    • Filtration
│   filtration)   │    • Level control
└─────────────────┘

Benefits:
• Extremely high power density
• Precise force control
• Fast response
• Self-lubricating
• Heat dissipation
```

## Pneumatic Actuators

### 1. Artificial Muscles
```
McKibben Muscle Design:
┌─────────────────┐
│   Bladder       │ ← Inner rubber tube
│  (Inflatable    │    • Expands radially
│   chamber)      │    • Contracts axially
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Braid         │ ← Woven fiber sleeve
│  (Fiber mesh)   │    • Constrains expansion
│                 │    • Converts to linear
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   End Fittings  │ ← Attachment points
│  (Air inlet,    │    • Sealed connections
│   mounting)     │    • Load transfer
└─────────────────┘

Characteristics:
• Contraction: 20-30%
• Force: 100-2000 N
• Pressure: 0.4-0.8 MPa
• Response time: 50-200 ms

Advantages:
• Muscle-like behavior
• Lightweight
• Compliance
• Safe interaction
• Low cost
```

### 2. Pneumatic Systems
```
System Components:
┌─────────────────┐
│   Compressor    │ ← Air source
│  (Pressure      │    • Air compression
│   generation)   │    • Storage tank
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Air Treatment │ ← Air preparation
│  (Filtration,   │    • Remove moisture
│   drying,       │    • Oil removal
│   regulation)    │    • Pressure control
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Valves        │ ← Control system
│  (Directional,  │    • On/off control
│   proportional, │    • Flow regulation
│   servo)        │    • Pressure control
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Actuators     │ ← Motion generation
│  (Muscles,      │    • Linear contraction
│   cylinders)     │    • Compliance
└─────────────────┘

Benefits:
• Inherent compliance
• Clean operation
• Fast response
• Safe failure modes
• Low weight
```

## Advanced Actuation Technologies

### 1. Shape Memory Alloys (SMA)
```
Principle:
┌─────────────────┐
│   Cold State    │ ← Martensite phase
│  (Deformed      │    • Soft, easily deformed
│   shape)        │    • Low temperature
└─────────┬───────┘
          │ Heat
          ▼
┌─────────▼───────┐
│   Hot State     │ ← Austenite phase
│  (Original      │    • Rigid, remembers shape
│   shape)        │    • High temperature
└─────────────────┘

Materials:
• Nitinol (Ni-Ti alloy)
• Cu-Al-Ni alloys
• Fe-Mn-Si alloys

Properties:
• Strain recovery: 4-8%
• Stress: 200-800 MPa
• Temperature: 50-100°C
• Response time: 1-10 seconds

Applications:
• Small joint actuators
• Gripper fingers
• Micro-robots
• Morphing structures
```

### 2. Electroactive Polymers (EAP)
```
Dielectric Elastomers:
┌─────────────────┐
│   Electrodes    │ ← Conductive layers
│  (Carbon,       │    • Flexible
│   silver,       │    • Stretchable
│   graphene)     │    • Transparent
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Polymer       │ ← Active material
│  (Silicone,     │    • High dielectric
│   acrylic)      │    • Elastic
│                 │    • Large strain
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Electric      │ ← Actuation principle
│  Field          │    • Voltage: 1-10 kV
│  (Compression,  │    • Thickness reduction
│   expansion)    │    • Area expansion
└─────────────────┘

Performance:
• Strain: 10-300%
• Stress: 0.1-10 MPa
• Response time: 1-100 ms
• Efficiency: 60-80%

Applications:
• Artificial muscles
• Soft grippers
• Haptic devices
• Biomimetic robots
```

### 3. Tendon-Driven Systems
```
Architecture:
        ┌─────┐
        │Motor│ ← Actuator source
        └──┬──┘    • Electric motors
           │       • Hydraulic/pneumatic
           ▼
    ┌─────────────┐
    │   Tendon    │ ← Cable transmission
    │  (Steel,    │    • High strength
    │   synthetic) │    • Low weight
    └─────┬───────┘    • Flexible
          │
          ▼
    ┌─────────────┐
    │   Pulley     │ ← Routing system
    │  (Guides,    │    • Direction change
    │   tensioners)│    • Force multiplication
    └─────┬───────┘
          │
          ▼
    ┌─────────────┐
    │   Joint      │ ← Motion output
    │  (Rotation,  │    • Multi-DOF
    │   articulation)│  • Human-like movement
    └─────────────┘

Benefits:
• Remote actuation
• Weight reduction
• Human-like biomechanics
• High force-to-weight ratio
• Compact joint design
```

## Bipedal Locomotion

### 1. Walking Gait Cycle
```
Gait Phases:
    Stance Phase (60%):
    ┌─────────────────┐
    │ Heel Strike     │ ← Initial contact
    │ │               │    • Weight acceptance
    │ ▼               │    • Shock absorption
    │ ●─────●         │
    │       │         │
    │       ▼         │
    │ Foot Flat       │ ← Full contact
    │                 │    • Weight support
    │                 │    • Stability
    │                 │
    │ Mid Stance      │ ← Body over foot
    │                 │    • Maximum stability
    │                 │    • Forward progression
    │                 │
    │ Heel Off        │ ← Push-off preparation
    │       ▲         │    • Weight transfer
    │       │         │    • Propulsion
    │ ●─────●         │
    └─────────────────┘

    Swing Phase (40%):
    ┌─────────────────┐
    │ Toe Off        │ ← Leave ground
    │                 │    • Propulsion
    │                 │    • Leg clearance
    │                 │
    │ Initial Swing   │ ← Forward motion
    │                 │    • Foot clearance
    │                 │    • Acceleration
    │                 │
    │ Mid Swing      │ ← Maximum clearance
    │                 │    • Forward reach
    │                 │    • Balance
    │                 │
    │ Terminal Swing │ ← Deceleration
    │                 │    • Positioning
    │                 │    • Preparation
    └─────────────────┘
```

### 2. Balance Control
```
Zero Moment Point (ZMP):
    Stable Region:
    ┌─────────────────┐
    │   Support       │
    │   Polygon       │ ← Foot contact area
    │  ┌───────┐      │    • Stability boundary
    │  │       │      │    • ZMP must stay inside
    │  │   ●   │ ← ZMP │    • Center of pressure
    │  │       │      │    • Balance indicator
    │  └───────┘      │
    └─────────────────┘

    Unstable:
    ┌─────────────────┐
    │   Support       │
    │   Polygon       │
    │  ┌───────┐      │
    │  │       │      │
    │  │       │      │
    │  │       │      │
    │  └───────┘      │
    │       ● ← ZMP    │ ← Outside support
    └─────────────────┘    • Imminent fall

Balance Strategies:
• Ankle strategy (small disturbances)
• Hip strategy (medium disturbances)
• Step strategy (large disturbances)
```

### 3. Walking Patterns
```
Different Gaits:
    Normal Walking:
    ┌─┐   ┌─┐
    │ │   │ │ ← Alternating steps
    │ │   │ │    • 1.0-1.5 m/s
    └─┘   └─┘    • Natural rhythm

    Fast Walking:
    ┌─┐ ┌─┐
    │ │ │ │ ← Double support phase
    │ │ │ │    • 1.5-2.0 m/s
    └─┘ └─┘    • Reduced stability

    Running:
    ┌─┐   ┌─┐
    │ │   │ │ ← Flight phase
    │ │   │ │    • No double support
    └─┘   └─┘    • 2.0+ m/s

    Stair Climbing:
    ┌─┐
    │ │ ← Lift leg
    │ │    • Higher step
    └─┘    • More power
```

## Manipulation and Grasping

### 1. Hand Design
```
Human-like Hand Structure:
        ┌─────┐
        │Thumb│ ← Opposable
        │  ●  │    • 2-3 DOF
        └──┬──┘    • Critical for grasp
           │
    ┌──────┼──────┐
    │Index │Middle │ ← Fingers
    │  ●   │  ●   │    • 3-4 DOF each
    └──┬───┴──┬───┘    • Fine manipulation
       │       │
    ┌──┴───┐ ┌─┴───┐
    │Ring  │ │Pinky│ ← Little finger
    │  ●   │ │  ●  │    • 2-3 DOF
    └──────┘ └─────┘    • Stability

Total DOF: 20-25
Actuators: 15-20
Sensors: 50-100 tactile sensors
```

### 2. Grasp Types
```
Power Grasps:
┌─────────────────┐
│ Cylindrical     │ ← Cylindrical objects
│  ┌─────┐        │    • Power grip
│  │     │        │    • Multiple fingers
│  │ ● ● │        │    • High force
│  │     │        │
│  └─────┘        │
├─────────────────┤
│ Spherical       │ ← Spherical objects
│     ●●●         │    • Enveloping grasp
│    ●   ●        │    • Stability
│     ●●●         │    • Moderate force
└─────────────────┘

Precision Grasps:
┌─────────────────┐
│ Pinch           │ ← Small objects
│   ●   ●         │    • Thumb-index
│       │         │    • Fine control
│       │         │    • Low force
├─────────────────┤
│ Tripod          │ ← Medium objects
│   ●   ●         │    • Thumb-index-middle
│    ●  ●         │    • Stability + precision
│       │         │    • Moderate force
└─────────────────┘
```

## Actuator Selection and Design

### 1. Performance Metrics
```
Comparison Table:
┌─────────────────┬─────────┬─────────┬─────────┬─────────┐
│     Metric      │Electric │Hydraulic│Pneumatic│Advanced │
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Power Density   │  200-500│ 1000-5000│  50-200 │  100-300│
│   (W/kg)        │         │         │         │         │
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Efficiency      │   85-95 │   70-90 │   20-40 │   30-60 │
│   (%)           │         │         │         │         │
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Response Time   │  1-10ms │ 10-50ms │ 50-200ms│ 10-100ms│
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Control         │Excellent│   Good  │   Fair  │   Fair  │
│   Precision     │         │         │         │         │
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Compliance      │   Low   │   Low   │   High  │ Variable│
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Complexity      │   Low   │   High  │ Medium  │   High  │
├─────────────────┼─────────┼─────────┼─────────┼─────────┤
│ Cost            │   Low   │   High  │ Medium  │   High  │
└─────────────────┴─────────┴─────────┴─────────┴─────────┘
```

### 2. Design Considerations
```
Selection Criteria:
┌─────────────────┐
│ Application     │
│ • Required force │
│ • Speed range   │
│ • Precision     │
│ • Duty cycle    │
├─────────────────┤
│ Environment     │
│ • Size/weight   │
│ • Power source  │
│ • Temperature   │
│ • Cleanliness   │
├─────────────────┤
│ Safety          │
│ • Human interaction│
│ • Failure modes │
│ • Compliance    │
│ • Force limits  │
├─────────────────┤
│ Cost            │
│ • Initial cost  │
│ • Maintenance   │
│ • Energy consumption│
│ • Lifecycle     │
└─────────────────┘
```

## Energy Efficiency and Power Management

### 1. Energy Recovery Systems
```
Regenerative Systems:
┌─────────────────┐
│   Kinetic       │ ← Motion energy
│  Energy Recovery│    • Braking energy
│  (Deceleration, │    • Gravity assist
│   downhill)     │    • Inertial storage
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Storage       │ ← Energy storage
│  (Batteries,    │    • Capacitors
│   supercaps,    │    • Springs
│   flywheels)    │    • Compressed air
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Reuse         │ ← Energy utilization
│  (Acceleration,│    • Peak power
│   assistance,   │    • Extended range
│   peak shaving) │    • Efficiency
└─────────────────┘

Efficiency Gains:
• 10-30% energy recovery
• Extended battery life
• Reduced heat generation
• Smaller power systems
```

### 2. Power Management
```
Hierarchical Control:
┌─────────────────┐
│   High-Level    │ ← Mission planning
│  (Task planning,│    • Energy budgeting
│   gait selection)│   • Power allocation
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Mid-Level     │ ← Motion control
│  (Trajectory,   │    • Efficient paths
│   force control)│   • Power optimization
└─────────┬───────┘
          │
          ▼
┌─────────▼───────┐
│   Low-Level     │ ← Actuator control
│  (Motor control,│    • Efficiency modes
│   valve control)│   • Sleep states
└─────────────────┘

Strategies:
• Variable impedance control
• Gravity compensation
• Cocontraction reduction
• Sleep modes
• Predictive control
```

## Chapter Summary

### Key Takeaways:
1. **Multiple actuator types** are needed for different humanoid robot functions
2. **Electric actuators** provide precision and control for manipulation
3. **Hydraulic systems** deliver high power for locomotion
4. **Compliance** is essential for safe human interaction
5. **Energy efficiency** is critical for practical deployment

### Important Terms:
- **DOF (Degrees of Freedom)**: Independent movement capabilities
- **ZMP (Zero Moment Point)**: Balance stability metric
- **SEA (Series Elastic Actuator)**: Compliant actuator design
- **SMA (Shape Memory Alloy)**: Thermally activated actuator
- **EAP (Electroactive Polymer)**: Electrically activated polymer

### Next Chapter Preview:
In the next chapter, we'll explore **Humanoid Control Systems** - the sophisticated algorithms and architectures that coordinate all actuators and sensors to achieve coherent, intelligent behavior in humanoid robots.

## Review Questions

1. Why do humanoid robots need different types of actuators?
2. How do series elastic actuators improve safety in human-robot interaction?
3. What are the main challenges of bipedal locomotion?
4. Compare the advantages and disadvantages of hydraulic vs electric actuators.
5. How does balance control work in humanoid robots?
6. What role does compliance play in humanoid robot actuation?

## Practical Exercise

**Design an Actuation System:**
Choose a humanoid robot subsystem (arm, leg, hand) and design an actuation system considering:
- Required force and speed specifications
- Actuator type selection and justification
- Power requirements and efficiency
- Safety considerations
- Control system integration

This will help you apply actuation concepts to real-world humanoid robot design challenges!