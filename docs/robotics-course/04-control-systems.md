---
title: 04. Control Systems
sidebar_position: 4
---

# 04. Control Systems

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand control system architecture and feedback loops
- Design and tune PID controllers for robot applications
- Analyze advanced control techniques including MPC and adaptive control
- Implement real-time control systems for robotics

## Introduction to Control Systems

**Control systems are the brain of a robot** - they process sensor data and command actuators to achieve desired behaviors. Just as human brain coordinates muscles based on sensory input, robot controllers coordinate actuators based on sensor feedback.

### Control System Hierarchy

```
    ┌─────────────────┐
    │   High-Level    │
    │   Planning      │ ← Mission planning
    │   (What to do)  │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Path          │
    │   Planning      │ ← Trajectory generation
    │   (How to move) │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Motion        │
    │   Control      │ ← Low-level control
    │   (Execute move) │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Actuators     │ ← Physical execution
    │   (Muscles)     │
    └─────────────────┘
```

## Feedback Control Fundamentals

### Basic Feedback Loop

```
    Reference → [Controller] → Plant → Output
       ▲                                    │
       │                                    ▼
       └────── [Sensor] ←───────────────────┘
    
    Components:
    • Reference: Desired state/position
    • Controller: Makes decisions
    • Plant: System being controlled
    • Sensor: Measures actual state
    • Output: Actual result
```

### Why Feedback is Essential

```
    Without Feedback (Open-Loop):
    Command: "Move 10cm"
    Result: 8-12cm (depends on load, friction)
    
    With Feedback (Closed-Loop):
    Command: "Move 10cm"
    Measure: 8cm
    Adjust: "Move 2cm more"
    Result: 10cm (accurate)
```

## PID Controllers

### PID Structure

PID (Proportional-Integral-Derivative) is the most widely used control algorithm in robotics:

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
```

### Understanding Each Term

#### Proportional (P) Term
```
    Output = Kp × Error
    
    Characteristics:
    • Responds to current error
    • Larger Kp = faster response
    • Too large Kp = oscillation
    
    Example:
    Error = 5cm, Kp = 2
    Output = 10 units of control
```

#### Integral (I) Term
```
    Output = Ki × ∫Error dt
    
    Characteristics:
    • Eliminates steady-state error
    • Accumulates past errors
    • Can cause windup
    
    Example:
    Persistent small error
    → Integral term grows
    → Eliminates error
```

#### Derivative (D) Term
```
    Output = Kd × d(Error)/dt
    
    Characteristics:
    • Predicts future error
    • Reduces overshoot
    • Sensitive to noise
    
    Example:
    Error changing rapidly
    → Large derivative term
    → Damps response
```

### PID Tuning Response

```
    Setpoint: ────────────────────────
    
    Underdamped (Kp too high):
    ╱╲    ╱╲    ╱╲
   ╱  ╲  ╱  ╲  ╱  ╲
  ╱    ╲╱    ╲╱    ╲
    
    Critically Damped (Optimal):
    ╱╲
   ╱  ╲
  ╱    ╲
 ╱      ╲
╱        ╲
    
    Overdamped (Kp too low):
    ╱╲
   ╱  ╲
  ╱    ╲
 ╱      ╲
╱        ╲
```

### Tuning Methods

#### Manual Tuning
```
    Step 1: Set Ki = 0, Kd = 0
    Step 2: Increase Kp until oscillation
    Step 3: Reduce Kp by half
    Step 4: Increase Ki to eliminate steady-state error
    Step 5: Increase Kd to reduce overshoot
```

#### Ziegler-Nichols Method
```
    Step 1: Set Ki = 0, Kd = 0
    Step 2: Increase Kp until sustained oscillation
    Step 3: Note Ku (ultimate gain) and Tu (period)
    Step 4: Use Z-N table:
    
    Controller | Kp       | Ti       | Td
    P         | 0.5·Ku   | -        | -
    PI        | 0.45·Ku  | Tu/1.2   | -
    PID       | 0.6·Ku   | Tu/2     | Tu/8
```

## Advanced Control Techniques

### Model Predictive Control (MPC)

MPC predicts future behavior and optimizes control actions:

```
    Current State
        │
        ▼
    ┌─────────────────┐
    │   Prediction     │
    │   Horizon       │
    │  t → t+N        │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Optimization  │
    │   (Cost Func)   │
    │   Minimize:     │
    │   • Error       │
    │   • Control     │
    │   • Effort      │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Control       │
    │   Sequence      │
    └─────────┬───────┘
              │
              ▼
    Apply First Control Action
```

### Adaptive Control

Adapts to changing system parameters:

```
    ┌─────────────────┐
    │   Plant         │
    │   Dynamics      │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Parameter     │
    │   Estimation    │
    │   (RLS, EKF)    │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Controller    │
    │   Adaptation    │
    │   (Gain         │
    │   Scheduling)   │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Updated        │
    │   Control Law    │
    └─────────────────┘
```

### Robust Control

Handles uncertainties and disturbances:

```
    ┌─────────────────┐
    │   Nominal       │
    │   Controller    │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Robustness    │
    │   Analysis      │
    │   (H-infinity,  │
    │   Sliding Mode) │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Guaranteed    │
    │   Performance   │
    │   Despite       │
    │   Uncertainty   │
    └─────────────────┘
```

## State Space Control

### State Space Representation

Modern control theory uses state space formulation:

```
    State Equation:  ẋ = Ax + Bu
    Output Equation: y = Cx + Du
    
    Where:
    x = State vector
    u = Control input
    y = Output
    A, B, C, D = System matrices
    
    Example: Inverted Pendulum
    States: [θ, θ̇, x, ẋ]
    Input: [F]
    Output: [θ, x]
```

### State Observer (Kalman Filter)

Estimates system states from measurements:

```
    ┌─────────────────┐
    │   System        │
    │   Dynamics      │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Measurement   │
    │   + Noise       │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Kalman        │
    │   Filter        │
    │   (Predict     │
    │    + Update)   │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Estimated     │
    │   State        │
    └─────────────────┘
```

### LQR Control

Linear Quadratic Regulator provides optimal control:

```
    Cost Function:
    J = ∫(xᵀQx + uᵀRu)dt
    
    Where:
    Q = State cost matrix
    R = Control cost matrix
    
    Solution:
    u = -Kx
    K = R⁻¹BᵀP
    P solves Riccati equation
```

## Robot-Specific Control

### Jacobian Control

For robot arms, use Jacobian matrix:

```
    Joint Space → [Jacobian] → Cartesian Space
        θ              J           x
    
    ┌─────────────────┐
    │   Joint         │
    │   Angles        │
    │   [θ1, θ2, θ3]  │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Jacobian      │
    │   Matrix J      │
    │   ∂x/∂θ        │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   End-Effector  │
    │   Velocity      │
    │   [vx, vy, vz]  │
    └─────────────────┘
```

### Impedance Control

Controls force and position relationship:

```
    Desired → [Impedance] → Force
    Position    Controller    Output
    
    ┌─────────────────┐
    │   Stiffness     │
    │   K             │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Damping       │
    │   B             │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Inertia       │
    │   M             │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Force         │
    │   Output        │
    └─────────────────┘
    
    Equation: F = M·ẍ + B·ẋ + K·x
```

### Computed Torque Control

For robot arm dynamics:

```
    Desired → [Inverse     ] → Torque
    Trajectory  Dynamics    Commands
    
    Robot Dynamics:
    τ = M(q)q̈ + C(q,q̇)q̇ + G(q)
    
    Control Law:
    τ = M(q)(q̈d + Kv·ė + Kp·e) + C(q,q̇)q̇ + G(q)
```

## Real-Time Control Systems

### Real-Time Operating System (RTOS)

RTOS ensures deterministic timing:

```
    ┌─────────────────┐
    │   High Priority │
    │   (Safety)      │ ← 1 kHz
    │   • Emergency   │
    │     Stop       │
    └─────────────────┘
    
    ┌─────────────────┐
    │   Medium        │
    │   (Control)     │ ← 100 Hz
    │   • PID         │
    │   • Trajectory  │
    └─────────────────┘
    
    ┌─────────────────┐
    │   Low Priority  │
    │   (Planning)    │ ← 10 Hz
    │   • Path        │
    │   • Navigation  │
    └─────────────────┘
```

### Control Loop Timing

```
    Time →
    ┌───┬───┬───┬───┬───┬───┬───┬───┐
    │ S │ C │ S │ C │ S │ C │ S │ C │
    └───┴───┴───┴───┴───┴───┴───┴───┘
    
    S = Sensor reading
    C = Control computation
    
    Requirements:
    • Period: 1-10ms for motion control
    • Jitter: < 10% of period
    • Latency: < 1ms for safety
```

### Hardware Considerations

```
    Processor Requirements:
    • Speed: >100 MHz for basic control
    • FPU: Floating point unit
    • ADC/DAC: Analog/digital conversion
    • Timers: Precise timing
    • Interrupts: Fast response
    
    Communication:
    • CAN bus: Industrial control
    • Ethernet: High-speed data
    • SPI/I2C: Sensor communication
```

## Fault Detection and Recovery

### Fault Detection

```
    ┌─────────────────┐
    │   Sensor        │
    │   Monitoring    │
    │   • Range check │
    │   • Rate limit  │
    │   • Consistency │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Redundancy    │
    │   Check         │
    │   • Voting      │
    │   • Cross-check │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Fault         │
    │   Detection     │
    │   • Isolation   │
    │   • Identification│
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Alarm         │
    │   System        │
    └─────────────────┘
```

### Recovery Strategies

```
    Fault Detected
          │
          ▼
    ┌─────────────────┐
    │   Safe Mode     │ ← Stop motion
    │   • Brakes on   │
    │   • Power down  │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Backup        │ ← Use redundant
    │   System        │   system
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Degraded      │ ← Reduced
    │   Operation     │   capability
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Shutdown      │ ← Safe
    │   Procedure     │   shutdown
    └─────────────────┘
```

## Performance Metrics

### Time Domain Metrics

```
    Step Response:
    Setpoint: ────────────────
    
    Response:  ╱╲
              ╱  ╲
             ╱    ╲─────────────
            ╱      ╲
           ╱        ╲
    
    Metrics:
    • Rise time: 10% → 90%
    • Settling time: Within ±2%
    • Overshoot: Maximum deviation
    • Steady-state error: Final error
```

### Frequency Domain Metrics

```
    Bode Plot:
    Gain (dB)
      │
      │    ┌─────┐
      │   ╱       ╲
      │  ╱         ╲
      │ ╱           ╲
    ──┼───────────────── Frequency
      │
      │
    Phase (degrees)
      │
      │
      │
    ──┼───────────────── Frequency
    
    Metrics:
    • Bandwidth: -3dB point
    • Phase margin: Stability
    • Gain margin: Robustness
```

## Implementation Examples

### Mobile Robot Control

```
    Architecture:
    ┌─────────────────┐
    │   Path          │
    │   Following     │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Velocity      │
    │   Control      │
    │   (PID)        │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Motor         │
    │   Drivers       │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Wheels        │
    └─────────────────┘
```

### Robot Arm Control

```
    Cascade Control:
    Position → [Position PID] → Velocity Ref
       ▲                                            │
       │                                            ▼
       └────── [Position Sensor] ←─────────────────┘
    
    Velocity Ref → [Velocity PID] → Torque Command
         ▲                                            │
         │                                            ▼
         └────── [Velocity Sensor] ←─────────────────┘
    
    Torque → [Current Control] → Motor Current
       ▲                                            │
       │                                            ▼
       └────── [Current Sensor] ←─────────────────┘
```

## Best Practices

### Controller Design

1. **Model the System**: Understand dynamics first
2. **Start Simple**: Begin with PID, advance if needed
3. **Simulate First**: Test before implementation
4. **Tune Systematically**: Use proven methods
5. **Validate Extensively**: Test all conditions

### Implementation

1. **Use Fixed-Point**: For embedded systems
2. **Handle Saturation**: Prevent windup
3. **Filter Measurements**: Reduce noise
4. **Implement Limits**: Safety constraints
5. **Monitor Performance**: Real-time metrics

### Safety

1. **Emergency Stop**: Always available
2. **Redundancy**: Critical systems
3. **Fail-Safe**: Default to safe state
4. **Testing**: Comprehensive validation
5. **Documentation**: Clear procedures

## Chapter Summary

### Key Takeaways:
1. **Control systems coordinate robot behavior** through feedback
2. **PID controllers** are widely used and effective
3. **Advanced techniques** like MPC handle complex systems
4. **Real-time constraints** are critical for performance
5. **Safety and reliability** are essential considerations

### Important Terms:
- **Feedback**: Using sensor output to adjust control
- **PID**: Proportional-Integral-Derivative control
- **State Space**: Mathematical representation of system dynamics
- **Real-Time**: Deterministic timing requirements
- **Fault Tolerance**: Ability to handle failures

### Next Chapter Preview:
In the next chapter, we'll explore **Path Planning and Navigation** - how robots plan and execute movements through complex environments.

## Review Questions

1. Explain the role of feedback in control systems with examples.
2. Describe each term in PID control and its effect on system response.
3. Compare open-loop vs closed-loop control systems.
4. What are the advantages of Model Predictive Control?
5. How does a Kalman filter work in state estimation?
6. Why are real-time constraints important in robot control?

## Practical Exercise

**PID Controller Implementation:**
Design a PID controller for a DC motor position control system:
1. Define system requirements (settling time, overshoot)
2. Choose initial PID gains
3. Simulate step response
4. Tune gains for optimal performance
5. Test disturbance rejection

This exercise will help you understand practical control system design!