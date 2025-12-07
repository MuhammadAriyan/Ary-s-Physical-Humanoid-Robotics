---
title: 05. Path Planning and Navigation
sidebar_position: 5
---

# 05. Path Planning and Navigation

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamental concepts of path planning and navigation
- Implement various path planning algorithms for different scenarios
- Analyze the trade-offs between different planning approaches
- Design navigation systems for mobile robots

## Introduction to Path Planning

**Path planning enables robots to move from start to goal** while avoiding obstacles and optimizing for various criteria like time, energy, or safety. It's a fundamental capability for autonomous robots, from vacuum cleaners to self-driving cars.

### The Navigation Problem

```
    Start ●─────────────● Goal
          │             │
          │    Path     │
          │             │
          ▼             ▼
    ┌─────────────────┐
    │   Obstacles     │
    │   ████ ███      │
    │   █    █ █      │
    │   ███  ███      │
    └─────────────────┘
    
    Challenges:
    • Avoid obstacles
    • Find optimal path
    • Handle dynamic environments
    • Real-time computation
```

## Configuration Space

### Workspace vs Configuration Space

The first step in path planning is transforming the problem:

```
    Workspace (Physical):
    ┌─────────────────┐
    │                 │
    │     Robot       │
    │      ●          │
    │                 │
    │   Obstacle      │
    │   ████          │
    └─────────────────┘
    
    Configuration Space (C-Space):
    ┌─────────────────┐
    │                 │
    │     ●           │ ← Robot as point
    │                 │
    │   XXXXXX        │ ← Grown obstacles
    │   XXXXXX        │
    └─────────────────┘
```

### Obstacle Growing

```
    Original Obstacle:
    ████
    
    Robot Size:
    ○○
    ○○
    
    Grown Obstacle (with robot radius):
    XXXXXXXX
    XXXXXXXX
    XXXXXXXX
    XXXXXXXX
    
    Formula:
    Grown = Original ⊕ Robot
    (Minkowski sum)
```

## Graph-Based Planning

### Grid-Based Planning

Discretize the environment into a grid:

```
    ┌─┬─┬─┬─┬─┬─┬─┬─┐
    │S│ │ │ │█│ │ │ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │█│ │█│█│ │█│ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │ │ │ │█│ │ │ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │█│ │█│ │ │ │█│ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │ │ │█│█│ │ │ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │█│ │ │ │ │█│ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │ │█│ │█│ │ │ │
    ├─┼─┼─┼─┼─┼─┼─┼─┤
    │ │ │ │ │ │ │ │G│
    └─┴─┴─┴─┴─┴─┴─┴─┘
    
    S = Start, G = Goal, █ = Obstacle
    
    Advantages:
    • Simple to implement
    • Easy to visualize
    • Works with any obstacle shape
    
    Disadvantages:
    • Memory intensive
    • Resolution dependent
    • Not optimal in continuous space
```

### Visibility Graph

Connect visible vertices for optimal paths:

```
    ●─────●─────●
    │ \   │   / │
    │   \ │ /   │
    │     ●     │
    │   / │ \   │
    │ /   │   \ │
    ●─────●─────●
    
    Construction:
    1. Find all obstacle vertices
    2. Connect visible vertices
    3. Add start and goal points
    4. Find shortest path
    
    Advantages:
    • Optimal in 2D
    • Relatively sparse
    • Fast path finding
    
    Disadvantages:
    • Only works with polygonal obstacles
    • Paths touch obstacles
    • Limited to 2D
```

### Voronoi Diagram

Maximize distance from obstacles:

```
    ┌─────────────────┐
    │     │     │     │
    │  ╱  │  ╲  │  ╲  │
    │ ╱   │   ╲ │   ╲ │
    │╱    │    ╲│    ╲│
    ├─────●─────●─────┤
    │    ╲│    ╱│    ╱│
    │   ╲ │   ╱ │   ╱ │
    │  ╲  │  ╱  │  ╱  │
    │     │     │     │
    └─────────────────┘
    
    Properties:
    • Maximizes clearance
    • Safe paths
    • Naturally follows corridors
    
    Applications:
    • Safe navigation
    • Robot path planning
    • Facility layout
```

## Optimal Planning Algorithms

### Dijkstra's Algorithm

Guarantees shortest path in weighted graphs:

```
    Start ●
          │ 2
          ▼
        ●─●─●
       3│1│4│
        ▼ ▼ ▼
       ●─●─●─●
      2│3│5│1│
        ▼ ▼ ▼ ▼
       ●─●─●─●─Goal
    
    Algorithm:
    1. Initialize distances (∞)
    2. Set start distance = 0
    3. Visit unvisited node with min distance
    4. Update neighbor distances
    5. Repeat until goal reached
    
    Time Complexity: O(V²) or O(E + V log V)
    Space Complexity: O(V)
```

### A* Algorithm

Combines Dijkstra with heuristic for faster search:

```
    f(n) = g(n) + h(n)
    
    g(n) = cost from start to n
    h(n) = heuristic cost from n to goal
    
    Start ●
          │ g=2, h=6
          ▼ f=8
        ●─●─●
       g│h│g│h│
        ▼ ▼ ▼
       ●─●─●─●
      g│h│g│h│g│h
        ▼ ▼ ▼ ▼
       ●─●─●─●─Goal
    
    Heuristic Requirements:
    • Admissible: h(n) ≤ true cost
    • Consistent: h(n) ≤ cost(n,n') + h(n')
    
    Common Heuristics:
    • Euclidean distance: √((x₂-x₁)² + (y₂-y₁)²)
    • Manhattan distance: |x₂-x₁| + |y₂-y₁|
    • Diagonal distance: max(|x₂-x₁|, |y₂-y₁|)
```

### D* Algorithm

Dynamic A* for changing environments:

```
    Initial Path:
    ●─────●─────●
    
    Obstacle Appears:
    ●─────█─────●
    
    D* Replanning:
    ●─┐   ┌─●
      │   │
      ●─●─●
    
    Advantages:
    • Fast replanning
    • Reuses previous information
    • Good for dynamic environments
```

## Sampling-Based Planning

### Rapidly-exploring Random Tree (RRT)

Randomly explores configuration space:

```
    Start ●
          │
          │
          ●─────●
         ╱       │
        ╱         ●
       ╱         ╱
      ●         ●
               ╱
              ╱
             ●
             │
             ●─────● Goal
    
    Algorithm:
    1. Start with tree containing start
    2. Sample random configuration
    3. Find nearest tree node
    4. Extend toward random point
    5. Add new node to tree
    6. Repeat until goal reached
    
    Advantages:
    • Works in high dimensions
    • Handles complex obstacles
    • Probabilistically complete
    
    Disadvantages:
    • Not optimal
    • Path quality varies
    • May require many samples
```

### RRT* (Optimal RRT)

Improves RRT with optimality:

```
    Start ●
          │
          │
          ●─────●
         ╱ \     │
        ╱   \    ●
       ╱     \  ╱
      ●       ●
     ╱ \     ╱
    ╱   \   ●
   ●     \ │
          ●─────● Goal
          │
          ● (rewired)
    
    Improvements:
    • Parent selection: Choose best parent
    • Rewiring: Improve tree connections
    • Asymptotic optimality: Converges to optimal
    
    Cost Function:
    c(q) = Σ distance along path
```

### PRM (Probabilistic Roadmap)

Multi-query planning approach:

```
    Learning Phase:
    ●─────●─────●
     \   / \   /
      \ /   \ /
       ●─────●
      / \   / \
     /   \ /   \
    ●─────●─────●
    
    Query Phase:
    Start ●─────● Goal
    
    Advantages:
    • Fast for multiple queries
    • Good for known environments
    • Can be precomputed
    
    Disadvantages:
    • Not good for dynamic environments
    • Memory intensive
    • May miss narrow passages
```

## Dynamic Window Approach

### Velocity Space Planning

Considers robot dynamics in velocity space:

```
    Velocity Space (v, ω):
          ω
          ▲
          │
    ●─────┼─────●
    │     │     │
    │     │     │
    ◄─────┼─────► v
    │     │     │
    │     │     │
    ●─────┼─────●
    
    Dynamic Window = Reachable velocities
    Admissible Velocities = Safe velocities
    Optimal Velocity = Best objective
```

### Trajectory Evaluation

```
    Current Time: t
    Future Time: t + Δt
    
    Robot at t: ●
    
    Possible trajectories:
    1. ●─────● (straight)
    2. ● ╱   (left turn)
    3. ● ╲   (right turn)
    
    Evaluation Criteria:
    • Distance to obstacles
    • Progress toward goal
    • Velocity constraints
    • Path smoothness
    
    Objective Function:
    G(v,ω) = α·Heading(v,ω) + β·Dist(v,ω) + γ·Vel(v,ω)
```

## Multi-Robot Path Planning

### Priority-Based Planning

Assign priorities and plan sequentially:

```
    Robot 1 (High Priority):
    Start ●─────● Goal
    
    Robot 2 (Low Priority):
    Start ●    ╲ Goal
              ╲
               ● (wait)
    
    Algorithm:
    1. Assign priorities to robots
    2. Plan path for highest priority
    3. Treat robot as obstacle for lower priority
    4. Repeat for all robots
    
    Advantages:
    • Simple to implement
    • Guarantees collision-free
    • Computationally efficient
    
    Disadvantages:
    • Not optimal
    • Priority assignment affects quality
    • May cause deadlocks
```

### Cooperative Planning

Plan all robots simultaneously:

```
    Robot 1: ●─────●
    Robot 2: ●     ╲
              ╲   ●
    
    Coordination Points:
    ●─────●─────●
    │     │     │
    ●     ●     ●
    
    Methods:
    • Centralized planning
    • Decentralized with communication
    • Market-based approaches
```

## Local Planning vs Global Planning

### Two-Level Planning Architecture

```
    Global Planner:
    Start ●─────────────● Goal
          │             │
          │    Path     │
          │             │
          ▼             ▼
    
    Local Planner:
    ●─●─●─●─●─●─●─●─●─●
    (reacts to dynamic obstacles)
    
    Global: Long-term, optimal path
    Local: Short-term, reactive avoidance
```

### Sensor-Based Reactive Planning

#### Potential Fields

Use attractive and repulsive forces:

```
    Goal (Attractive):
         ○
        ╱ ╲
       ╱   ╲
      ╱     ╲
    
    Obstacle (Repulsive):
      XXXXX
      XXXXX
      XXXXX
    
    Combined Field:
    Start ●→→→→→→→→→● Goal
          ↗     ↖
         ↗       ↖
        ↗   XXX   ↖
       ↗  XXXXXX  ↖
    
    Force Calculation:
    F_total = F_goal + ΣF_obstacles
    
    Advantages:
    • Simple implementation
    • Real-time execution
    • Smooth paths
    
    Disadvantages:
    • Local minima
    • Oscillations in narrow passages
    • Parameter tuning required
```

#### Bug Algorithms

Simple but robust navigation strategies:

```
    Bug 0:
    Start ●─────█─────● Goal
          │     │
          │     │
          └─────┘ (follow obstacle)
    
    Bug 1:
    Start ●─┐   █   ┌─● Goal
          │  │   │   │
          │  └───┘   │
          └──────────┘
    
    Bug 2:
    Start ●─┐   █   ┌─● Goal
          │  │   │   │
          │  └─●─┘   │
          └─────┬─────┘
                │
                ● (leave point)
    
    Guarantees:
    • Reach goal if reachable
    • Simple to implement
    • No global map required
```

## Path Optimization

### Path Smoothing

Improve path quality after planning:

```
    Original Path:
    ●─●─●─●─●─●
    
    Smoothed Path:
    ●─────●─────●
    
    Methods:
    • Spline fitting
    • Bezier curves
    • Polynomial smoothing
    • Shortcut heuristics
```

### Trajectory Parameterization

Add timing to geometric path:

```
    Waypoints:
    ●─────●─────●
    
    Time Parameterization:
    t0: ●
    t1:   ●
    t2:     ●
    t3:       ●
    
    Velocity Profile:
    ╱╲    ╱╲
   ╱  ╲  ╱  ╲
  ╱    ╲╱    ╲
    
    Constraints:
    • Max velocity
    • Max acceleration
    • Max jerk
```

## Real-World Considerations

### Kinematic Constraints

Different robot types have different motion constraints:

```
    Car-like Robot:
    ○─────○
     \   /
      \ /
       ● (steering angle limit)
    
    Differential Drive:
    ○     ○
     \   /
      \ /
       ●
    
    Omni-directional:
    ○─────○
    │  ●  │
    ○─────○
```

### Dynamic Constraints

Physical limits on motion:

```
    Velocity Limits:
    v_min ≤ v ≤ v_max
    
    Acceleration Limits:
    a_min ≤ a ≤ a_max
    
    Jerk Limits:
    j_min ≤ j ≤ j_max
    
    Comfort Constraints:
    • Lateral acceleration < 2 m/s²
    • Jerk < 1 m/s³
```

### Uncertainty Handling

Account for sensor and actuator noise:

```
    Planned Path:
    ●─────●─────●
    
    Actual Path (with noise):
    ●↗ ↘ ●↗ ↘ ●
    
    Uncertainty Ellipse:
    ┌─────┐
    │     │
    │  ●  │
    │     │
    └─────┘
    
    Strategies:
    • Buffer around obstacles
    • Probabilistic roadmaps
    • Robust planning
```

## Performance Metrics

### Path Quality Metrics

```
    Path Length:
    L = Σ distance between waypoints
    
    Path Smoothness:
    S = Σ |curvature| along path
    
    Clearance:
    C = min distance to obstacles
    
    Computation Time:
    T = time to find path
    
    Energy Consumption:
    E = ∫ power dt along path
```

### Algorithm Comparison

```
    Algorithm    | Optimal | Complete | Fast | Memory
    ------------|---------|----------|-------|--------
    Dijkstra   |    ✓    |    ✓     |   ✗   |   O(V)
    A*         |    ✓    |    ✓     |   ✓   |   O(V)
    RRT        |    ✗    |    ✓     |   ✓   |   O(n)
    RRT*       |    ✓    |    ✓     |   ✗   |   O(n)
    PRM        |    ✗    |    ✓     |   ✓   |   O(n²)
```

## Implementation Examples

### Autonomous Vehicle Navigation

```
    Highway Driving:
    ┌─────────────────┐
    │   Lane         │
    │   Following    │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Adaptive      │
    │   Cruise       │
    │   Control      │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Lane         │
    │   Changing     │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Emergency    │
    │   Braking     │
    └─────────────────┘
```

### Indoor Mobile Robot

```
    Room Navigation:
    ┌─────────────────┐
    │   Global       │
    │   Path         │
    │   Planning     │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Local        │
    │   Avoidance    │
    └─────────┬───────┘
              │
              ▼
    ┌─────────────────┐
    │   Door         │
    │   Passage      │
    │   Detection    │
    └─────────────────┘
```

## Best Practices

### Algorithm Selection

1. **Environment**: Static vs dynamic
2. **Dimensionality**: 2D vs 3D vs high-D
3. **Constraints**: Kinematic, dynamic
4. **Optimality**: Required vs not critical
5. **Computation**: Real-time vs offline

### Implementation Tips

1. **Discretization**: Balance resolution vs computation
2. **Heuristics**: Choose appropriate for A*
3. **Hybrid Approaches**: Combine global and local
4. **Replanning**: Update when environment changes
5. **Validation**: Simulate before real deployment

### Safety Considerations

1. **Buffer Zones**: Keep distance from obstacles
2. **Emergency Stops**: Always available
3. **Fail-Safe**: Default to safe behavior
4. **Testing**: Comprehensive edge cases
5. **Monitoring**: Real-time performance checks

## Chapter Summary

### Key Takeaways:
1. **Path planning finds collision-free routes** from start to goal
2. **Different algorithms** suit different scenarios
3. **Configuration space** simplifies planning problems
4. **Global and local planning** work together for robustness
5. **Real-world constraints** must be considered in implementation

### Important Terms:
- **Configuration Space**: Robot as point with grown obstacles
- **A* Algorithm**: Optimal search with heuristic
- **RRT**: Sampling-based planner for high dimensions
- **Dynamic Window**: Velocity space planning
- **Potential Fields**: Attractive/repulsive forces

### Next Chapter Preview:
In the final chapter, we'll explore **Machine Learning in Robotics** - how AI techniques enable robots to learn, adapt, and improve their performance over time.

## Review Questions

1. Explain the difference between workspace and configuration space.
2. Compare A* and RRT algorithms - when would you use each?
3. What are the advantages of two-level planning (global + local)?
4. How do potential fields work and what are their limitations?
5. Describe the Dynamic Window Approach and its applications.
6. What factors should be considered when selecting a path planning algorithm?

## Practical Exercise

**Path Planning Implementation:**
Implement a path planner for a 2D grid environment:
1. Create a grid with obstacles
2. Implement A* algorithm
3. Add path smoothing
4. Test with different scenarios
5. Compare with RRT implementation

This exercise will help you understand practical path planning challenges!