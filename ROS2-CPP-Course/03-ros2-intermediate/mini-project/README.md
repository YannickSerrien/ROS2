# Mini-Project: 2D Robot Arm Controller

## Project Overview

Build a complete 2D robot arm control system that integrates **Actions**, **TF2**, and **Launch Files** to achieve goal-based manipulation with real-time coordinate transformations.

## Learning Integration

This project synthesizes all Module 3 concepts:
- **Actions**: Goal-based arm movement with feedback
- **TF2**: Forward kinematics and coordinate frames
- **Launch Files**: System configuration and startup
- **Composition** (Bonus): Performance optimization

## System Architecture

```
                    ┌─────────────────────┐
                    │  Goal Position      │
                    │  (x, y) in world    │
                    └──────────┬──────────┘
                               ↓
                    ┌─────────────────────┐
                    │  MoveArm Action     │
                    │  - Inverse Kinematics│
                    │  - Trajectory Plan   │
                    │  - Execute Motion    │
                    └──────────┬──────────┘
                               ↓
                    ┌─────────────────────┐
                    │  Arm Controller     │
                    │  - Joint Control     │
                    │  - TF Broadcasting   │
                    └──────────┬──────────┘
                               ↓
                    ┌─────────────────────┐
                    │  TF Tree            │
                    │  world → base →     │
                    │  link1 → link2 →    │
                    │  end_effector       │
                    └─────────────────────┘
```

## Robot Specification

**2-Link Planar Arm**:
- **Link 1**: Length = 1.0m, joint at base
- **Link 2**: Length = 0.8m, joint at link1 end
- **Workspace**: Circular, radius ~1.8m
- **Joints**: Revolute (continuous rotation)

**Coordinate Frames**:
```
world (fixed)
 └─ base_link (arm base)
     └─ link1 (first link, rotates via joint1)
         └─ link2 (second link, rotates via joint2)
             └─ end_effector (gripper/tool)
```

## Project Components

### 1. Arm Controller Node

**Responsibilities**:
- Maintain joint states (θ₁, θ₂)
- Publish TF tree (forward kinematics)
- Accept joint commands
- Provide current state service

**Publishers**:
- `/joint_states` (sensor_msgs/JointState)
- `/tf` (TF tree updates)

**Subscribers**:
- `/joint_commands` (for direct control)

**Services**:
- `/get_end_effector_pose` (current position)

### 2. Move Arm Action Server

**Action**: `MoveArm.action`
```
# Goal: Target end-effector position
float64 target_x
float64 target_y
---
# Result: Success and final position
bool success
string message
float64 final_x
float64 final_y
float64 final_theta
---
# Feedback: Current progress
float64 current_x
float64 current_y
float64 percent_complete
```

**Responsibilities**:
- Inverse kinematics (x,y → θ₁, θ₂)
- Trajectory planning (smooth motion)
- Collision/workspace checking
- Progress feedback

### 3. Arm Visualizer (Optional)

**Responsibilities**:
- Subscribe to joint states
- Publish visualization markers
- Display in RViz

### 4. Launch System

**Files**:
- `arm_system.launch.py`: Full system
- `arm_bringup.launch.py`: Hardware/sim selection
- `arm_demo.launch.py`: With RViz

**Configuration**:
- `config/arm_params.yaml`: Arm dimensions, limits
- `config/rviz_config.rviz`: Visualization

## Implementation Requirements

### Part 1: Forward Kinematics (TF2)

Implement TF broadcasting for 2-link arm:

```
Position of link1 end:
  x1 = L1 * cos(θ₁)
  y1 = L1 * sin(θ₁)

Position of end-effector:
  x = L1*cos(θ₁) + L2*cos(θ₁+θ₂)
  y = L1*sin(θ₁) + L2*sin(θ₁+θ₂)
```

**TF Tree Updates**:
- `world` → `base_link` (static)
- `base_link` → `link1` (dynamic, rotates by θ₁)
- `link1` → `link2` (dynamic, rotates by θ₂)
- `link2` → `end_effector` (static offset)

### Part 2: Inverse Kinematics (Actions)

Solve for joint angles given target (x, y):

```
Given: target_x, target_y
Find: θ₁, θ₂

r² = x² + y²
θ₂ = ±arccos((r² - L1² - L2²) / (2*L1*L2))
θ₁ = arctan2(y, x) - arctan2(L2*sin(θ₂), L1 + L2*cos(θ₂))
```

**Handle**:
- Multiple solutions (elbow up/down)
- Unreachable targets
- Singularities

### Part 3: Trajectory Planning

Smooth motion from current to target:
- Interpolate joint angles
- Constant velocity or minimum jerk
- Configurable duration

### Part 4: Action Integration

**Server**:
1. Receive goal (target x, y)
2. Validate reachability
3. Compute IK solution
4. Plan trajectory
5. Execute with feedback
6. Handle cancellation

**Client**:
- Send goals
- Monitor feedback
- Display progress

## Starter Code Structure

```
robot_arm_project/
├── package.xml
├── CMakeLists.txt
├── action/
│   └── MoveArm.action
├── config/
│   ├── arm_params.yaml
│   └── rviz_config.rviz
├── launch/
│   ├── arm_system.launch.py
│   └── arm_demo.launch.py
├── src/
│   ├── arm_controller.cpp          # TODO: Complete
│   ├── move_arm_action_server.cpp  # TODO: Complete
│   └── arm_kinematics.cpp          # Helpers provided
└── include/
    └── robot_arm_project/
        └── kinematics.hpp
```

## Step-by-Step Guide

### Phase 1: Arm Controller & TF (Week 1)

1. **Implement Arm Controller**
   - Declare joint state variables
   - Create TF broadcaster
   - Implement forward kinematics
   - Publish TF tree at 50 Hz

2. **Test TF Tree**
   ```bash
   ros2 run robot_arm_project arm_controller
   ros2 run tf2_tools view_frames
   ros2 run tf2_ros tf2_echo world end_effector
   ```

3. **Verify Forward Kinematics**
   - Set joints to known values
   - Check end-effector position matches equations

### Phase 2: Action Server & IK (Week 1-2)

1. **Implement Inverse Kinematics**
   - Code IK equations
   - Handle edge cases
   - Test with known positions

2. **Create Action Server**
   - Goal validation
   - IK computation
   - Trajectory generation
   - Feedback publishing

3. **Test Action**
   ```bash
   ros2 run robot_arm_project move_arm_server
   ros2 action send_goal /move_arm robot_arm_project/action/MoveArm "{target_x: 1.0, target_y: 1.0}"
   ```

### Phase 3: Launch & Integration (Week 2)

1. **Create Launch Files**
   - Start all nodes
   - Load parameters
   - Support arguments

2. **Add Configuration**
   - Arm dimensions
   - Joint limits
   - Motion parameters

3. **Full System Test**
   ```bash
   ros2 launch robot_arm_project arm_system.launch.py
   ```

### Phase 4: Visualization (Bonus)

1. **RViz Configuration**
   - Add TF display
   - Robot model (optional URDF)
   - Interactive markers

2. **Demo Launch**
   ```bash
   ros2 launch robot_arm_project arm_demo.launch.py
   ```

## Testing Scenarios

### Test 1: Forward Kinematics

**Input**: θ₁ = 45°, θ₂ = -45°
**Expected**: End-effector at specific (x, y)
**Verify**: `ros2 run tf2_ros tf2_echo world end_effector`

### Test 2: Inverse Kinematics

**Input**: Target (1.5, 0.5)
**Expected**: Valid joint angles
**Verify**: Forward kinematics gives back (1.5, 0.5)

### Test 3: Reachability

**Input**: Target (3.0, 3.0) - unreachable
**Expected**: Action rejects goal
**Verify**: Appropriate error message

### Test 4: Smooth Motion

**Input**: Move from (0.5, 1.0) to (1.0, 1.5)
**Expected**: Smooth interpolation
**Verify**: Feedback shows progress 0% → 100%

### Test 5: Cancellation

**Input**: Start long motion, cancel mid-way
**Expected**: Motion stops, returns canceled
**Verify**: Arm stops at intermediate position

## Success Criteria

**Minimum Requirements** (Pass):
- [ ] TF tree publishes correctly
- [ ] Forward kinematics accurate
- [ ] Inverse kinematics works for reachable targets
- [ ] Action server accepts and executes goals
- [ ] Feedback published during motion
- [ ] Launch file starts system

**Complete Implementation** (Excellent):
- [ ] All above +
- [ ] Workspace validation
- [ ] Smooth trajectory interpolation
- [ ] Cancellation support
- [ ] Configuration via YAML
- [ ] RViz visualization
- [ ] Error handling

**Bonus Features** (Outstanding):
- [ ] Obstacle avoidance
- [ ] Multiple IK solutions with selection
- [ ] Velocity/acceleration limits
- [ ] Composition for performance
- [ ] Joint state publisher integration

## Common Challenges & Hints

<details>
<summary>Challenge 1: IK Has No Solution</summary>

**Problem**: Target unreachable
**Solution**:
```cpp
double r = sqrt(x*x + y*y);
if (r > L1 + L2 || r < abs(L1 - L2)) {
    // Unreachable
    return false;
}
```
</details>

<details>
<summary>Challenge 2: TF Updates Too Slow</summary>

**Problem**: Jerky visualization
**Solution**: Publish at 50-100 Hz using timer
```cpp
timer_ = create_wall_timer(20ms, [this]() { publish_tf(); });
```
</details>

<details>
<summary>Challenge 3: Singularity at Full Extension</summary>

**Problem**: IK unstable when arm fully extended
**Solution**: Add small epsilon, avoid exact workspace boundary
```cpp
if (r > L1 + L2 - 0.01) { /* reject */ }
```
</details>

<details>
<summary>Challenge 4: Action Doesn't Accept Goals</summary>

**Check**:
- Server running: `ros2 action list`
- Action definition compiled
- Client waits for server
</details>

## Evaluation Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| TF Tree | 20 | Correct frame hierarchy and transforms |
| Forward Kinematics | 15 | Accurate end-effector position |
| Inverse Kinematics | 20 | Correct joint angles for targets |
| Action Server | 20 | Goal handling, feedback, results |
| Launch Files | 10 | System starts correctly |
| Error Handling | 10 | Graceful failure modes |
| Code Quality | 5 | Clean, commented, documented |
| **Total** | **100** | |

**Bonus**: +10 for RViz integration, +10 for composition

## Resources

**Kinematics References**:
- [Introduction to Robotics](http://www.mech.sharif.ir/c/document_library/get_file?uuid=5a4bb247-1430-4e46-942c-d692dead831f&groupId=14040) - Chapter 3
- [Modern Robotics](http://hades.mech.northwestern.edu/images/7/7f/MR.pdf) - Chapter 6

**ROS2 Documentation**:
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
- [Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)

**Example Code**:
- Module 3 examples (actions_example, tf2_example)
- Exercises 1-3

## Submission Requirements

1. **Code**: Complete package with all nodes
2. **Documentation**:
   - README with build/run instructions
   - Comments explaining key algorithms
3. **Demo Video** (optional): 3-minute demonstration
4. **Report** (1-2 pages):
   - System architecture
   - Kinematic equations used
   - Testing results
   - Challenges encountered

## Timeline

**Week 1**:
- Days 1-2: Arm controller + TF tree
- Days 3-4: Inverse kinematics
- Day 5: Action server basics

**Week 2**:
- Days 1-2: Trajectory planning
- Days 3-4: Launch files and integration
- Day 5: Testing and polish

**Total Time**: ~30-40 hours

## What's Next?

After completing this mini-project:
- ✅ Module 3 Complete!
- → Module 4: Simulation with Gazebo
- → Module 5: Hardware Integration
- → Module 7: Advanced Projects

This project demonstrates real-world robotics integration - the exact pattern used in industrial robot arms, surgical robots, and manipulators!

---

**Good luck! You're building a real robot arm controller! 🦾**
