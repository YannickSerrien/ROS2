# Gazebo World Example

Demonstrates Gazebo world building with custom models and environments.

## What This Demonstrates

- **Custom Gazebo models**: box_obstacle, cylinder_obstacle
- **World files**: Obstacle course with walls and obstacles
- **Model organization**: Proper model directory structure
- **GAZEBO_MODEL_PATH**: Finding custom models

## Package Contents

### Custom Models

**box_obstacle/**
- model.config: Metadata
- model.sdf: Red box (1.0 x 0.5 x 0.5 m)
- Static obstacle for testing

**cylinder_obstacle/**
- model.config: Metadata
- model.sdf: Green cylinder (radius 0.3m, height 1.2m)
- Static obstacle for testing

### Worlds

**obstacle_course.world**
- 16m x 16m arena with walls
- 2 box obstacles
- 2 cylinder obstacles
- Perfect for navigation testing

## Quick Start

```bash
# Build
colcon build --packages-select gazebo_world_example
source install/setup.bash

# Launch obstacle course
ros2 launch gazebo_world_example obstacle_course.launch.py
```

**Result**: Gazebo opens with arena and obstacles

## Model Structure

Gazebo models follow this structure:
```
model_name/
├── model.config  # Metadata (name, version, description)
└── model.sdf     # Actual model (geometry, physics)
```

## Environment Variables

The launch file sets `GAZEBO_MODEL_PATH` to find custom models:

```python
SetEnvironmentVariable(
    name='GAZEBO_MODEL_PATH',
    value=models_dir
)
```

## Manual Testing

```bash
# Set model path manually
export GAZEBO_MODEL_PATH=/path/to/gazebo_world_example/models:$GAZEBO_MODEL_PATH

# Launch Gazebo with world
gazebo obstacle_course.world
```

## Customization

### Add More Obstacles

Edit `obstacle_course.world`:

```xml
<include>
  <uri>model://box_obstacle</uri>
  <name>box3</name>
  <pose>5 5 0.25 0 0 0</pose>
</include>
```

### Create New Models

1. Create directory: `models/my_model/`
2. Add `model.config`
3. Add `model.sdf`
4. Reference in world file

## Related Lessons

- Module 4, Lesson 6: Gazebo Basics
- Module 4, Lesson 7: World Building

## Key Concepts

**SDF**: Simulation Description Format (Gazebo's format)
**Static models**: `<static>true</static>` - no physics
**World files**: Environment description with models, lighting, physics
**Model library**: Organized collection of reusable models
