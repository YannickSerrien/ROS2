# Lesson 7: World Building

## Learning Objectives

- Create custom Gazebo models
- Build structured environments
- Add terrain and elevation
- Configure materials and textures
- Organize model libraries
- Import CAD models (STL, Collada)
- Use model plugins
- Create reusable world templates

## Model Structure

### Gazebo Model Anatomy

**Model**: Self-contained simulation object (robot, obstacle, building, etc.)

**Directory structure**:
```
model_name/
├── model.config      # Metadata (name, author, description)
├── model.sdf         # Actual model definition (geometry, physics)
├── meshes/           # 3D mesh files (optional)
│   ├── visual.dae
│   └── collision.stl
└── materials/        # Textures and shaders (optional)
    ├── scripts/
    └── textures/
```

### model.config

**Purpose**: Metadata for Gazebo model database

```xml
<?xml version="1.0"?>
<model>
  <name>my_custom_obstacle</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>you@example.com</email>
  </author>

  <description>
    A custom obstacle for testing navigation algorithms.
  </description>
</model>
```

### model.sdf

**Purpose**: Actual model definition

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="my_custom_obstacle">

    <!-- Static (doesn't move) or dynamic -->
    <static>true</static>

    <!-- Pose in world (if included in world file) -->
    <pose>0 0 0 0 0 0</pose>

    <!-- Link (rigid body) -->
    <link name="link">

      <!-- Inertial properties (if dynamic) -->
      <inertial>
        <mass>50</mass>
        <inertia>
          <ixx>1</ixx> <iyy>1</iyy> <izz>1</izz>
          <ixy>0</ixy> <ixz>0</ixz> <iyz>0</iyz>
        </inertia>
      </inertial>

      <!-- Visual appearance -->
      <visual name="visual">
        <geometry>
          <box>
            <size>2 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>

      <!-- Collision geometry -->
      <collision name="collision">
        <geometry>
          <box>
            <size>2 1 1</size>
          </box>
        </geometry>
      </collision>

    </link>

  </model>
</sdf>
```

## Creating Simple Models

### Example 1: Wall

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="wall">
    <static>true</static>

    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>5 0.2 2</size>  <!-- Length Width Height -->
          </box>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.7 1</ambient>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>5 0.2 2</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Example 2: Cylinder Obstacle

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="cylinder_obstacle">
    <static>true</static>

    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.8 0.2 1</ambient>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.5</radius>
            <length>1.5</length>
          </cylinder>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Example 3: Multi-Link Structure

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="table">
    <static>true</static>

    <!-- Table top -->
    <link name="top">
      <pose>0 0 0.75 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <box><size>1.5 0.8 0.05</size></box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>1.5 0.8 0.05</size></box>
        </geometry>
      </collision>
    </link>

    <!-- Leg 1 -->
    <link name="leg1">
      <pose>0.6 0.3 0.375 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.75</length></cylinder>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder><radius>0.05</radius><length>0.75</length></cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Legs 2-4 (similar structure) -->
    <!-- ... -->

  </model>
</sdf>
```

## Materials and Appearance

### Basic Colors (RGBA)

```xml
<material>
  <!-- Ambient: Color in shadow -->
  <ambient>0.8 0.2 0.2 1</ambient>

  <!-- Diffuse: Color in light -->
  <diffuse>0.8 0.2 0.2 1</diffuse>

  <!-- Specular: Shininess/reflection -->
  <specular>0.5 0.5 0.5 1</specular>

  <!-- Emissive: Glow (like LED) -->
  <emissive>0 0 0 0</emissive>
</material>
```

**Color values**: RGBA (Red Green Blue Alpha), range 0-1

**Examples**:
```xml
<!-- Red -->
<ambient>1 0 0 1</ambient>

<!-- Green -->
<ambient>0 1 0 1</ambient>

<!-- Blue -->
<ambient>0 0 1 1</ambient>

<!-- White -->
<ambient>1 1 1 1</ambient>

<!-- Black -->
<ambient>0 0 0 1</ambient>

<!-- Gray -->
<ambient>0.5 0.5 0.5 1</ambient>

<!-- Transparent -->
<ambient>1 0 0 0.5</ambient>  <!-- Semi-transparent red -->
```

### Textures from Images

**Material script** (materials/scripts/my_material.material):
```
material MyTexture
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture my_texture.png
      }
    }
  }
}
```

**Using textured material**:
```xml
<visual name="visual">
  <geometry>
    <box><size>1 1 1</size></box>
  </geometry>
  <material>
    <script>
      <uri>model://my_model/materials/scripts</uri>
      <uri>model://my_model/materials/textures</uri>
      <name>MyTexture</name>
    </script>
  </material>
</visual>
```

## Using Meshes

### Supported Formats

- **COLLADA (.dae)**: Recommended, includes materials
- **STL (.stl)**: Simple geometry only
- **OBJ (.obj)**: With materials (.mtl file)

### Mesh in Model

```xml
<visual name="visual">
  <geometry>
    <mesh>
      <uri>model://my_robot/meshes/chassis.dae</uri>
      <scale>1 1 1</scale>  <!-- Scale factor -->
    </mesh>
  </geometry>
</visual>

<collision name="collision">
  <geometry>
    <mesh>
      <!-- Use simplified mesh for collision -->
      <uri>model://my_robot/meshes/chassis_collision.stl</uri>
    </mesh>
  </geometry>
</collision>
```

**Best practice**:
- **Visual**: Detailed mesh for appearance
- **Collision**: Simplified mesh or primitive shapes for performance

### Importing from CAD

**Workflow**:
1. Design in CAD (SolidWorks, Fusion 360, Blender)
2. Export as COLLADA (.dae) or STL
3. Place in `meshes/` directory
4. Reference in model.sdf

**Tips**:
- **Origin**: Set correctly in CAD software
- **Units**: Gazebo uses meters
- **Triangulate**: Export with triangulated meshes
- **Simplify collision**: Create low-poly version for physics

## Terrain and Heightmaps

### Heightmap Terrain

**Purpose**: Create realistic terrain from grayscale image

**Heightmap image**:
- Grayscale PNG/JPG
- Black (0) = lowest elevation
- White (255) = highest elevation

**In world file**:
```xml
<model name="terrain">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <heightmap>
          <uri>model://terrain/materials/textures/heightmap.png</uri>
          <size>100 100 10</size>  <!-- X Y Z size in meters -->
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
    </collision>

    <visual name="visual">
      <geometry>
        <heightmap>
          <uri>model://terrain/materials/textures/heightmap.png</uri>
          <size>100 100 10</size>
          <texture>
            <diffuse>model://terrain/materials/textures/grass.jpg</diffuse>
            <normal>model://terrain/materials/textures/grass_normal.jpg</normal>
            <size>10</size>  <!-- Texture repeat -->
          </texture>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

**Creating heightmaps**:
- Use image editor (GIMP, Photoshop)
- Grayscale = elevation
- Resolution affects detail (higher = more triangles = slower)

### Simple Slopes and Ramps

```xml
<model name="ramp">
  <static>true</static>
  <link name="link">
    <pose>0 0 0.25 0 0.3 0</pose>  <!-- Pitched 0.3 radians -->
    <visual name="visual">
      <geometry>
        <box><size>2 1 0.1</size></box>
      </geometry>
      <material>
        <ambient>0.5 0.5 0.5 1</ambient>
      </material>
    </visual>
    <collision name="collision">
      <geometry>
        <box><size>2 1 0.1</size></box>
      </geometry>
    </collision>
  </link>
</model>
```

## Building Complete Worlds

### Test Environment Example

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="test_arena">

    <!-- Physics and scene -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.7 0.8 1 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Lighting -->
    <include><uri>model://sun</uri></include>

    <!-- Additional light -->
    <light type="point" name="light1">
      <pose>5 5 5 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>20</range>
      </attenuation>
    </light>

    <!-- Ground -->
    <include><uri>model://ground_plane</uri></include>

    <!-- Walls forming square arena -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 10 1 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>20 0.2 2</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><box><size>20 0.2 2</size></box></geometry>
        </collision>
      </link>
    </model>

    <!-- South, East, West walls (similar) -->
    <!-- ... -->

    <!-- Obstacles -->
    <model name="obstacle1">
      <static>true</static>
      <pose>3 3 0.5 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.5</radius><length>1</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient></material>
        </visual>
        <collision name="collision">
          <geometry><cylinder><radius>0.5</radius><length>1</length></cylinder></geometry>
        </collision>
      </link>
    </model>

    <!-- More obstacles -->
    <!-- ... -->

  </world>
</sdf>
```

### Modular World with Includes

**Main world file**:
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="modular_world">

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Include custom models -->
    <include>
      <uri>model://my_wall</uri>
      <name>wall1</name>
      <pose>5 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://my_obstacle</uri>
      <name>obstacle1</name>
      <pose>2 2 0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

**Benefit**: Reuse models across worlds!

## Model Libraries

### Gazebo Model Path

**Gazebo searches for models in**:
1. `~/.gazebo/models/` (user models)
2. `/usr/share/gazebo-11/models/` (system models)
3. Directories in `GAZEBO_MODEL_PATH` environment variable

### Adding Custom Model Path

```bash
# Add to ~/.bashrc
export GAZEBO_MODEL_PATH=/path/to/my_models:$GAZEBO_MODEL_PATH

# Or in launch file
os.environ['GAZEBO_MODEL_PATH'] = '/path/to/my_models'
```

### Organizing Models

```
my_models/
├── obstacles/
│   ├── box_obstacle/
│   │   ├── model.config
│   │   └── model.sdf
│   └── cylinder_obstacle/
│       ├── model.config
│       └── model.sdf
├── structures/
│   ├── wall/
│   └── building/
└── robots/
    └── my_robot/
```

## Plugins for Interactive Worlds

### Model Plugin Example

**Moving platform**:
```xml
<model name="moving_platform">
  <static>false</static>

  <link name="link">
    <visual name="visual">
      <geometry><box><size>2 2 0.1</size></box></geometry>
    </visual>
    <collision name="collision">
      <geometry><box><size>2 2 0.1</size></box></geometry>
    </collision>
    <inertial>
      <mass>100</mass>
      <inertia><ixx>10</ixx><iyy>10</iyy><izz>10</izz></inertia>
    </inertial>
  </link>

  <!-- Plugin to move platform -->
  <plugin name="linear_movement" filename="liblinear_movement_plugin.so">
    <velocity>0.5</velocity>  <!-- m/s -->
    <direction>1 0 0</direction>  <!-- X-axis -->
    <distance>5</distance>  <!-- Meters -->
  </plugin>
</model>
```

### Wind Plugin

```xml
<world name="windy_world">
  <!-- ... -->

  <plugin name="wind" filename="libgazebo_ros_wind.so">
    <frame_id>world</frame_id>
    <wind_direction>1 0 0</wind_direction>  <!-- X-direction -->
    <wind_force_mean>5</wind_force_mean>
    <wind_force_variance>2</wind_force_variance>
  </plugin>

</world>
```

## Best Practices

**1. Use static for non-moving objects**:
```xml
<!-- Saves physics calculations -->
<model name="building">
  <static>true</static>
  <!-- ... -->
</model>
```

**2. Simplify collision geometry**:
```xml
<visual>
  <geometry><mesh><uri>detailed_mesh.dae</uri></mesh></geometry>
</visual>

<collision>
  <geometry><box size="2 2 2"/></geometry>  <!-- Simple box -->
</collision>
```

**3. Organize by function**:
```
worlds/
├── test_empty.world
├── test_obstacles.world
├── test_terrain.world
└── competition_arena.world
```

**4. Use meaningful names**:
```xml
<!-- ✅ Good -->
<model name="north_wall">

<!-- ❌ Bad -->
<model name="model_1">
```

**5. Set proper inertia** (for dynamic objects):
```xml
<!-- Use realistic masses and inertias -->
<inertial>
  <mass>50</mass>  <!-- kg -->
  <inertia><!-- Calculate properly --></inertia>
</inertial>
```

**6. Test incrementally**:
- Build world piece by piece
- Test each model individually
- Verify physics behavior
- Check performance

## Debugging World Issues

**Model not appearing**:
```bash
# Check model path
echo $GAZEBO_MODEL_PATH

# Verify model.config exists
ls ~/.gazebo/models/my_model/model.config

# Check Gazebo output for errors
gazebo --verbose my_world.world
```

**Performance issues**:
- Reduce mesh complexity
- Use `<static>true</static>` where possible
- Simplify collision meshes
- Lower physics update rate

**Physics instability**:
- Check for overlapping geometries
- Verify inertia values
- Reduce max_step_size
- Add damping to joints

## Summary

**World building involves**:
- Creating custom models (geometry, materials)
- Organizing model libraries
- Building environments with terrain
- Adding lighting and atmosphere
- Using plugins for interactivity

**Key components**:
- **model.config**: Metadata
- **model.sdf**: Geometry and physics
- **Meshes**: Visual detail
- **Materials**: Colors and textures
- **Heightmaps**: Terrain

**Best practices**:
- Static models for non-moving objects
- Simple collision geometry
- Proper organization
- Incremental testing

**Tools**:
- CAD software for mesh creation
- Image editors for heightmaps
- Gazebo model editor
- `GAZEBO_MODEL_PATH` for organization

## What's Next?

**Next Lesson**: Sensor Simulation - Lidar, cameras, IMU in Gazebo

**You'll learn**:
- Adding sensors to robot models
- Gazebo sensor plugins
- Publishing sensor data to ROS2
- Simulating realistic noise
- Sensor visualization

---

**Key Takeaway**: Custom worlds let you create controlled test environments - build scenarios that challenge your robot!
