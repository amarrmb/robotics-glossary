# URDF & Xacro Research Notes

**Topic**: URDF & Xacro - Robot Description Formats for ROS
**Target file**: `src/content/docs/software/ros2/urdf-xacro.mdx`
**Research date**: 2026-01-21

---

## Key Concepts to Cover

### 1. What is URDF?

- **Unified Robot Description Format** - XML specification for representing robot models
- Introduced in 2009 within the ROS ecosystem
- Describes robot as a kinematic tree of links and joints
- Encodes geometry, kinematics, dynamics, and visual/collision properties
- De facto standard adopted beyond ROS: Gazebo, PyBullet, MuJoCo, Pinocchio, Unity, MATLAB, Isaac Sim
- Machine-readable schema for robot model storage and interchange

### 2. What is Xacro?

- **XML Macros** - macro language for simplifying URDF files
- Reduces redundant XML code through parameterized macros
- Supports properties (variables), math operations, conditionals, and includes
- Must be processed/compiled into standard URDF before use
- Name comes from "XML macro" = "Xacro"
- Not a separate format, but a preprocessor for URDF

### 3. URDF Structure - Links

Links represent rigid bodies (physical robot components):

```xml
<link name="base_link">
  <!-- Visual - appearance/rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/base.stl"/>
    </geometry>
    <material name="gray"/>
  </visual>

  <!-- Collision - physics/collision detection -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial - mass properties for dynamics -->
  <inertial>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

**Key Components**:
- **Visual**: Shape for rendering (mesh, box, cylinder, sphere)
- **Collision**: Shape for physics/collision detection (often simplified)
- **Inertial**: Mass, center of mass, and 3x3 inertia matrix (symmetric, 6 values)

**Geometry Types**:
- `<box size="x y z"/>` - rectangular prism
- `<cylinder radius="r" length="l"/>` - cylinder
- `<sphere radius="r"/>` - sphere
- `<mesh filename="package://..." scale="1 1 1"/>` - STL or COLLADA mesh

### 4. URDF Structure - Joints

Joints define kinematic relationships between links:

```xml
<joint name="joint_1" type="revolute">
  <parent link="base_link"/>
  <child link="link_1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" velocity="1.0" effort="100"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

**Joint Types**:
| Type | Description | Limits |
|------|-------------|--------|
| `revolute` | Rotation about axis | min/max angle |
| `continuous` | Rotation without limits | none |
| `prismatic` | Linear sliding | min/max position |
| `fixed` | No relative motion | N/A |
| `floating` | 6-DOF motion | none |
| `planar` | 2D motion in plane | none |

**Key Elements**:
- `<parent link="..."/>` - parent link name
- `<child link="..."/>` - child link name
- `<origin xyz="..." rpy="..."/>` - transform from parent to joint
- `<axis xyz="..."/>` - axis of rotation/translation
- `<limit lower="..." upper="..." velocity="..." effort="..."/>` - motion limits
- `<dynamics damping="..." friction="..."/>` - physical properties

### 5. Xacro Features

#### Properties (Variables)
```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="wheel_width" value="0.05"/>
<xacro:property name="pi" value="3.14159"/>

<!-- Usage with ${} -->
<cylinder radius="${wheel_radius}" length="${wheel_width}"/>
```

#### Math Operations
```xml
<!-- Basic math -->
<origin xyz="${base_length/2} 0 ${base_height + wheel_radius}"/>

<!-- Trigonometry -->
<origin xyz="${cos(pi/4)} ${sin(pi/4)} 0"/>

<!-- Complex expressions -->
<mass value="${density * pi * wheel_radius**2 * wheel_width}"/>
```

#### Macros
```xml
<!-- Define macro -->
<xacro:macro name="wheel" params="prefix reflect">
  <link name="${prefix}_wheel_link">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
      </geometry>
    </visual>
  </link>

  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="0 ${reflect * wheel_separation/2} 0" rpy="${-pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</xacro:macro>

<!-- Use macro -->
<xacro:wheel prefix="left" reflect="1"/>
<xacro:wheel prefix="right" reflect="-1"/>
```

#### Block Parameters (for complex XML)
```xml
<xacro:macro name="link_with_inertia" params="name mass *inertia">
  <link name="${name}">
    <inertial>
      <mass value="${mass}"/>
      <xacro:insert_block name="inertia"/>
    </inertial>
  </link>
</xacro:macro>

<!-- Usage with block parameter -->
<xacro:link_with_inertia name="heavy_link" mass="10.0">
  <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
</xacro:link_with_inertia>
```

#### Conditionals
```xml
<xacro:if value="${use_gazebo}">
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
  </gazebo>
</xacro:if>

<xacro:unless value="${simplified_collision}">
  <collision>
    <geometry>
      <mesh filename="package://robot/meshes/detailed.stl"/>
    </geometry>
  </collision>
</xacro:unless>
```

#### Includes
```xml
<!-- Include other xacro files -->
<xacro:include filename="$(find robot_description)/urdf/materials.xacro"/>
<xacro:include filename="$(find robot_description)/urdf/sensors.xacro"/>

<!-- Include with namespace to avoid conflicts -->
<xacro:include filename="$(find gripper_description)/urdf/gripper.xacro" ns="gripper"/>
```

#### Arguments (command-line parameters)
```xml
<!-- Define argument with default -->
<xacro:arg name="use_sim" default="false"/>
<xacro:arg name="robot_name" default="my_robot"/>

<!-- Use argument -->
<xacro:property name="sim_mode" value="$(arg use_sim)"/>
<robot name="$(arg robot_name)">
```

### 6. ros2_control Integration

URDF includes ros2_control tags for hardware abstraction:

```xml
<ros2_control name="robot_control" type="system">
  <hardware>
    <!-- For real hardware -->
    <plugin>my_robot/MyRobotHardware</plugin>

    <!-- OR for simulation -->
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>

  <joint name="joint_1">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <command_interface name="velocity">
      <param name="min">-1.0</param>
      <param name="max">1.0</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>

  <sensor name="force_torque_sensor">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
  </sensor>
</ros2_control>
```

**Command Interface Types**:
- `position` - target joint position
- `velocity` - target joint velocity
- `effort` - force/torque applied to joint

**State Interface Types**:
- `position` - current joint position
- `velocity` - current joint velocity
- `effort` - current force/torque

### 7. Gazebo Integration (Modern - Ionic)

**Important**: Gazebo Classic (numbered versions) reached end-of-life January 2025.
Use modern Gazebo (Ionic for Jazzy/Rolling) with gz_ros2_control.

```xml
<!-- Load gz_ros2_control plugin -->
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find robot_bringup)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- Gazebo-specific link properties -->
<gazebo reference="base_link">
  <material>Gazebo/Gray</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>

<!-- Gazebo sensor plugins -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"/>
  </sensor>
</gazebo>
```

### 8. Package Organization Best Practices

Recommended directory structure for robot description packages:

```
my_robot_description/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   ├── robot.urdf.xacro          # Main robot file
│   ├── materials.xacro            # Color definitions
│   ├── sensors/
│   │   ├── camera.xacro
│   │   ├── lidar.xacro
│   │   └── imu.xacro
│   ├── control/
│   │   └── ros2_control.xacro     # ros2_control tags
│   └── gazebo/
│       └── gazebo.xacro           # Gazebo-specific tags
├── meshes/
│   ├── visual/                    # Detailed meshes (.stl, .dae)
│   │   ├── base.stl
│   │   └── arm_link.stl
│   └── collision/                 # Simplified meshes
│       ├── base_collision.stl
│       └── arm_collision.stl
├── config/
│   └── joint_limits.yaml
├── launch/
│   └── view_robot.launch.py
└── rviz/
    └── view_robot.rviz
```

### 9. Processing Xacro Files

**Command Line**:
```bash
# Generate URDF from xacro
ros2 run xacro xacro robot.urdf.xacro > robot.urdf

# With arguments
ros2 run xacro xacro robot.urdf.xacro use_sim:=true robot_name:=my_robot > robot.urdf

# Check for errors
ros2 run urdf_parser check_urdf robot.urdf
```

**In Launch Files**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ]),
        ' use_sim:=true'
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([robot_state_publisher])
```

### 10. Common Inertia Calculations

**Primitive Shapes**:
```
Box (mass m, dimensions x, y, z):
  Ixx = (1/12) * m * (y² + z²)
  Iyy = (1/12) * m * (x² + z²)
  Izz = (1/12) * m * (x² + y²)

Cylinder (mass m, radius r, length l, axis along z):
  Ixx = Iyy = (1/12) * m * (3r² + l²)
  Izz = (1/2) * m * r²

Sphere (mass m, radius r):
  Ixx = Iyy = Izz = (2/5) * m * r²
```

**Xacro Helper Macros** (common pattern):
```xml
<xacro:macro name="cylinder_inertia" params="mass radius length">
  <inertia
    ixx="${(1/12) * mass * (3 * radius**2 + length**2)}"
    iyy="${(1/12) * mass * (3 * radius**2 + length**2)}"
    izz="${(1/2) * mass * radius**2}"
    ixy="0" ixz="0" iyz="0"/>
</xacro:macro>
```

---

## Current Version Information

### ROS 2 Distribution Support

| Distribution | xacro Version | Gazebo | Status |
|--------------|---------------|--------|--------|
| **Jazzy** | 2.0.x | Harmonic | **Recommended** |
| Humble | 2.0.x | Fortress (Classic EOL) | LTS until 2027 |
| Rolling | Latest | Harmonic | Development |
| **Kilted** | 2.0.x | Ionic | Released May 2025 |

### Installation
```bash
# Jazzy
sudo apt install ros-jazzy-xacro ros-jazzy-urdf ros-jazzy-urdf-parser-plugin

# Joint state publisher (for visualization)
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui

# Robot state publisher
sudo apt install ros-jazzy-robot-state-publisher
```

### Key ROS 2 Packages
- `xacro` - Xacro processor
- `urdf` - URDF parser library
- `urdf_parser_plugin` - Plugin for parsing URDF
- `robot_state_publisher` - Publishes TF transforms from URDF + joint states
- `joint_state_publisher` - Publishes fake joint states for testing
- `joint_state_publisher_gui` - GUI sliders for joint states

---

## NVIDIA + ROS 2 Ecosystem Integration

### Isaac Sim URDF Import

Isaac Sim supports URDF import through multiple methods:

**Method 1: Direct Import (GUI)**
- File > Import > Select URDF file
- Extension: `isaacsim.asset.importer.urdf` (enabled by default)
- Converts URDF to USD (Universal Scene Description) format

**Method 2: ROS 2 Node Import (supports Xacro)**
- Extension: `isaacsim.ros2.urdf`
- Reads robot_description from ROS 2 topic
- Automatically handles Xacro-to-URDF conversion via ROS 2

**Method 3: Python API**
```python
from isaacsim.asset.importer.urdf import UrdfImporter

importer = UrdfImporter()
result = importer.import_robot(
    urdf_path="/path/to/robot.urdf",
    import_config=UrdfImportConfig(
        fix_base=True,
        self_collision=False,
        default_drive_type=DriveType.POSITION
    )
)
```

**Import Configuration Options**:
- `fix_base` - Fixed or moveable base
- `self_collision` - Enable self-collision (default: false for performance)
- `default_drive_type` - Position, Velocity, or Effort

### Converting Xacro for Isaac Sim
```bash
# Convert xacro to URDF first
ros2 run xacro xacro robot.urdf.xacro > robot.urdf

# Then import into Isaac Sim
# OR use ROS 2 bridge to publish robot_description topic
```

### cuRobo Robot Configuration

cuRobo (CUDA Robotics Library) uses URDF as input:

```yaml
# robot_config.yaml
robot_cfg:
  urdf_path: "robot.urdf"
  base_link: "base_link"
  ee_link: "tool0"

  kinematics:
    collision_spheres: "spheres.yaml"  # Collision representation

  cspace:
    joint_names:
      - joint_1
      - joint_2
      - joint_3
```

### Isaac ROS Integration
- Isaac ROS packages use standard URDF/Xacro for robot descriptions
- Compatible with MoveIt 2 and ros2_control
- cuMotion MoveIt plugin requires URDF + XRDF (cuMotion-specific extension)

---

## Code Examples to Include

### 1. Minimal URDF Example
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Rotating joint -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0.25 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" velocity="1.0" effort="10"/>
  </joint>

  <!-- First link -->
  <link name="link_1">
    <visual>
      <origin xyz="0.1 0 0"/>
      <geometry>
        <cylinder radius="0.02" length="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>
</robot>
```

### 2. Xacro with Macros
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_arm">

  <!-- Properties -->
  <xacro:property name="link_length" value="0.3"/>
  <xacro:property name="link_radius" value="0.02"/>
  <xacro:property name="link_mass" value="1.0"/>
  <xacro:property name="pi" value="3.14159265359"/>

  <!-- Inertia macro for cylinder -->
  <xacro:macro name="cylinder_inertia" params="mass radius length">
    <inertial>
      <mass value="${mass}"/>
      <inertia
        ixx="${(1/12) * mass * (3 * radius**2 + length**2)}"
        iyy="${(1/12) * mass * (3 * radius**2 + length**2)}"
        izz="${(1/2) * mass * radius**2}"
        ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </xacro:macro>

  <!-- Arm link macro -->
  <xacro:macro name="arm_link" params="name length radius mass color">
    <link name="${name}">
      <visual>
        <origin xyz="${length/2} 0 0" rpy="0 ${pi/2} 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <origin xyz="${length/2} 0 0" rpy="0 ${pi/2} 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia mass="${mass}" radius="${radius}" length="${length}"/>
    </link>
  </xacro:macro>

  <!-- Materials -->
  <material name="blue"><color rgba="0 0 0.8 1"/></material>
  <material name="red"><color rgba="0.8 0 0 1"/></material>

  <!-- Base -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05"/></geometry>
      <material name="blue"/>
    </visual>
  </link>

  <!-- Use macros to create links -->
  <xacro:arm_link name="link_1" length="${link_length}" radius="${link_radius}"
                   mass="${link_mass}" color="red"/>
  <xacro:arm_link name="link_2" length="${link_length}" radius="${link_radius}"
                   mass="${link_mass}" color="blue"/>

  <!-- Joints -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.025" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-pi}" upper="${pi}" velocity="1.0" effort="10"/>
  </joint>

  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="${link_length} 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-pi/2}" upper="${pi/2}" velocity="1.0" effort="10"/>
  </joint>
</robot>
```

### 3. Launch File with Robot State Publisher
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf', 'robot.urdf.xacro'
        ])
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'rviz', 'view_robot.rviz'
        ])]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
```

### 4. ros2_control Configuration
```xml
<!-- In robot.urdf.xacro -->
<ros2_control name="robot_control" type="system">
  <hardware>
    <xacro:if value="$(arg use_sim)">
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </xacro:if>
    <xacro:unless value="$(arg use_sim)">
      <plugin>my_robot_hardware/MyRobotHardware</plugin>
      <param name="serial_port">/dev/ttyUSB0</param>
    </xacro:unless>
  </hardware>

  <joint name="joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="joint_2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

---

## Diagrams Needed

### 1. URDF Tree Structure
```
             robot (root)
                 │
    ┌────────────┼────────────┐
    │            │            │
  link_0    ─ joint_1 ─    link_1
 (base)     (revolute)    (arm_1)
                              │
                         ─ joint_2 ─
                          (revolute)
                              │
                           link_2
                          (arm_2)
                              │
                         ─ joint_3 ─
                           (fixed)
                              │
                          tool_link
                         (end_effector)
```

### 2. Link Element Structure
```
┌─────────────────────────────────────────────────────┐
│                      <link>                          │
├─────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐ │
│  │   <visual>  │  │ <collision> │  │  <inertial> │ │
│  ├─────────────┤  ├─────────────┤  ├─────────────┤ │
│  │  <origin>   │  │  <origin>   │  │  <origin>   │ │
│  │  <geometry> │  │  <geometry> │  │   <mass>    │ │
│  │  <material> │  │             │  │  <inertia>  │ │
│  └─────────────┘  └─────────────┘  └─────────────┘ │
│                                                      │
│   Rendering      Physics Engine    Dynamics Sim     │
│   (RViz)         (Gazebo/Isaac)   (Gazebo/Isaac)   │
└─────────────────────────────────────────────────────┘
```

### 3. Joint Element Structure
```
┌──────────────────────────────────────────┐
│               <joint>                     │
├──────────────────────────────────────────┤
│                                           │
│  type: revolute | continuous | prismatic │
│        | fixed | floating | planar       │
│                                           │
│  ┌─────────────────────────────────────┐ │
│  │  <parent link="parent_link"/>       │ │
│  │  <child link="child_link"/>         │ │
│  │  <origin xyz="x y z" rpy="r p y"/>  │ │
│  │  <axis xyz="ax ay az"/>             │ │
│  │  <limit lower="" upper=""           │ │
│  │         velocity="" effort=""/>     │ │
│  │  <dynamics damping="" friction=""/> │ │
│  └─────────────────────────────────────┘ │
│                                           │
│  parent ──[origin]──[axis]── child        │
│            (transform)  (motion)          │
└──────────────────────────────────────────┘
```

### 4. Xacro Processing Pipeline
```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   .xacro file   │────▶│  xacro parser   │────▶│   .urdf file    │
│  (with macros)  │     │ (ros2 run xacro)│     │  (expanded XML) │
└─────────────────┘     └─────────────────┘     └────────┬────────┘
                                                          │
                                                          ▼
                        ┌─────────────────────────────────────────┐
                        │         robot_state_publisher           │
                        │  (publishes TF transforms + /robot_description) │
                        └────────────────────┬────────────────────┘
                                             │
                    ┌────────────────────────┼────────────────────────┐
                    ▼                        ▼                        ▼
             ┌──────────┐             ┌──────────┐             ┌──────────┐
             │   RViz   │             │  Gazebo  │             │ MoveIt 2 │
             │ (visual) │             │  (sim)   │             │(planning)│
             └──────────┘             └──────────┘             └──────────┘
```

### 5. ros2_control Integration
```
┌────────────────────────────────────────────────────────────────┐
│                        URDF / Xacro                             │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    <ros2_control>                         │  │
│  │   <hardware>                                              │  │
│  │     <plugin>hardware_interface_plugin</plugin>            │  │
│  │   </hardware>                                             │  │
│  │   <joint name="joint_1">                                  │  │
│  │     <command_interface name="position"/>                  │  │
│  │     <state_interface name="position"/>                    │  │
│  │   </joint>                                                │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌────────────────────────────────────────────────────────────────┐
│                    Controller Manager                           │
│  ┌─────────────────┐  ┌─────────────────┐  ┌────────────────┐ │
│  │ Joint Trajectory│  │   Joint State   │  │  Other         │ │
│  │   Controller    │  │   Broadcaster   │  │  Controllers   │ │
│  └────────┬────────┘  └────────┬────────┘  └────────────────┘ │
└───────────┼─────────────────────┼───────────────────────────────┘
            │                     │
            ▼                     ▼
┌────────────────────────────────────────────────────────────────┐
│                    Hardware Interface                           │
│           (Real Robot / Gazebo / Mock Hardware)                │
└────────────────────────────────────────────────────────────────┘
```

---

## Prerequisites / Related Concepts

1. **ROS 2** - Understanding of ROS 2 fundamentals
2. **TF2** - Transform library for coordinate frames
3. **Coordinate Frames** - Understanding of reference frames
4. **XML** - Basic XML syntax knowledge
5. **ros2_control** - Controller framework (for control integration)
6. **Kinematics** - For understanding joint configurations

---

## Common Issues & Solutions

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot appears at origin in Gazebo | Missing inertial properties | Add `<inertial>` to all links |
| Joints don't move | Inertia values too small/zero | Use realistic inertia values |
| Self-intersection at startup | Incorrect joint origins | Check `<origin>` transforms |
| xacro parsing error | Syntax error in expressions | Check `${}` syntax, escape `$${}` for literal |
| Mesh not found | Incorrect package path | Use `package://pkg_name/path` format |
| TF tree broken | Missing or duplicate links | Ensure single-parent tree structure |
| Collision not working | Missing collision geometry | Add `<collision>` to links |
| Robot flies away in sim | Unrealistic inertia/mass | Verify mass and inertia calculations |

---

## Source URLs for Citations

### Official ROS 2 Documentation
- [URDF Tutorials (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) - Official ROS 2 URDF tutorials
- [Building Visual Robot Model](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Building-a-Visual-Robot-Model-with-URDF-from-Scratch.html) - Building URDF from scratch
- [Using Xacro (Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Using-Xacro-to-Clean-Up-a-URDF-File.html) - Xacro tutorial
- [Adding Physical Properties](https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html) - Collision and inertial properties
- [URDF XML Specification](http://wiki.ros.org/urdf/XML) - Complete URDF XML reference

### ros2_control Documentation
- [ros2_control Overview](https://control.ros.org/rolling/doc/getting_started/getting_started.html) - Getting started with ros2_control
- [gz_ros2_control](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) - Gazebo integration for ros2_control
- [Hardware Interface](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) - Writing hardware interfaces
- [6DOF Robot Tutorial](https://control.ros.org/rolling/doc/ros2_control_demos/example_7/doc/userdoc.html) - Complete robot arm example

### NVIDIA Isaac Documentation
- [Isaac Sim URDF Import](https://docs.isaacsim.omniverse.nvidia.com/latest/importer_exporter/ext_isaacsim_asset_importer_urdf.html) - URDF Importer Extension
- [URDF Import Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/robot_setup/import_urdf.html) - Step-by-step import guide
- [URDF Importer GitHub](https://github.com/isaac-sim/urdf-importer-extension) - Open source URDF importer
- [cuRobo Robot Configuration](https://curobo.org/tutorials/1_robot_configuration.html) - Configuring robots for cuRobo

### Nav2 Documentation
- [Nav2 URDF Setup](https://docs.nav2.org/setup_guides/urdf/setup_urdf.html) - URDF setup for navigation robots

### Universal Robots Documentation
- [UR ROS 2 Description](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_description/doc/index.html) - UR robot description packages
- [Assembling URDF](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_tutorials/my_robot_cell/doc/assemble_urdf.html) - Creating robot cells with URDF

### Tutorials & Community Resources
- [Articulated Robotics - URDF](https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/) - Comprehensive URDF guide
- [Automatic Addison - URDF Jazzy](https://automaticaddison.com/create-and-visualize-a-robotic-arm-with-urdf-ros-2-jazzy/) - Robot arm URDF tutorial
- [MOGI ROS Gazebo Basics](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics) - URDF with Gazebo Harmonic
- [TU Delft - Xacro Limitations](https://ocw.tudelft.nl/course-lectures/2-2-2-limitations-and-xacro/) - Academic resource on URDF/Xacro

### Xacro Reference
- [Xacro Wiki](http://wiki.ros.org/xacro) - Xacro syntax and features
- [Xacro GitHub](https://github.com/ros/xacro) - Source repository

### ROS 2 Releases
- [Kilted Kaiju Release](https://docs.ros.org/en/jazzy/Releases/Release-Kilted-Kaiju.html) - Latest ROS 2 release notes

---

## Style Notes (from existing entries)

Based on review of moveit2.research.md and other entries:

1. **Frontmatter**: title, description, last_validated, sidebar badge
2. **Badge type**: **Practical** - hands-on, implementation-focused
3. **Opening**: Bold term definition, one-liner summary
4. **Aside notes**: For key callouts (use tip for best practices, caution for warnings)
5. **Prerequisites section**: CardGrid with LinkCards
6. **Key sections to include**:
   - Why URDF/Xacro Matters
   - URDF Structure (Links & Joints)
   - Xacro Features (Properties, Math, Macros)
   - ros2_control Integration
   - Processing & Visualization
   - NVIDIA Integration
   - Common Issues
7. **Code Examples**: Use Tabs for different approaches
8. **Related Terms**: CardGrid with LinkCards
9. **Learn More**: External tutorial links
10. **Sources**: Citation links at bottom

---

## Notes for Content Creation

- Entry should be comprehensive but practical (not just reference docs)
- Emphasize Xacro as the preferred approach over raw URDF
- Include ros2_control integration prominently (key for modern ROS 2)
- Show both command-line and launch file processing
- Cover NVIDIA Isaac Sim integration for import workflow
- Common issues table is essential for debugging
- Link to related concepts: tf2, coordinate-frames, ros2_control, kinematics
- Use Gazebo Ionic (not Classic) for examples
- Target ROS 2 Jazzy as primary, mention Kilted where relevant
