# Code Example Template

This template provides a standard format for all code examples in the Digital Twin simulation module.

## Template Structure

Each code example should follow this structure:

### Example Title
**Description**: Brief explanation of what the code demonstrates
**Language**: [Language identifier for syntax highlighting]
**File Location**: [Where this code would be placed in a real project]

```[language]
# Paste code here with proper syntax highlighting
```

**Explanation**: Detailed explanation of the code functionality
**Key Points**:
- Important concepts demonstrated
- Common pitfalls to avoid
- Related concepts or extensions

## Standard Code Example Formats

### ROS 2 Launch Files (Python)
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'worlds',
                'simple_world.world'
            ])
        }.items()
    )
    ld.add_action(gazebo)

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True}]
    )
    ld.add_action(robot_state_publisher)

    return ld
```

### URDF Robot Models (XML)
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Additional links and joints -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

### Gazebo World Files (SDF)
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### ROS 2 Control Nodes (Python)
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Simple controller initialized')

    def scan_callback(self, msg):
        # Process laser scan data
        pass

    def control_loop(self):
        # Implement control logic
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Unity C# Scripts
```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry;

public class RobotController : MonoBehaviour
{
    [SerializeField]
    string topicName = "/cmd_vel";

    ROSConnection ros;
    public float linearVelocity = 1.0f;
    public float angularVelocity = 1.0f;

    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    // Update is called once per frame
    void Update()
    {
        // Example: Publish velocity commands based on input
        if (Input.GetKey(KeyCode.UpArrow))
        {
            var twist = new TwistMsg();
            twist.linear = new Vector3Msg(linearVelocity, 0, 0);
            ros.Publish(topicName, twist);
        }
    }
}
```

## Code Example Standards

### Required Elements
1. **Clear Purpose**: Each example must have a specific, well-defined purpose
2. **Complete Functionality**: Examples should be runnable with minimal modification
3. **Proper Comments**: Include comments explaining complex or non-obvious code
4. **Error Handling**: Include appropriate error handling where relevant
5. **Documentation**: Brief explanation of what the code does and why

### Quality Checks
- [ ] Code is properly formatted with correct syntax highlighting
- [ ] All variable names are descriptive and follow conventions
- [ ] Comments explain the "why" not just the "what"
- [ ] Example includes necessary imports/includes
- [ ] Code is tested and functional
- [ ] Example demonstrates a practical, real-world scenario
- [ ] Error handling is included where appropriate

### Language-Specific Guidelines

#### Python/ROS 2
- Use 4-space indentation
- Follow PEP 8 style guide
- Include proper type hints where beneficial
- Use descriptive variable and function names

#### XML (URDF/SDF)
- Use 2-space indentation for readability
- Include proper namespace declarations
- Use descriptive names for elements
- Maintain consistent attribute ordering

#### C++/ROS 2
- Use 2-space indentation
- Follow ROS 2 style guide
- Include proper header guards
- Use appropriate namespaces

#### C# (Unity)
- Use 4-space indentation
- Follow Unity/C# conventions
- Include proper serialization attributes
- Use appropriate access modifiers

## Common Code Example Categories

### Setup and Configuration
- Environment setup scripts
- Configuration file examples
- Package initialization code

### Basic Operations
- Simple movement commands
- Sensor data processing
- Basic control loops

### Advanced Features
- Complex algorithms
- Multi-robot coordination
- Advanced sensor fusion

### Integration Examples
- ROS 2 to Gazebo connections
- Unity to ROS communication
- External system integration

Use this template to ensure all code examples maintain consistent quality and format throughout the Digital Twin module.
