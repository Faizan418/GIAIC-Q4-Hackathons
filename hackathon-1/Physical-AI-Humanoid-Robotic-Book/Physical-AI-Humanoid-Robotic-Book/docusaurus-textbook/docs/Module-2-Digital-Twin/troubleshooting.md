# Simulation Troubleshooting Guide

This guide provides solutions to common issues encountered when working with Gazebo simulation, ROS 2 integration, and Unity digital twins.

## Common Gazebo Issues

### Simulation Instability
**Problem**: Objects sinking into surfaces or exhibiting explosive behavior.
**Solutions**:
- Reduce physics step size in world file (try 0.001 or smaller)
- Increase solver iterations in physics configuration
- Verify URDF mass properties are physically realistic
- Check that inertia tensors are properly defined

### Robot Not Moving
**Problem**: Robot ignores velocity commands.
**Solutions**:
- Verify joint names match between URDF and controller configuration
- Check that joint state publisher is running
- Confirm controller manager is properly configured and running
- Validate that command topics are correctly connected

### Sensor Data Issues
**Problem**: Sensors not publishing data or publishing invalid values.
**Solutions**:
- Verify sensor plugins are properly configured in URDF
- Check that sensor topics are being published (use `ros2 topic list`)
- Confirm sensor frame names match TF tree
- Validate sensor noise parameters aren't causing extreme values

## ROS 2 Integration Problems

### Connection Issues
**Problem**: ROS 2 nodes can't communicate with Gazebo.
**Solutions**:
- Verify gazebo_ros_pkgs are installed and sourced
- Check that ROS_DOMAIN_ID is consistent across processes
- Confirm network configuration allows communication
- Verify Gazebo plugins are properly loaded

### TF Tree Problems
**Problem**: TF tree is incomplete or missing transforms.
**Solutions**:
- Ensure robot_state_publisher is running
- Verify joint_state_publisher is publishing joint states
- Check that URDF joint names match actual joints
- Confirm base_link is properly defined in URDF

### Topic Communication Issues
**Problem**: Topics not receiving or sending messages.
**Solutions**:
- Verify topic names match between publishers and subscribers
- Check QoS profiles are compatible between nodes
- Confirm nodes are on the same ROS domain
- Use `ros2 topic echo` to verify data flow

## Unity Integration Issues

### URDF Import Problems
**Problem**: URDF not importing correctly into Unity.
**Solutions**:
- Ensure all mesh files are accessible and in supported formats (FBX, OBJ)
- Verify URDF file has valid XML syntax
- Check that joint names and types are supported by importer
- Confirm coordinate systems align between ROS and Unity

### ROS-TCP-Connector Issues
**Problem**: Unity and ROS 2 can't communicate.
**Solutions**:
- Verify IP addresses and ports are correctly configured
- Check firewall settings aren't blocking TCP connections
- Confirm both Unity and ROS sides are using same protocol version
- Verify ROS 2 nodes are properly connected to TCP bridge

### Performance Problems
**Problem**: Low frame rates or lag in Unity simulation.
**Solutions**:
- Reduce complexity of visual meshes
- Implement Level of Detail (LOD) systems
- Optimize lighting and shadow settings
- Consider using occlusion culling for complex scenes

## URDF Validation

### Common URDF Errors
**Problem**: URDF fails to load or behaves unexpectedly.
**Solutions**:
- Validate XML syntax using `check_urdf` command
- Verify all joint limits are properly defined
- Check that mass values are positive and realistic
- Confirm inertia tensors follow positive-definite rules
- Ensure all referenced mesh files exist and are accessible

### Inertia Issues
**Problem**: Robot behaves unrealistically in simulation.
**Solutions**:
- Use CAD software to calculate accurate inertia tensors
- Verify center of mass is within the physical bounds of the link
- Check that inertia values are appropriate for the link's mass
- Consider using simplified inertia calculations for basic shapes

## Simulation Optimization

### Performance Tips
- Use simplified collision geometry where detailed shapes aren't necessary
- Reduce physics update rate if real-time performance isn't required
- Limit the number of active sensors during development
- Use fixed joints instead of revolute joints when no motion is needed

### Debugging Strategies
- Use Gazebo's built-in visualization tools (contact points, joint axes)
- Monitor ROS 2 topics with `ros2 topic echo` and `rqt`
- Enable detailed logging for simulation components
- Test components individually before integration

## Unity-Specific Troubleshooting

### Scene Setup Issues
**Problem**: Robot appears incorrectly in Unity scene.
**Solutions**:
- Verify coordinate system conversion (ROS uses right-handed, Unity uses left-handed)
- Check that robot is properly scaled (ROS typically uses meters)
- Confirm initial robot pose matches Gazebo simulation
- Validate that joint axes align between URDF and Unity

### Physics Discrepancies
**Problem**: Robot behavior differs between Gazebo and Unity.
**Solutions**:
- Verify physics parameters match between both environments
- Check that mass properties are consistent
- Confirm joint limits and constraints are identical
- Validate that control inputs are applied identically

## Network and Communication

### Connection Timeouts
**Problem**: Network connections between systems time out.
**Solutions**:
- Check network latency between systems
- Verify firewall settings allow required ports
- Consider using local loopback if systems are on same machine
- Increase timeout values if appropriate for application

### Data Synchronization
**Problem**: Unity visualization doesn't match ROS 2 state.
**Solutions**:
- Verify timestamp synchronization between systems
- Check message queue sizes aren't causing delays
- Confirm that state updates are processed in correct order
- Consider implementing interpolation for smoother visualization

## Development Workflow

### Iterative Testing
- Test individual components before integration
- Use simple test cases to verify basic functionality
- Gradually increase complexity as components work correctly
- Document working configurations for future reference

### Version Control
- Keep URDF files under version control
- Document ROS 2 and Gazebo version requirements
- Track Unity project settings that affect simulation
- Maintain backup configurations for different scenarios
