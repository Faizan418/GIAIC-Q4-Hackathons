from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.world",
        description="Choose one of the world files from `/usr/share/gazebo-11/worlds` or specify custom world path",
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "world": [
                PathJoinSubstitution(
                    [FindPackageShare("my_robot_gazebo"), "worlds", "basic_room.world"]
                )
            ]
        }.items(),
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": True, "publish_frequency": 50.0}],
        remappings=[("/joint_states", "joint_states")],
    )

    # Joint State Publisher node (for non-fixed joints)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True, "rate": 50}],
        remappings=[("/joint_states", "joint_states")],
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "my_robot",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.5",  # Start 0.5m above ground to ensure proper spawning
        ],
        output="screen",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch arguments and nodes to the launch description
    ld.add_action(world_arg)
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(spawn_robot)

    return ld
