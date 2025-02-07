import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Add this import

def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    # Declare the model argument with a default value
    model_arg = DeclareLaunchArgument(name="model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "bumperbot.urdf.xacro"
                                        ),
                                      description="Absolute path to robot urdf file"
    )
    
    # Declare the world_name argument (this will be passed from the command line)
    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="small_house.world")

    # Correct world path construction using PathJoinSubstitution
    world_path = PathJoinSubstitution([
        bumperbot_description,
        "worlds",
        LaunchConfiguration("world_name")  # This will be substituted with the world name
    ])

    # Set up the model path for Gazebo
    model_path = os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")

    # Set environment variable for Gazebo model path
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    # Load robot description from the urdf file using xacro
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    # Create robot state publisher node to publish robot state
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Start the Gazebo server with the specified world
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()  # Pass the full world path here
    )

    # Start the Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(package="gazebo_ros", executable="spawn_entity.py",
                        arguments=["-entity", "bumperbot",
                                   "-topic", "robot_description",
                                  ],
                        output="screen"
    )

    # Set up ROS 2 bridge for communication with Gazebo
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[('/imu', '/imu/out')],
    )

    # Return the complete launch description
    return LaunchDescription([
        env_var,
        model_arg,
        world_name_arg,
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher_node,
        spawn_robot,
        gz_ros2_bridge
    ])
