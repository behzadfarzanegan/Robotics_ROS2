from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",  # Fixed typo here
        default_value="0.17",
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bumperbot_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )
    simple_controller_py = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],  # Fixed typo
        condition=IfCondition(use_python),
    )

    simple_controller_cpp = Node(
        package="bumperbot_controller",
        executable="simple_controller",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],  # Fixed typo
        condition=UnlessCondition(use_python),
    )

    return LaunchDescription(
        [
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            simple_controller,
            simple_controller_py,
            simple_controller_cpp,
        ]
    )
