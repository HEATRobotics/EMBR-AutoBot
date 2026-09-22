from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("embr_description")
    model_path = PathJoinSubstitution(
        [package_share, "urdf", "planetary_eagle", "urdf", "eagle_power_test_stand_assembly.xacro"]
    )
    controllers_file = PathJoinSubstitution(
        [package_share, "config", "planetary_eagle_controllers.yaml"]
    )
    rviz_config = PathJoinSubstitution([package_share, "rviz", "planetary_eagle.rviz"])
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", model_path]), value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop",
                default_value="true",
                description="Start the TeleCmd-to-simulated-joint command bridge.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description, "publish_frequency": 50.0}],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[{"robot_description": robot_description}, controllers_file],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "planetary_eagle_controller"],
                output="screen",
            ),
            Node(
                package="embr_core",
                executable="cp_helper",
                arguments=["--sim"],
                condition=IfCondition(LaunchConfiguration("teleop")),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
