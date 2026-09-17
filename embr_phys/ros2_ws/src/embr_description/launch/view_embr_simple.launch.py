from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("embr_description")
    model_path = PathJoinSubstitution(
        [package_share, "urdf", "embr_simple", "urdf", "embr_simple.urdf"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "embr_simple.rviz"]
    )

    robot_description = ParameterValue(
                    Command([FindExecutable(name="xacro"), " ", model_path]),
                    value_type=str,
                )

    controllers_file = PathJoinSubstitution(
        [package_share, "config", "embr_simple_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "publish_frequency": 50.0}],
        output="screen",
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        remappings=[('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')],
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "diff_drive_controller"],
            output="screen",
        ),
        rviz2        
    ])
