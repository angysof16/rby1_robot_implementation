from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        name="gui",
        default_value="true",
        choices=["true", "false"],
        description="Enable joint_state_publisher_gui",
    ),
    DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution(
            [
                get_package_share_directory("urdf_rby"),
                "urdf",
                "robots",
                "rby_standalone.urdf.xacro",
            ]
        ),
        description="Path to the robot URDF/Xacro",
    ),
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    ),
]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(
        IncludeLaunchDescription(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch", "display.launch.py"]
            ),
            launch_arguments={
                "urdf_package": "urdf_rby",
                "urdf_package_path": LaunchConfiguration("model"),
                "jsp_gui": LaunchConfiguration("gui"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rviz_config": PathJoinSubstitution(
                    [
                        FindPackageShare("urdf_rby"),
                        "launch",
                        "config.rviz",
                    ]
                ),
            }.items(),
        )
    )
    return ld
