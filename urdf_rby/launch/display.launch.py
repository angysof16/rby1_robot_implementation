from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = FindPackageShare("urdf_rby")

    default_model = PathJoinSubstitution([pkg_share, "urdf", "robots", "rby_standalone.urdf.xacro"])
    default_rviz  = PathJoinSubstitution([pkg_share, "launch", "config.rviz"])

    # --- Args ---
    model_arg       = DeclareLaunchArgument("model", default_value=default_model, description="Ruta al URDF/Xacro")
    gui_arg         = DeclareLaunchArgument("gui", default_value="true", choices=["true","false"], description="Usar joint_state_publisher_gui")
    use_sim_time_arg= DeclareLaunchArgument("use_sim_time", default_value="false", description="Usar reloj de simulación")
    rviz_arg        = DeclareLaunchArgument("rviz_config", default_value=default_rviz, description="Config de RViz")

    # --- robot_description desde Xacro ---
    robot_description = {
        "robot_description": ParameterValue(Command([
            "xacro ", LaunchConfiguration("model"),
            # Si tu Xacro acepta más params, puedes añadirlos aquí, p.ej.:
            " use_sensors:=true"
        ]), value_type=str) # Envuelve el Command en ParameterValue y especifica value_type=str
    }

    # --- Nodos ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )

    jsp_cli = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[robot_description],
        condition=UnlessCondition(LaunchConfiguration("gui")),
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    return LaunchDescription([
        model_arg, gui_arg, use_sim_time_arg, rviz_arg,
        rsp, jsp_gui, jsp_cli, rviz2
    ])
