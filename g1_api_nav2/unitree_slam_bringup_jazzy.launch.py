"""
Unitree SLAM + Nav2 bringup for ROS 2 Jazzy using nav2_ws overlay.

Startup sequence:
  1. RealSense camera + unitree bridges + hand detection (parallel)
  2. wait until /unitree/odom and /utlidar/pcl2 are publishing
  3. static TFs + map_server + Nav2 stack + showroom workflow
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

_G1_API_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.dirname(_G1_API_DIR)
if _WS_ROOT not in sys.path:
    sys.path.insert(0, _WS_ROOT)

from g1_api_nav2.paths import (  # noqa: E402
    CAMERA_HANDDET,
    MAP_YAML,
    PARAMS_YAML,
    PCL_BRIDGE,
    RELOCATION_ODOM_BRIDGE,
    SHOWROOM_WORKFLOW,
    START_CAMERA,
    WAIT_FOR_TOPICS,
)


def _named_python_cmd(process_name: str, script_path: str, extra_args=None):
    """Run a Python script with exec -a so rcutils log files use process_name."""
    if extra_args:
        return [
            "bash",
            "-c",
            [f"exec -a {process_name} python3 ", script_path, *extra_args],
        ]
    return [
        "bash",
        "-c",
        f"exec -a {process_name} python3 {script_path}",
    ]


def generate_launch_description():
    nav2_ws = os.environ.get("NAV2_WS", "/workspace/nav2_ws")
    nav2_install_prefix = os.path.join(nav2_ws, "install")
    g1_fun_ws = os.environ.get("G1_FUN_WS", "/workspace/g1_fun_ws")
    g1_fun_install_prefix = os.path.join(g1_fun_ws, "install")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    bridge_wait_timeout = LaunchConfiguration("bridge_wait_timeout")
    bridge_startup_delay = LaunchConfiguration("bridge_startup_delay")
    bridge_output = LaunchConfiguration("bridge_output")
    nav2_output = LaunchConfiguration("nav2_output")
    workflow_output = LaunchConfiguration("workflow_output")
    camera_output = LaunchConfiguration("camera_output")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
        },
        convert_types=True,
    )

    odom_bridge = ExecuteProcess(
        cmd=_named_python_cmd("unitree_relocation_odom_bridge", RELOCATION_ODOM_BRIDGE),
        name="unitree_relocation_odom_bridge",
        output=bridge_output,
    )

    realsense_camera = ExecuteProcess(
        cmd=["bash", START_CAMERA],
        name="realsense_camera",
        output=camera_output,
    )

    hand_3d_node = ExecuteProcess(
        cmd=_named_python_cmd("hand_3d_node", CAMERA_HANDDET),
        name="hand_3d_node",
        output=camera_output,
    )

    pcl_bridge = ExecuteProcess(
        cmd=_named_python_cmd("utlidar_pcl_bridge", PCL_BRIDGE),
        name="utlidar_pcl_bridge",
        output=bridge_output,
    )

    wait_for_bridge_topics = ExecuteProcess(
        cmd=_named_python_cmd(
            "wait_for_bridge_topics",
            WAIT_FOR_TOPICS,
            [" /unitree/odom /utlidar/pcl2 --timeout ", bridge_wait_timeout],
        ),
        name="wait_for_bridge_topics",
        output=bridge_output,
    )

    post_wait_stack = GroupAction(
        [
            Node(
                package="cmd_vel_bridge",
                executable="cmd_vel_bridge",
                name="cmd_vel_bridge",
                output=bridge_output,
                parameters=[
                    {
                        "print_cmd_vel_log": True,
                        "enable_min_yaw_clamp": True,
                    }
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output=bridge_output,
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_livox_frame",
                arguments=[
                    "0",
                    "0",
                    "1.3",
                    "0",
                    "0.047",
                    "3.141592653589793",
                    "base_link",
                    "livox_frame",
                ],
                output=bridge_output,
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output=nav2_output,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output=nav2_output,
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output=nav2_output,
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": [
                            "controller_server",
                            "planner_server",
                            "behavior_server",
                            "bt_navigator",
                            "waypoint_follower",
                        ],
                    }
                ],
            ),
            ExecuteProcess(
                cmd=_named_python_cmd("showroom_workflow", SHOWROOM_WORKFLOW),
                name="showroom_workflow",
                output=workflow_output,
            ),
        ]
    )

    def on_bridge_wait_exit(event, context):
        if event.returncode != 0:
            return [
                LogInfo(
                    msg=(
                        f"ERROR: Bridge topic wait failed (exit {event.returncode}); "
                        "Nav2 stack will not start. Check odom/pcl bridges and robot SLAM."
                    )
                ),
                Shutdown(reason="bridge topics not ready"),
            ]
        return [post_wait_stack]

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
            SetEnvironmentVariable(
                "AMENT_PREFIX_PATH",
                [
                    g1_fun_install_prefix,
                    ":",
                    nav2_install_prefix,
                    ":",
                    EnvironmentVariable("AMENT_PREFIX_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "COLCON_PREFIX_PATH",
                [
                    g1_fun_install_prefix,
                    ":",
                    nav2_install_prefix,
                    ":",
                    EnvironmentVariable("COLCON_PREFIX_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                "CMAKE_PREFIX_PATH",
                [
                    g1_fun_install_prefix,
                    ":",
                    nav2_install_prefix,
                    ":",
                    EnvironmentVariable("CMAKE_PREFIX_PATH", default_value=""),
                ],
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PARAMS_YAML,
                description="Nav2 params yaml",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=MAP_YAML,
                description="Occupancy grid yaml (must match Unitree SLAM map)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "bridge_wait_timeout",
                default_value="60",
                description="Seconds to wait for each bridge topic before starting Nav2",
            ),
            DeclareLaunchArgument(
                "bridge_startup_delay",
                default_value="1.0",
                description="Seconds after bridge launch before topic wait begins",
            ),
            DeclareLaunchArgument(
                "bridge_output",
                default_value="screen",
                description=(
                    "Logging for bridges/cmd_vel/static TFs/wait script "
                    "(log | screen | both)"
                ),
            ),
            DeclareLaunchArgument(
                "nav2_output",
                default_value="screen",
                description="Logging for map_server and Nav2 stack (log | screen | both)",
            ),
            DeclareLaunchArgument(
                "workflow_output",
                default_value="screen",
                description="Logging for showroom workflow node (log | screen | both)",
            ),
            DeclareLaunchArgument(
                "camera_output",
                default_value="screen",
                description="Logging for RealSense camera and hand detection (log | screen | both)",
            ),
            realsense_camera,
            hand_3d_node,
            odom_bridge,
            pcl_bridge,
            TimerAction(
                period=bridge_startup_delay,
                actions=[wait_for_bridge_topics],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_bridge_topics,
                    on_exit=on_bridge_wait_exit,
                )
            ),
        ]
    )
