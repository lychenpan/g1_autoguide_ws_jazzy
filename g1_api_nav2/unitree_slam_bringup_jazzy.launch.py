"""
Unitree SLAM + Nav2 bringup for ROS 2 Jazzy using nav2_ws overlay.

This launch starts:
  - static map->odom TF
  - map_server
  - unitree_relocation_odom_bridge (python3 script)
  - Nav2 navigation stack from /workspace/nav2_ws
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

_G1_API_DIR = os.path.dirname(os.path.abspath(__file__))
_WS_ROOT = os.path.dirname(_G1_API_DIR)
if _WS_ROOT not in sys.path:
    sys.path.insert(0, _WS_ROOT)

from g1_api_nav2.paths import (  # noqa: E402
    MAP_YAML,
    PARAMS_YAML,
    RELOCATION_ODOM_BRIDGE,
)
def generate_launch_description():
    nav2_ws = os.environ.get("NAV2_WS", "/workspace/nav2_ws")
    nav2_install_prefix = os.path.join(nav2_ws, "install")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

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

    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "1"),
            # Ensure nav2_ws overlay takes precedence over /opt/ros.
            SetEnvironmentVariable(
                "AMENT_PREFIX_PATH",
                [nav2_install_prefix, ":", EnvironmentVariable("AMENT_PREFIX_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                "COLCON_PREFIX_PATH",
                [nav2_install_prefix, ":", EnvironmentVariable("COLCON_PREFIX_PATH", default_value="")],
            ),
            SetEnvironmentVariable(
                "CMAKE_PREFIX_PATH",
                [nav2_install_prefix, ":", EnvironmentVariable("CMAKE_PREFIX_PATH", default_value="")],
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
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="screen",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": autostart,
                        "node_names": ["map_server"],
                    }
                ],
            ),
            ExecuteProcess(
                cmd=["python3", RELOCATION_ODOM_BRIDGE],
                name="unitree_relocation_odom_bridge",
                output="screen",
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
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
        ]
    )
