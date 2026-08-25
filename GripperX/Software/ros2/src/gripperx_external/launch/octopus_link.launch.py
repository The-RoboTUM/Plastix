"""Launch the external-goal link: transport node + gateway.

    # rollout stage 1 - telemetry only, no goal ingress, no action client
    ros2 launch gripperx_external octopus_link.launch.py

    # rollout stage 2 - goal path, disarmed and dry-run
    ros2 launch gripperx_external octopus_link.launch.py goal_ingress:=true

    # rollout stage 3 - dispatch possible, STILL DISARMED until an explicit
    # SetArming call; dry_run:=false only removes the second block
    ros2 launch gripperx_external octopus_link.launch.py goal_ingress:=true dry_run:=false

Both nodes live in the namespace ``/gripperx/external``, which is where SR-15
puts the arming service (``/gripperx/external/set_arming``) and where the config
files key their parameters.

NOTHING HERE CAN ARM ANYTHING
=============================
There is deliberately no ``arm`` or ``allow_arm`` launch argument. SR-15 rule 4:
no parameter, launch argument, environment variable, config file or service
default may result in ``armed == true`` at startup. Arming is a runtime
``SetArming`` service call and nothing else. ``dry_run`` is exposed only in the
safe direction - it is a launch argument so a run can be made *more* restricted,
never less; the default is ``true`` and the disarmed state blocks dispatch
independently of it.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ---------------------------------------------------------------------------
# additional_env={} - DELIBERATELY NOT GRIPPERX_ROSDEPS_LIB
# ---------------------------------------------------------------------------
# sim_navigation.launch.py and navigation.launch.py prepend
# GRIPPERX_ROSDEPS_LIB to LD_LIBRARY_PATH for the Nav2 C++ nodes, because those
# .rosdeps_local debs were built against a NEWER diagnostic_updater ABI than the
# system ROS on this laptop.
#
# These two nodes must NOT get that treatment:
#   * nav2_msgs is system-installed (ros-jazzy-nav2-msgs 1.3.12) and
#     byte-identical to the .rosdeps_local copy bt_navigator uses - both 1.3.12,
#     .action files diff-clean - so there is nothing to gain;
#   * prepending those libs would drag the newer-ABI diagnostic_updater into a
#     pure-python node for no reason at all.
# For the same reason the DiagnosticArray in diagnostics.py is assembled by hand
# instead of via diagnostic_updater. Leave this empty.
_NO_ROSDEPS_ENV: dict = {}

_NAMESPACE = "gripperx/external"


def generate_launch_description():
    package_share = get_package_share_directory("gripperx_external")

    env = LaunchConfiguration("env")
    goal_ingress = LaunchConfiguration("goal_ingress")
    dry_run = LaunchConfiguration("dry_run")
    url = LaunchConfiguration("url")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    params_file = PathJoinSubstitution(
        [
            FindPackageShare("gripperx_external"),
            "config",
            PythonExpression(["'octopus_link_' + '", env, "' + '.yaml'"]),
        ]
    )

    # Only the two stage switches are overridable from the command line, and
    # only these. Everything else comes from the config file, so a run is
    # reproducible from one reviewable artefact.
    overrides = {
        "goal_ingress_enabled": goal_ingress,
        "use_sim_time": use_sim_time,
    }

    link_node = Node(
        package="gripperx_external",
        executable="octopus_link_node",
        name="octopus_link_node",
        namespace=_NAMESPACE,
        output="screen",
        parameters=[params_file, dict(overrides, url=url)],
        additional_env=_NO_ROSDEPS_ENV,
    )

    gateway_node = Node(
        package="gripperx_external",
        executable="goal_gateway_node",
        name="goal_gateway_node",
        namespace=_NAMESPACE,
        output="screen",
        parameters=[params_file, dict(overrides, dry_run=dry_run)],
        additional_env=_NO_ROSDEPS_ENV,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_octopus_goals",
        output="screen",
        condition=IfCondition(rviz),
        arguments=["-d", os.path.join(package_share, "rviz", "octopus_goals.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "env",
                default_value="twin",
                choices=["twin", "real"],
                description=(
                    "Which config file to load. twin requires ROS_DOMAIN_ID=220, "
                    "real requires 20 (SR-8); a mismatch is FATAL at startup."
                ),
            ),
            DeclareLaunchArgument(
                "goal_ingress",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "false (stage 1): the external goal topics are not subscribed "
                    "at all and no goal can enter the process. true (stage 2/3): "
                    "goals are validated and previewed, and from stage 3 they can "
                    "be dispatched - but only after an explicit SetArming call, "
                    "which no launch argument can perform."
                ),
            ),
            DeclareLaunchArgument(
                "dry_run",
                default_value="true",
                choices=["true", "false"],
                description=(
                    "Second, independent block on dispatch. Setting it false "
                    "arms nothing: the gateway still starts disarmed, and "
                    "disarmed alone still prevents every dispatch (SR-15 rule 2)."
                ),
            ),
            DeclareLaunchArgument(
                "url",
                default_value="ws://127.0.0.1:9090",
                description="rosbridge WebSocket. Default: test/fake_octopus.py.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                # DERIVED FROM `env`, not a fixed default (SAFETY.md F-24). It
                # used to default to "true" for both envs, and these overrides
                # are applied AFTER the config file, so `env:=real` silently
                # overrode `octopus_link_real.yaml`'s `use_sim_time: false` and
                # put the REAL ROBOT - where there is no /clock at all - on a
                # clock that never advances, with the arming expiry, the link
                # watchdog and every in-flight re-check measured on it. The
                # gateway now refuses to start in that combination; this makes
                # the combination stop happening by default as well.
                default_value=PythonExpression(
                    ["'true' if '", env, "' == 'twin' else 'false'"]
                ),
                choices=["true", "false"],
                description=(
                    "true against the twin, false on the real robot; defaults to "
                    "whichever `env` implies. true REQUIRES a live /clock "
                    "publisher (Gazebo): with sim time and no /clock every "
                    "timer-driven safety mechanism in the gateway is inert, "
                    "which the gateway now detects, reports at ERROR and refuses "
                    "to arm on (SAFETY.md F-24). Running the twin stack WITHOUT "
                    "Gazebo - against mocks, as the acceptance suite does - is "
                    "use_sim_time:=false."
                ),
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Start RViz with the preview config. Prefer standalone RViz "
                    "(DIGITAL_TWIN_PLAN.md 9.4); launch-started RViz is a known "
                    "flakiness issue."
                ),
            ),
            link_node,
            gateway_node,
            rviz_node,
        ]
    )
