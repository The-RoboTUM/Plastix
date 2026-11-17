"""The ONE TF-freshness bound shared by goal_gateway_node and arm_action_server
(SR-16 E3, audit F4, user decision 2026-11-13 option a): its file in the source
tree and its parameters, read from that ACTIVE file - never a copy."""

import os

import yaml

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SHARED_TF_FILE = os.path.join(_SRC, "gripperx_arm_msgs", "config", "tf_freshness.yaml")


def shared_tf_params():
    with open(SHARED_TF_FILE) as f:
        return dict(yaml.safe_load(f)["/**"]["ros__parameters"])


def shared_tf_overrides():
    from rclpy.parameter import Parameter
    return [Parameter(k, value=v) for k, v in shared_tf_params().items()]
