"""Configuration classes for robots in CRISP."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml


@dataclass
class RobotConfig:
    """Base configuration for a robot in CRISP.

    Attributes:
        robot_type: The type of robot (e.g., "franka", "ur").
        namespace: ROS namespace for the robot.
        base_frame: The base frame of the robot.
        end_effector_frame: The end-effector frame of the robot.
        home_config: Default joint configuration for the home position.
        target_pose_topic: Topic for publishing target poses.
        target_joint_topic: Topic for publishing target joint configurations.
        current_pose_topic: Topic for the current pose.
        joint_states_topic: Topic for joint states.
        target_stiffness_topic: Topic for publishing target Cartesian stiffness.
        target_admittance_stiffness_topic: Topic for admittance stiffness.
        cartesian_controller_name: Name of the Cartesian controller.
        cartesian_admittance_controller_name: Name of the admittance controller.
        joint_controller_name: Name of the joint controller.
    """

    robot_type: str = "generic"
    namespace: str = ""
    base_frame: str = "base_link"
    end_effector_frame: str = "end_effector_link"
    home_config: Optional[List[float]] = None

    target_pose_topic: str = "target_pose"
    target_joint_topic: str = "target_joint"
    current_pose_topic: str = "current_pose"
    joint_states_topic: str = "joint_states"
    target_stiffness_topic: str = "target_stiffness"
    target_admittance_stiffness_topic: str = "target_admittance_stiffness"

    cartesian_controller_name: str = "cartesian_impedance_controller"
    cartesian_admittance_controller_name: str = "cartesian_admittance_controller"
    joint_controller_name: str = "joint_trajectory_controller"

    def num_joints(self) -> int:
        """Return the number of joints for this robot."""
        raise NotImplementedError

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> "RobotConfig":
        """Load a robot configuration from a YAML file."""
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        return make_robot_config(**data)


@dataclass
class FrankaConfig(RobotConfig):
    """Configuration for Franka Emika Panda / FR3 arms."""

    robot_type: str = "franka"
    end_effector_frame: str = "fr3_hand_tcp"

    def num_joints(self) -> int:
        return 7


@dataclass
class FrankaAdmittanceConfig(FrankaConfig):
    """Franka configuration with admittance control enabled."""

    cartesian_controller_name: str = "cartesian_admittance_controller"


@dataclass
class URConfig(RobotConfig):
    """Configuration for Universal Robots arms."""

    robot_type: str = "ur"
    base_frame: str = "base"
    end_effector_frame: str = "tool0"
    ur_type: str = "ur5e"

    def num_joints(self) -> int:
        return 6


def make_robot_config(robot_type: str = "generic", **kwargs) -> RobotConfig:
    """Factory to create a robot configuration based on robot_type."""
    mapping = {
        "franka": FrankaConfig,
        "franka_admittance": FrankaAdmittanceConfig,
        "ur": URConfig,
    }
    config_cls = mapping.get(robot_type, RobotConfig)
    valid = {f.name for f in __import__("dataclasses").fields(config_cls)}
    return config_cls(**{k: v for k, v in kwargs.items() if k in valid})
