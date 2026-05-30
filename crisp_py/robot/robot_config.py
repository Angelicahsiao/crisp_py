"""Robot configuration classes for crisp_py."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml


@dataclass
class RobotConfig:
    """Base configuration for a robot arm."""

    robot_type: str = "generic"
    home_config: Optional[list[float]] = None
    namespace: str = ""
    base_frame: str = "base_link"
    end_effector_frame: str = "end_effector_link"

    target_pose_topic: str = "target_pose"
    current_pose_topic: str = "current_pose"
    joint_states_topic: str = "joint_states"
    target_joint_topic: str = "target_joint"

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

    def num_joints(self) -> int:
        return 7

    @property
    def home_joint_config(self) -> list[float]:
        """Default home joint configuration for Franka arms."""
        return [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


@dataclass
class URConfig(RobotConfig):
    """Configuration for Universal Robots arms."""

    robot_type: str = "ur"
    ur_model: str = "ur5e"

    def num_joints(self) -> int:
        return 6

    @property
    def home_joint_config(self) -> list[float]:
        """Default home joint configuration for UR arms."""
        return [0.0, -1.571, 1.571, -1.571, -1.571, 0.0]

    @property
    def ur_dh_params(self) -> dict:
        """DH parameters per UR model."""
        dh = {
            "ur5e": {"d1": 0.1625, "a2": -0.425, "a3": -0.3922, "d4": 0.1333, "d5": 0.0997, "d6": 0.0996},
            "ur7e": {"d1": 0.1807, "a2": -0.4784, "a3": -0.36, "d4": 0.17415, "d5": 0.11985, "d6": 0.11655},
            "ur10e": {"d1": 0.1807, "a2": -0.6127, "a3": -0.57155, "d4": 0.17415, "d5": 0.11985, "d6": 0.11655},
        }
        return dh.get(self.ur_model, dh["ur5e"])


def make_robot_config(robot_type: str = "generic", **kwargs) -> RobotConfig:
    """Factory to create a robot configuration based on robot_type."""
    mapping = {
        "franka": FrankaConfig,
        "ur": URConfig,
    }
    config_cls = mapping.get(robot_type, RobotConfig)
    return config_cls(**{k: v for k, v in kwargs.items() if k in _config_fields(config_cls)})


def _config_fields(cls) -> set:
    """Return the set of valid field names for a config dataclass."""
    import dataclasses

    return {f.name for f in dataclasses.fields(cls)}
