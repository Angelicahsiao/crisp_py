"""N-DOF gripper class for grippers like the Tesollo Delto DG3F.

The :class:`crisp_py.gripper.gripper.Gripper` interface treats the gripper as a
single normalized [0, 1] value. For multi-finger grippers (e.g. the DG3F has
12 actuated joints), each joint must be commanded independently. This module
provides a parallel implementation that:

- Publishes a ``std_msgs/Float64MultiArray`` of length ``num_joints`` to the
  command topic.
- Subscribes to a ``sensor_msgs/JointState`` topic and reads the joints listed
  in ``joint_indices``.
- Normalizes each joint independently between ``min_values[i]`` and
  ``max_values[i]``.
"""

import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from crisp_py.config.path import find_config
from crisp_py.gripper.gripper_config import GripperConfig
from crisp_py.utils.callback_monitor import CallbackMonitor


@dataclass
class MultiDofGripperConfig(GripperConfig):
    """Config for an N-DOF gripper with per-joint normalization.

    Attributes:
        num_joints: Number of actuated joints to command and observe.
        min_values: Per-joint minimum raw values (length ``num_joints``).
        max_values: Per-joint maximum raw values (length ``num_joints``).
        joint_indices: Indices into the ``JointState`` message to read for
            each of the ``num_joints`` joints.
        joint_names: Optional human-readable joint names (length ``num_joints``).
    """

    num_joints: int = 1
    min_values: List[float] = field(default_factory=list)
    max_values: List[float] = field(default_factory=list)
    joint_indices: List[int] = field(default_factory=list)
    joint_names: List[str] = field(default_factory=list)

    def __post_init__(self):
        if not self.min_values:
            self.min_values = [float(self.min_value)] * self.num_joints
        if not self.max_values:
            self.max_values = [float(self.max_value)] * self.num_joints
        if not self.joint_indices:
            self.joint_indices = list(range(self.num_joints))
        if len(self.min_values) != self.num_joints:
            raise ValueError(
                f"min_values length ({len(self.min_values)}) does not match num_joints ({self.num_joints})."
            )
        if len(self.max_values) != self.num_joints:
            raise ValueError(
                f"max_values length ({len(self.max_values)}) does not match num_joints ({self.num_joints})."
            )
        if len(self.joint_indices) != self.num_joints:
            raise ValueError(
                f"joint_indices length ({len(self.joint_indices)}) does not match num_joints ({self.num_joints})."
            )

    @classmethod
    def from_yaml(cls, path, **overrides) -> "MultiDofGripperConfig":  # noqa: ANN001, ANN003
        """Load a multi-DOF gripper config from YAML."""
        if isinstance(path, str):
            found_path = find_config(path)
            full_path = found_path if found_path is not None else Path(path)
        else:
            full_path = path

        with open(full_path, "r") as f:
            data = yaml.safe_load(f) or {}
        data.pop("type", None)
        data.update(overrides)
        return cls(**data)


class MultiDofGripper:
    """ROS2 client for an N-DOF gripper.

    Mirrors the public surface of :class:`crisp_py.gripper.gripper.Gripper` but
    operates on per-joint arrays of shape ``(num_joints,)`` instead of scalars.
    """

    THREADS_REQUIRED = 2

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        gripper_config: MultiDofGripperConfig | None = None,
        spin_node: bool = True,
    ):
        if not rclpy.ok() and node is None:
            rclpy.init()

        self.node = (
            rclpy.create_node(
                node_name="multi_dof_gripper_client",
                namespace=namespace,
                parameter_overrides=[],
            )
            if not node
            else node
        )
        self.config: MultiDofGripperConfig = (
            gripper_config
            if gripper_config is not None
            else MultiDofGripperConfig(min_value=0.0, max_value=1.0, num_joints=1)
        )

        self._prefix = f"{namespace}_" if namespace else ""
        self._values: np.ndarray | None = None
        self._target: np.ndarray | None = None
        self._mins = np.asarray(self.config.min_values, dtype=np.float64)
        self._maxs = np.asarray(self.config.max_values, dtype=np.float64)
        self._range = self._maxs - self._mins
        self._range = np.where(self._range == 0.0, 1.0, self._range)

        self._callback_monitor = CallbackMonitor(
            self.node, stale_threshold=self.config.max_joint_delay
        )

        self._command_publisher = self.node.create_publisher(
            Float64MultiArray,
            self.config.command_topic,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self._joint_subscriber = self.node.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} MultiDofGripper Joint State",
                self._callback_joint_state,
            ),
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} MultiDofGripper Target Publisher",
                self._callback_publish_target,
            ),
            ReentrantCallbackGroup(),
        )

        self.reboot_client = self.node.create_client(Trigger, self.config.reboot_service)
        self.enable_torque_client = self.node.create_client(
            SetBool, self.config.enable_torque_service
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    @property
    def num_joints(self) -> int:
        return self.config.num_joints

    @property
    def value(self) -> np.ndarray:
        """Per-joint normalized [0, 1] values, shape ``(num_joints,)``."""
        if self._values is None:
            raise RuntimeError(
                f"{self._prefix}MultiDofGripper is not initialized. Call wait_until_ready() first."
            )
        return np.clip(self._normalize(self._values), 0.0, 1.0)

    @property
    def raw_value(self) -> np.ndarray | None:
        return self._values

    @property
    def target(self) -> np.ndarray | None:
        if self._target is None:
            return None
        return np.clip(self._normalize(self._target), 0.0, 1.0)

    def is_ready(self) -> bool:
        return self._values is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError(
                    f"Timeout waiting for multi-DOF gripper. Is {self._joint_subscriber.topic_name} being published?"
                )

    def is_open(self, open_threshold: float = 0.5) -> bool:
        return bool(np.mean(self.value) > open_threshold)

    def open(self):
        self.set_target(np.ones(self.num_joints))

    def close(self):
        self.set_target(np.zeros(self.num_joints))

    def set_target(self, target, *, epsilon: float = 0.05):  # noqa: ANN001
        """Set per-joint normalized target.

        Args:
            target: Array-like of shape ``(num_joints,)`` with values in [0, 1].
            epsilon: Tolerance allowed outside [0, 1] before clipping.
        """
        target = np.asarray(target, dtype=np.float64)
        if target.shape != (self.num_joints,):
            raise ValueError(
                f"Target shape {target.shape} does not match num_joints={self.num_joints}."
            )
        if np.any(target < -epsilon) or np.any(target > 1.0 + epsilon):
            raise ValueError(
                f"Target values must be in [0, 1] (with eps={epsilon}). Got: {target}"
            )
        self._target = self._unnormalize(np.clip(target, 0.0, 1.0))

    def _normalize(self, raw: np.ndarray) -> np.ndarray:
        return (raw - self._mins) / self._range

    def _unnormalize(self, normalized: np.ndarray) -> np.ndarray:
        return normalized * self._range + self._mins

    def _callback_joint_state(self, msg: JointState):
        positions = np.asarray(msg.position, dtype=np.float64)
        try:
            self._values = positions[self.config.joint_indices]
        except IndexError:
            self.node.get_logger().warning(
                f"JointState only has {len(positions)} joints, but joint_indices includes {max(self.config.joint_indices)}"
            )

    def _callback_publish_target(self):
        if self._target is None or self._values is None:
            return
        delta = np.clip(
            self._target - self._values,
            -self.config.max_delta,
            self.config.max_delta,
        )
        cmd = self._values + delta
        msg = Float64MultiArray()
        msg.data = cmd.tolist()
        self._command_publisher.publish(msg)

    def shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def reboot(self, block: bool = False):
        if not self.reboot_client.service_is_ready():
            raise RuntimeError(
                f"Reboot service {self.config.reboot_service} is not available."
            )
        if block:
            self.reboot_client.call(Trigger.Request())
        else:
            self.reboot_client.call_async(Trigger.Request())

    def enable_torque(self, block: bool = False):
        self._set_torque_holding(enable=True, block=block)

    def disable_torque(self, block: bool = False):
        self._set_torque_holding(enable=False, block=block)

    def _set_torque_holding(self, enable: bool, block: bool = False):
        if not self.enable_torque_client.service_is_ready():
            # The DG3F driver does not expose the dynamixel torque service;
            # warn rather than fail so envs can still run.
            self.node.get_logger().warning(
                f"Torque service {self.config.enable_torque_service} not available; skipping."
            )
            return
        req = SetBool.Request()
        req.data = enable
        if block:
            self.enable_torque_client.call(req)
        else:
            self.enable_torque_client.call_async(req)

    @classmethod
    def from_yaml(
        cls,
        config_name: str,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        **overrides,  # noqa: ANN003
    ) -> "MultiDofGripper":
        if not config_name.endswith(".yaml"):
            config_name += ".yaml"
        config_path = find_config(f"grippers/{config_name}") or find_config(config_name)
        if config_path is None:
            raise FileNotFoundError(
                f"Multi-DOF gripper config '{config_name}' not found in CRISP config paths."
            )
        config = MultiDofGripperConfig.from_yaml(config_path, **overrides)
        return cls(
            node=node, namespace=namespace, gripper_config=config, spin_node=spin_node
        )
