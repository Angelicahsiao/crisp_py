"""Generic class for a gripper based on a simple ros2 topic."""

import numpy as np
import yaml
from control_msgs.action import GripperCommand
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from crisp_py.config.path import find_config, list_configs_in_folder
from crisp_py.gripper.gripper_base import GRIPPER_REGISTRY, GripperBase, register_gripper
from crisp_py.gripper.gripper_config import GripperConfig


@register_gripper("gripper")
class Gripper(GripperBase):
    """Single-DOF gripper client: one normalized [0, 1] value.

    Node/spin/monitor/reboot/torque machinery lives in GripperBase.
    """

    NODE_NAME = "gripper_client"

    def _setup_io(self):
        """Create the command publisher/action client, joint subscriber and target timer."""
        self._value = None
        self._torque = None
        self._target = None
        self._index = self.config.index

        self._command_publisher = (
            self.node.create_publisher(
                Float64MultiArray,
                self.config.command_topic,
                qos_profile_system_default,
                callback_group=ReentrantCallbackGroup(),
            )
            if not self.config.use_gripper_command_action
            else None
        )
        self._command_action_client = (
            ActionClient(
                self.node,
                GripperCommand,
                self.config.command_topic,
                callback_group=ReentrantCallbackGroup(),
            )
            if self.config.use_gripper_command_action
            else None
        )

        self._joint_subscriber = self.node.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._callback_monitor.monitor(
                f"{self._namespace.capitalize()} Gripper Joint State", self._callback_joint_state
            ),
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{self._namespace.capitalize()} Gripper Target Publisher",
                self._callback_publish_target,
            ),
            ReentrantCallbackGroup(),
        )

    @classmethod
    def from_yaml(
        cls,
        config_name: str,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        **overrides,  # noqa: ANN003
    ) -> "Gripper":
        """Create a Gripper instance from a YAML configuration file.

        Args:
            config_name: Name of the config file (with or without .yaml extension)
            node: ROS2 node to use. If None, creates a new node.
            namespace: ROS2 namespace for the gripper.
            spin_node: Whether to spin the node in a separate thread.
            **overrides: Additional parameters to override YAML values

        Returns:
            Gripper: Configured gripper instance

        Raises:
            FileNotFoundError: If the config file is not found
        """
        if not config_name.endswith(".yaml"):
            config_name += ".yaml"

        config_path = find_config(f"grippers/{config_name}")
        if config_path is None:
            config_path = find_config(config_name)

        if config_path is None:
            raise FileNotFoundError(
                f"Gripper config file '{config_name}' not found in any CRISP config paths"
            )

        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}

        data.pop("type", None)  # dispatch key consumed by make_gripper
        data.update(overrides)

        namespace = data.pop("namespace", namespace)
        config_data = data.pop("gripper_config", data)

        gripper_config = GripperConfig(**config_data)

        return cls(
            node=node,
            namespace=namespace,
            gripper_config=gripper_config,
            spin_node=spin_node,
        )

    @property
    def min_value(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.min_value

    @property
    def max_value(self) -> float:
        """Returns the maximum width of the gripper."""
        return self.config.max_value

    @property
    def torque(self) -> float | None:
        """Returns the current torque of the gripper or None if not initialized."""
        return self._torque

    @property
    def is_valid(self) -> bool:
        """Returns true if the joint values received are valid."""
        if self._value is None:
            self.node.get_logger().error(
                f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first."
            )
            return False
        if self._normalize(self._value) > 1.05 or self._normalize(self._value) < -0.05:
            self.node.get_logger().error(
                f"{self._prefix}Gripper value {self._value} is out of bounds [0.0, 1.0]. Please check the gripper configuration, and eventually calibrate the gripper."
            )
            self.node.get_logger().error(
                f"The raw value of the gripper is {self.raw_value}, which is not in the range [{self.min_value}, {self.max_value}]."
            )
            return False
        return True

    @property
    def value(self) -> float | None:
        """Returns the current value of the gripper or None if not initialized."""
        if self._value is None:
            raise RuntimeError(
                f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first."
            )
        namespace_part = self._prefix.rstrip("_").capitalize() if self._prefix else ""
        callback_name = f"{namespace_part} Gripper Joint State".strip()
        try:
            joint_callback_data = self._callback_monitor.get_callback_data(callback_name)
            if joint_callback_data and joint_callback_data.is_stale:
                self.node.get_logger().warn(f"{self._prefix}Gripper joint state is stale")
        except ValueError:
            pass
        return np.clip(self._normalize(self._value), 0.0, 1.0)

    @property
    def raw_value(self) -> float | None:
        """Returns the current raw value of the gripper or None if not initialized."""
        return self._value

    @property
    def target(self) -> float:
        """Returns the target value of the gripper."""
        return np.clip(self._normalize(self._target), 0.0, 1.0)

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        action_client_ready = (
            self._command_action_client.wait_for_server(timeout_sec=0.0)
            if self._command_action_client
            else True
        )
        return self._value is not None and action_client_ready

    def is_open(self, open_threshold: float = 0.1) -> bool:
        """Returns True if the gripper is open."""
        if self.value is None:
            raise RuntimeError("Gripper value is not initialized. Call wait_until_ready() first.")
        return self.value > open_threshold

    def close(self):
        """Close the gripper."""
        self.set_target(target=0.0)

    def open(self):
        """Open the gripper."""
        self.set_target(target=1.0)

    def _callback_publish_target(self):
        """Publish the target command."""
        if self._target is None:
            return

        if self.config.use_gripper_command_action:
            if self._command_action_client is None:
                raise RuntimeError("Command action client is not initialized.")

            goal = GripperCommand.Goal()
            goal.command.position = self._unnormalize(
                self.value
                + np.clip(
                    self._normalize(self._target) - self.value,
                    -self.config.max_delta,
                    self.config.max_delta,
                )
            )
            goal.command.max_effort = self.config.max_effort
            self._command_action_client.send_goal_async(goal)
            return

        if self._command_publisher is None:
            raise RuntimeError("Command publisher is not initialized.")

        msg = Float64MultiArray()
        msg.data = [
            self._unnormalize(
                self.value
                + np.clip(
                    self._normalize(self._target) - self.value,
                    -self.config.max_delta,
                    self.config.max_delta,
                )
            )
        ]
        self._command_publisher.publish(msg)

    def _callback_joint_state(self, msg: JointState):
        """Save the latest joint state values.

        Note: we assume that the gripper value is the first element of the joint message.

        Args:
            msg (JointState): the message containing the joint state.
        """
        if self._index >= len(msg.position):
            # Wrong topic / shorter JointState than expected: warn instead of
            # raising IndexError inside the executor thread (which would kill
            # the subscription silently). MultiDofGripper guards the same way.
            self.node.get_logger().warning(
                f"{self._prefix}JointState has {len(msg.position)} positions but "
                f"gripper joint index is {self._index} — check the gripper "
                "joint_state topic/config.",
                throttle_duration_sec=5.0,
            )
            return
        self._value = msg.position[self._index]
        self._torque = (
            msg.effort[self._index] if len(msg.effort) > self._index else None
        )

    def set_target(self, target: float, *, epsilon: float = 0.1):
        """Grasp with the gripper by setting a target. This can be a position, velocity or effort depending on the active controller.

        Args:
            target (float): The target value for the gripper between 0 and 1 from closed to open respectively.
            epsilon (float): allowed zone around the target limits that are allowed to be set.
        """
        assert 0.0 - epsilon <= target <= 1.0 + epsilon, (
            f"The target should be normalized between 0 and 1, but is currently {target}"
        )
        self._target = self._unnormalize(target)

    def _normalize(self, unormalized_value: float) -> float:
        """Normalize a raw value between 0.0 and 1.0."""
        return (unormalized_value - self.min_value) / (self.max_value - self.min_value)

    def _unnormalize(self, normalized_value: float) -> float:
        """Normalize a raw value between 0.0 and 1.0."""
        return (self.max_value - self.min_value) * normalized_value + self.min_value

def make_gripper(
    config_name: str | None,
    gripper_config: GripperConfig | None = None,
    node: "Node | None" = None,
    namespace: str = "",
    spin_node: bool = True,
    **overrides,  # noqa: ANN003
) -> GripperBase:
    """Factory: create a gripper of the right TYPE from a config file or config.

    Dispatch:
      * YAML path: an optional top-level ``type:`` key selects the
        implementation from the gripper registry (``gripper`` default,
        ``multi_dof`` for MultiDofGripper). No ``type:`` key -> plain Gripper,
        exactly as before.
      * gripper_config instance: dispatched on the config class
        (MultiDofGripperConfig -> MultiDofGripper, else Gripper).

    Args:
        config_name: Name of the gripper config file
        gripper_config: Directly provide a GripperConfig instance instead of loading from file.
        node: ROS2 node to use. If None, creates a new node.
        namespace: ROS2 namespace for the gripper.
        spin_node: Whether to spin the node in a separate thread.
        **overrides: Additional parameters to override config values

    Returns:
        GripperBase: Configured gripper instance of the dispatched type.

    Raises:
        FileNotFoundError: If the config file is not found
        ValueError: If the `type:` key names an unregistered gripper type.
    """
    # ensure all built-in implementations are registered
    from crisp_py.gripper.multi_dof_gripper import MultiDofGripper, MultiDofGripperConfig  # noqa: F401

    if not ((not config_name and gripper_config) or (config_name and not gripper_config)):
        raise ValueError("Either config_name or gripper_config must be provided, not both.")

    if config_name is not None:
        yaml_name = config_name if config_name.endswith(".yaml") else config_name + ".yaml"
        config_path = find_config(f"grippers/{yaml_name}") or find_config(yaml_name)
        if config_path is None:
            raise FileNotFoundError(
                f"Gripper config file '{config_name}' not found in any CRISP config paths"
            )
        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}
        type_name = overrides.pop("type", data.get("type", "gripper"))
        cls = GRIPPER_REGISTRY.get(type_name)
        if cls is None:
            raise ValueError(
                f"Unknown gripper type '{type_name}' in {config_path}. "
                f"Registered types: {sorted(GRIPPER_REGISTRY)}"
            )
        return cls.from_yaml(
            config_name=config_name,
            node=node,
            namespace=namespace,
            spin_node=spin_node,
            **overrides,
        )

    cls = MultiDofGripper if isinstance(gripper_config, MultiDofGripperConfig) else Gripper
    return cls(
        gripper_config=gripper_config, node=node, namespace=namespace, spin_node=spin_node
    )


def list_gripper_configs() -> list[str]:
    """List all available gripper configurations."""
    configs = list_configs_in_folder("grippers")
    return [config.stem for config in configs if config.suffix == ".yaml"]
