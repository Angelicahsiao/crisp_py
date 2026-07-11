"""Shared base class + registry for gripper clients.

`Gripper` (single normalized scalar) and `MultiDofGripper` (per-joint arrays)
share everything protocol-independent through :class:`GripperBase`:
node creation/ownership, spin thread, callback monitoring, the reboot/torque
service clients with the tri-state ``torque_interface`` semantics, YAML
loading plumbing and shutdown. Subclasses implement only the value/target
protocol and the joint-state callback.

The registry (:func:`register_gripper` / :func:`make_gripper_from_type`)
implements the HANDOFF-agreed extensibility direction (modeled on the sensor
registry): a gripper YAML may carry ``type: multi_dof`` (default ``gripper``)
and the factory dispatches on it.

torque_interface tri-state (single source of truth — do NOT reimplement in
subclasses):
    None  -> best effort: use the service if available, warn+skip otherwise
    False -> silently skip (gripper has no Dynamixel-style interface)
    True  -> required: raise if the service is unavailable
"""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from typing import Callable, Dict, Type

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from crisp_py.gripper.gripper_config import GripperConfig
from crisp_py.utils.callback_monitor import CallbackMonitor

# ── registry ──────────────────────────────────────────────────────────────────

GRIPPER_REGISTRY: Dict[str, Type["GripperBase"]] = {}


def register_gripper(type_name: str) -> Callable[[type], type]:
    """Class decorator registering a gripper implementation under a YAML `type:` key."""

    def _decorator(cls: type) -> type:
        GRIPPER_REGISTRY[type_name] = cls
        cls.GRIPPER_TYPE = type_name
        return cls

    return _decorator


class GripperBase(ABC):
    """Common machinery for gripper clients (see module docstring)."""

    THREADS_REQUIRED = 2
    GRIPPER_TYPE = "gripper"
    NODE_NAME = "gripper_client"
    SERVICE_CALL_TIMEOUT = 10.0

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        gripper_config: GripperConfig | None = None,
        spin_node: bool = True,
    ):
        """Initialize node, monitor and service clients (shared by all grippers).

        Args:
            node: ROS2 node to use. If None, creates (and later owns) a new node.
            namespace: ROS2 namespace for the gripper.
            gripper_config: Configuration; subclasses provide their default.
            spin_node: Whether to spin the node in a separate daemon thread.
        """
        if not rclpy.ok() and node is None:
            rclpy.init()

        if node is None:
            self.node = rclpy.create_node(
                node_name=self.NODE_NAME, namespace=namespace, parameter_overrides=[]
            )
            self._owns_node = True
        else:
            self.node = node
            self._owns_node = False

        self.config = gripper_config if gripper_config is not None else self._default_config()

        self._prefix = f"{namespace}_" if namespace else ""
        self._namespace = namespace
        self._callback_monitor = CallbackMonitor(
            self.node, stale_threshold=self.config.max_joint_delay
        )

        self.reboot_client = self.node.create_client(Trigger, self.config.reboot_service)
        self.enable_torque_client = self.node.create_client(
            SetBool, self.config.enable_torque_service
        )

        # Subclass wiring: publishers/subscribers/timers for the value protocol.
        self._setup_io()

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    # ── subclass protocol ─────────────────────────────────────────────────────

    @staticmethod
    def _default_config() -> GripperConfig:
        """Config used when none is provided."""
        return GripperConfig(min_value=0.0, max_value=1.0)

    @abstractmethod
    def _setup_io(self) -> None:
        """Create the command publisher / joint subscriber / target timer."""

    @property
    @abstractmethod
    def value(self):
        """Current normalized value(s)."""

    @abstractmethod
    def set_target(self, target, *, epsilon: float = 0.1) -> None:
        """Set the normalized target value(s)."""

    @abstractmethod
    def is_ready(self) -> bool:
        """True once joint values have been received."""

    # ── shared behavior ───────────────────────────────────────────────────────

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the first joint values arrive or raise TimeoutError."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError(
                    f"Timeout waiting for {type(self).__name__}. Is the joint state "
                    f"topic '{self.config.joint_state_topic}' being published?"
                )

    def shutdown(self):
        """Destroy this gripper's node (if it created one) and shut down rclpy.

        Note: rclpy.shutdown() is global — in a multi-object process, shut down
        the LAST object only, or manage rclpy lifetime yourself.
        """
        if self._owns_node:
            try:
                self.node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    # ── reboot / torque (tri-state, single implementation) ───────────────────

    def _call_service(self, client, request, block: bool):
        """Fire a service call; when blocking, use call_async + timeout (a
        synchronous client call deadlocks with an externally spun node)."""
        future = client.call_async(request)
        if not block:
            return None
        t_start = time.time()
        while not future.done():
            time.sleep(0.01)
            if time.time() - t_start > self.SERVICE_CALL_TIMEOUT:
                raise TimeoutError(
                    f"Service call on {type(self).__name__} timed out after "
                    f"{self.SERVICE_CALL_TIMEOUT}s."
                )
        return future.result()

    def reboot(self, block: bool = False):
        """Reboot the gripper if the reboot service is available (tri-state)."""
        if self.config.torque_interface is False:
            return
        if not self.reboot_client.service_is_ready():
            if self.config.torque_interface is True:
                raise RuntimeError(
                    f"Reboot service {self.config.reboot_service} is not available "
                    "although the gripper config declares torque_interface: true."
                )
            self.node.get_logger().warning(
                f"Reboot service {self.config.reboot_service} not available; skipping."
            )
            return
        self._call_service(self.reboot_client, Trigger.Request(), block)

    def enable_torque(self, block: bool = False):
        """Enable torque holding (tri-state, see module docstring)."""
        self._set_torque_holding(enable=True, block=block)

    def disable_torque(self, block: bool = False):
        """Disable torque holding (tri-state, see module docstring)."""
        self._set_torque_holding(enable=False, block=block)

    def _set_torque_holding(self, enable: bool, block: bool = False):
        if self.config.torque_interface is False:
            return
        if not self.enable_torque_client.service_is_ready():
            if self.config.torque_interface is True:
                raise RuntimeError(
                    f"Torque service {self.config.enable_torque_service} is not "
                    "available although the gripper config declares "
                    "torque_interface: true."
                )
            self.node.get_logger().warning(
                f"Torque service {self.config.enable_torque_service} not available; skipping."
            )
            return
        req = SetBool.Request()
        req.data = enable
        self._call_service(self.enable_torque_client, req, block)
