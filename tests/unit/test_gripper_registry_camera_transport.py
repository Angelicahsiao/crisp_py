"""Tests for the GripperBase registry/factory and the camera image transport.

Runs in two modes:
  * On a machine with ROS 2: uses the real rclpy/message packages with Mock
    nodes (same style as the other unit tests).
  * Without ROS (CI sandbox): installs minimal module stubs before importing
    crisp_py, exercising the same code paths.

Run:  python -m pytest tests/unit/test_gripper_registry_camera_transport.py -v
      (or directly: python tests/unit/test_gripper_registry_camera_transport.py)
"""

import sys
import tempfile
import types
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# ── optional ROS stubs (only when rclpy is unavailable) ───────────────────────

def _stub(name: str, **attrs) -> types.ModuleType:
    mod = types.ModuleType(name)
    mod.__path__ = []
    for k, v in attrs.items():
        setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


# crisp_py/__init__ reads the installed package version; shim when not installed
import importlib.metadata as _im

try:
    _im.version("crisp_python")
except _im.PackageNotFoundError:
    _orig_version = _im.version
    _im.version = lambda name: (
        "0.0.0-test" if name == "crisp_python" else _orig_version(name)
    )

try:
    import rclpy  # noqa: F401
except ImportError:
    rclpy = _stub("rclpy", ok=lambda: False, init=lambda: None, shutdown=lambda: None,
                  create_node=lambda *a, **k: MagicMock())
    _stub("rclpy.node", Node=object)
    _stub("rclpy.callback_groups", ReentrantCallbackGroup=MagicMock)
    _stub("rclpy.executors", MultiThreadedExecutor=MagicMock)
    _stub("rclpy.qos", qos_profile_system_default=object(), qos_profile_sensor_data=object())
    _stub("rclpy.action", client=None)
    _stub("rclpy.action.client", ActionClient=MagicMock)

    class _Msg:
        class Request:  # noqa: D106
            def __init__(self):
                self.data = None

        class Goal:  # noqa: D106
            def __init__(self):
                self.command = types.SimpleNamespace(position=0.0, max_effort=0.0)

    _stub("sensor_msgs")
    _stub("sensor_msgs.msg", JointState=type("JointState", (), {}),
          CameraInfo=type("CameraInfo", (), {}),
          CompressedImage=type("CompressedImage", (), {}),
          Image=type("Image", (), {}))
    _stub("std_msgs")
    _stub("std_msgs.msg", Float64MultiArray=type("Float64MultiArray", (), {"data": None}),
          Float32=type("Float32", (), {}))
    _stub("std_srvs")
    _stub("std_srvs.srv", SetBool=_Msg, Trigger=_Msg)
    _stub("control_msgs")
    _stub("control_msgs.action", GripperCommand=_Msg)
    _stub("diagnostic_msgs")
    _stub("diagnostic_msgs.msg", DiagnosticArray=type("DiagnosticArray", (), {}),
          DiagnosticStatus=type("DiagnosticStatus", (), {"OK": 0, "WARN": 1}),
          KeyValue=type("KeyValue", (), {}))
    _stub("geometry_msgs")
    _stub("geometry_msgs.msg", PoseStamped=type("PoseStamped", (), {}),
          TwistStamped=type("TwistStamped", (), {}),
          WrenchStamped=type("WrenchStamped", (), {}),
          TransformStamped=type("TransformStamped", (), {}))
    _stub("cv_bridge", CvBridge=MagicMock)
    if "cv2" not in sys.modules:
        _stub("cv2", resize=lambda *a, **k: None, INTER_AREA=0)

from crisp_py.camera.camera import Camera  # noqa: E402
from crisp_py.camera.camera_config import CameraConfig  # noqa: E402
from crisp_py.gripper.gripper import Gripper, make_gripper  # noqa: E402
from crisp_py.gripper.gripper_base import GRIPPER_REGISTRY  # noqa: E402
from crisp_py.gripper.gripper_config import GripperConfig  # noqa: E402
from crisp_py.gripper.multi_dof_gripper import (  # noqa: E402
    MultiDofGripper,
    MultiDofGripperConfig,
)
from sensor_msgs.msg import CompressedImage, Image  # noqa: E402


def _mock_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node


def _make_gripper(cls=Gripper, config=None, **cfg_kwargs):
    config = config or GripperConfig(min_value=0.0, max_value=1.0, **cfg_kwargs)
    return cls(node=_mock_node(), gripper_config=config, spin_node=False)


# ── registry / factory ────────────────────────────────────────────────────────

class TestGripperRegistry:
    def test_builtin_types_registered(self):
        assert GRIPPER_REGISTRY["gripper"] is Gripper
        assert GRIPPER_REGISTRY["multi_dof"] is MultiDofGripper

    def _write_yaml(self, text: str) -> Path:
        f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        f.write(text)
        f.close()
        return Path(f.name)

    def test_make_gripper_dispatches_on_type_key(self):
        path = self._write_yaml(
            "type: multi_dof\nmin_value: 0.0\nmax_value: 1.0\nnum_joints: 3\n"
        )
        with patch("crisp_py.gripper.gripper.find_config", return_value=path), \
             patch("crisp_py.gripper.multi_dof_gripper.find_config", return_value=path), \
             patch.object(MultiDofGripper, "__init__", return_value=None) as init:
            g = make_gripper("whatever")
            assert isinstance(g, MultiDofGripper)
            assert init.called

    def test_make_gripper_defaults_to_plain_gripper(self):
        path = self._write_yaml("min_value: 0.0\nmax_value: 1.0\n")
        with patch("crisp_py.gripper.gripper.find_config", return_value=path), \
             patch.object(Gripper, "__init__", return_value=None) as init:
            g = make_gripper("whatever")
            assert isinstance(g, Gripper)
            assert init.called

    def test_make_gripper_rejects_unknown_type(self):
        path = self._write_yaml("type: pneumatic\nmin_value: 0.0\nmax_value: 1.0\n")
        with patch("crisp_py.gripper.gripper.find_config", return_value=path):
            with pytest.raises(ValueError, match="Unknown gripper type"):
                make_gripper("whatever")

    def test_make_gripper_dispatches_on_config_instance(self):
        cfg = MultiDofGripperConfig(min_value=0.0, max_value=1.0, num_joints=2)
        with patch.object(MultiDofGripper, "__init__", return_value=None):
            g = make_gripper(None, gripper_config=cfg)
            assert isinstance(g, MultiDofGripper)


# ── shared base behavior ──────────────────────────────────────────────────────

class TestGripperBaseBehavior:
    @pytest.mark.parametrize("cls,config", [
        (Gripper, GripperConfig(min_value=0.0, max_value=1.0, torque_interface=True)),
        (MultiDofGripper, MultiDofGripperConfig(
            min_value=0.0, max_value=1.0, num_joints=2, torque_interface=True)),
    ])
    def test_torque_interface_true_raises_when_service_missing(self, cls, config):
        """torque_interface: true must RAISE on both classes (MultiDof used to
        silently ignore it — the divergence this refactor fixes)."""
        g = cls(node=_mock_node(), gripper_config=config, spin_node=False)
        g.enable_torque_client.service_is_ready = MagicMock(return_value=False)
        with pytest.raises(RuntimeError, match="torque_interface: true"):
            g.enable_torque()
        g.reboot_client.service_is_ready = MagicMock(return_value=False)
        with pytest.raises(RuntimeError, match="torque_interface: true"):
            g.reboot()

    @pytest.mark.parametrize("cls,config", [
        (Gripper, GripperConfig(min_value=0.0, max_value=1.0, torque_interface=False)),
        (MultiDofGripper, MultiDofGripperConfig(
            min_value=0.0, max_value=1.0, num_joints=2, torque_interface=False)),
    ])
    def test_torque_interface_false_is_silent_noop(self, cls, config):
        g = cls(node=_mock_node(), gripper_config=config, spin_node=False)
        g.enable_torque()
        g.disable_torque()
        g.reboot()
        assert not g.enable_torque_client.call_async.called
        assert not g.reboot_client.call_async.called

    def test_torque_interface_none_warns_and_skips(self):
        g = _make_gripper()
        g.enable_torque_client.service_is_ready = MagicMock(return_value=False)
        g.enable_torque()  # must not raise
        assert g.node.get_logger.return_value.warning.called
        assert not g.enable_torque_client.call_async.called

    def test_torque_call_fires_async_when_service_ready(self):
        g = _make_gripper()
        g.enable_torque_client.service_is_ready = MagicMock(return_value=True)
        g.enable_torque(block=False)
        assert g.enable_torque_client.call_async.called

    def test_shutdown_does_not_destroy_external_node(self):
        g = _make_gripper()
        assert g._owns_node is False
        g.shutdown()
        assert not g.node.destroy_node.called

    def test_shutdown_destroys_owned_node(self):
        g = _make_gripper()
        g._owns_node = True  # simulate self-created node
        g.shutdown()
        assert g.node.destroy_node.called

    def test_scalar_normalization_unchanged(self):
        g = _make_gripper(config=GripperConfig(min_value=0.8, max_value=0.0))  # inverted
        assert np.isclose(g._normalize(0.8), 0.0)
        assert np.isclose(g._normalize(0.0), 1.0)
        assert np.isclose(g._unnormalize(0.5), 0.4)

    def test_multi_dof_normalization_unchanged(self):
        cfg = MultiDofGripperConfig(
            min_value=0.0, max_value=1.0, num_joints=2,
            min_values=[0.0, 1.0], max_values=[2.0, 3.0])
        g = MultiDofGripper(node=_mock_node(), gripper_config=cfg, spin_node=False)
        np.testing.assert_allclose(g._normalize(np.array([1.0, 2.0])), [0.5, 0.5])
        np.testing.assert_allclose(g._unnormalize(np.array([0.5, 0.5])), [1.0, 2.0])


# ── camera transport ──────────────────────────────────────────────────────────

def _make_camera(**cfg_kwargs) -> tuple:
    cfg = CameraConfig(
        camera_color_image_topic="/cam/color/image_raw",
        camera_color_info_topic=None,
        resolution=[64, 64],
        **cfg_kwargs,
    )
    node = _mock_node()
    cam = Camera(node=node, config=cfg, spin_node=False)
    msg_type, topic = node.create_subscription.call_args_list[0][0][:2]
    return cam, node, msg_type, topic


class TestCameraTransport:
    def test_compressed_default(self):
        cam, node, msg_type, topic = _make_camera()
        assert msg_type is CompressedImage
        assert topic == "/cam/color/image_raw/compressed"
        assert not node.get_logger.return_value.warning.called

    def test_suffix_override(self):
        _, _, msg_type, topic = _make_camera(compressed_topic_suffix="/compressed_fast")
        assert msg_type is CompressedImage
        assert topic == "/cam/color/image_raw/compressed_fast"

    def test_raw_transport_subscribes_image_and_warns_about_storage(self):
        cam, node, msg_type, topic = _make_camera(image_transport="raw")
        assert msg_type is Image
        assert topic == "/cam/color/image_raw"
        warning = node.get_logger.return_value.warning
        assert warning.called
        assert "RAW" in warning.call_args[0][0]
        assert "storage" in warning.call_args[0][0]

    def test_invalid_transport_rejected(self):
        with pytest.raises(ValueError, match="image_transport"):
            CameraConfig(
                camera_color_image_topic="/x", camera_color_info_topic=None,
                resolution=[64, 64], image_transport="jpeg",
            )

    def test_decode_routes_by_transport(self):
        cam, *_ = _make_camera()
        cam._uncompress = MagicMock(return_value="compressed_decoded")
        cam._image_to_array = MagicMock(return_value="raw_decoded")
        assert cam._decode(object()) == "compressed_decoded"
        cam.config.image_transport = "raw"
        assert cam._decode(object()) == "raw_decoded"


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
