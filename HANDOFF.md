# Development Handoff Notes — crisp_py

Instructions for future AI-assisted development sessions on this repo.
crisp_py is the ROS2 client library (robots, grippers, cameras, sensors,
controller clients) consumed by `crisp_gym`. Changes here ripple into
crisp_gym's recording and deployment pipelines — read `crisp_gym/HANDOFF.md`
too before touching anything pose- or gripper-related.

---

## 1. Things that must not change silently

### 1.1 OrientationRepresentation (`crisp_py/utils/geometry.py`)
- Members: `QUATERNION`, `EULER`, `ANGLE_AXIS`, `ROTATION_6D` ("rotation_6d").
- `ROTATION_6D` encodes the **first two ROWS of the rotation matrix, flattened
  row-major** (UMI / pytorch3d convention). NOT columns. Decoding requires
  Gram-Schmidt. crisp_gym's recorded datasets and trained models depend on
  this exact convention — changing it invalidates existing data/checkpoints.
- crisp_gym's test suite stubs this enum
  (`crisp_gym/tests/test_lerobot_record.py::_OrientationRepresentation`);
  if a member is added/renamed here, update that stub in the same change set.

### 1.2 Config field parity
- `GripperConfig.from_yaml` manually enumerates fields instead of `cls(**data)`.
  This already caused a bug once (`use_gripper_command_action` and `max_effort`
  were silently dropped; fixed on main). When adding ANY field to
  `GripperConfig`, add it in BOTH the dataclass and the `from_yaml` dict —
  or better, refactor to `cls(**data)` like `CameraConfig` does.
- `RobotConfig` has `has_effort_feedback: bool = False` — effort observation in
  crisp_gym only activates when a robot config sets this true AND the JointState
  messages actually carry effort values.

### 1.3 TCP frame definition
- The TCP frame is `RobotConfig.target_frame` (e.g. `fr3_hand_tcp`), realized by
  the crisp_controllers CIC. crisp_gym's UMI handheld pipeline calibrates its
  handheld TCP (`tx_body_tcp`) to match THIS frame's convention. Renaming or
  re-orienting a robot's target_frame breaks handheld<->robot data compatibility.

---

## 2. Extensibility: the agreed direction (assessed, partially pending)

The repo has FOUR different extension patterns; new code should follow the
best one (the sensor registry) rather than adding to the worst:

| Family | Current mechanism | State |
|---|---|---|
| Sensors | decorator registry (`sensors/sensor.py`: `@register_sensor`) | GOOD — the model to copy |
| Robots | `make_robot_config()` if/elif chain (`robot/robot_config.py`) + subclass + manual `__init__.py` export (exports already drifted: Panda/UR/DynaArm configs not exported) | to be replaced by a `@register_robot_config` registry |
| Grippers | single concrete `Gripper` class; variation via config booleans (`use_gripper_command_action`); `MultiDofGripper` added as a parallel standalone class | wants a `GripperBase` + registry so MultiDofGripper and future protocols slot in |
| Cameras | single concrete `Camera`; hardcoded `CompressedImage` transport (`/compressed` suffix, rgb8) | wants a base class if raw/depth transports are ever needed |

Known small bugs (agreed to fix):
- FIXED: `Robot.home()` now uses `config.home_controller_name` (a NEW field —
  `joint_trajectory_controller_name` could not be reused because, despite its
  name, it is the parameter-client target of the streaming joint controller).
  `JointTrajectoryControllerClient` takes `controller_name`.
- Broadcaster detection by `name.endswith("broadcaster")` in
  `control/controller_switcher.py`.
- Duplicated crop/resize validation between `CameraConfig.__post_init__` and
  `Camera._pre_crop`.

Shared boilerplate (`_spin_node`, `from_yaml`, `wait_until_ready`, `make_*`,
`list_*_configs`) is re-implemented in Robot/Gripper/Camera/Sensor — a shared
base/mixin is welcome, but do it as its own PR, not mixed into feature work.

## 3. ROS coupling map (relevant when decoupling inference from ROS)

- Pure Python (safe to import anywhere, keep them that way):
  all `*_config.py` dataclasses, `config/path.py`, `utils/geometry.py`,
  `utils/sliding_buffer.py`.
- Hard ROS2-coupled (rclpy / ROS msgs at module top): `robot/robot.py`,
  `gripper/*.py`, `camera/camera.py`, `sensors/*.py`, all of `control/`,
  `utils/tf_pose.py`.
- Do not add rclpy imports (even indirect) to the pure-Python modules —
  crisp_gym's training/inference code paths import `geometry.py` on machines
  without ROS.

## 4. Process rules

- Develop on the designated `claude/...` branch; the owner merges PRs via the
  GitHub website. Never push to other branches without explicit permission.
- Check ALL remote branches (`git ls-remote --heads origin`) before assessing
  repo state — side branches (UR7e fixes, DG3F gripper) have carried unmerged
  work before.
- ROS2 Humble pins Python to 3.11 on robot machines; anything intended to run
  on a training/GPU machine must not require crisp_py at all (crisp_gym keeps
  those files dependency-free — don't break that by "helpfully" importing
  crisp_py utilities there).
