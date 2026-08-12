"""Module for JointState-effort sensor specifications."""

import numpy as np
from sensor_msgs.msg import JointState

from crisp_py.sensors.sensor import SensorSpec, register_sensor


@register_sensor("joint_state_effort")
def get_joint_state_effort_sensor_spec() -> SensorSpec:
    """Get the sensor specification for the `effort` field of a JointState.

    For publishers that carry a joint-space torque in a sensor_msgs/JointState
    rather than a bare array — notably franka_ros2's
    `franka_robot_state_broadcaster/external_joint_torques`, which reports the
    EXTERNAL joint torque with gravity and arm dynamics already removed by the
    robot's firmware.

    Prefer this over `robot.joint_efforts` when what you want is contact:
    Robot.current_joint_effort reads the effort field of the robot's own
    /joint_states, which is the TOTAL measured torque and is dominated by
    gravity when the arm is merely holding position. It also degrades to a
    silent vector of zeros when effort is absent, whereas a missing sensor
    raises.

    Values are returned in the publisher's own joint order — unlike
    Robot._ros_msg_to_joint_effort, a sensor has no robot config to reorder
    against. Check `ros2 topic echo <topic> --field name` once and confirm it
    matches the order you expect before trusting the recorded columns.

    Returns:
        SensorSpec: Tuple containing the ROS message type and conversion function.
    """
    return (
        JointState,
        lambda msg: np.array(msg.effort, dtype=np.float32),
    )
