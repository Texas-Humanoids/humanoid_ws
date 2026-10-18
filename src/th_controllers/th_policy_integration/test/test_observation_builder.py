import numpy as np
import pytest
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from th_policy_integration.observation_builder import (
    ordered_joint_state,
    projected_gravity,
)
from th_policy_node.profile import BERKELEY_HUMANOID_JOINTS


def test_joint_feedback_is_reordered_by_name():
    names = list(reversed(BERKELEY_HUMANOID_JOINTS))
    message = JointState()
    message.name = names
    message.position = [float(i) for i in range(22)]
    message.velocity = [float(i) / 10 for i in range(22)]
    positions, velocities = ordered_joint_state(message, BERKELEY_HUMANOID_JOINTS)
    assert positions.tolist() == [float(i) for i in reversed(range(22))]
    assert np.allclose(velocities, [i / 10 for i in reversed(range(22))])


def test_missing_velocity_is_rejected():
    message = JointState()
    message.name = BERKELEY_HUMANOID_JOINTS
    message.position = [0.0] * 22
    with pytest.raises(ValueError, match="positions and velocities"):
        ordered_joint_state(message, BERKELEY_HUMANOID_JOINTS)


def test_projected_gravity_matches_body_rotation():
    assert np.allclose(projected_gravity(Quaternion(w=1.0)), [0, 0, -1])
    half = 2 ** -0.5
    assert np.allclose(
        projected_gravity(Quaternion(x=half, w=half)), [0, -1, 0], atol=1e-6
    )


def test_invalid_orientation_is_rejected():
    with pytest.raises(ValueError, match="orientation"):
        projected_gravity(Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))
