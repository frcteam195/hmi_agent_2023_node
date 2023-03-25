import typing

from ck_ros_msgs_node.msg import Arm_Goal, Arm_Status
from ck_utilities_py_node.ckmath import *

def limit_drive_power(arm_status: Arm_Status, forward_velocity: float, angular_rotation: float) -> typing.Tuple[float, float]:
    """
    Limit the drive power depending on the current arm position.
    """
    forward_limit = 1.0
    angular_limit = 1.0

    if arm_status is not None:
        overall_arm_angle = abs(arm_status.arm_base_angle + arm_status.arm_upper_angle)
        overall_arm_angle = limit(overall_arm_angle, 0.0, 150)

        forward_limit = -0.006666667 * overall_arm_angle + 1.2
        angular_limit = -0.006666667 * overall_arm_angle + 1.2

        if arm_status.extended or arm_status.goal in (Arm_Goal.SHELF_CONE_PICKUP, Arm_Goal.SHELF_CUBE_PICKUP):
            forward_limit -= 0.1
            angular_limit -= 0.1

    forward_limit = limit(forward_limit, 0.0, 1.0)
    angular_limit = limit(angular_limit, 0.0, 1.0)

    return limit(forward_velocity, -forward_limit, forward_limit), limit(angular_rotation, -angular_limit, angular_limit)
