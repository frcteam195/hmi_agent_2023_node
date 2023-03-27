from ck_ros_msgs_node.msg import Arm_Goal

from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import Alliance

AWAY_GOALS = [
    Arm_Goal.GROUND_DEAD_CONE,
    Arm_Goal.GROUND_CONE,
    Arm_Goal.GROUND_CUBE,
    Arm_Goal.PRE_DEAD_CONE,
    Arm_Goal.SIDEWAYS_DEAD_CONE,
    Arm_Goal.SHELF_PICKUP
]

INTAKE_GOALS = [
    Arm_Goal.GROUND_DEAD_CONE,
    Arm_Goal.GROUND_CONE,
    Arm_Goal.GROUND_CUBE,
    Arm_Goal.PRE_DEAD_CONE,
    Arm_Goal.SIDEWAYS_DEAD_CONE
]

def get_away_side(heading: float) -> int:
    """
    Gets the side of the robot facing your opponent's alliance.
    """
    front_alliance = Alliance.RED if (90 < heading < 270) else Alliance.BLUE
    away_side = Arm_Goal.SIDE_FRONT if front_alliance != robot_status.get_alliance() else Arm_Goal.SIDE_BACK
    return away_side

def get_home_side(heading: float) -> int:
    """
    Gets the side of the robot facing your own alliance.
    """
    front_alliance = Alliance.RED if (90 < heading < 270) else Alliance.BLUE
    home_side = Arm_Goal.SIDE_BACK if front_alliance != robot_status.get_alliance() else Arm_Goal.SIDE_FRONT
    return home_side
