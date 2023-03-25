"""
Class definition of the HMI agent node.
"""

from dataclasses import dataclass
import rospy

import numpy as np

from hmi_agent_node.arm_control import get_away_side, get_home_side, INTAKE_GOALS
from hmi_agent_node.drive import limit_drive_power

from actions_node.ActionRunner import ActionRunner
from actions_node.game_specific_actions.Subsystem import Subsystem
from ck_ros_msgs_node.msg import HMI_Signals, Intake_Control, Led_Control, Arm_Goal, Arm_Status
from nav_msgs.msg import Odometry
from ck_ros_base_msgs_node.msg import Joystick_Status

from ck_utilities_py_node.ckmath import *
from ck_utilities_py_node.geometry import *
from ck_utilities_py_node.joystick import Joystick
from ck_utilities_py_node.pid_controller import PIDController
from ck_utilities_py_node.rosparam_helper import load_parameter_class
from frc_robot_utilities_py_node.frc_robot_utilities_py import *
from frc_robot_utilities_py_node.RobotStatusHelperPy import BufferedROSMsgHandlerPy

# import cProfile

NUM_LEDS = 50
solid_purple = Led_Control(Led_Control.SET_LED, 0, 57, 3, 87, 0, 1, 0, NUM_LEDS, "")
solid_yellow = Led_Control(Led_Control.SET_LED, 0, 255, 255, 0, 0, 1, 0, NUM_LEDS, "")
strobe_purple = Led_Control(Led_Control.ANIMATE, Led_Control.STROBE, 57, 3, 87, 0, 0, 0.25, NUM_LEDS, "")
strobe_red = Led_Control(Led_Control.ANIMATE, Led_Control.STROBE, 96, 0, 0, 0, 0, 0.10, NUM_LEDS, "")
larson_purple = Led_Control(Led_Control.ANIMATE, Led_Control.LARSON, 57, 3, 87, 0, 0, 0.20, NUM_LEDS, "")
fire_animation = Led_Control(Led_Control.ANIMATE, Led_Control.FIRE, 0, 0, 0, 0, 0.5, 0.5, NUM_LEDS, "")
rainbow = Led_Control(Led_Control.ANIMATE, Led_Control.RAINBOW, 0, 0, 0, 0, 0.5, 0.60, NUM_LEDS, "")
twinkle_purple = Led_Control(Led_Control.ANIMATE, Led_Control.TWINKLE_OFF, 57, 3, 87, 0, 0, 0.25, NUM_LEDS, "")

@dataclass
class DriverParams:
    """
    Driver parameters. Must match the configuration YAML loaded.
    """
    drive_fwd_back_axis_id: int = -1
    drive_fwd_back_axis_inverted: bool = False

    drive_left_right_axis_id: int = -1
    drive_left_right_axis_inverted: bool = False

    drive_z_axis_id: int = -1
    drive_z_axis_inverted: bool = False

    drive_axis_deadband: float = 0.05
    drive_z_axis_deadband: float = 0.05
    drive_z_axis_min_value_after_deadband : float = 0

    reset_odometry_button_id: int = -1
    robot_xmode_driver_id: int = -1
    robot_orient_button_id: int = -1
    field_centric_button_id: int = -1

@dataclass
class OperatorSplitParams:
    """
    Operator parameters for the split joystick/button box. Must match the configuration YAML.
    """
    # Button Box
    home_button_id: int = -1


    high_cone_button_id: int = -1
    mid_cone_button_id: int = -1
    shelf_cone_button_id: int = -1
    low_cone_button_id: int = -1
    pickup_cone_button_id: int = -1
    pickup_dead_cone_button_id: int = -1
    pre_dead_cone_button_id: int = -1

    high_cube_button_id: int = -1
    mid_cube_button_id: int = -1
    pickup_cube_button_id: int = -1
    shelf_cube_button_id: int = -1
    low_cube_button_id: int = -1

    led_toggle_id: int = -1

    # Joystick
    intake_in_button_id: int = -1
    intake_out_button_id: int = -1

    pre_score_position_button_id: int = -1

    robot_xmode_id: int = -1

    toggle_reverse_side_button_id: int = -1


class HmiAgentNode():
    """
    The HMI agent node.
    """

    def __init__(self) -> None:
        register_for_robot_updates()

        self.action_runner = ActionRunner()

        self.driver_joystick = Joystick(0)
        self.operator_button_box = Joystick(1)
        self.operator_joystick = Joystick(2)

        self.driver_params = DriverParams()
        self.operator_params = OperatorSplitParams()

        load_parameter_class(self.driver_params)
        load_parameter_class(self.operator_params)

        self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        self.led_control_message = solid_purple
        self.led_timer = 0
        self.party_time = False

        self.current_color = solid_yellow

        self.heading = 0.0

        self.pinch_active = True

        self.current_goal = Arm_Goal.HOME

        self.hmi_publisher = rospy.Publisher(name="/HMISignals", data_class=HMI_Signals, queue_size=10, tcp_nodelay=True)
        self.intake_publisher = rospy.Publisher(name="/IntakeControl", data_class=Intake_Control, queue_size=10, tcp_nodelay=True)
        self.led_control_publisher = rospy.Publisher(name="/LedControl", data_class=Led_Control, queue_size=10, tcp_nodelay=True)

        self.arm_goal_publisher = rospy.Publisher(name="/ArmGoal", data_class=Arm_Goal, queue_size=10, tcp_nodelay=True)
        self.arm_goal = Arm_Goal()
        self.arm_goal.goal = Arm_Goal.HOME
        self.arm_goal.wrist_goal = Arm_Goal.WRIST_ZERO

        self.odometry_subscriber = BufferedROSMsgHandlerPy(Odometry)
        self.odometry_subscriber.register_for_updates("odometry/filtered")

        self.arm_subscriber = BufferedROSMsgHandlerPy(Arm_Status)
        self.arm_subscriber.register_for_updates("/ArmStatus")

        self.orientation_helper = PIDController(kP=0.0047, kD=0.001, filter_r=0.6)

        self.reverse_intake = False
        self.intake_side = None

        rospy.Subscriber(name="/JoystickStatus", data_class=Joystick_Status, callback=self.joystick_callback, queue_size=1, tcp_nodelay=True)
        # profiler = cProfile.Profile()
        # profiler.enable()
        rospy.spin()
        # profiler.disable()
        # profiler.dump_stats("/mnt/working/hmi_agent_node.stats")


    def joystick_callback(self, message: Joystick_Status):
        """
        Joystick callback function. This runs everytime a new joystick status message is received.
        """

        #DO NOT REMOVE THIS CHECK!!!!!!!!!! DID YOU LEARN NOTHING FROM 2022?!
        if robot_status.get_mode() != RobotMode.TELEOP:
            self.process_leds()
            return

        Joystick.update(message)

        hmi_update_message = HMI_Signals()
        hmi_update_message.drivetrain_brake = True


        #######################################################################
        ###                         DRIVER CONTROLS                         ###
        #######################################################################
        invert_axis_fwd_back = -1 if self.driver_params.drive_fwd_back_axis_inverted else 1
        invert_axis_left_right = -1 if self.driver_params.drive_left_right_axis_inverted else 1

        fwd_back_value = self.driver_joystick.getFilteredAxis(self.driver_params.drive_fwd_back_axis_id, self.driver_params.drive_axis_deadband)
        hmi_update_message.drivetrain_fwd_back = invert_axis_fwd_back * fwd_back_value

        left_right_value = self.driver_joystick.getFilteredAxis(self.driver_params.drive_left_right_axis_id, self.driver_params.drive_axis_deadband)
        hmi_update_message.drivetrain_left_right = invert_axis_left_right * left_right_value

        x = hmi_update_message.drivetrain_fwd_back
        y = hmi_update_message.drivetrain_left_right

        invert_axis_z = -1 if self.driver_params.drive_z_axis_inverted else 1
        z = invert_axis_z * self.driver_joystick.getFilteredAxis(self.driver_params.drive_z_axis_id, self.driver_params.drive_z_axis_deadband, self.driver_params.drive_z_axis_min_value_after_deadband)

        r = hypotenuse(x, y)
        theta = polar_angle_rad(x, y)

        z = np.sign(z) * pow(z, 2)
        active_theta = theta
        if r > self.driver_params.drive_axis_deadband:
            active_theta = theta

        hmi_update_message.drivetrain_swerve_direction = active_theta

        # Scale the drive power based on current arm position.
        arm_status_message = self.arm_subscriber.get()
        limited_forward_velocity, limited_angular_rotation = limit_drive_power(arm_status_message, r, z)

        hmi_update_message.drivetrain_swerve_percent_fwd_vel = limited_forward_velocity
        hmi_update_message.drivetrain_swerve_percent_angular_rot = limited_angular_rotation

        # Swap between field centric and robot oriented drive.
        if self.driver_joystick.getButton(self.driver_params.robot_orient_button_id):
            self.drivetrain_orientation = HMI_Signals.ROBOT_ORIENTED
        elif self.driver_joystick.getButton(self.driver_params.field_centric_button_id):
            self.drivetrain_orientation = HMI_Signals.FIELD_CENTRIC

        hmi_update_message.drivetrain_orientation = self.drivetrain_orientation

        hmi_update_message.drivetrain_xmode = False
        if self.driver_joystick.getButton(self.driver_params.robot_xmode_driver_id) or self.operator_joystick.getButton(self.operator_params.robot_xmode_id):
            hmi_update_message.drivetrain_xmode = True

        if self.driver_joystick.getRisingEdgeButton(self.driver_params.reset_odometry_button_id):
            reset_robot_pose(robot_status.get_alliance())

        #######################################################################
        ###                        OPERATOR CONTROLS                        ###
        #######################################################################
        self.process_leds()

        # Determine the alliance station the robot is facing.
        if self.odometry_subscriber.get() is not None:
            odometry_message = self.odometry_subscriber.get()
            rotation = Rotation(odometry_message.pose.pose.orientation)
            yaw = rotation.yaw
            yaw = normalize_to_2_pi(yaw)
            self.heading = np.degrees(yaw)

        hmi_update_message.drivetrain_heading = self.heading


        ################################################################################
        ###                         CONTROL MAPPINGS                                 ###
        ################################################################################
        if self.operator_button_box.getRisingEdgeButton(self.operator_params.home_button_id):
            self.current_goal = Arm_Goal.HOME

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.shelf_cone_button_id):
            self.current_goal = Arm_Goal.SHELF_CONE_PICKUP
    
        if self.operator_button_box.getRisingEdgeButton(self.operator_params.shelf_cube_button_id):
            self.current_goal = Arm_Goal.SHELF_CUBE_PICKUP

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cone_button_id):
            self.current_goal = Arm_Goal.HIGH_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cone_button_id):
            self.current_goal = Arm_Goal.MID_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cone_button_id):
            self.current_goal = Arm_Goal.GROUND_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_dead_cone_button_id):
            self.current_goal = Arm_Goal.GROUND_DEAD_CONE

        if self.operator_joystick.getRisingEdgeButton(self.operator_params.pre_score_position_button_id):
            self.current_goal = Arm_Goal.PRE_SCORE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.high_cube_button_id):
            self.current_goal = Arm_Goal.HIGH_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.mid_cube_button_id):
            self.current_goal = Arm_Goal.MID_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pickup_cube_button_id):
            self.current_goal = Arm_Goal.GROUND_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.low_cone_button_id):
            self.current_goal = Arm_Goal.LOW_CONE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.low_cube_button_id):
            self.current_goal = Arm_Goal.LOW_CUBE

        if self.operator_button_box.getRisingEdgeButton(self.operator_params.pre_dead_cone_button_id):
            self.current_goal = Arm_Goal.PRE_DEAD_CONE

        # If the new arm goal is not an intake goal, reset the intake parameters.
        if self.current_goal != self.arm_goal.goal and self.current_goal not in INTAKE_GOALS:
            self.intake_side = None
            self.reverse_intake = False

        self.arm_goal.goal = self.current_goal

        if self.operator_joystick.getRisingEdgeButton(self.operator_params.toggle_reverse_side_button_id):
            self.reverse_intake = not self.reverse_intake


        # Arm should point away for shelf pick-up.
        if self.arm_goal.goal in (Arm_Goal.SHELF_PICKUP, Arm_Goal.GROUND_CONE, Arm_Goal.GROUND_CUBE, Arm_Goal.GROUND_DEAD_CONE, Arm_Goal.PRE_DEAD_CONE):
            self.arm_goal.goal_side = get_away_side(self.heading)
        else:
            self.arm_goal.goal_side = get_home_side(self.heading)

        # Lock the intake positions to their original side.
        if self.arm_goal.goal in (Arm_Goal.GROUND_CONE, Arm_Goal.GROUND_CUBE, Arm_Goal.GROUND_DEAD_CONE, Arm_Goal.PRE_DEAD_CONE):
            if self.intake_side is None:
                if self.reverse_intake:
                    self.intake_side = Arm_Goal.SIDE_FRONT if self.arm_goal.goal_side == Arm_Goal.SIDE_BACK else Arm_Goal.SIDE_BACK
                else:
                    self.intake_side = Arm_Goal.SIDE_BACK if self.arm_goal.goal_side == Arm_Goal.SIDE_BACK else Arm_Goal.SIDE_FRONT

            self.arm_goal.goal_side = self.intake_side

        # Control the intake rollers.
        if self.operator_joystick.getButton(self.operator_params.intake_in_button_id):
            self.arm_goal.intake_goal = Arm_Goal.INTAKE_CONE
            self.arm_goal.speed = 0.70
        elif self.operator_joystick.getButton(self.operator_params.intake_out_button_id):
            self.arm_goal.intake_goal = Arm_Goal.INTAKE_CUBE
            self.arm_goal.speed = 0.35
        else:
            self.arm_goal.intake_goal = Arm_Goal.INTAKE_OFF

        self.arm_goal_publisher.publish(self.arm_goal)
        self.hmi_publisher.publish(hmi_update_message)

        self.action_runner.loop(robot_status.get_mode())

    def process_leds(self):
        """
        Handles all the LED changes.
        """
        if not robot_status.is_connected():
            self.led_control_message = strobe_red
        elif robot_status.get_mode() != RobotMode.TELEOP:
            self.led_control_message = larson_purple
        else:
            if self.operator_joystick.getRisingEdgeButton(self.operator_params.led_toggle_id):
                self.current_color = solid_yellow if self.current_color == solid_purple else solid_purple
            self.led_control_message = self.current_color

        self.led_control_publisher.publish(self.led_control_message)
