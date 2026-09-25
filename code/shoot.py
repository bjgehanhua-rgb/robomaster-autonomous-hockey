#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


# ---------------------------------------------------------------------------
# Robot and Vicon topic configuration
# ---------------------------------------------------------------------------

ROBOT_ID = 9

# Puck colour used for the current test.
PUCK_COLOR = "green"

# Final distance from the robot rotation center to the puck, in metres.
# The robot final position is constrained to the line through the puck that is
# exactly 90 degrees to the puck-to-goal direction.
ROBOT_TO_PUCK_DISTANCE = 0.35

# Select which side of the puck the robot uses:
# +1 and -1 are the two opposite points on the 90-degree line.
SHOT_SIDE = 1.0

# Stop this many metres before the exact final point, along the same
# perpendicular line. Positive means stop farther away from the puck.
FINAL_STOP_EARLY_OFFSET = 0.05

# Optional correction along the puck-to-goal direction.
# Keep this at 0.0 to preserve an exact 90-degree robot-puck-goal angle.
SHOT_DIRECTION_OFFSET = 0.00

ROBOT_POSE_TOPIC = f"/vrpn_mocap/dji_robot_{ROBOT_ID}/pose"
PUCK_POSE_TOPIC = f"/vrpn_mocap/hockey_puck_{PUCK_COLOR}/pose"
GOAL_POSE_TOPIC = "/vrpn_mocap/hockey_goal_1/pose"


# ---------------------------------------------------------------------------
# Arm and 180-degree shooting action
# ---------------------------------------------------------------------------

ARM_SPEED = 0.08
ARM_PUBLISH_RATE = 20.0
LOWER_ARM_Z_SPEED = -ARM_SPEED

# Calibrate this so the raised stick just reaches the floor.
LOWER_STICK_TIME = 1.0

# First rotate 180 degrees while the stick is raised.
PRE_ROTATION_ANGLE = math.pi
PRE_ROTATION_SPEED = 0.55

# Then lower the stick and rotate back 180 degrees at high speed.
SHOT_ROTATION_ANGLE = math.pi
SHOT_ROTATION_SPEED = 1.60

# +1 = counterclockwise, -1 = clockwise.
# These must be opposite.
PRE_ROTATION_SIGN = 1.0
SHOT_ROTATION_SIGN = -1.0

STOP_SETTLE_TIME = 0.50
STICK_GROUND_SETTLE_TIME = 0.40

MAX_PRE_ROTATION_TIME = 8.0
MAX_SHOT_ROTATION_TIME = 10.0


class RotateShotController(Node):
    """
    Assumptions:
    - The robot has already grabbed and raised the hockey stick.
    - The stick points approximately along the robot's forward x-axis.
    - The ceiling-camera/Vicon system publishes robot, puck, and goal poses.
    - The robot shoots by rotating in place so the stick tip sweeps through
      the puck with tangential velocity directed toward the goal.

    State sequence:
        wait_for_poses
        -> calculate_shot
        -> go_to_pre_swing
        -> go_to_swing_center
        -> align_for_pre_rotation
        -> pre_rotate_180
        -> settle_after_pre_rotation
        -> lower_stick
        -> settle_stick_on_ground
        -> fast_shot_rotate_180
        -> done
    """

    def __init__(self):
        super().__init__(f"rotate_shot_controller_robot{ROBOT_ID}")

        self.robot_id = ROBOT_ID

        self.robot_pose = None
        self.puck_pose = None
        self.goal_pose = None

        # -------------------------------------------------------------------
        # Approximate-linearization navigation parameters
        # -------------------------------------------------------------------
        self.lookahead_l = 0.25
        self.kp_position = 0.55
        self.max_linear_speed = 0.28
        self.max_angular_speed = 0.75
        self.position_tolerance = 0.07

        # -------------------------------------------------------------------
        # Shot geometry parameters
        # -------------------------------------------------------------------

        # Distance from the robot's rotation center to the stick contact point.
        # Measure this physically and replace this initial estimate.
        self.STICK_RADIUS = 0.70

        # Small radial correction. Positive means placing the robot slightly
        # closer to the puck than the ideal stick radius.
        self.CONTACT_RADIUS_CORRECTION = 0.02

        # Extra distance outside the final swing-center point. The robot first
        # reaches this pre-swing point, then approaches the final point.
        self.PRE_SWING_CLEARANCE = 0.45

        self.PRE_ROTATION_ANGLE = PRE_ROTATION_ANGLE
        self.PRE_ROTATION_SPEED = PRE_ROTATION_SPEED
        self.SHOT_ROTATION_ANGLE = SHOT_ROTATION_ANGLE
        self.SHOT_ROTATION_SPEED = SHOT_ROTATION_SPEED
        self.PRE_ROTATION_SIGN = PRE_ROTATION_SIGN
        self.SHOT_ROTATION_SIGN = SHOT_ROTATION_SIGN

        if self.PRE_ROTATION_SIGN not in (-1.0, 1.0):
            raise ValueError("PRE_ROTATION_SIGN must be +1.0 or -1.0")
        if self.SHOT_ROTATION_SIGN != -self.PRE_ROTATION_SIGN:
            raise ValueError(
                "SHOT_ROTATION_SIGN must be opposite PRE_ROTATION_SIGN"
            )

        # Slow alignment parameters.
        self.heading_tolerance = math.radians(3.0)
        self.kp_heading = 1.5
        self.max_align_angular_speed = 0.55

        # Final low-speed approach to the swing-center point.
        self.max_final_approach_speed = 0.10
        self.min_final_approach_speed = 0.025
        self.final_speed_kp = 0.60
        self.final_heading_kp = 0.8
        self.max_final_heading_correction = 0.20
        self.final_position_tolerance = 0.04

        # Final-position geometry and robust stopping parameters.
        self.ROBOT_TO_PUCK_DISTANCE = ROBOT_TO_PUCK_DISTANCE
        self.SHOT_SIDE = SHOT_SIDE
        self.FINAL_STOP_EARLY_OFFSET = FINAL_STOP_EARLY_OFFSET
        self.SHOT_DIRECTION_OFFSET = SHOT_DIRECTION_OFFSET
        self.stop_line_margin = 0.02
        self.max_safe_lateral_error = 0.25

        self.LOWER_STICK_TIME = LOWER_STICK_TIME
        self.STOP_SETTLE_TIME = STOP_SETTLE_TIME
        self.STICK_GROUND_SETTLE_TIME = STICK_GROUND_SETTLE_TIME
        self.MAX_PRE_ROTATION_TIME = MAX_PRE_ROTATION_TIME
        self.MAX_SHOT_ROTATION_TIME = MAX_SHOT_ROTATION_TIME

        # -------------------------------------------------------------------
        # Runtime state
        # -------------------------------------------------------------------
        self.stage = "wait_for_poses"
        self.last_status = None

        self.pre_swing_target = None
        self.swing_center_target = None
        self.contact_heading = None
        self.approach_heading = None
        self.pre_rotation_start_heading = None

        self.stage_start_time = None
        self.rotation_start_time = None
        self.previous_rotation_theta = None
        self.accumulated_rotation_angle = 0.0

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            f"/robot{self.robot_id}/cmd_vel",
            10,
        )

        self.arm_pub = self.create_publisher(
            Vector3,
            f"/robot{self.robot_id}/cmd_arm",
            10,
        )

        self.create_subscription(
            PoseStamped,
            ROBOT_POSE_TOPIC,
            self.robot_pose_callback,
            best_effort_qos,
        )

        self.create_subscription(
            PoseStamped,
            PUCK_POSE_TOPIC,
            self.puck_pose_callback,
            best_effort_qos,
        )

        self.create_subscription(
            PoseStamped,
            GOAL_POSE_TOPIC,
            self.goal_pose_callback,
            best_effort_qos,
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Rotate-shot controller created for robot {self.robot_id}."
        )
        self.get_logger().info(f"Robot pose topic: {ROBOT_POSE_TOPIC}")
        self.get_logger().info(f"Puck pose topic: {PUCK_POSE_TOPIC}")
        self.get_logger().info(f"Goal pose topic: {GOAL_POSE_TOPIC}")
        self.get_logger().info(
            f"Robot-to-puck distance: "
            f"{self.ROBOT_TO_PUCK_DISTANCE:.3f} m"
        )
        self.get_logger().info(
            f"Shot side: {self.SHOT_SIDE:+.0f}"
        )
        self.get_logger().info(
            f"Final stop early offset: "
            f"{self.FINAL_STOP_EARLY_OFFSET:.3f} m"
        )
        self.get_logger().info(
            f"Shot-direction offset: "
            f"{self.SHOT_DIRECTION_OFFSET:.3f} m"
        )

    # -----------------------------------------------------------------------
    # Pose callbacks and utilities
    # -----------------------------------------------------------------------

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def puck_pose_callback(self, msg):
        self.puck_pose = msg

    def goal_pose_callback(self, msg):
        self.goal_pose = msg

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(min(value, maximum), minimum)

    def robot_xytheta(self):
        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        theta = self.yaw_from_quaternion(
            self.robot_pose.pose.orientation
        )
        return x, y, theta

    def log_once(self, message):
        if message != self.last_status:
            self.get_logger().info(message)
            self.last_status = message

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def hard_stop_robot(self, repetitions=8, delay=0.02):
        for _ in range(repetitions):
            self.stop_robot()
            time.sleep(delay)

    def stop_arm(self):
        self.arm_pub.publish(Vector3())

    def move_arm_z(self, z_speed, duration):
        cmd = Vector3()
        cmd.z = float(z_speed)

        period = 1.0 / ARM_PUBLISH_RATE
        end_time = time.monotonic() + duration

        try:
            while rclpy.ok() and time.monotonic() < end_time:
                self.arm_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(period)
        finally:
            self.stop_arm()
            time.sleep(0.3)

    # -----------------------------------------------------------------------
    # Shot geometry
    # -----------------------------------------------------------------------

    def calculate_shot_geometry(self):
        puck_x = self.puck_pose.pose.position.x
        puck_y = self.puck_pose.pose.position.y

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        shot_dx = goal_x - puck_x
        shot_dy = goal_y - puck_y
        shot_length = math.hypot(shot_dx, shot_dy)

        if shot_length < 1e-6:
            self.get_logger().error(
                "Puck and goal positions are too close to define a shot direction."
            )
            return False

        # Unit vector pointing from puck toward goal.
        ux = shot_dx / shot_length
        uy = shot_dy / shot_length

        # Right-hand unit normal to the puck-to-goal direction.
        # This vector is exactly 90 degrees from (ux, uy).
        normal_x = uy
        normal_y = -ux

        if self.SHOT_SIDE not in (-1.0, 1.0):
            self.get_logger().error(
                "SHOT_SIDE must be either +1.0 or -1.0."
            )
            return False

        # Exact final robot-center position:
        #
        # robot_center = puck
        #              + side * radius * perpendicular_direction
        #              + optional_along-shot correction
        #
        # Keep SHOT_DIRECTION_OFFSET = 0.0 for an exact 90-degree angle.
        self.swing_center_target = (
            puck_x
            + self.SHOT_SIDE
            * self.ROBOT_TO_PUCK_DISTANCE
            * normal_x
            + self.SHOT_DIRECTION_OFFSET * ux,
            puck_y
            + self.SHOT_SIDE
            * self.ROBOT_TO_PUCK_DISTANCE
            * normal_y
            + self.SHOT_DIRECTION_OFFSET * uy,
        )

        # The pre-swing point lies farther outward on the same perpendicular
        # line, so the final approach is directly toward the puck side position.
        self.pre_swing_target = (
            puck_x
            + self.SHOT_SIDE
            * (
                self.ROBOT_TO_PUCK_DISTANCE
                + self.PRE_SWING_CLEARANCE
            )
            * normal_x
            + self.SHOT_DIRECTION_OFFSET * ux,
            puck_y
            + self.SHOT_SIDE
            * (
                self.ROBOT_TO_PUCK_DISTANCE
                + self.PRE_SWING_CLEARANCE
            )
            * normal_y
            + self.SHOT_DIRECTION_OFFSET * uy,
        )

        # Direction from the robot's final rotation center to the puck.
        radial_x = puck_x - self.swing_center_target[0]
        radial_y = puck_y - self.swing_center_target[1]

        self.contact_heading = math.atan2(radial_y, radial_x)

        # Before the raised-stick 180-degree wind-up, point the stick
        # directly from the robot rotation center toward the puck.
        self.pre_rotation_start_heading = self.contact_heading

        # During the final straight approach, point from the pre-swing point
        # toward the final swing-center point.
        approach_dx = (
            self.swing_center_target[0] - self.pre_swing_target[0]
        )
        approach_dy = (
            self.swing_center_target[1] - self.pre_swing_target[1]
        )
        self.approach_heading = math.atan2(
            approach_dy,
            approach_dx,
        )

        self.get_logger().info(
            "Shot geometry calculated: "
            f"puck=({puck_x:.3f}, {puck_y:.3f}), "
            f"goal=({goal_x:.3f}, {goal_y:.3f})"
        )
        self.get_logger().info(
            f"pre_swing=({self.pre_swing_target[0]:.3f}, "
            f"{self.pre_swing_target[1]:.3f}), "
            f"swing_center=({self.swing_center_target[0]:.3f}, "
            f"{self.swing_center_target[1]:.3f})"
        )
        self.get_logger().info(
            f"contact_heading="
            f"{math.degrees(self.contact_heading):.1f} deg, "
            f"pre_rotation=180.0 deg, shot_rotation=180.0 deg"
        )

        return True

    # -----------------------------------------------------------------------
    # Navigation and heading control
    # -----------------------------------------------------------------------

    def drive_to_point(self, target_x, target_y, tolerance):
        x, y, theta = self.robot_xytheta()

        px = x + self.lookahead_l * math.cos(theta)
        py = y + self.lookahead_l * math.sin(theta)

        ex = target_x - px
        ey = target_y - py
        distance = math.hypot(ex, ey)

        if distance < tolerance:
            self.stop_robot()
            return True

        pdot_x = self.kp_position * ex
        pdot_y = self.kp_position * ey

        v = (
            math.cos(theta) * pdot_x
            + math.sin(theta) * pdot_y
        )

        w = (
            -math.sin(theta) * pdot_x
            + math.cos(theta) * pdot_y
        ) / self.lookahead_l

        v = self.clamp(
            v,
            -self.max_linear_speed,
            self.max_linear_speed,
        )

        w = self.clamp(
            w,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        return False

    def align_to_heading(self, desired_heading):
        _, _, theta = self.robot_xytheta()

        heading_error = self.wrap_angle(
            desired_heading - theta
        )

        if abs(heading_error) < self.heading_tolerance:
            self.stop_robot()
            return True

        w = self.kp_heading * heading_error
        w = self.clamp(
            w,
            -self.max_align_angular_speed,
            self.max_align_angular_speed,
        )

        cmd = Twist()
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        return False

    def drive_straight_to_swing_center(self):
        """
        Drive toward the swing-center target and stop at a target line.

        The old version only stopped when the robot entered a small circle
        around the target. If the robot passed beside that circle, it kept
        moving and could push the puck.

        This version stops when the robot reaches a line located
        FINAL_STOP_EARLY_OFFSET metres before the calculated swing center.
        """
        x, y, theta = self.robot_xytheta()
        target_x, target_y = self.swing_center_target

        forward_x = math.cos(self.approach_heading)
        forward_y = math.sin(self.approach_heading)

        # Move the actual stopping point backward along the final-approach
        # direction, so a positive offset makes the robot stop earlier.
        stop_x = (
            target_x
            - self.FINAL_STOP_EARLY_OFFSET * forward_x
        )
        stop_y = (
            target_y
            - self.FINAL_STOP_EARLY_OFFSET * forward_y
        )

        error_x = stop_x - x
        error_y = stop_y - y

        # Positive while the stopping line is still in front of the robot.
        remaining_forward = (
            error_x * forward_x
            + error_y * forward_y
        )

        # Lateral error relative to the final straight approach.
        lateral_error = (
            -error_x * forward_y
            + error_y * forward_x
        )

        distance = math.hypot(error_x, error_y)

        # Main stop condition: the robot has reached or crossed the stop line.
        if remaining_forward <= self.stop_line_margin:
            self.stop_robot()
            self.get_logger().info(
                "Swing-center stopping line reached: "
                f"remaining_forward={remaining_forward:.3f} m, "
                f"lateral_error={lateral_error:.3f} m"
            )
            return True

        # Backup circular stop condition.
        if distance <= self.final_position_tolerance:
            self.stop_robot()
            return True

        # Safety condition: do not continue toward the puck when the path is
        # severely offset sideways.
        if abs(lateral_error) > self.max_safe_lateral_error:
            self.stop_robot()
            self.get_logger().error(
                "Final approach stopped because lateral error is unsafe: "
                f"{lateral_error:.3f} m"
            )
            self.stage = "error"
            return False

        heading_error = self.wrap_angle(
            self.approach_heading - theta
        )

        w = self.final_heading_kp * heading_error
        w = self.clamp(
            w,
            -self.max_final_heading_correction,
            self.max_final_heading_correction,
        )

        # Slow down as the robot approaches the stopping line.
        v = self.final_speed_kp * remaining_forward
        v = self.clamp(
            v,
            self.min_final_approach_speed,
            self.max_final_approach_speed,
        )

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

        return False

    # -----------------------------------------------------------------------
    # Measured in-place rotations
    # -----------------------------------------------------------------------

    def start_measured_rotation(self):
        _, _, theta = self.robot_xytheta()
        self.previous_rotation_theta = theta
        self.accumulated_rotation_angle = 0.0
        self.rotation_start_time = self.get_clock().now()

    def update_measured_rotation(
        self,
        direction_sign,
        target_angle,
        angular_speed,
        timeout,
        label,
    ):
        _, _, theta = self.robot_xytheta()

        delta_theta = self.wrap_angle(
            theta - self.previous_rotation_theta
        )
        self.previous_rotation_theta = theta

        signed_increment = direction_sign * delta_theta
        if signed_increment > 0.0:
            self.accumulated_rotation_angle += signed_increment

        elapsed = (
            self.get_clock().now() - self.rotation_start_time
        ).nanoseconds / 1e9

        if self.accumulated_rotation_angle >= target_angle:
            self.hard_stop_robot()
            self.get_logger().info(
                f"{label} reached "
                f"{math.degrees(self.accumulated_rotation_angle):.1f} deg."
            )
            return True

        if elapsed >= timeout:
            self.hard_stop_robot()
            self.get_logger().error(
                f"{label} timed out at "
                f"{math.degrees(self.accumulated_rotation_angle):.1f} deg."
            )
            self.stage = "error"
            return False

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = direction_sign * angular_speed
        self.cmd_pub.publish(cmd)
        return False

    # -----------------------------------------------------------------------
    # State machine
    # -----------------------------------------------------------------------

    def control_loop(self):
        if self.stage == "wait_for_poses":
            self.stop_robot()

            if self.robot_pose is None:
                self.log_once("Waiting for robot pose...")
                return

            if self.puck_pose is None:
                self.log_once("Waiting for puck pose...")
                return

            if self.goal_pose is None:
                self.log_once("Waiting for goal pose...")
                return

            self.last_status = None
            self.stage = "calculate_shot"

        if self.stage == "calculate_shot":
            self.stop_robot()

            if not self.calculate_shot_geometry():
                self.stage = "error"
                return

            self.stage = "go_to_pre_swing"
            self.last_status = None
            return

        if self.stage == "go_to_pre_swing":
            self.log_once("Stage: GO_TO_PRE_SWING")

            reached = self.drive_to_point(
                self.pre_swing_target[0],
                self.pre_swing_target[1],
                self.position_tolerance,
            )

            if reached:
                self.get_logger().info(
                    "Reached the pre-swing point."
                )
                self.stage = "align_for_final_approach"
                self.last_status = None
            return

        if self.stage == "align_for_final_approach":
            self.log_once(
                "Stage: ALIGN_FOR_FINAL_APPROACH"
            )

            aligned = self.align_to_heading(
                self.approach_heading
            )

            if aligned:
                self.get_logger().info(
                    "Aligned for final approach."
                )
                self.stage = "go_to_swing_center"
                self.last_status = None
            return

        if self.stage == "go_to_swing_center":
            self.log_once("Stage: GO_TO_SWING_CENTER")

            reached = self.drive_straight_to_swing_center()

            if reached:
                self.get_logger().info(
                    "Reached the final swing-center position."
                )
                self.stage = "align_for_pre_rotation"
                self.last_status = None
            return

        if self.stage == "align_for_pre_rotation":
            self.log_once("Stage: ALIGN_FOR_PRE_ROTATION")

            aligned = self.align_to_heading(
                self.pre_rotation_start_heading
            )

            if aligned:
                self.hard_stop_robot()
                self.get_logger().info(
                    "Raised stick aligned toward puck."
                )
                self.start_measured_rotation()
                self.stage = "pre_rotate_180"
                self.last_status = None
            return

        if self.stage == "pre_rotate_180":
            self.log_once("Stage: PRE_ROTATE_180")

            completed = self.update_measured_rotation(
                direction_sign=self.PRE_ROTATION_SIGN,
                target_angle=self.PRE_ROTATION_ANGLE,
                angular_speed=self.PRE_ROTATION_SPEED,
                timeout=self.MAX_PRE_ROTATION_TIME,
                label="Raised-stick pre-rotation",
            )

            if self.stage == "error":
                return

            if completed:
                self.stage_start_time = self.get_clock().now()
                self.stage = "settle_after_pre_rotation"
                self.last_status = None
            return

        if self.stage == "settle_after_pre_rotation":
            self.log_once("Stage: SETTLE_AFTER_PRE_ROTATION")
            self.stop_robot()

            elapsed = (
                self.get_clock().now() - self.stage_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.STOP_SETTLE_TIME:
                self.stage = "lower_stick"
                self.last_status = None
            return

        if self.stage == "lower_stick":
            self.log_once("Stage: LOWER_STICK")
            self.hard_stop_robot()

            self.get_logger().info(
                "Lowering arm so the stick touches the floor."
            )
            self.move_arm_z(
                LOWER_ARM_Z_SPEED,
                self.LOWER_STICK_TIME,
            )

            self.stage_start_time = self.get_clock().now()
            self.stage = "settle_stick_on_ground"
            self.last_status = None
            return

        if self.stage == "settle_stick_on_ground":
            self.log_once("Stage: SETTLE_STICK_ON_GROUND")
            self.stop_robot()
            self.stop_arm()

            elapsed = (
                self.get_clock().now() - self.stage_start_time
            ).nanoseconds / 1e9

            if elapsed >= self.STICK_GROUND_SETTLE_TIME:
                self.start_measured_rotation()
                self.get_logger().info(
                    "Starting fast 180-degree shot rotation."
                )
                self.stage = "fast_shot_rotate_180"
                self.last_status = None
            return

        if self.stage == "fast_shot_rotate_180":
            self.log_once("Stage: FAST_SHOT_ROTATE_180")

            completed = self.update_measured_rotation(
                direction_sign=self.SHOT_ROTATION_SIGN,
                target_angle=self.SHOT_ROTATION_ANGLE,
                angular_speed=self.SHOT_ROTATION_SPEED,
                timeout=self.MAX_SHOT_ROTATION_TIME,
                label="Shot rotation",
            )

            if self.stage == "error":
                return

            if completed:
                self.stage = "done"
                self.last_status = None
            return

        if self.stage == "done":
            self.log_once("Stage: DONE")
            self.stop_robot()
            self.stop_arm()
            return

        if self.stage == "error":
            self.log_once("Stage: ERROR")
            self.stop_robot()
            self.stop_arm()


def main(args=None):
    rclpy.init(args=args)
    node = RotateShotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warning(
            "Rotate-shot controller interrupted by user."
        )
    finally:
        node.hard_stop_robot()
        node.stop_arm()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
