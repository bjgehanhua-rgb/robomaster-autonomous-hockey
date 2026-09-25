#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Set this to True when running with turtlesim.
# In simulation, gripper commands are treated as successful so navigation can run.
SIMULATION_MODE = True

try:
    from robomaster_msgs.action import GripperControl
    ROBOMASTER_MSGS_AVAILABLE = True
except ImportError:
    GripperControl = None
    ROBOMASTER_MSGS_AVAILABLE = False


# ---------------------------------------------------------------------------
# Robot and gripper configuration
# ---------------------------------------------------------------------------

ROBOT_ID = 6

# Select which hockey stick to use:
# 1 -> /vrpn_mocap/hockey_sticks_1/pose
# 2 -> /vrpn_mocap/hockey_sticks_2/pose
# 3 -> /vrpn_mocap/hockey_sticks_3/pose
# 4 -> /vrpn_mocap/hockey_sticks_4/pose
STICK_ID = 1

# Final distance from the selected base edge, in metres.
FINAL_EDGE_CLEARANCE = 0.30

# Lateral path correction, in metres.
# Positive: shift to the robot's right while facing the base.
# Negative: shift to the robot's left while facing the base.
LATERAL_OFFSET = 0.0

GRIPPER_OPEN = 1
GRIPPER_CLOSE = 2
GRIPPER_POWER = 0.5

ARM_SPEED = 0.08
ARM_PUBLISH_RATE = 20.0

RAISE_Z_SPEED = ARM_SPEED
LOWER_Z_SPEED = -ARM_SPEED

PREPARE_RAISE_TIME = 1.0

# Diagonal grab motion parameters.
# Positive x is assumed to move the arm forward; negative z moves it down.
# If positive x moves backward on the real robot, reverse the sign.
GRAB_FORWARD_SPEED = 0.04
GRAB_DOWN_SPEED = -ARM_SPEED
GRAB_DIAGONAL_TIME = 1.0

# Return motion after closing the gripper.
GRAB_RETURN_X_SPEED = -GRAB_FORWARD_SPEED
GRAB_RETURN_Z_SPEED = ARM_SPEED
GRAB_RETURN_TIME = GRAB_DIAGONAL_TIME


class NavigateAndGrabStick(Node):
    """
    Complete sequence:

    1. Open the gripper.
    2. Raise the arm.
    3. Navigate to a pre-approach point outside the selected three-marker edge.
    4. Align perpendicular to the base.
    5. Drive straight toward the final stopping line.
    6. Stop once the robot reaches or passes that stopping line.
    7. Move the arm diagonally forward and downward.
    8. Close the gripper.
    9. Move the arm diagonally backward and upward with the stick.
    """

    def __init__(self):
        super().__init__(
            f"navigate_and_grab_stick_robot{ROBOT_ID}_stick{STICK_ID}"
        )

        if STICK_ID not in (1, 2, 3, 4):
            raise ValueError("STICK_ID must be 1, 2, 3, 4")

        self.robot_id = ROBOT_ID
        self.stick_id = STICK_ID

        self.robot_pose = None
        self.base_pose = None

        # -------------------------------------------------------------------
        # Stick-base geometry calibration
        # -------------------------------------------------------------------

        # The selected hockey-stick Vicon topic is chosen from stick_id.
        self.stick_pose_topic = (
            f"/vrpn_mocap/hockey_sticks_{self.stick_id}/pose"
        )

        # Which local base axis points toward the three-marker edge.
        self.THREE_MARKER_EDGE_AXIS = "x"
        self.THREE_MARKER_EDGE_SIGN = 1.0

        # Approximate half-length from the Vicon base origin to the edge.
        self.BASE_HALF_LENGTH = 0.15

        # First stop this far outside the selected edge.
        self.PRE_APPROACH_CLEARANCE = 1.00

        # Final clearance from the selected edge.
        # This parameter moves the final stopping line forward/backward.
        self.FINAL_EDGE_CLEARANCE = float(FINAL_EDGE_CLEARANCE)

        # Lateral target correction in metres.
        #
        # Positive value: shifts the target to the robot's right when the
        # robot is facing the base.
        # Negative value: shifts the target to the robot's left.
        #
        # Example:
        #   +0.05 means 5 cm right.
        #   -0.05 means 5 cm left.
        self.FINAL_LATERAL_OFFSET = float(LATERAL_OFFSET)

        # Apply the same lateral correction to the pre-approach point so the
        # final straight segment stays parallel to the base normal.
        self.PRE_APPROACH_LATERAL_OFFSET = float(LATERAL_OFFSET)

        # -------------------------------------------------------------------
        # Navigation parameters
        # -------------------------------------------------------------------

        self.lookahead_l = 0.05
        self.kp_position = 0.55
        self.max_linear_speed = 0.30
        self.max_angular_speed = 0.75

        self.pre_approach_tolerance = 0.10
        self.heading_tolerance = math.radians(4.0)

        self.kp_heading = 1.5
        self.max_align_angular_speed = 0.60

        # Final straight approach.
        self.max_final_approach_speed = 0.12
        self.min_final_approach_speed = 0.025
        self.final_speed_kp = 0.60

        self.final_heading_kp = 0.8
        self.max_final_heading_correction = 0.20

        # Stop based on the target line, not only a small circular target.
        self.stop_line_margin = 0.025

        # Optional backup Euclidean tolerance.
        self.final_position_tolerance = 0.05

        # If lateral error becomes very large during final approach, stop
        # rather than continuing toward the base.
        self.max_safe_lateral_error = 0.25

        self.stage = "waiting_for_preparation"

        self.pre_target = None
        self.final_target = None
        self.approach_heading = None
        self.last_status = None

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

        if SIMULATION_MODE:
            self.gripper_client = None
            self.get_logger().info(
                "SIMULATION_MODE enabled: gripper commands will return success."
            )
        else:
            if not ROBOMASTER_MSGS_AVAILABLE:
                raise ImportError(
                    "robomaster_msgs is required when SIMULATION_MODE is False."
                )

            self.gripper_client = ActionClient(
                self,
                GripperControl,
                f"/robot{self.robot_id}/gripper",
            )

        self.create_subscription(
            PoseStamped,
            f"/vrpn_mocap/dji_robot_{self.robot_id}/pose",
            self.robot_pose_callback,
            best_effort_qos,
        )

        self.create_subscription(
            PoseStamped,
            self.stick_pose_topic,
            self.base_pose_callback,
            best_effort_qos,
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"Controller created for robot {self.robot_id}, stick {self.stick_id}."
        )
        self.get_logger().info(
            f"Stick topic: {self.stick_pose_topic}"
        )
        self.get_logger().info(
            f"Final edge clearance: {self.FINAL_EDGE_CLEARANCE:.3f} m"
        )
        self.get_logger().info(
            f"Lateral offset: {self.FINAL_LATERAL_OFFSET:.3f} m"
        )

    # -----------------------------------------------------------------------
    # Pose utilities
    # -----------------------------------------------------------------------

    def robot_pose_callback(self, msg):
        self.robot_pose = msg

    def base_pose_callback(self, msg):
        self.base_pose = msg

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

    # -----------------------------------------------------------------------
    # Base, arm, and gripper commands
    # -----------------------------------------------------------------------

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def hard_stop_robot(self, repetitions=8, delay=0.02):
        """
        Publish several zero-velocity commands to make the stop command robust.
        """
        for _ in range(repetitions):
            self.stop_robot()
            time.sleep(delay)

    def stop_arm(self):
        self.arm_pub.publish(Vector3())

    def move_arm_xyz(self, x_speed, y_speed, z_speed, duration):
        cmd = Vector3()
        cmd.x = float(x_speed)
        cmd.y = float(y_speed)
        cmd.z = float(z_speed)

        period = 1.0 / ARM_PUBLISH_RATE
        end_time = time.monotonic() + duration

        try:
            while rclpy.ok() and time.monotonic() < end_time:
                self.arm_pub.publish(cmd)
                time.sleep(period)
        finally:
            self.stop_arm()
            time.sleep(0.3)

    def move_arm_z(self, z_speed, duration):
        self.move_arm_xyz(
            x_speed=0.0,
            y_speed=0.0,
            z_speed=z_speed,
            duration=duration,
        )

    def set_gripper(self, target_state):
        if SIMULATION_MODE:
            if target_state == GRIPPER_OPEN:
                action_name = "OPEN"
            elif target_state == GRIPPER_CLOSE:
                action_name = "CLOSE"
            else:
                action_name = str(target_state)

            self.get_logger().info(
                f"SIMULATION: gripper {action_name} treated as successful."
            )
            return True

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Gripper action server unavailable: "
                f"/robot{self.robot_id}/gripper"
            )
            return False

        goal = GripperControl.Goal()
        goal.target_state = int(target_state)
        goal.power = float(GRIPPER_POWER)

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done() or send_future.result() is None:
            self.get_logger().error(
                "No response from gripper action server."
            )
            return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "Gripper command was rejected."
            )
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=5.0,
        )

        return True

    # -----------------------------------------------------------------------
    # Preparation and grab sequences
    # -----------------------------------------------------------------------

    def prepare_arm_for_navigation(self):
        self.hard_stop_robot()
        self.stop_arm()

        self.get_logger().info(
            "Preparation step 1: opening gripper."
        )

        if not self.set_gripper(GRIPPER_OPEN):
            return False

        time.sleep(0.5)

        self.get_logger().info(
            "Preparation step 2: raising arm."
        )

        self.move_arm_z(
            RAISE_Z_SPEED,
            PREPARE_RAISE_TIME,
        )

        self.stop_arm()

        self.get_logger().info(
            "Arm raised and gripper left open."
        )
        return True

    def grab_stick(self):
        self.hard_stop_robot()
        self.stop_arm()

        self.get_logger().info(
            "Grab step 1: confirming gripper is open."
        )

        if not self.set_gripper(GRIPPER_OPEN):
            return False

        time.sleep(0.3)

        self.get_logger().info(
            "Grab step 2: moving arm diagonally forward and downward."
        )

        self.move_arm_xyz(
            x_speed=GRAB_FORWARD_SPEED,
            y_speed=0.0,
            z_speed=GRAB_DOWN_SPEED,
            duration=GRAB_DIAGONAL_TIME,
        )

        self.get_logger().info(
            "Grab step 3: closing gripper."
        )

        if not self.set_gripper(GRIPPER_CLOSE):
            return False

        time.sleep(0.8)

        self.get_logger().info(
            "Grab step 4: moving arm diagonally backward and upward "
            "with the stick."
        )

        self.move_arm_xyz(
            x_speed=GRAB_RETURN_X_SPEED,
            y_speed=0.0,
            z_speed=GRAB_RETURN_Z_SPEED,
            duration=GRAB_RETURN_TIME,
        )

        self.stop_arm()
        self.get_logger().info(
            "Navigation and diagonal grab sequence completed."
        )
        return True

    # -----------------------------------------------------------------------
    # Target calculation
    # -----------------------------------------------------------------------

    def get_three_marker_edge_direction(self):
        base_yaw = self.yaw_from_quaternion(
            self.base_pose.pose.orientation
        )

        if self.THREE_MARKER_EDGE_AXIS == "x":
            dx = math.cos(base_yaw)
            dy = math.sin(base_yaw)
        elif self.THREE_MARKER_EDGE_AXIS == "y":
            dx = -math.sin(base_yaw)
            dy = math.cos(base_yaw)
        else:
            raise ValueError(
                'THREE_MARKER_EDGE_AXIS must be "x" or "y".'
            )

        dx *= self.THREE_MARKER_EDGE_SIGN
        dy *= self.THREE_MARKER_EDGE_SIGN
        return dx, dy

    def calculate_targets(self):
        base_x = self.base_pose.pose.position.x
        base_y = self.base_pose.pose.position.y

        # Outward normal from the base toward the robot approach side.
        dx, dy = self.get_three_marker_edge_direction()

        # Robot faces in the opposite direction during final approach.
        self.approach_heading = math.atan2(-dy, -dx)

        # Unit vector toward the robot's right when facing the base.
        right_x = math.sin(self.approach_heading)
        right_y = -math.cos(self.approach_heading)

        pre_distance = (
            self.BASE_HALF_LENGTH
            + self.PRE_APPROACH_CLEARANCE
        )

        final_distance = (
            self.BASE_HALF_LENGTH
            + self.FINAL_EDGE_CLEARANCE
        )

        self.pre_target = (
            base_x
            + pre_distance * dx
            + self.PRE_APPROACH_LATERAL_OFFSET * right_x,
            base_y
            + pre_distance * dy
            + self.PRE_APPROACH_LATERAL_OFFSET * right_y,
        )

        self.final_target = (
            base_x
            + final_distance * dx
            + self.FINAL_LATERAL_OFFSET * right_x,
            base_y
            + final_distance * dy
            + self.FINAL_LATERAL_OFFSET * right_y,
        )

        self.get_logger().info(
            "Targets calculated: "
            f"pre=({self.pre_target[0]:.3f}, "
            f"{self.pre_target[1]:.3f}), "
            f"final=({self.final_target[0]:.3f}, "
            f"{self.final_target[1]:.3f}), "
            f"heading={math.degrees(self.approach_heading):.1f} deg"
        )

    # -----------------------------------------------------------------------
    # Navigation controllers
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

    def drive_straight_to_final_target(self):
        """
        Final approach with robust stopping.

        The previous version only stopped inside a small circle around the
        target. If the robot passed beside that circle, it continued forever.

        This version projects the target error onto the approach direction.
        Once the robot reaches or passes the final stopping line, it stops
        regardless of small lateral error.
        """
        x, y, theta = self.robot_xytheta()
        final_x, final_y = self.final_target

        error_x = final_x - x
        error_y = final_y - y

        forward_x = math.cos(self.approach_heading)
        forward_y = math.sin(self.approach_heading)

        # Positive while the target line is still in front of the robot.
        remaining_forward = (
            error_x * forward_x
            + error_y * forward_y
        )

        # Positive means target is to the robot's left of the approach line.
        lateral_error = (
            -error_x * forward_y
            + error_y * forward_x
        )

        distance = math.hypot(error_x, error_y)

        # Primary stop condition: target stopping line reached or crossed.
        if remaining_forward <= self.stop_line_margin:
            self.hard_stop_robot()
            self.get_logger().info(
                "Final stopping line reached. "
                f"remaining_forward={remaining_forward:.3f} m, "
                f"lateral_error={lateral_error:.3f} m"
            )
            return True

        # Backup stop condition.
        if distance <= self.final_position_tolerance:
            self.hard_stop_robot()
            self.get_logger().info(
                "Final target tolerance reached."
            )
            return True

        # Safety stop: do not keep driving toward the base if the final path
        # has become severely laterally misaligned.
        if abs(lateral_error) > self.max_safe_lateral_error:
            self.hard_stop_robot()
            self.get_logger().error(
                "Final approach stopped because lateral error became unsafe: "
                f"{lateral_error:.3f} m"
            )
            self.stage = "failed"
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

        # Slow down continuously as the robot approaches the stopping line.
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
    # State machine
    # -----------------------------------------------------------------------

    def control_loop(self):
        if self.stage == "waiting_for_preparation":
            self.stop_robot()
            return

        if self.stage in ("navigation_done", "done", "failed"):
            self.stop_robot()
            return

        if self.robot_pose is None:
            self.log_once("Waiting for robot pose...")
            return

        if self.base_pose is None:
            self.log_once(
                f"Waiting for stick {self.stick_id} pose..."
            )
            return

        if self.pre_target is None:
            self.calculate_targets()

        if self.stage == "go_to_pre_approach":
            self.log_once("Stage: GO_TO_PRE_APPROACH")

            reached = self.drive_to_point(
                self.pre_target[0],
                self.pre_target[1],
                self.pre_approach_tolerance,
            )

            if reached:
                self.get_logger().info(
                    "Reached the pre-approach point."
                )
                self.stage = "align_to_base"
                self.last_status = None

        elif self.stage == "align_to_base":
            self.log_once("Stage: ALIGN_TO_BASE")

            aligned = self.align_to_heading(
                self.approach_heading
            )

            if aligned:
                self.get_logger().info(
                    "Robot aligned perpendicular to the base."
                )
                self.stage = "straight_to_base"
                self.last_status = None

        elif self.stage == "straight_to_base":
            self.log_once("Stage: STRAIGHT_TO_BASE")

            reached = self.drive_straight_to_final_target()

            if self.stage == "failed":
                return

            if reached:
                self.hard_stop_robot()
                self.get_logger().info(
                    "Reached the final grab position."
                )
                self.stage = "navigation_done"
                self.last_status = None


def main(args=None):
    rclpy.init(args=args)

    node = NavigateAndGrabStick()

    try:
        prepared = node.prepare_arm_for_navigation()

        if not prepared:
            node.get_logger().error(
                "Sequence cancelled because preparation failed."
            )
            node.stage = "failed"
            return

        node.stage = "go_to_pre_approach"

        node.get_logger().info(
            f"Starting navigation to stick {node.stick_id}."
        )

        while (
            rclpy.ok()
            and node.stage not in ("navigation_done", "failed")
        ):
            rclpy.spin_once(node, timeout_sec=0.1)

        if node.stage == "navigation_done":
            node.hard_stop_robot()
            time.sleep(0.5)

            grabbed = node.grab_stick()

            if grabbed:
                node.stage = "done"
            else:
                node.stage = "failed"
                node.get_logger().error(
                    "Robot reached the stick, but the grab sequence failed."
                )

    except KeyboardInterrupt:
        node.get_logger().warning(
            "Controller interrupted by user."
        )

    finally:
        node.hard_stop_robot()
        node.stop_arm()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
