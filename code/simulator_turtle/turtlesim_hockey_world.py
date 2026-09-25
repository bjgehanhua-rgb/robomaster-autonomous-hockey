#!/usr/bin/env python3
"""Turtlesim hockey world for ECE486 controller testing.

Objects:
- turtle1: robot
- puck: movable puck (second turtle)
- red U-shaped marking: goal
- stick and goal poses: simulated Vicon PoseStamped topics

Run only one robot controller at a time.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Kill, SetPen, Spawn, TeleportAbsolute


# =============================================================================
# INITIAL CONFIGURATION
# Change only the values in this section to test different starting scenes.
# Turtlesim coordinates are approximately 0.0 to 11.0.
#
# Common directions:
#   0.0          = right
#   math.pi / 2  = up
#   math.pi      = left
#  -math.pi / 2  = down
# =============================================================================

# Robot: turtle1
ROBOT_INIT_X = 5
ROBOT_INIT_Y = 5
ROBOT_INIT_THETA = 0.0

# Puck: second turtle
PUCK_INIT_X = 3.0
PUCK_INIT_Y = 6.0
PUCK_INIT_THETA = 0.0

# Fixed stick pose published to the simulated Vicon topic
STICK_INIT_X = 2.0
STICK_INIT_Y = 2.0
STICK_INIT_THETA = math.pi/2

# Stick marker display
DRAW_STICK_MARKER = True
STICK_MARKER_STEM_LENGTH = 0.70
STICK_MARKER_BAR_HALF_LENGTH = 0.30
STICK_MARKER_PEN_WIDTH = 6
STICK_MARKER_R = 245
STICK_MARKER_G = 205
STICK_MARKER_B = 25

# Goal center and orientation
# GOAL_INIT_THETA is the direction in which the U-shaped goal opens.
GOAL_INIT_X = 10.0
GOAL_INIT_Y = 2.5
GOAL_INIT_THETA = 3*math.pi/4

# Goal display and scoring parameters
GOAL_RADIUS = 0.45
GOAL_WIDTH = 1.10
GOAL_DEPTH = 0.40
GOAL_PEN_WIDTH = 8


class TurtlesimHockeyWorld(Node):
    def __init__(self) -> None:
        super().__init__("turtlesim_hockey_world")

        # Initial configuration.
        self.robot_init_x = ROBOT_INIT_X
        self.robot_init_y = ROBOT_INIT_Y
        self.robot_init_theta = ROBOT_INIT_THETA

        self.stick_x = STICK_INIT_X
        self.stick_y = STICK_INIT_Y
        self.stick_theta = STICK_INIT_THETA

        self.draw_stick_marker = DRAW_STICK_MARKER
        self.stick_marker_stem_length = STICK_MARKER_STEM_LENGTH
        self.stick_marker_bar_half_length = STICK_MARKER_BAR_HALF_LENGTH
        self.stick_marker_pen_width = STICK_MARKER_PEN_WIDTH
        self.stick_marker_r = STICK_MARKER_R
        self.stick_marker_g = STICK_MARKER_G
        self.stick_marker_b = STICK_MARKER_B

        self.puck_start_x = PUCK_INIT_X
        self.puck_start_y = PUCK_INIT_Y
        self.puck_start_theta = PUCK_INIT_THETA

        self.goal_x = GOAL_INIT_X
        self.goal_y = GOAL_INIT_Y
        self.goal_theta = GOAL_INIT_THETA
        self.goal_radius = GOAL_RADIUS

        self.goal_width = GOAL_WIDTH
        self.goal_depth = GOAL_DEPTH
        self.goal_pen_width = GOAL_PEN_WIDTH

        # Simple shot detection / puck motion.
        self.contact_distance = 0.85
        self.min_shot_angular_speed = 1.0
        self.puck_speed = 2.2
        self.puck_active = False
        self.puck_reached_goal = False

        self.robot_pose: Optional[Pose] = None
        self.puck_pose: Optional[Pose] = None
        self.last_robot_cmd = Twist()

        # Command publishers.
        self.robot_cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.puck_cmd_pub = self.create_publisher(Twist, "/puck/cmd_vel", 10)

        # Simulated Vicon publishers expected by robot controllers 1 through 10.
        self.robot_pose_pubs = {
            robot_id: self.create_publisher(
                PoseStamped,
                f"/vrpn_mocap/dji_robot_{robot_id}/pose",
                10,
            )
            for robot_id in range(1, 11)
        }
        self.puck_pose_pub = self.create_publisher(
            PoseStamped, "/vrpn_mocap/hockey_puck_blue/pose", 10
        )
        self.stick_pose_pub = self.create_publisher(
            PoseStamped, "/vrpn_mocap/hockey_sticks_1/pose", 10
        )
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, "/vrpn_mocap/hockey_goal_1/pose", 10
        )

        # Pose subscriptions.
        self.create_subscription(Pose, "/turtle1/pose", self.robot_pose_callback, 10)
        self.create_subscription(Pose, "/puck/pose", self.puck_pose_callback, 10)

        # Bridge robot controller IDs 1 through 10 to the same turtle1.
        self.robot_cmd_subscriptions = [
            self.create_subscription(
                Twist,
                f"/robot{robot_id}/cmd_vel",
                self.robot_cmd_callback,
                10,
            )
            for robot_id in range(1, 11)
        ]

        # Main turtlesim services.
        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.kill_client = self.create_client(Kill, "/kill")
        self.robot_pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        self.robot_teleport_client = self.create_client(
            TeleportAbsolute, "/turtle1/teleport_absolute"
        )
        self.puck_pen_client = self.create_client(SetPen, "/puck/set_pen")
        self.puck_teleport_client = self.create_client(
            TeleportAbsolute, "/puck/teleport_absolute"
        )

        self.robot_initialized = False
        self.puck_spawn_requested = False
        self.puck_spawned = False
        self.pen_disabled = False
        self.last_spawn_wait_log_ns = 0

        # Goal drawing state. A temporary turtle draws the red U, then is killed.
        # The local U opens in the +x direction and is then rotated by goal_theta.
        half_w = self.goal_width / 2.0
        half_d = self.goal_depth / 2.0
        local_goal_points = [
            (half_d, half_w),
            (-half_d, half_w),
            (-half_d, -half_w),
            (half_d, -half_w),
        ]

        cos_goal = math.cos(self.goal_theta)
        sin_goal = math.sin(self.goal_theta)
        self.goal_points = [
            (
                self.goal_x + local_x * cos_goal - local_y * sin_goal,
                self.goal_y + local_x * sin_goal + local_y * cos_goal,
            )
            for local_x, local_y in local_goal_points
        ]
        self.goal_spawn_requested = False
        self.goal_drawer_spawned = False
        self.goal_drawn = False
        self.goal_draw_stage = "spawn"
        self.goal_point_index = 0
        self.goal_future = None
        self.goal_pen_client = None
        self.goal_teleport_client = None

        # Stick marker drawing state. A temporary turtle draws a blue T.
        # stem_end_local = (-self.stick_marker_stem_length, 0.0)

        # bar_left_local = (
        #     -self.stick_marker_stem_length,
        #     self.stick_marker_bar_half_length,
        # )

        # bar_right_local = (
        #     -self.stick_marker_stem_length,
        #     -self.stick_marker_bar_half_length,
        # )
        stem_end_local = (
            self.stick_marker_stem_length,
            0.0,
        )

        bar_left_local = (
            0.0,
            self.stick_marker_bar_half_length,
        )

        bar_right_local = (
            0.0,
            -self.stick_marker_bar_half_length,
        )

        cos_stick = math.cos(self.stick_theta)
        sin_stick = math.sin(self.stick_theta)

        def rotate_stick_point(local_x, local_y):
            return (
                self.stick_x + local_x * cos_stick - local_y * sin_stick,
                self.stick_y + local_x * sin_stick + local_y * cos_stick,
            )

        # self.stick_marker_points = [
        #     (self.stick_x, self.stick_y),
        #     rotate_stick_point(*stem_end_local),
        #     rotate_stick_point(*bar_left_local),
        #     rotate_stick_point(*bar_right_local),
        # ]
        self.stick_marker_points = [
            (self.stick_x, self.stick_y),       # T-junction
            rotate_stick_point(*stem_end_local),
            rotate_stick_point(*bar_left_local),
            rotate_stick_point(*bar_right_local),
        ]

        self.stick_spawn_requested = False
        self.stick_marker_drawn = not self.draw_stick_marker
        self.stick_draw_stage = "spawn"
        self.stick_future = None
        self.stick_pen_client = None
        self.stick_teleport_client = None

        self.timer = self.create_timer(0.05, self.update)

        self.get_logger().info(
            "Hockey world started: turtle1=robot, puck=second turtle."
        )
        self.get_logger().info(
            f"robot=({self.robot_init_x:.2f}, {self.robot_init_y:.2f}, "
            f"{self.robot_init_theta:.2f} rad), "
            f"stick=({self.stick_x:.2f}, {self.stick_y:.2f}), "
            f"puck=({self.puck_start_x:.2f}, {self.puck_start_y:.2f}, "
            f"{self.puck_start_theta:.2f} rad), "
            f"goal=({self.goal_x:.2f}, {self.goal_y:.2f}, "
            f"{self.goal_theta:.2f} rad)"
        )

    @staticmethod
    def yaw_to_quaternion(theta: float):
        half = 0.5 * theta
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def pose_stamped(self, x: float, y: float, theta: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(theta)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        return msg

    def robot_pose_callback(self, msg: Pose) -> None:
        self.robot_pose = msg

    def puck_pose_callback(self, msg: Pose) -> None:
        self.puck_pose = msg

    def robot_cmd_callback(self, msg: Twist) -> None:
        self.last_robot_cmd = msg
        self.robot_cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Move turtle1 to the configured initial pose.
    # ------------------------------------------------------------------
    def initialize_robot_pose(self) -> None:
        if self.robot_initialized:
            return

        if not (
            self.robot_pen_client.wait_for_service(timeout_sec=0.1)
            and self.robot_teleport_client.wait_for_service(timeout_sec=0.1)
        ):
            return

        pen_request = SetPen.Request()
        pen_request.r = 255
        pen_request.g = 255
        pen_request.b = 255
        pen_request.width = 1
        pen_request.off = 1
        self.robot_pen_client.call_async(pen_request)

        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = float(self.robot_init_x)
        teleport_request.y = float(self.robot_init_y)
        teleport_request.theta = float(self.robot_init_theta)
        self.robot_teleport_client.call_async(teleport_request)

        self.robot_initialized = True
        self.get_logger().info("Robot moved to configured initial pose.")

    # ------------------------------------------------------------------
    # Spawn the movable puck turtle.
    # ------------------------------------------------------------------
    def request_puck_spawn(self) -> None:
        if self.puck_spawn_requested:
            return

        # service_is_ready() can remain false silently while ROS discovery is
        # still settling. wait_for_service() actively checks the service and
        # makes startup much more reliable after reopening VS Code/WSL.
        if not self.spawn_client.wait_for_service(timeout_sec=0.5):
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_spawn_wait_log_ns > 2_000_000_000:
                self.get_logger().info("Waiting for turtlesim /spawn service...")
                self.last_spawn_wait_log_ns = now_ns
            return

        request = Spawn.Request()
        request.x = float(self.puck_start_x)
        request.y = float(self.puck_start_y)
        request.theta = float(self.puck_start_theta)
        request.name = "puck"

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_done_callback)
        self.puck_spawn_requested = True

    def spawn_done_callback(self, future) -> None:
        try:
            response = future.result()
            self.puck_spawned = True
            self.get_logger().info(f"Spawned second turtle as '{response.name}'.")
        except Exception as exc:
            text = str(exc).lower()
            if "already exists" in text:
                self.puck_spawned = True
                self.get_logger().warning("Puck already exists; using /puck.")
            else:
                self.get_logger().error(f"Failed to spawn puck: {exc}")
                self.puck_spawn_requested = False

    # ------------------------------------------------------------------
    # Draw the fixed red goal using a temporary third turtle.
    # ------------------------------------------------------------------
    def update_goal_drawing(self) -> None:
        if self.goal_drawn:
            return

        if self.goal_draw_stage == "spawn":
            if self.goal_spawn_requested or not self.spawn_client.service_is_ready():
                return
            request = Spawn.Request()
            request.x = float(self.goal_points[0][0])
            request.y = float(self.goal_points[0][1])
            request.theta = 0.0
            request.name = "goal_drawer"
            self.goal_future = self.spawn_client.call_async(request)
            self.goal_spawn_requested = True
            self.goal_draw_stage = "wait_spawn"
            return

        if self.goal_draw_stage == "wait_spawn":
            if not self.goal_future.done():
                return
            try:
                self.goal_future.result()
            except Exception as exc:
                if "already exists" not in str(exc).lower():
                    self.get_logger().error(f"Could not spawn goal drawer: {exc}")
                    return
            self.goal_drawer_spawned = True
            self.goal_pen_client = self.create_client(
                SetPen, "/goal_drawer/set_pen"
            )
            self.goal_teleport_client = self.create_client(
                TeleportAbsolute, "/goal_drawer/teleport_absolute"
            )
            self.goal_draw_stage = "wait_services"
            return

        if self.goal_draw_stage == "wait_services":
            if not (
                self.goal_pen_client.service_is_ready()
                and self.goal_teleport_client.service_is_ready()
            ):
                return
            request = SetPen.Request()
            request.r = 230
            request.g = 30
            request.b = 30
            request.width = self.goal_pen_width
            request.off = 0
            self.goal_future = self.goal_pen_client.call_async(request)
            self.goal_point_index = 1
            self.goal_draw_stage = "wait_pen"
            return

        if self.goal_draw_stage == "wait_pen":
            if not self.goal_future.done():
                return
            self.goal_draw_stage = "draw_next"

        if self.goal_draw_stage == "draw_next":
            if self.goal_point_index >= len(self.goal_points):
                self.goal_draw_stage = "kill"
                return
            x, y = self.goal_points[self.goal_point_index]
            request = TeleportAbsolute.Request()
            request.x = float(x)
            request.y = float(y)
            request.theta = 0.0
            self.goal_future = self.goal_teleport_client.call_async(request)
            self.goal_point_index += 1
            self.goal_draw_stage = "wait_point"
            return

        if self.goal_draw_stage == "wait_point":
            if self.goal_future.done():
                self.goal_draw_stage = "draw_next"
            return

        if self.goal_draw_stage == "kill":
            if not self.kill_client.service_is_ready():
                return
            request = Kill.Request()
            request.name = "goal_drawer"
            self.goal_future = self.kill_client.call_async(request)
            self.goal_draw_stage = "wait_kill"
            return

        if self.goal_draw_stage == "wait_kill":
            if not self.goal_future.done():
                return
            self.goal_drawn = True
            self.get_logger().info("Red goal marking drawn in turtlesim.")


    # ------------------------------------------------------------------
    # Draw the fixed stick marker using a temporary turtle.
    # ------------------------------------------------------------------
    def update_stick_marker_drawing(self) -> None:
        if self.stick_marker_drawn:
            return

        if self.stick_draw_stage == "spawn":
            if self.stick_spawn_requested or not self.spawn_client.service_is_ready():
                return
            request = Spawn.Request()
            request.x = float(self.stick_marker_points[0][0])
            request.y = float(self.stick_marker_points[0][1])
            request.theta = 0.0
            request.name = "stick_drawer"
            self.stick_future = self.spawn_client.call_async(request)
            self.stick_spawn_requested = True
            self.stick_draw_stage = "wait_spawn"
            return

        if self.stick_draw_stage == "wait_spawn":
            if not self.stick_future.done():
                return
            try:
                self.stick_future.result()
            except Exception as exc:
                if "already exists" not in str(exc).lower():
                    self.get_logger().error(f"Could not spawn stick drawer: {exc}")
                    return
            self.stick_pen_client = self.create_client(
                SetPen, "/stick_drawer/set_pen"
            )
            self.stick_teleport_client = self.create_client(
                TeleportAbsolute, "/stick_drawer/teleport_absolute"
            )
            self.stick_draw_stage = "wait_services"
            return

        if self.stick_draw_stage == "wait_services":
            if not (
                self.stick_pen_client.service_is_ready()
                and self.stick_teleport_client.service_is_ready()
            ):
                return
            request = SetPen.Request()
            request.r = int(self.stick_marker_r)
            request.g = int(self.stick_marker_g)
            request.b = int(self.stick_marker_b)
            request.width = int(self.stick_marker_pen_width)
            request.off = 0
            self.stick_future = self.stick_pen_client.call_async(request)
            self.stick_draw_stage = "wait_pen"
            return

        if self.stick_draw_stage == "wait_pen":
            if not self.stick_future.done():
                return
            x, y = self.stick_marker_points[1]
            request = TeleportAbsolute.Request()
            request.x = float(x)
            request.y = float(y)
            request.theta = 0.0
            self.stick_future = self.stick_teleport_client.call_async(request)
            self.stick_draw_stage = "wait_stem"
            return

        if self.stick_draw_stage == "wait_stem":
            if not self.stick_future.done():
                return
            request = SetPen.Request()
            request.r = int(self.stick_marker_r)
            request.g = int(self.stick_marker_g)
            request.b = int(self.stick_marker_b)
            request.width = int(self.stick_marker_pen_width)
            request.off = 1
            self.stick_future = self.stick_pen_client.call_async(request)
            self.stick_draw_stage = "wait_pen_off"
            return

        if self.stick_draw_stage == "wait_pen_off":
            if not self.stick_future.done():
                return
            x, y = self.stick_marker_points[2]
            request = TeleportAbsolute.Request()
            request.x = float(x)
            request.y = float(y)
            request.theta = 0.0
            self.stick_future = self.stick_teleport_client.call_async(request)
            self.stick_draw_stage = "wait_bar_start"
            return

        if self.stick_draw_stage == "wait_bar_start":
            if not self.stick_future.done():
                return
            request = SetPen.Request()
            request.r = int(self.stick_marker_r)
            request.g = int(self.stick_marker_g)
            request.b = int(self.stick_marker_b)
            request.width = int(self.stick_marker_pen_width)
            request.off = 0
            self.stick_future = self.stick_pen_client.call_async(request)
            self.stick_draw_stage = "wait_pen_on"
            return

        if self.stick_draw_stage == "wait_pen_on":
            if not self.stick_future.done():
                return
            x, y = self.stick_marker_points[3]
            request = TeleportAbsolute.Request()
            request.x = float(x)
            request.y = float(y)
            request.theta = 0.0
            self.stick_future = self.stick_teleport_client.call_async(request)
            self.stick_draw_stage = "wait_bar_end"
            return

        if self.stick_draw_stage == "wait_bar_end":
            if not self.stick_future.done():
                return
            self.stick_draw_stage = "kill"
            return

        if self.stick_draw_stage == "kill":
            if not self.kill_client.service_is_ready():
                return
            request = Kill.Request()
            request.name = "stick_drawer"
            self.stick_future = self.kill_client.call_async(request)
            self.stick_draw_stage = "wait_kill"
            return

        if self.stick_draw_stage == "wait_kill":
            if not self.stick_future.done():
                return
            self.stick_marker_drawn = True
            self.get_logger().info("Yellow stick marker drawn in turtlesim.")

    def disable_pen_once(self) -> None:
        if self.pen_disabled:
            return
        if not (
            self.robot_pen_client.service_is_ready()
            and self.puck_pen_client.service_is_ready()
        ):
            return

        request = SetPen.Request()
        request.r = 255
        request.g = 255
        request.b = 255
        request.width = 1
        request.off = 1
        self.robot_pen_client.call_async(request)
        self.puck_pen_client.call_async(request)
        self.pen_disabled = True

    def publish_simulated_vicon(self) -> None:
        if self.robot_pose is not None:
            robot_msg = self.pose_stamped(
                self.robot_pose.x, self.robot_pose.y, self.robot_pose.theta
            )
            for publisher in self.robot_pose_pubs.values():
                publisher.publish(robot_msg)

        if self.puck_pose is not None:
            self.puck_pose_pub.publish(
                self.pose_stamped(
                    self.puck_pose.x, self.puck_pose.y, self.puck_pose.theta
                )
            )

        self.stick_pose_pub.publish(
            self.pose_stamped(self.stick_x, self.stick_y, self.stick_theta)
        )
        self.goal_pose_pub.publish(
            self.pose_stamped(self.goal_x, self.goal_y, self.goal_theta)
        )

    def maybe_launch_puck(self) -> None:
        if (
            self.robot_pose is None
            or self.puck_pose is None
            or self.puck_active
            or self.puck_reached_goal
        ):
            return

        distance = math.hypot(
            self.robot_pose.x - self.puck_pose.x,
            self.robot_pose.y - self.puck_pose.y,
        )

        if (
            distance <= self.contact_distance
            and abs(self.last_robot_cmd.angular.z) >= self.min_shot_angular_speed
        ):
            heading = math.atan2(
                self.goal_y - self.puck_pose.y,
                self.goal_x - self.puck_pose.x,
            )
            if self.puck_teleport_client.service_is_ready():
                request = TeleportAbsolute.Request()
                request.x = float(self.puck_pose.x)
                request.y = float(self.puck_pose.y)
                request.theta = float(heading)
                self.puck_teleport_client.call_async(request)
                self.puck_active = True
                self.get_logger().info("SHOT detected: puck launched toward goal.")

    def update_puck_motion(self) -> None:
        cmd = Twist()

        if not self.puck_active or self.puck_pose is None:
            self.puck_cmd_pub.publish(cmd)
            return

        distance_to_goal = math.hypot(
            self.goal_x - self.puck_pose.x,
            self.goal_y - self.puck_pose.y,
        )

        if distance_to_goal <= self.goal_radius:
            self.puck_active = False
            self.puck_reached_goal = True
            self.puck_cmd_pub.publish(cmd)
            self.get_logger().info("GOAL!")
            return

        cmd.linear.x = self.puck_speed
        self.puck_cmd_pub.publish(cmd)

    def update(self) -> None:
        self.initialize_robot_pose()

        if not self.puck_spawned:
            self.request_puck_spawn()

        # Goal drawing and fixed-pose publication do not need to be blocked
        # while the puck spawn request is still being completed.
        self.update_goal_drawing()
        self.update_stick_marker_drawing()

        if self.puck_spawned:
            self.disable_pen_once()

        self.publish_simulated_vicon()

        if self.puck_spawned:
            self.maybe_launch_puck()
            self.update_puck_motion()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurtlesimHockeyWorld()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.robot_cmd_pub.publish(Twist())
        node.puck_cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()