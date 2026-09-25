import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import ColorRGBA, Bool
from math import cos, sin, pi
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import threading


# =============================================================================
# Initial scene configuration
# Each vector uses the format: [x, y, theta]
# theta is in radians. The puck's theta is currently unused.
# =============================================================================
ROBOT_INITIAL_POSE = np.array([-2, -2, pi/2], dtype=float)
PUCK_INITIAL_POSE = np.array([-1, 1, 0.00], dtype=float)
GOAL_INITIAL_POSE = np.array([2, 2, pi/4], dtype=float)
STICK_INITIAL_POSE = np.array([2, 0, pi], dtype=float)

class MultiRoboMasterSim(Node):
    def __init__(self):
        super().__init__('multi_robomaster_sim')

        # constants
        # robots
        self.ROBOT_IDS = [6]
        self.N = len(self.ROBOT_IDS)
        # time
        self.TIMEOUT_SET_MOBILE_BASE_SPEED = 20 # milliseconds
        self.TIMEOUT_GET_POSES = 10 # milliseconds
        self.TIMEOUT_CHASSIS_SPEED = 500 # milliseconds
        self.DT = (self.TIMEOUT_SET_MOBILE_BASE_SPEED + self.TIMEOUT_GET_POSES) / 1000.
        # robot control
        self.MAX_LINEAR_SPEED = 1.0 # meters / second
        self.MAX_ANGULAR_SPEED = 360 * np.pi / 180 # radians / second
        # dimensions
        self.ENV = [-3, -3, 6, 6] # (x, y) can vary from (ENV[0], ENV[1]) to (ENV[0]+ENV[2], ENV[1]+ENV[3])
        self.ROBOT_SIZE = [0.3, 0.4] # [w, l]
        self.GRIPPER_SIZE = 0.2
        
        # State: [x, y, theta]
        self.states = {}
        self.leds = {}
        self.velocities = {rid: np.array([0.0, 0.0, 0.0]) for rid in self.ROBOT_IDS}
        self.last_cmd_time = {rid: self.get_clock().now() for rid in self.ROBOT_IDS}
        
        # Initialize a single test robot at a deterministic pose.
        # State format: [x, y, theta].
        for rid in self.ROBOT_IDS:
            self.states[rid] = ROBOT_INITIAL_POSE.copy()
            self.leds[rid] = np.array([0., 0., 0.])

        # Hockey-scene objects.
        self.puck_position = PUCK_INITIAL_POSE[:2].copy()
        self.puck_velocity = np.array([0.0, 0.0], dtype=float)
        self.PUCK_RADIUS = 0.07
        self.PUCK_FRICTION = 1.25       # exponential velocity decay, 1/s
        self.PUCK_RESTITUTION = 0.70    # wall bounce
        self.PUCK_STOP_SPEED = 0.015    # m/s
        self.goal_position = GOAL_INITIAL_POSE[:2].copy()
        self.goal_yaw = float(GOAL_INITIAL_POSE[2])

        # T-shaped hockey stick.
        # The published stick pose is the T intersection (the base coordinate).
        # The stem starts at that point and points along stick_yaw.
        self.stick_position = STICK_INITIAL_POSE[:2].copy()
        self.stick_yaw = float(STICK_INITIAL_POSE[2])
        self.STICK_STEM_LENGTH = 0.52
        self.STICK_STEM_WIDTH = 0.055
        self.STICK_CROSSBAR_LENGTH = 0.25
        self.STICK_CROSSBAR_WIDTH = 0.065

        # When held, the T intersection is just in front of the robot.
        # 0.18 + 0.52 = 0.70 m from robot center to stick tip, matching rs3.
        self.stick_attached = False
        self.STICK_BASE_OFFSET = 0.18

        # Simple stick-puck collision model.
        self.STICK_CONTACT_RADIUS = self.PUCK_RADIUS + 0.055
        self.STICK_IMPULSE_GAIN = 1.45
        self.last_stick_tip = None
        self.last_hit_time = -1e9
        self.HIT_COOLDOWN = 0.25

        # Pubs and Subs
        self.pubs = {}
        self.subs_vel = {}
        self.subs_led = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        # Fixed-object pose publishers. Topic names exactly match rs3(10).py.
        self.puck_pub = self.create_publisher(
            PoseStamped, '/vrpn_mocap/hockey_puck_blue/pose', qos)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/vrpn_mocap/hockey_goal_1/pose', qos)
        self.stick_pub = self.create_publisher(
            PoseStamped, '/vrpn_mocap/hockey_sticks_1/pose', qos)
        self.gripper_sub = self.create_subscription(
            Bool,
            '/robot6/gripper_closed',
            self.gripper_callback,
            qos,
        )

        for rid in self.ROBOT_IDS:
            # Publisher: Mimics VRPN motion capture system
            self.pubs[rid] = self.create_publisher(
                PoseStamped, f'/vrpn_mocap/dji_robot_{rid}/pose', qos)
            
            # Subscriber: Listen to the controller's cmd_vel
            self.subs_vel[rid] = self.create_subscription(
                Twist, f'/robot{rid}/cmd_vel', 
                lambda msg, rid=rid: self.vel_callback(msg, rid), qos)
            
            # Subscriber: Listen to the controller's leds
            self.subs_led[rid] = self.create_subscription(
                ColorRGBA, f'/robot{rid}/leds/color', 
                lambda msg, rid=rid: self.led_callback(msg, rid), qos)

        self.timer = self.create_timer(self.DT, self.update_and_publish)
        self.get_logger().info(f"Simulator started for robots: {self.ROBOT_IDS}")

        # Plots
        self.figure = []
        self.axes = []
        self.patches_robots = {rid: [] for rid in self.ROBOT_IDS}
        self.patches_grippers = {rid: [] for rid in self.ROBOT_IDS}
        self.text_ids = {rid: [] for rid in self.ROBOT_IDS}
        self.patch_puck = None
        self.patch_goal = None
        self.patch_stick_stem = None
        self.patch_stick_crossbar = None
        self.text_stick = None
        self.__init_plot()
        self.__update_plot()
    
    def __init_plot(self):
        self.figure, self.axes = plt.subplots()
        p_env = patches.Rectangle(np.array([self.ENV[0], self.ENV[1]]), self.ENV[2], self.ENV[3], edgecolor=(0, 0, 0, 1), fill=False, linewidth=4)
        self.axes.add_patch(p_env)

        # Blue puck and goal visualization.
        self.patch_puck = patches.Circle(
            self.puck_position, radius=self.PUCK_RADIUS, facecolor='tab:blue', edgecolor='k')
        # U-shaped goal frame. The open side points along the goal's
        # local +x direction, and the whole frame rotates with goal_yaw.
        goal_local_corners = np.array([
            [-0.06,  0.40],
            [ 0.06,  0.40],
            [ 0.06, -0.40],
            [-0.06, -0.40],
        ])
        goal_rotation = np.array([
            [cos(self.goal_yaw), -sin(self.goal_yaw)],
            [sin(self.goal_yaw),  cos(self.goal_yaw)],
        ])
        goal_corners = (
            goal_local_corners @ goal_rotation.T
            + self.goal_position
        )
        self.patch_goal = patches.Polygon(
            goal_corners,
            closed=False,
            fill=False,
            edgecolor='tab:green',
            linewidth=4,
        )
        self.axes.add_patch(self.patch_puck)
        self.axes.add_patch(self.patch_goal)
        self.axes.text(self.puck_position[0], self.puck_position[1] + 0.13,
                       'puck', ha='center')
        self.axes.text(self.goal_position[0], self.goal_position[1] + 0.48,
                       'goal', ha='center')

        # T-shaped stick visualization.
        stem_xy, crossbar_xy = self.get_stick_polygons()
        self.patch_stick_stem = patches.Polygon(
            stem_xy,
            closed=True,
            facecolor='saddlebrown',
            edgecolor='saddlebrown',
            linewidth=2,
        )
        self.patch_stick_crossbar = patches.Polygon(
            crossbar_xy,
            closed=True,
            facecolor='peru',
            edgecolor='peru',
            linewidth=2,
        )
        self.axes.add_patch(self.patch_stick_stem)
        self.axes.add_patch(self.patch_stick_crossbar)
        self.text_stick = self.axes.text(
            self.stick_position[0],
            self.stick_position[1] + 0.20,
            'stick 1',
            ha='center',
        )

        for i, rid in enumerate(self.ROBOT_IDS):
            R = np.array([[cos(self.states[rid][2]), -sin(self.states[rid][2])], [sin(self.states[rid][2]), cos(self.states[rid][2])]])
            t = np.array([self.states[rid][0], self.states[rid][1]])
            p_robot = patches.Polygon(t + (np.array([[self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                                     [-self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                                     [-self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0],
                                                     [self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0]]) @ R.T),
                                                     facecolor='k')
            p_gripper = patches.Polygon(t + (np.array([[self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, 0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, 0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, -0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -0.8 * self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -self.GRIPPER_SIZE / 2.0],
                                                       [self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0]]) @ R.T),
                                                       facecolor='k')
            text_id = plt.text(self.states[rid][0] + max(self.ROBOT_SIZE) / 2.0, self.states[rid][1] + max(self.ROBOT_SIZE) / 2.0, s=str(self.ROBOT_IDS[i]), color="red")
            self.patches_robots[rid] = p_robot
            self.patches_grippers[rid] = p_gripper
            self.text_ids[rid] = text_id
            self.axes.add_patch(p_robot)
            self.axes.add_patch(p_gripper)
        
        self.axes.set_xlim(self.ENV[0] - max(self.ROBOT_SIZE), self.ENV[0] + self.ENV[2] + max(self.ROBOT_SIZE))
        self.axes.set_ylim(self.ENV[1] - max(self.ROBOT_SIZE), self.ENV[1] + self.ENV[3] + max(self.ROBOT_SIZE))
        self.axes.grid()
        # self.axes.set_axis_off()
        self.axes.axis('equal')

        # The Qt window is started from main(), on the main thread.
        # ROS spins separately so pose publishing is not blocked by plt.show().
    
    def __update_plot(self):
        for rid in self.ROBOT_IDS:
            R = np.array([[cos(self.states[rid][2]), -sin(self.states[rid][2])], [sin(self.states[rid][2]), cos(self.states[rid][2])]])
            t = np.array([self.states[rid][0], self.states[rid][1]])
            xy_robot = t + (np.array([[self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                      [-self.ROBOT_SIZE[1] / 2.0, self.ROBOT_SIZE[0] / 2.0],
                                      [-self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0],
                                      [self.ROBOT_SIZE[1] / 2.0, -self.ROBOT_SIZE[0] / 2.0]]) @ R.T)
            xy_gripper = t + (np.array([[self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0, self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, 0.8 * self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0, 0.8 * self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0, -0.8 * self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -0.8 * self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0 + self.GRIPPER_SIZE, -self.GRIPPER_SIZE / 2.0],
                                        [self.ROBOT_SIZE[1] / 2.0, -self.GRIPPER_SIZE / 2.0]]) @ R.T)
        
            self.patches_robots[rid].xy = xy_robot
            self.patches_grippers[rid].xy = xy_gripper

            self.patches_robots[rid].set_facecolor(self.leds[rid])

            self.text_ids[rid].set_position((self.states[rid][0] + max(self.ROBOT_SIZE) / 2.0, self.states[rid][1] + max(self.ROBOT_SIZE) / 2.0))

        self.figure.canvas.draw_idle()
    
    @staticmethod
    def transform_velocity_local_to_global(robots_speeds, theta):
        # robots_speeds : list of 3
        # theta : scalar
        robots_speeds_global = [0] * 3
        x_dot = robots_speeds[0]
        y_dot = robots_speeds[1]
        th_dot = robots_speeds[2]
        c_th = cos(theta)
        s_th = sin(theta)
        robots_speeds_global[0] = c_th * x_dot - s_th * y_dot
        robots_speeds_global[1] = s_th * x_dot + c_th * y_dot
        robots_speeds_global[2] = robots_speeds[2]
        return robots_speeds_global

    @staticmethod
    def rotation_matrix(yaw):
        return np.array([
            [cos(yaw), -sin(yaw)],
            [sin(yaw),  cos(yaw)],
        ])

    def get_stick_polygons(self):
        """Return stem and crossbar polygons for the T-shaped stick.

        stick_position is the T intersection. The stem extends forward along
        stick_yaw, making the direction visually unambiguous.
        """
        direction = np.array([cos(self.stick_yaw), sin(self.stick_yaw)])
        normal = np.array([-sin(self.stick_yaw), cos(self.stick_yaw)])
        base = self.stick_position

        stem_half_width = self.STICK_STEM_WIDTH / 2.0
        tip = base + self.STICK_STEM_LENGTH * direction
        stem_xy = np.array([
            base + stem_half_width * normal,
            tip + stem_half_width * normal,
            tip - stem_half_width * normal,
            base - stem_half_width * normal,
        ])

        half_cross = self.STICK_CROSSBAR_LENGTH / 2.0
        half_cross_width = self.STICK_CROSSBAR_WIDTH / 2.0
        crossbar_xy = np.array([
            base + half_cross * normal + half_cross_width * direction,
            base - half_cross * normal + half_cross_width * direction,
            base - half_cross * normal - half_cross_width * direction,
            base + half_cross * normal - half_cross_width * direction,
        ])
        return stem_xy, crossbar_xy

    def stick_tip_position(self):
        return self.stick_position + self.STICK_STEM_LENGTH * np.array([
            cos(self.stick_yaw),
            sin(self.stick_yaw),
        ])

    @staticmethod
    def point_to_segment_distance(point, seg_start, seg_end):
        segment = seg_end - seg_start
        length_sq = float(np.dot(segment, segment))
        if length_sq < 1e-12:
            return float(np.linalg.norm(point - seg_start))

        t = float(np.dot(point - seg_start, segment) / length_sq)
        t = max(0.0, min(1.0, t))
        closest = seg_start + t * segment
        return float(np.linalg.norm(point - closest))

    def update_stick_pose(self):
        if self.stick_attached:
            robot_x, robot_y, robot_theta = self.states[6]
            direction = np.array([cos(robot_theta), sin(robot_theta)])
            self.stick_position = np.array([robot_x, robot_y]) + (
                self.STICK_BASE_OFFSET * direction
            )
            self.stick_yaw = robot_theta

        stem_xy, crossbar_xy = self.get_stick_polygons()
        self.patch_stick_stem.xy = stem_xy
        self.patch_stick_crossbar.xy = crossbar_xy
        self.text_stick.set_position((
            self.stick_position[0],
            self.stick_position[1] + 0.20,
        ))

    def apply_stick_puck_collision(self, current_time):
        """Give the puck momentum when the moving attached stick contacts it."""
        tip = self.stick_tip_position()

        if self.last_stick_tip is None:
            self.last_stick_tip = tip.copy()
            return

        if not self.stick_attached:
            self.last_stick_tip = tip.copy()
            return

        distance = self.point_to_segment_distance(
            self.puck_position,
            self.last_stick_tip,
            tip,
        )

        time_since_hit = current_time - self.last_hit_time
        if distance <= self.STICK_CONTACT_RADIUS and time_since_hit >= self.HIT_COOLDOWN:
            robot_x, robot_y, _ = self.states[6]
            robot_velocity = self.velocities[6][:2]
            omega = float(self.velocities[6][2])

            radius_vector = tip - np.array([robot_x, robot_y])
            rotational_tip_velocity = omega * np.array([
                -radius_vector[1],
                radius_vector[0],
            ])
            tip_velocity = robot_velocity + rotational_tip_velocity
            tip_speed = float(np.linalg.norm(tip_velocity))

            if tip_speed >= 0.05:
                self.puck_velocity = self.STICK_IMPULSE_GAIN * tip_velocity
                self.last_hit_time = current_time
                self.get_logger().info(
                    "Puck hit: "
                    f"tip_speed={tip_speed:.3f} m/s, "
                    f"puck_velocity=({self.puck_velocity[0]:.3f}, "
                    f"{self.puck_velocity[1]:.3f}) m/s"
                )

        self.last_stick_tip = tip.copy()

    def update_puck_physics(self):
        self.puck_position += self.puck_velocity * self.DT

        # Exponential rolling/sliding friction.
        self.puck_velocity *= np.exp(-self.PUCK_FRICTION * self.DT)
        if np.linalg.norm(self.puck_velocity) < self.PUCK_STOP_SPEED:
            self.puck_velocity[:] = 0.0

        # Keep the puck inside the 5 m x 5 m field and bounce off the walls.
        xmin, ymin, width, height = self.ENV
        xmax = xmin + width
        ymax = ymin + height

        if self.puck_position[0] - self.PUCK_RADIUS < xmin:
            self.puck_position[0] = xmin + self.PUCK_RADIUS
            self.puck_velocity[0] = abs(self.puck_velocity[0]) * self.PUCK_RESTITUTION
        elif self.puck_position[0] + self.PUCK_RADIUS > xmax:
            self.puck_position[0] = xmax - self.PUCK_RADIUS
            self.puck_velocity[0] = -abs(self.puck_velocity[0]) * self.PUCK_RESTITUTION

        if self.puck_position[1] - self.PUCK_RADIUS < ymin:
            self.puck_position[1] = ymin + self.PUCK_RADIUS
            self.puck_velocity[1] = abs(self.puck_velocity[1]) * self.PUCK_RESTITUTION
        elif self.puck_position[1] + self.PUCK_RADIUS > ymax:
            self.puck_position[1] = ymax - self.PUCK_RADIUS
            self.puck_velocity[1] = -abs(self.puck_velocity[1]) * self.PUCK_RESTITUTION

        self.patch_puck.center = self.puck_position

    def gripper_callback(self, msg):
        if msg.data:
            # Attach only when the robot is reasonably close to the stick.
            robot_xy = self.states[6][:2]
            distance = np.linalg.norm(robot_xy - self.stick_position)
            if distance <= 0.70:
                self.stick_attached = True
                self.get_logger().info(
                    f"Stick attached to robot 6 at distance {distance:.3f} m."
                )
            else:
                self.get_logger().warning(
                    f"Gripper closed too far from stick: {distance:.3f} m."
                )
        else:
            self.stick_attached = False

    def vel_callback(self, msg, rid):
        # Store commanded velocities
        robot_speeds = MultiRoboMasterSim.transform_velocity_local_to_global([msg.linear.x, msg.linear.y, msg.angular.z], self.states[rid][2])
        self.velocities[rid] = np.array(robot_speeds)
        self.last_cmd_time[rid] = self.get_clock().now() # Update heartbeat

    def led_callback(self, msg, rid):
        # Store commanded velocities
        self.leds[rid] = np.array([msg.r, msg.g, msg.b])
        
    def publish_fixed_pose(self, publisher, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        half_yaw = float(yaw) * 0.5
        msg.pose.orientation.z = sin(half_yaw)
        msg.pose.orientation.w = cos(half_yaw)
        publisher.publish(msg)

    def update_and_publish(self):
        current_clock = self.get_clock().now()
        current_time = current_clock.nanoseconds / 1e9

        # 1. Integrate robot motion.
        for rid in self.ROBOT_IDS:
            elapsed = (
                current_clock - self.last_cmd_time[rid]
            ).nanoseconds / 1e9

            if elapsed > self.TIMEOUT_CHASSIS_SPEED / 1e3:
                v_cmd = np.array([0.0, 0.0, 0.0])
            else:
                v_cmd = self.velocities[rid]

            self.states[rid][0] += v_cmd[0] * self.DT
            self.states[rid][1] += v_cmd[1] * self.DT
            self.states[rid][2] += v_cmd[2] * self.DT

        # 2. Move the held T-stick, detect impact, then integrate puck momentum.
        self.update_stick_pose()
        self.apply_stick_puck_collision(current_time)
        self.update_puck_physics()

        # 3. Publish dynamic scene-object poses.
        self.publish_fixed_pose(
            self.puck_pub,
            self.puck_position[0],
            self.puck_position[1],
        )
        self.publish_fixed_pose(
            self.goal_pub,
            self.goal_position[0],
            self.goal_position[1],
            self.goal_yaw,
        )
        self.publish_fixed_pose(
            self.stick_pub,
            self.stick_position[0],
            self.stick_position[1],
            self.stick_yaw,
        )

        # 4. Publish robot poses.
        for rid in self.ROBOT_IDS:
            msg = PoseStamped()
            msg.header.stamp = current_clock.to_msg()
            msg.header.frame_id = 'world'

            msg.pose.position.x = float(self.states[rid][0])
            msg.pose.position.y = float(self.states[rid][1])
            msg.pose.position.z = 0.0

            half_yaw = self.states[rid][2] * 0.5
            msg.pose.orientation.z = sin(half_yaw)
            msg.pose.orientation.w = cos(half_yaw)

            self.pubs[rid].publish(msg)

        # Plot updates are handled by a Matplotlib timer on the GUI thread.

def main(args=None):
    rclpy.init(args=args)
    node = MultiRoboMasterSim()

    # ROS callbacks run in a background thread. Qt/Matplotlib remains on the
    # main thread, which is required for reliable GUI operation.
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True,
    )
    ros_thread.start()

    # Refresh the drawing from the Qt main thread at roughly the simulator rate.
    gui_timer = node.figure.canvas.new_timer(
        interval=max(1, int(node.DT * 1000)),
    )
    gui_timer.add_callback(node._MultiRoboMasterSim__update_plot)
    gui_timer.start()

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        gui_timer.stop()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
        node.destroy_node()

if __name__ == '__main__':
    main()
