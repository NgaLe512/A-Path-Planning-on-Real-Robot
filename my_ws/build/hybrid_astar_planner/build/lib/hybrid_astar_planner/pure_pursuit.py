import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
import math


class PurePursuit(Node):

    # ------------------------------------------------------------------
    # Tuning parameters
    # ------------------------------------------------------------------
    LOOKAHEAD_DIST    = 0.6    # metres — look-ahead circle radius
    GOAL_TOLERANCE    = 0.3    # metres — declare goal reached inside this
    BASE_LINEAR_SPEED = 0.4    # m/s    — max forward speed
    MIN_LINEAR_SPEED  = 0.05   # m/s    — floor speed (keeps robot moving)
    MAX_ANGULAR_SPEED = 1.2    # rad/s  — FIX: clamp prevents odometry blowup
    KP_ANGULAR        = 1.8    # proportional gain on heading error
    # Stop translating when heading error exceeds this threshold.
    # Prevents the robot from arcing sideways on large corrections.
    HEADING_STOP_THRESH = math.pi / 3.0   # 60 °

    def __init__(self):
        super().__init__('pure_pursuit')

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)

        self.path         = None
        self.current_pose = None
        self.create_timer(0.1, self.control_loop)   # 10 Hz control loop

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def path_cb(self, msg):
        if msg.poses:
            self.path = msg.poses
            self.get_logger().info(f"New path received: {len(self.path)} waypoints.")
        else:
            self.get_logger().warn("Received empty path — ignoring.")

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return

        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y

        # --- 1. Check if we've reached the FINAL goal ------------------
        final = self.path[-1].pose.position
        dist_to_goal = math.hypot(final.x - curr_x, final.y - curr_y)

        if dist_to_goal < self.GOAL_TOLERANCE:
            self.get_logger().info(
                f"Goal reached! (dist={dist_to_goal:.2f} m)")
            self.path = None
            self.cmd_pub.publish(Twist())   # publish zero-velocity to stop
            return

        # --- 2. Find the closest path point to the robot ---------------
        closest_idx = 0
        min_dist    = float('inf')
        for i, pose_stamped in enumerate(self.path):
            pt   = pose_stamped.pose.position
            dist = math.hypot(pt.x - curr_x, pt.y - curr_y)
            if dist < min_dist:
                min_dist    = dist
                closest_idx = i

        # --- 3. Find the look-ahead target -----------------------------
        # Walk forward from closest_idx until a point at >= LOOKAHEAD_DIST
        target = None
        for i in range(closest_idx, len(self.path)):
            pt   = self.path[i].pose.position
            dist = math.hypot(pt.x - curr_x, pt.y - curr_y)
            if dist >= self.LOOKAHEAD_DIST:
                target = pt
                break

        # If we're very close to the end, aim directly at the final goal
        if target is None:
            target = final

        # --- 4. Compute heading error ----------------------------------
        current_yaw   = self._yaw_from_quaternion(self.current_pose.orientation)
        global_angle  = math.atan2(target.y - curr_y, target.x - curr_x)
        heading_error = self._wrap_angle(global_angle - current_yaw)

        # --- 5. Compute velocity commands ------------------------------
        twist = Twist()

        # FIX: when facing the wrong direction, rotate in place first.
        # This prevents the robot from physically driving sideways/backward
        # which was corrupting odometry.
        if abs(heading_error) > self.HEADING_STOP_THRESH:
            # Pure rotation — no forward motion
            twist.linear.x  = 0.0
        else:
            # Scale down speed smoothly as heading error grows
            heading_fraction = 1.0 - abs(heading_error) / self.HEADING_STOP_THRESH
            twist.linear.x   = max(
                self.MIN_LINEAR_SPEED,
                self.BASE_LINEAR_SPEED * heading_fraction,
            )

        # FIX: clamp angular velocity — unclamped gain × π was sending
        # ~4.7 rad/s which overwhelmed the IMU and broke EKF estimation.
        raw_angular     = self.KP_ANGULAR * heading_error
        twist.angular.z = max(
            -self.MAX_ANGULAR_SPEED,
            min(self.MAX_ANGULAR_SPEED, raw_angular),
        )

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle):
        """Wrap angle to (−π, π]."""
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = PurePursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
