"""
planner_node.py
===============
Hybrid A* planner node.

Key change from original
─────────────────────────
The planner now subscribes to /global_costmap (produced by
global_costmap_node) instead of raw /map.  /global_costmap contains:
    • the static or SLAM-updated occupancy grid
    • social cost blobs around tracked humans

This means the planner automatically routes around both walls AND people
without any changes to the HybridAStar algorithm itself.

Map source tracking is preserved for logging so you can verify in the
terminal which source is feeding the planner.

/map is still subscribed (read-only) purely for logging/diagnostics so
you can see when SLAM has updated the underlying map.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer
from tf2_ros.transform_listener import TransformListener
from hybrid_astar_planner.hybrid_astar import HybridAStar


class PlannerNode(Node):

    def __init__(self):
        super().__init__('planner_node')

        # ── Subscriptions ─────────────────────────────────────────────────
        # /global_costmap: merged static/SLAM map + social costs — used for planning
        self.gcm_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap', self.global_costmap_cb, 10)

        # /map: subscribed only to track SLAM updates for logging
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # ── TF2 ───────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ─────────────────────────────────────────────────────────
        self.planner         = HybridAStar()
        self.global_costmap: OccupancyGrid | None = None
        self._map_seq        = 0
        self._map_source     = 'none'

        self.get_logger().info(
            'PlannerNode ready — waiting for /global_costmap and /goal_pose.')

    # ──────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────

    def global_costmap_cb(self, msg: OccupancyGrid):
        """Store the latest merged costmap for use at the next goal request."""
        self.global_costmap = msg

    def map_cb(self, msg: OccupancyGrid):
        """
        Track /map updates for logging only — planner uses /global_costmap.
        map_server publishes once; slam_toolbox publishes repeatedly.
        """
        self._map_seq += 1
        if self._map_seq == 1:
            self._map_source = 'map_server (static)'
        elif self._map_seq == 2:
            self._map_source = 'slam_toolbox (live)'
            self.get_logger().info(
                'slam_toolbox is now publishing /map — '
                '/global_costmap will reflect live updates.')

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))

            q = trans.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                yaw,
            )
        except Exception as e:
            self.get_logger().info(f'TF lookup failed: {e}')
            return None

    def goal_cb(self, msg: PoseStamped):
        # ── Guard: need the global costmap ──────────────────────────────
        if self.global_costmap is None:
            self.get_logger().error(
                'No /global_costmap received yet! '
                'Ensure global_costmap_node and map_server are running.')
            return

        # ── Guard: need robot pose ───────────────────────────────────────
        start = self.get_robot_pose()
        if start is None:
            return

        gcm = self.global_costmap
        res = gcm.info.resolution
        ox  = gcm.info.origin.position.x
        oy  = gcm.info.origin.position.y
        w   = gcm.info.width
        h   = gcm.info.height

        # ── Verify robot is inside the costmap ──────────────────────────
        gx = int((start[0] - ox) / res)
        gy = int((start[1] - oy) / res)

        if 0 <= gx < w and 0 <= gy < h:
            val = gcm.data[gy * w + gx]
            self.get_logger().info(
                f'[{self._map_source}] '
                f'Robot ({start[0]:.2f}, {start[1]:.2f}) → '
                f'GlobalCostmap[{gx},{gy}] = {val}  '
                f'| costmap size: {w}×{h} cells')
        else:
            self.get_logger().warn(
                f'Robot is OUTSIDE global costmap bounds! '
                f'start={start[:2]}  origin=({ox:.2f},{oy:.2f})  size={w}×{h}')
            return

        # ── Extract goal yaw ─────────────────────────────────────────────
        qg = msg.pose.orientation
        siny = 2.0 * (qg.w * qg.z + qg.x * qg.y)
        cosy = 1.0 - 2.0 * (qg.y * qg.y + qg.z * qg.z)
        goal_yaw = math.atan2(siny, cosy)

        goal = (msg.pose.position.x, msg.pose.position.y, goal_yaw)
        self.get_logger().info(
            f'Planning to goal ({goal[0]:.2f}, {goal[1]:.2f}, '
            f'θ={math.degrees(goal[2]):.1f}°) | source: {self._map_source}')

        # ── Run Hybrid A* on the GLOBAL costmap ─────────────────────────
        raw_path = self.planner.plan(
            start, goal,
            gcm.data,      # ← merged static + SLAM + social costs
            w, h, ox, oy, res,
        )

        # ── Publish path ─────────────────────────────────────────────────
        if raw_path:
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp    = self.get_clock().now().to_msg()

            for p in raw_path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = p[0]
                pose.pose.position.y = p[1]
                pose.pose.orientation.z = math.sin(p[2] / 2.0)
                pose.pose.orientation.w = math.cos(p[2] / 2.0)
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(
                f'SUCCESS — path has {len(raw_path)} waypoints.')
        else:
            self.get_logger().error(
                'PLANNER FAIL — no path found.  '
                'Possible causes: goal inside wall/inflation zone, '
                'goal blocked by human social cost, or search budget exceeded.')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()