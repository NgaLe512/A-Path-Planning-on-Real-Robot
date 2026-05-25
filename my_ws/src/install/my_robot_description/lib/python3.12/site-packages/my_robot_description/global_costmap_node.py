"""
global_costmap_node.py  (FIXED)
================================
Merges /map (static or live SLAM) with /social_costmap into /global_costmap
for the planner.

Fix applied
-----------
Same QoS fix as social_costmap_node: publish /global_costmap with
RELIABLE + TRANSIENT_LOCAL so RViz2's Map display plugin can receive it.
Also subscribe to both input maps with the matching QoS so we receive
map_server's TRANSIENT_LOCAL /map publication correctly.

Merge rule (per cell):
    • Unknown (-1) in base map → stays -1
    • Known cells             → max(base_value, social_value)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


# ── QoS profile matching map_server / RViz2 Map plugin ────────────────────
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class GlobalCostmapNode(Node):
    def __init__(self):
        super().__init__('global_costmap')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('publish_rate', 5.0)
        rate = self.get_parameter('publish_rate').value

        # ── Subscriptions ─────────────────────────────────────────────────
        # /map uses TRANSIENT_LOCAL (map_server convention)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, MAP_QOS)

        # /social_costmap is now also TRANSIENT_LOCAL (fixed in social node)
        self.social_sub = self.create_subscription(
            OccupancyGrid, '/social_costmap', self.social_cb, MAP_QOS)

        # ── Publisher — TRANSIENT_LOCAL so RViz receives it ───────────────
        self.pub = self.create_publisher(
            OccupancyGrid, '/global_costmap', MAP_QOS)

        # ── State ─────────────────────────────────────────────────────────
        self.base_map:   OccupancyGrid | None = None
        self.social_map: OccupancyGrid | None = None

        # ── Timer ─────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self.publish_global_costmap)

        self.get_logger().info(
            f'GlobalCostmapNode ready — publishing /global_costmap '
            f'at {rate} Hz with TRANSIENT_LOCAL QoS.')

    # ──────────────────────────────────────────────────────────────────────
    def map_cb(self, msg: OccupancyGrid):
        self.base_map = msg

    def social_cb(self, msg: OccupancyGrid):
        self.social_map = msg

    # ──────────────────────────────────────────────────────────────────────
    def publish_global_costmap(self):
        if self.base_map is None:
            self.get_logger().warn(
                'Waiting for /map …', throttle_duration_sec=5.0)
            return

        base = self.base_map
        w    = base.info.width
        h    = base.info.height

        base_arr     = np.array(base.data, dtype=np.int16)
        unknown_mask = base_arr < 0
        known_arr    = np.where(unknown_mask, np.int16(0), base_arr)

        if self.social_map is not None:
            sm = self.social_map
            if sm.info.width == w and sm.info.height == h:
                social_arr = np.clip(
                    np.array(sm.data, dtype=np.int16), 0, 100)
                merged = np.maximum(known_arr, social_arr)
            else:
                self.get_logger().warn(
                    f'Social map size {sm.info.width}×{sm.info.height} '
                    f'!= base map {w}×{h} — social layer skipped.',
                    throttle_duration_sec=2.0)
                merged = known_arr
        else:
            merged = known_arr

        result = np.where(unknown_mask, np.int16(-1), merged).astype(np.int8)

        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp    = self.get_clock().now().to_msg()
        out.info            = base.info
        out.data            = result.tolist()

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCostmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()