import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math

# ── QoS profile that matches map_server / RViz2 Map plugin ────────────────
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

class SocialCostmap(Node):
    def __init__(self):
        super().__init__('social_costmap')

        # Subscribe /human_pose
        self.sub = self.create_subscription(
            Int32MultiArray,
            '/human_pose',
            self.human_callback,
            10
        )

        # Subscribe /map để đồng bộ origin + size
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publish social costmap
        self.pub = self.create_publisher(
            OccupancyGrid,
            '/social_costmap',
            MAP_QOS,
        )

        # Map params — sẽ được cập nhật từ /map
        self.resolution = 0.05
        self.width = 200
        self.height = 200
        self.origin_x = -5.0
        self.origin_y = -5.0
        self.map_ready = False  # chờ nhận /map trước

        self.humans = {}
        self.human_timeout = 1.0
        self.timer = self.create_timer(0.5, self.publish_costmap)

    def map_callback(self, msg):
        """Đồng bộ origin + kích thước từ slam_toolbox map."""
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_ready = True

    def human_callback(self, msg):
        data = msg.data
        if len(data) < 3:
            return

        track_id = data[0]
        x = data[1]/100 # đơn vị: int (mét, đã truncate)
        y = data[2]/100
        now= self.get_clock().now().nanoseconds/1e9

        self.humans[track_id] = (x, y,now)

    def publish_costmap(self):
        if not self.map_ready:
            self.get_logger().warn('Chờ /map từ slam_toolbox...', throttle_duration_sec=5.0)
            return
        now = self.get_clock().now().nanoseconds / 1e9
        
        expired_ids = [ track_id for track_id, (_, _, t_last) in self.humans.items()
                       if now - t_last > self.human_timeout]
        
        for track_id in expired_ids:
            del self.humans[track_id]

        grid = np.zeros((self.height, self.width), dtype=np.float32)

        for (x, y, _) in self.humans.values():
            self.add_gaussian(grid, x, y)

        msg = OccupancyGrid()

        # Header — frame phải là "map"
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        # Info — khớp hoàn toàn với slam map
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        # Flatten và scale 0→100
        flat = (grid.flatten() * 100).astype(np.int8).tolist()
        msg.data = flat

        self.pub.publish(msg)

    def add_gaussian(self, grid, cx, cy):
        """
        Vẽ Gaussian quanh vị trí người (cx, cy) tính bằng mét.
        Dùng vectorized numpy thay vì double for-loop để nhanh hơn.
        """
        sigma = 0.1  # bán kính ảnh hưởng (mét)

        # Tọa độ thế giới của từng cell
        cols = np.arange(self.width)
        rows = np.arange(self.height)
        wx = self.origin_x + (cols+0.5) * self.resolution          # shape (W,)
        wy = self.origin_y + (rows+0.5) * self.resolution          # shape (H,)

        # Broadcast thành lưới 2D
        WX, WY = np.meshgrid(wx, wy)                         # shape (H, W)

        dx = WX - cx
        dy = WY - cy

        cost = np.exp(-(dx**2 + dy**2) / (2 * sigma**2))    # shape (H, W)

        np.maximum(grid, cost, out=grid)


def main(args=None):
    rclpy.init(args=args)
    node = SocialCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()