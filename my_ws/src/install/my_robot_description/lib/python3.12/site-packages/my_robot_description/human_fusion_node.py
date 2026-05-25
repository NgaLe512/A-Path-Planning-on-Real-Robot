import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
import math


class HumanFusion(Node):
    def __init__(self):
        super().__init__('human_fusion')

        self.bbox_sub = self.create_subscription(
            Int32MultiArray, '/human_bbox', self.bbox_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.pub = self.create_publisher(
            Int32MultiArray, '/human_pose', 10)

        self.ranges = None
        self.angle_min = 0.0
        self.angle_increment = 0.0

        # Smooth trên map frame mới đúng
        self.prev_positions = {}
        self.alpha = 0.7

        # TF để convert base_link → map
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lidar_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def bbox_callback(self, msg):
        if self.ranges is None:
            return

        data = msg.data
        for i in range(0, len(data), 5):
            track_id = data[i]
            x1, y1, x2, y2 = data[i+1], data[i+2], data[i+3], data[i+4]

            cx = int((x1 + x2) / 2)

            fx = 528.433723449707
            cx0 = 320.0
            yaw_offset = -0.05

            left_angle = -math.atan((x1 - cx0) / fx) + yaw_offset
            right_angle = -math.atan((x2 - cx0) / fx) + yaw_offset
            center_angle = -math.atan((cx - cx0) / fx) + yaw_offset

            i1 = int((left_angle - self.angle_min) / self.angle_increment)
            i2 = int((right_angle - self.angle_min) / self.angle_increment)

            i1 = max(0, min(i1, len(self.ranges) - 1))
            i2 = max(0, min(i2, len(self.ranges) - 1))

            if i1 > i2:
                i1, i2 = i2, i1

            window = [
                r for r in self.ranges[i1:i2+1]
                if math.isfinite(r) and r > 0.05
                ]

            if not window:
                continue

            distance = min(window)
            angle = center_angle

            

            # Tọa độ trong frame base_link (mét)
            x_local = distance * math.cos(angle)
            y_local = distance * math.sin(angle)

          
            try:
                point = PointStamped()
                point.header.frame_id = 'base_link'
                point.header.stamp = rclpy.time.Time().to_msg()
                point.point.x = x_local
                point.point.y = y_local
                point.point.z = 0.0

                point_map = self.tf_buffer.transform(
                    point, 'map',
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                x_map = point_map.point.x
                y_map = point_map.point.y

            except Exception as e:
                self.get_logger().warn(f'TF error: {e}', throttle_duration_sec=2.0)
                continue

        
            if track_id in self.prev_positions:
                prev_x, prev_y = self.prev_positions[track_id]
                x_map = self.alpha * prev_x + (1 - self.alpha) * x_map
                y_map = self.alpha * prev_y + (1 - self.alpha) * y_map

            self.prev_positions[track_id] = (x_map, y_map)

            
            msg_out = Int32MultiArray()
            msg_out.data = [track_id, int(x_map * 100), int(y_map * 100)]
            self.pub.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = HumanFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()