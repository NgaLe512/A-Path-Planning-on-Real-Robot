import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Pose, PoseArray


class HumanToTEBObstacles(Node):
    def __init__(self):
        super().__init__('human_to_teb_obstacles')

        self.sub = self.create_subscription(
            Int32MultiArray,
            '/human_pose',
            self.human_callback,
            10
        )

        self.pub = self.create_publisher(
            PoseArray,
            '/teb_obstacles',
            10
        )

        # {track_id: (x, y, t_last)}
        self.humans = {}

        self.declare_parameter('human_timeout', 1.0)
        self.human_timeout = float(self.get_parameter('human_timeout').value)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('human_to_teb_obstacles started.')

    def human_callback(self, msg: Int32MultiArray):
        data = msg.data
        if len(data) < 3:
            return

        track_id = int(data[0])
        x = data[1] / 100.0
        y = data[2] / 100.0
        now = self.get_clock().now().nanoseconds / 1e9

        self.humans[track_id] = (x, y, now)

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        expired_ids = [
            track_id
            for track_id, (_, _, t_last) in self.humans.items()
            if now - t_last > self.human_timeout
        ]

        for track_id in expired_ids:
            del self.humans[track_id]

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for track_id, (x, y, _) in self.humans.items():
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanToTEBObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()