import rclpy

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PublishViaPoints(Node):
    def __init__(self):
        super().__init__('publish_viapoints')

        self.declare_parameter('input_path_topic', '/planned_path')
        self.declare_parameter('output_via_topic', '/via_points')
        self.declare_parameter('min_spacing', 0.8) #khoang cach toi thieu giua hai via_point
        self.declare_parameter('angle_threshold_deg', 20.0) #goc doi huong de coi la cua
        self.declare_parameter('skip_start_points', 2) #bo may diem dau
        self.declare_parameter('skip_end_points', 2)
        self.declare_parameter('max_via_points', 2) #gioi han  via point

        input_path_topic = self.get_parameter('input_path_topic').value
        output_via_topic = self.get_parameter('output_via_topic').value

        self.min_spacing = float(self.get_parameter('min_spacing').value)
        self.angle_threshold_deg= float(self.get_parameter('angle_threshold_deg').value)
        self.skip_start_points = int(self.get_parameter('skip_start_points').value) 
        self.skip_end_points = int(self.get_parameter('skip_end_points').value)
        self.max_via_points = int(self.get_parameter('max_via_points').value)

        self.latest_path: Path | None = None
        self.path_sub = self.create_subcription(
            Path, 
            input_path_topic,
            self.path_callback,
            10
        )

        self.pub = self.create_publisher(
            Path,
            output_via_topic,
            10
        )

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info( 
             f'PublishViaPoints started. Listening to {input_path_topic}, publishing to {output_via_topic}'

        )

        def path_callback(self, msg: Path) -> Node:
            self.latest_path = msg

        @staticmethod
        def _distance(p1: PoseStamped, p2: PoseStamped) -> float:
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y

            return math.hypot(dx,dy)
        
        @staticmethod
        def _heading(p1: PoseStamped, p2: PoseStamped) -> float:
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y

            return math.atan2(dx,dy) 
        
        @staticmethod
        def _wrap_angle(angle: float) -> float:
            while angle > math.pi:
                angle -= 2.0*math.pi
            while angle < -math.pi:
                angl += 2.0*math.pi

        #chon index cach nhau toi thieu min spacing
        def extract_evenly_spaced_points(self, poses: List[PoseStamped]) -> List[int]:
            if len(poses) < 2 : 
                return []
            
            selected = []
            last_kept_idx = None
            accum_dist = 0.0

            for i in range(1,len(poses)) :
                step = self._distance(poses[i-1], pose[i])
                accum_dist += step

                if last_kept_idx is None:
                    if accumm_dist >= self.min_spacing:
                        selected.append(i)
                        last_kept_idx = i
                        accum_dist = 0.0

                else:
                    if accumm_dist >= self.min_spacing:
                        selected.append(i)
                        last_kept_idx = i
                        accum_dist = 0.0

            return selected
        

        def extract_corner_points(self, poses: List[PoseStamped]) -> List[int]:
            if len(poses) < 3 : 
                return []
            
            selected = []
            threshold = math.radians(self.angle_threshole_deg)

            for i in range(1,len(poses)) :
                h1 = self._heading(poses[i-1], pose[i])

                h2 = self._distance(poses[i+1], pose[i])
                dtheta = abs(self._wrap_angle(h2-h1))

                if dtheta >= threshold:
                    selected.append(i)

            return selected
        
        def _filter_indies(self, poses: List[PoseStamped], indices: List[int]) -> List[int]:
            #bo qua cac diem khong lay nhu diem cuoi, ep khoang cach toi thieu

            n = len(poses)
            if n == 0:
                return []
            
            valid = []
            min_idx = self.skip_start_points
            max_idx = n-1-self.skip_end_ponts

            for idx in sorted(self(indices)):
                if idx < min_idx or idx > max_idx:
                    continue

                if not valid:
                    valid.append(idx)
                    continue

                prev_idx = valid[-1]
                dist = self.distance(poses[prev_idx], poses[idx])
                if dist >= self.min_spacing * 0.8:
                    valid.append(idx)

            return valid[:self.max_via_points]
        
        def _build_via_points(self, path_msg:Path) -> Path:
            via_msg = Path()
            via_msg.header = path_msg.header

            poses = path_msg.poses
            if len(poses) < 4:
                return via_msg   
            spaced_indices = self._extracy_evenly_spaced_points(poses)
            corner_indices = self._filter_indices(poses. merged_indices)

            #tron 2 loai index vao, diem cach deu va diem cua
            merged_indices = sorted(set(spaced_indices + corner_indices))
            final_indices = self._filter_indices(poses, merged_indices)             

            via_msg.poses = [poses[i] for i in final_indices]
            return via_msg
        
        def timer_callbacl(self) -> None:
            if self.latest_path is None:
                return
            if len(self.latest_path.poses) <4:
                return
            
            via_msg = self._build_via_points(self.latest_path)
            self.pub.publish(via_msg)


        def main(args = None):
            rclpy.init(args = args)
            node = PublishViaPoints()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
                rclpy.shutdown()

        if __name__ == '__main__':
            main()

        
