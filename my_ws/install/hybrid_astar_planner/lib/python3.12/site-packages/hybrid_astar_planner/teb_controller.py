import math
import rclpy
import numpy as np 
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseArray, PoseStamped
from std_msgs.msg import Int32MultiArray

class CVKalman2D:
    def __init__(self,x,y):
        self.x= np.array([[x], [y],[0.0], [0.0]], dtype= float)
        self.P = np.eye(4) * 1.0

        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0]
        ], dtype=float)

        self.R = np.eye(2)*0.05
        self.Q_base = np.eye(4)*0.01

    def predict(self, dt:float):
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]

        ], dtype = float)

        self.x = F @ self.x
        self.P = F@self.P @ F.T + self.Q_base

    def update(self, z_x:float, z_y:float):
        z = np.array([[z_x], [z_y]], dtype=float)

        y= z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K= self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(4)
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        return (
    float(self.x[0, 0]),
    float(self.x[1, 0]),
    float(self.x[2, 0]),
    float(self.x[3, 0]),
)
                     
    def predict_trajectory(self, dt:float, steps:int):
        x, y, vx, vy = self.get_state()
        
        traj = []
        for i in range(1, steps+1):
            px = x + i*dt*vx
            py = y + i*dt*vy
            traj.append((px,py))

        return traj







class TEBController(Node):
    def __init__(self):
        super().__init__('teb_controller')

        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_cb, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10
        )

        self.obstacle_sub = self.create_subscription(
            PoseArray, '/teb_obstacles', self.teb_obstacles_cb, 10
        )

        self.human_pose_sub = self.create_subscription(
            Int32MultiArray, '/human_pose', self.human_pose_cb,10
        )

        self.via_sub = self.create_subscription(
            Path, '/via_points', self.via_points_cb, 10
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap', self.costmap_cb, 10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.debug_path_pub = self.create_publisher(Path, '/teb_debug_path', 10)

        self.path = None
        self.current_pose = None
        self.costmap = None
        self.dynamic_obstacles = []
        self.via_points = []
        self.human_filters = {} #track_ id
        self.human_last_time = {} #track -> time stamp
        self.predicted_human_trajs = {} #
        self.collision_points = [] # x_cc, y_cc
        self.robot_radius = 0.05
        self.person_radius = 0.05
        self.collision_radius = self.robot_radius + self.person_radius

        self.pred_dt = 0.2
        self.pred_steps = 8


        self._last_v = 0.0 #van toc chu ky truoc
        self._last_w = 0.0 #vantoc goc chu ky truoc

        #acceleration limit
        self.MAX_DV = 0.08
        self.MAX_DW = 0.25
        self.BREAK_DISTANCE = 0.6 #bat dau giam toc khi con <0.6 den goal
        self.GOAL_TOL = 0.15 # dung han

        self.prev_best_band = None
        self.prev_best_name = None

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info('TEBController started.')

    def path_cb(self, msg: Path):
        self.path = msg.poses

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def costmap_cb(self, msg: OccupancyGrid):
        self.costmap = msg

    def teb_obstacles_cb(self, msg: PoseArray):
        self.dynamic_obstacles = msg.poses

    def via_points_cb(self, msg: Path):
        self.via_points = msg.poses

    def clone_band(self, band):
        return [dict(pt) for pt in band]

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def wrap_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def extract_local_segment(self, max_points=15):
        if self.path is None or self.current_pose is None:
            return []

        curr_x = self.current_pose.position.x
        curr_y = self.current_pose.position.y

        closest_idx = 0
        min_dist = float('inf')

        for i, pose_stamped in enumerate(self.path):
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - curr_x, py - curr_y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        end_idx = min(len(self.path), closest_idx + max_points)
        return self.path[closest_idx:end_idx]

    def make_shifted_band(self, local_path, lateral_offset=0.0, dt_ref=0.2):
        band = []

        if len(local_path) < 2:
            return band

        for i in range(len(local_path)):
            p = local_path[i].pose.position
            x = p.x
            y = p.y

            if i < len(local_path) - 1:
                p_next = local_path[i + 1].pose.position
                theta = math.atan2(p_next.y - y, p_next.x - x)
            else:
                p_prev = local_path[i - 1].pose.position
                theta = math.atan2(y - p_prev.y, x - p_prev.x)

            nx = -math.sin(theta)
            ny = math.cos(theta)

            x_shifted = x + lateral_offset * nx
            y_shifted = y + lateral_offset * ny

            band.append({
                'x': x_shifted,
                'y': y_shifted,
                'theta': theta,
                'dt': dt_ref
            })

        return band

    def apply_via_point_attraction(self, band, gain=0.18):
        if not self.via_points or len(band) < 2:
            return

        for vp in self.via_points:
            vx = vp.pose.position.x
            vy = vp.pose.position.y

            best_i = None
            best_d = float('inf')

            for i in range(1, len(band) - 1):
                dx = band[i]['x'] - vx
                dy = band[i]['y'] - vy
                d = math.hypot(dx, dy)

                if d < best_d:
                    best_d = d
                    best_i = i

            if best_i is not None:
                band[best_i]['x'] += gain * (vx - band[best_i]['x'])
                band[best_i]['y'] += gain * (vy - band[best_i]['y'])

    def human_pose_cb(self, msg: Int32MultiArray):
        data = msg.data
        if len(data) < 3:
            return

        track_id = int(data[0])
        x = data[1] / 100.0
        y = data[2] / 100.0

        now = self.get_clock().now().nanoseconds / 1e9

        if track_id not in self.human_filters:
            self.human_filters[track_id] = CVKalman2D(x, y)
            self.human_last_time[track_id] = now
            return

        dt = max(now - self.human_last_time[track_id], 0.05)
        self.human_last_time[track_id] = now

        kf = self.human_filters[track_id]
        kf.predict(dt)
        kf.update(x, y)
        self.predicted_human_trajs[track_id] = kf.predict_trajectory(
        self.pred_dt, self.pred_steps
            )
                
        
    
    def apply_path_attraction(self, band, local_path, gain=0.12):
        n = min(len(band), len(local_path))
        for i in range(1, n - 1):
            ref = local_path[i].pose.position
            band[i]['x'] += gain * (ref.x - band[i]['x'])
            band[i]['y'] += gain * (ref.y - band[i]['y'])

    def apply_smoothing(self, band, gain=0.15):
        if len(band) < 3:
            return

        new_xy = []
        for i in range(len(band)):
            if i == 0 or i == len(band) - 1:
                new_xy.append((band[i]['x'], band[i]['y']))
                continue

            x_prev, y_prev = band[i - 1]['x'], band[i - 1]['y']
            x_curr, y_curr = band[i]['x'], band[i]['y']
            x_next, y_next = band[i + 1]['x'], band[i + 1]['y']

            x_smooth = x_curr + gain * (((x_prev + x_next) / 2.0) - x_curr)
            y_smooth = y_curr + gain * (((y_prev + y_next) / 2.0) - y_curr)

            new_xy.append((x_smooth, y_smooth))

        for i in range(len(band)):
            band[i]['x'], band[i]['y'] = new_xy[i]

    def world_to_grid(self, x, y):
        if self.costmap is None:
            return None

        info = self.costmap.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)

        if 0 <= gx < info.width and 0 <= gy < info.height:
            return gx, gy
        return None

    def get_cost(self, x, y):
        if self.costmap is None:
            return 0

        grid = self.world_to_grid(x, y)
        if grid is None:
            return 100

        gx, gy = grid
        idx = gy * self.costmap.info.width + gx
        val = self.costmap.data[idx]

        if val < 0:
            return 100
        return val

    def apply_obstacle_repulsion(self, band, gain=0.003, sample_offset=0.08):
        if self.costmap is None:
            return

        for i in range(1, len(band) - 1):
            x = band[i]['x']
            y = band[i]['y']

            c_center = self.get_cost(x, y)
            if c_center < 20:
                continue

            c_left = self.get_cost(x - sample_offset, y)
            c_right = self.get_cost(x + sample_offset, y)
            c_down = self.get_cost(x, y - sample_offset)
            c_up = self.get_cost(x, y + sample_offset)

            grad_x = float(c_right - c_left)
            grad_y = float(c_up - c_down)

            band[i]['x'] -= gain * grad_x
            band[i]['y'] -= gain * grad_y


        

    def apply_collision_repulsion(self, band, gain = 0.05):
        if not self.collision_points:
            return
        
        for i in range(1, len(band) - 1):
            x = band[i]['x']
            y = band[i]['y']

            for (x_cc, y_cc, r_cc) in self.collision_points:
                dx = x - x_cc
                dy = y- y_cc
                d = math.hypot(dx,dy)

                if d < r_cc + 0.5:
                    if d < 1e-3:
                        continue
                    push = gain * (r_cc + 0.5 - d) / d
                    band[i]['x'] += dx * push
                    band[i]['y'] += dy * push 



    def update_band_theta(self, band):
        if len(band) < 2:
            return

        for i in range(len(band) - 1):
            dx = band[i + 1]['x'] - band[i]['x']
            dy = band[i + 1]['y'] - band[i]['y']
            band[i]['theta'] = math.atan2(dy, dx)

        band[-1]['theta'] = band[-2]['theta']


    def update_predicted_human_trajectories(self):
        self.predicted_human_trajs = {}
        
        for track_id, kf in self.human_filters.items():
            traj =kf.predict_trajectory(self.pred_dt, self.pred_steps)
            self.predicted_human_trajs[track_id] = traj

    def build_robot_future_trajectory(self, band):
        #tra ve danh sach cac diem tuong lai cua robot lay tu band:

        traj = []
        n = min(len(band), self.pred_steps)
        for i in range(n):
            traj.append((band[i]['x'], band[i]['y']))
        return traj
    
    def predicted_collision_checking(self, band):
        self.collision_points = []

        robot_traj = self.build_robot_future_trajectory(band)
        if len(robot_traj) < 2:
            return

        for track_id, human_traj in self.predicted_human_trajs.items():
            n = min(len(robot_traj), len(human_traj))

            for i in range(n):
                rx, ry = robot_traj[i]
                hx, hy = human_traj[i]

                d = math.hypot(rx - hx, ry - hy)

                if d <= self.collision_radius:
                    x_cc = 0.5 * (rx + hx)
                    y_cc = 0.5 * (ry + hy)

                    self.collision_points.append(
                        (x_cc, y_cc, self.collision_radius)
                    )
                    break

                
    def optimize_band(self, band, local_path, iters=8):
        if len(band) < 2:
            return band

        for _ in range(iters):
            self.apply_via_point_attraction(band, gain=0.18)
            self.apply_path_attraction(band, local_path, gain=0.05)
            self.apply_smoothing(band, gain=0.12)
            self.apply_collision_repulsion(band, gain = 0.05)
            self.apply_obstacle_repulsion(band, gain=0.003, sample_offset=0.08)
            self.update_band_theta(band)

        return band

    def compute_band_cost(self, band, local_path):
        if len(band) < 2:
            return float('inf')

        cost = 0.0

        # 1. Lệch khỏi local path
        n = min(len(band), len(local_path))
        for i in range(n):
            ref = local_path[i].pose.position
            dx = band[i]['x'] - ref.x
            dy = band[i]['y'] - ref.y
            cost += 2.0 * (dx * dx + dy * dy)

        # 2. Costmap penalty
        for pt in band:
            c = self.get_cost(pt['x'], pt['y'])
            if c >= 100:
                cost += 500.0
            else:
                cost += 3.0 * c

        # 3. Dynamic obstacle penalty
        for pt in band:
            for obs in self.dynamic_obstacles:
                dx = pt['x'] - obs.position.x
                dy = pt['y'] - obs.position.y
                dist = math.hypot(dx, dy)

                if dist < 0.5:
                    cost += 1000.0
                elif dist < 1.0:
                    cost += 80.0 * (1.0 - dist)

        # 4. Smoothness penalty
        for i in range(1, len(band) - 1):
            x_prev, y_prev = band[i - 1]['x'], band[i - 1]['y']
            x_curr, y_curr = band[i]['x'], band[i]['y']
            x_next, y_next = band[i + 1]['x'], band[i + 1]['y']

            ddx = x_next - 2.0 * x_curr + x_prev
            ddy = y_next - 2.0 * y_curr + y_prev
            cost += 8.0 * (ddx * ddx + ddy * ddy)

        # 5. Via-point penalty
        for vp in self.via_points:
            vx = vp.pose.position.x
            vy = vp.pose.position.y

            best_d = float('inf')
            for pt in band:
                dx = pt['x'] - vx
                dy = pt['y'] - vy
                d = math.hypot(dx, dy)
                if d < best_d:
                    best_d = d

            cost += 5.0 * best_d

        # cpteb collision
        for pt in band:
            for (x_cc, y_cc, r_cc) in self.collision_points:
                d = math.hypot(pt['x']- x_cc, pt['y'] - y_cc)

                if d < r_cc:
                    cost += 2000.0
                elif d < r_cc + 0.5:
                    cost += 150.0*(r_cc + 0.5 -d)

        return cost
    
    @staticmethod
    def _apply_rate(current, target, max_delta):
        delta = target - current
        delta = max(-max_delta, min(max_delta, delta))
        return current + delta

    def band_to_twist(self, band):
        twist = Twist()

        if len(band) < 2 or self.current_pose is None:
            self._last_v = self._apply_rate(self._last_v, 0.0, self.MAX_DV)
            self._last_w = self._apply_rate(self._last_w, 0.0,self.MAX_DW)
            twist.linear.x = self._last_v
            twist.angular.z = self._last_w
            return twist

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = self.yaw_from_quaternion(self.current_pose.orientation)

        target_x = band[1]['x']
        target_y = band[1]['y']
        target_theta = band[1]['theta']
        dt = max(band[0]['dt'], 0.05)

        dx = target_x - robot_x
        dy = target_y - robot_y
        dist = math.hypot(dx, dy)

        #goal dung diem cuoi band
        goal = self.path[-1].pose.position
        
        dist_goal = math.hypot(goal.x - robot_x, goal.y - robot_y)

        #dung han neu da den goal

        if dist_goal < self.GOAL_TOL:
            self._last_v = self._apply_rate(self._last_v, 0.0, self.MAX_DV)
            self._last_w = self._apply_rate(self._last_w, 0.0, self.MAX_DW)
            twist.linear.x  = self._last_v
            twist.angular.z = self._last_w
            return twist

        desired_heading = math.atan2(dy, dx)
        heading_error = self.wrap_angle(desired_heading - robot_yaw)
        theta_error = self.wrap_angle(target_theta - robot_yaw)

        max_v = 0.35
        min_v = 0.03
        max_w = 1.0

        #target v
        if abs(heading_error) > math.pi / 3.0:
            v_target = 0.0
        else:
            heading_fraction = 1.0 - abs(heading_error)/(math.pi/3.0)
            v_target = min(max_v,dist/dt)*heading_fraction
            if v_target > 0.0:
                v_target = max(min_v, v_target)

        
        # Goal proximity scaling — giảm tốc mượt khi gần đích
        if dist_goal < self.BREAK_DISTANCE:
            brake_scale = min(1.0,dist_goal / self.BREAK_DISTANCE)
            # Dùng sqrt để giảm chậm ban đầu, nhanh cuối
            v_target *= math.sqrt(brake_scale)

        # --- Target w ---
        w_target = 1.5 * heading_error + 0.3 * theta_error
        w_target = max(-max_w, min(max_w, w_target))

        # --- Rate limiting — smooth! ---
        self._last_v = self._apply_rate(self._last_v, v_target, self.MAX_DV)
        self._last_w = self._apply_rate(self._last_w, w_target, self.MAX_DW)

        twist.linear.x  = self._last_v
        twist.angular.z = self._last_w
        return twist



    def publish_debug_band(self, band):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for pt in band:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pt['x']
            pose.pose.position.y = pt['y']
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.debug_path_pub.publish(msg)

    def adapt_band_to_new_path(self, band, local_path, gain=0.05):
        """
        Kéo band cũ về gần local_path mới nhưng không reset hoàn toàn.
        """
        if len(band) < 2 or len(local_path) < 2:
            return band

        n = min(len(band), len(local_path))
        for i in range(n):
            ref = local_path[i].pose.position
            band[i]['x'] += gain * (ref.x - band[i]['x'])
            band[i]['y'] += gain * (ref.y - band[i]['y'])

        self.update_band_theta(band)
        return band

    def build_candidate_bands(self, local_path):
        """
        Nếu có prev_best_band thì dùng làm center band khởi tạo.
        """
        if self.prev_best_band is not None and len(self.prev_best_band) >= 2:
            band_center = self.clone_band(self.prev_best_band)
            band_center = self.adapt_band_to_new_path(band_center, local_path, gain=0.15)
        else:
            band_center = self.make_shifted_band(local_path, lateral_offset=0.0, dt_ref=0.2)

        band_left = self.make_shifted_band(local_path, lateral_offset=0.35, dt_ref=0.2)
        band_right = self.make_shifted_band(local_path, lateral_offset=-0.35, dt_ref=0.2)

        return [
            ('center', band_center),
            ('left', band_left),
            ('right', band_right),
        ]

    def control_loop(self):
        if self.path is None or self.current_pose is None:
            return

        local_path = self.extract_local_segment(max_points=15)
        if len(local_path) < 2:
            self._last_v = 0.0
            self._last_w = 0.0
            self.cmd_pub.publish(Twist())
            return
        
        self.update_predicted_human_trajectories()
        
        if self.prev_best_band is not None and len(self.prev_best_band) >= 2:
            initial_band = self.clone_band(self.prev_best_band)
            initial_band = self.adapt_band_to_new_path(initial_band, local_path, gain=0.15)
        else:
            initial_band = self.make_shifted_band(local_path, lateral_offset=0.0, dt_ref=0.2)

        self.predicted_collision_checking(initial_band)

        candidates = self.build_candidate_bands(local_path)
        optimized_candidates = []

        for name, band in candidates:
            if len(band) < 2:
                continue

            band_opt = self.optimize_band(band, local_path, iters=8)
  
            band_cost = self.compute_band_cost(band_opt, local_path)

            # ưu tiên band giữa nhẹ một chút
            if name == 'center':
                band_cost *= 0.98

            optimized_candidates.append((name, band_opt, band_cost))

        if not optimized_candidates:
            self._last_v = 0.0
            self._last_w = 0.0
            self.cmd_pub.publish(Twist())
            return

        best_name, best_band, best_cost = min(
            optimized_candidates,
            key=lambda x: x[2]
        )

        # lưu lại để warm-start chu kỳ sau
        self.prev_best_band = self.clone_band(best_band)
        self.prev_best_name = best_name

        self.publish_debug_band(best_band)
        self.get_logger().info(
            f'Selected band: {best_name}, cost={best_cost:.2f}',
            throttle_duration_sec=1.0
        )

        twist = self.band_to_twist(best_band)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TEBController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()