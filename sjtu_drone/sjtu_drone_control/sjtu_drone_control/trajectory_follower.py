#!/usr/bin/env python3
import math
import csv
import os
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String


def quintic_coeff(p0, v0, a0, p1, v1, a1, T):
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T
    c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3)
    c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T4)
    c5 = (12 * (p1 - p0) - (6 * v1 + 6 * v0) * T - (a0 - a1) * T2) / (2 * T5)
    return (c0, c1, c2, c3, c4, c5)


def eval_quintic(c, t):
    c0, c1, c2, c3, c4, c5 = c
    return c0 + c1 * t + c2 * t * t + c3 * t**3 + c4 * t**4 + c5 * t**5


def eval_quintic_derivative(c, t):
    c0, c1, c2, c3, c4, c5 = c
    return c1 + 2.0 * c2 * t + 3.0 * c3 * t**2 + 4.0 * c4 * t**3 + 5.0 * c5 * t**4


def eval_quintic_second_derivative(c, t):
    c0, c1, c2, c3, c4, c5 = c
    return 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t**2 + 20.0 * c5 * t**3


def catmull_rom_point(p0, p1, p2, p3, t):
    t2 = t * t
    t3 = t2 * t
    a0 = -0.5 * t3 + t2 - 0.5 * t
    a1 = 1.5 * t3 - 2.5 * t2 + 1.0
    a2 = -1.5 * t3 + 2.0 * t2 + 0.5 * t
    a3 = 0.5 * t3 - 0.5 * t2
    x = a0 * p0[0] + a1 * p1[0] + a2 * p2[0] + a3 * p3[0]
    y = a0 * p0[1] + a1 * p1[1] + a2 * p2[1] + a3 * p3[1]
    z = a0 * p0[2] + a1 * p1[2] + a2 * p2[2] + a3 * p3[2]
    return (x, y, z)


def sample_catmull_rom(control_points: List[Tuple[float, float, float]], samples_per_segment: int) -> List[Tuple[float, float, float]]:
    if len(control_points) < 2:
        return control_points[:]
    pts = [control_points[0]] + control_points + [control_points[-1]]
    out: List[Tuple[float, float, float]] = []
    for i in range(0, len(pts) - 3):
        p0, p1, p2, p3 = pts[i], pts[i + 1], pts[i + 2], pts[i + 3]
        for s in range(samples_per_segment):
            t = s / float(samples_per_segment)
            out.append(catmull_rom_point(p0, p1, p2, p3, t))
    out.append(control_points[-1])
    return out


def _lin_interp(a: Tuple[float, float, float], b: Tuple[float, float, float], t: float):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t)


def densify_waypoints(waypoints: List[Tuple[float, float, float]], max_seg_len: float) -> List[Tuple[float, float, float]]:
    out: List[Tuple[float, float, float]] = []
    for i in range(len(waypoints) - 1):
        p0 = waypoints[i]
        p1 = waypoints[i + 1]
        out.append(p0)
        dx = p1[0] - p0[0]; dy = p1[1] - p0[1]; dz = p1[2] - p0[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist > max_seg_len:
            n = int(math.ceil(dist / max_seg_len))
            for k in range(1, n):
                tt = k / float(n)
                out.append(_lin_interp(p0, p1, tt))
    out.append(waypoints[-1])
    return out


# -----------------------------
# Path-following error helpers
# -----------------------------
def closest_point_on_segment_3d(p, a, b):
    ax, ay, az = a
    bx, by, bz = b
    px, py, pz = p
    abx, aby, abz = (bx - ax), (by - ay), (bz - az)
    apx, apy, apz = (px - ax), (py - ay), (pz - az)
    ab2 = abx*abx + aby*aby + abz*abz
    if ab2 < 1e-12:
        dx, dy, dz = (px - ax), (py - ay), (pz - az)
        return (a, 0.0, dx*dx + dy*dy + dz*dz)
    t = (apx*abx + apy*aby + apz*abz) / ab2
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    qx = ax + t*abx
    qy = ay + t*aby
    qz = az + t*abz
    dx, dy, dz = (px - qx), (py - qy), (pz - qz)
    return ((qx, qy, qz), t, dx*dx + dy*dy + dz*dz)


def closest_point_on_polyline_3d(p, pts, hint_i=0, window=250):
    n = len(pts)
    if n < 2:
        q = pts[0] if n == 1 else (0.0, 0.0, 0.0)
        return (q, 0, 0.0, float("inf"))

    i0 = max(0, hint_i - window)
    i1 = min(n - 2, hint_i + window)

    best_d2 = float("inf")
    best_q = pts[0]
    best_i = 0
    best_t = 0.0

    for i in range(i0, i1 + 1):
        q, tt, d2 = closest_point_on_segment_3d(p, pts[i], pts[i + 1])
        if d2 < best_d2:
            best_d2 = d2
            best_q = q
            best_i = i
            best_t = tt

    return (best_q, best_i, best_t, math.sqrt(best_d2))


def sample_quintic_segment_1d(c, T, dt):
    pts = []
    t = 0.0
    if dt <= 0.0:
        dt = T
    while t < T:
        pts.append(eval_quintic(c, t))
        t += dt
    pts.append(eval_quintic(c, T))
    return pts


class SimpleDroneTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("trajectory_follower")
        
        # Parameters
        self.declare_parameter("mode","lqr") # "lqr" o "pid"
        self.mode = self.get_parameter("mode").value

        # Subscriptions/Publishers
        self.pub_posctrl = self.create_publisher(Bool, "/simple_drone/posctrl", 10)
        self.pub_mode = self.create_publisher(String, "/simple_drone/control_mode", 10)
        self.pub_cmd = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        self.pub_ref = self.create_publisher(Odometry, "/simple_drone/ref_odom", 10)
        self.pub_takeoff = self.create_publisher(Empty, "/simple_drone/takeoff", 10)
        self.pub_land = self.create_publisher(Empty, "/simple_drone/land", 10)
        self.sub_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self._cb_pose, 10)
        self.current_pose: Optional[Pose] = None

        # RViz
        self.pub_path = self.create_publisher(Path, "/simple_drone/trajectory_path", 10)
        self.pub_marker = self.create_publisher(Marker, "/simple_drone/trajectory_marker", 10)
        
        self._publish_posctrl(self.mode == "pid")
        self._publish_mode()
        self.add_on_set_parameters_callback(self._on_params)
        
        # -------------------------
        # Parámetros
        # -------------------------
        self.rate_hz = 160.0
        self.v_ref = 2.0
        self.T_min = 0.6
        max_seg_len = 0.5
        v_scale_for_wp = 0.65
        alpha_T_filter = 0.6
        self.speed_scale = 0.75

        # Ajuste de aproximación
        approach_offset = 0.6
        approach_right_offset_A = 0.5
        approach_right_offset_B = 1.1

        # Seguridad al final
        self.land_at_end = False
        self.end_hold_seconds = 2.0
        self._ended = False
        self._end_hold_counter = 0
        self._end_hold_cycles = max(1, int(self.end_hold_seconds * self.rate_hz))

        # Waypoints base
        start = (0.0, 0.0, 7.0)
        pA_center = (-3.397180, -10.136400, 7.0)
        pB_center = (-9.971850, 3.162810, 7.0)
        pA_exit = (-1.5, -7.5, 7.1)
        pB_exit = (-11.5, 5.5, 7.2)
        p_end = (-13.0, 8.0, 8.5)

        # Vector helpers (locales)
        def vec_sub(a, b):
            return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

        def vec_add(a, b):
            return (a[0] + b[0], a[1] + b[1], a[2] + b[2])

        def vec_scale(a, s):
            return (a[0] * s, a[1] * s, a[2] * s)

        def norm(v):
            return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

        def normalize(v):
            n = max(1e-9, norm(v))
            return (v[0] / n, v[1] / n, v[2] / n)

        def cross(a, b):
            return (a[1] * b[2] - a[2] * b[1],
                    a[2] * b[0] - a[0] * b[2],
                    a[0] * b[1] - a[1] * b[0])

        # Compute approach points
        dir_to_A = normalize(vec_sub(pA_center, start))
        up = (0.0, 0.0, 1.0)
        right_A = normalize(cross(dir_to_A, up))
        pA_approach = vec_add(vec_sub(pA_center, vec_scale(dir_to_A, approach_offset)),
                              vec_scale(right_A, approach_right_offset_A))

        dir_to_B = normalize(vec_sub(pB_center, pA_exit))
        right_B = normalize(cross(dir_to_B, up))
        pB_approach = vec_add(vec_sub(pB_center, vec_scale(dir_to_B, approach_offset)),
                              vec_scale(right_B, approach_right_offset_B))

        # Build smooth control points and sample Catmull-Rom curve
        control_points: List[Tuple[float, float, float]] = [
            start,
            pA_approach,
            pA_center,
            pA_exit,
            pB_approach,
            pB_center,
            pB_exit,
            p_end
        ]

        samples_per_segment = 20
        curve_points = sample_catmull_rom(control_points, samples_per_segment)

        # Helix region between pA_exit and pB_center
        def lerp(a, b, t):
            return (a[0] + (b[0] - a[0]) * t,
                    a[1] + (b[1] - a[1]) * t,
                    a[2] + (b[2] - a[2]) * t)

        s0 = 0.35
        s1 = 0.75
        radius = 2.0
        turns = 3
        pts_per_turn = 24
        dz_total = 1.2

        d = (pB_center[0] - pA_exit[0], pB_center[1] - pA_exit[1], pB_center[2] - pA_exit[2])
        d_hat = normalize(d)
        w = (0.0, 0.0, 1.0)
        if abs(d_hat[2]) > 0.95:
            w = (1.0, 0.0, 0.0)
        u = normalize(cross(d_hat, w))
        v = cross(d_hat, u)

        helix_samples: List[Tuple[float, float, float]] = []
        N = turns * pts_per_turn
        for i in range(N + 1):
            tt = i / N
            s = s0 + (s1 - s0) * tt
            angle = 2.0 * math.pi * turns * tt
            cx_, cy_, cz_ = lerp(pA_exit, pB_center, s)
            ox = radius * (math.cos(angle) * u[0] + math.sin(angle) * v[0])
            oy = radius * (math.cos(angle) * u[1] + math.sin(angle) * v[1])
            oz = radius * (math.cos(angle) * u[2] + math.sin(angle) * v[2])
            upz = dz_total * tt
            helix_samples.append((cx_ + ox, cy_ + oy, cz_ + oz + upz))

        # Merge helix into curve_points replacing the segment between pA_exit and pB_center
        def find_nearest_index(points, target):
            best_i = 0
            best_d = float("inf")
            for i, p in enumerate(points):
                dx = p[0] - target[0]; dy = p[1] - target[1]; dz = p[2] - target[2]
                dd = dx*dx + dy*dy + dz*dz
                if dd < best_d:
                    best_d = dd
                    best_i = i
            return best_i

        i_start = find_nearest_index(curve_points, pA_exit)
        i_end = find_nearest_index(curve_points, pB_center)
        if i_start < i_end:
            curve_points = curve_points[:i_start] + helix_samples + curve_points[i_end + 1:]

        # Densify final waypoints
        self.waypoints: List[Tuple[float, float, float]] = densify_waypoints(curve_points, max_seg_len)

        # Hints
        self._nearest_seg_hint = 0
        self._ref_nearest_seg_hint = 0

        # Precompute times and waypoint derivatives
        self.seg_T_est: List[float] = []
        self.wp_v: List[Tuple[float, float, float]] = []
        self.wp_a: List[Tuple[float, float, float]] = []
        self._precompute_segment_times_and_waypoint_derivatives(v_scale=v_scale_for_wp, alpha_T_filter=alpha_T_filter)

        # Referencia continua muestreada al inicio (para el cálculo de error)
        self._build_reference_trajectory(dt=1.0 / 80.0)

        # Segment state
        self.seg_i = 0
        self.seg_T = 2.0
        self.seg_t0 = self._now_sec()
        self.cx = self.cy = self.cz = None

        # CSV logging
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.csv_path = os.path.join(script_dir, "trajectory_errors.csv")
        self._csv_fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_fh)
        self._csv.writerow(
            [
                "t_sec",
                "pose_x", "pose_y", "pose_z",
                "closest_x", "closest_y", "closest_z",
                "err_x", "err_y", "err_z", "err_norm",
                "closest_seg_i", "closest_seg_t"
            ]
        )
        self._t0_ns = self.get_clock().now().nanoseconds

        # RViz base messages
        self.frame_id = "world"
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = self.frame_id
        self.marker_msg.ns = "traj_exec"
        self.marker_msg.id = 0
        self.marker_msg.type = Marker.LINE_STRIP
        self.marker_msg.action = Marker.ADD
        self.marker_msg.scale.x = 0.03
        self.marker_msg.color.r = 1.0
        self.marker_msg.color.g = 0.2
        self.marker_msg.color.b = 0.2
        self.marker_msg.color.a = 1.0
        self.marker_msg.pose.orientation.w = 1.0

        # takeoff
        self.pub_takeoff.publish(Empty())

        # Publish planned path and start first segment
        self._publish_planned_trajectory()
        self._start_segment(0)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"Quintic speed-scaled follower listo. waypoints={len(self.waypoints)}, rate={self.rate_hz}Hz"
        )

    def _build_reference_trajectory(self, dt: float = 0.02):
        n = len(self.waypoints)
        if n < 2:
            self.ref_traj_pts = self.waypoints[:]
            self._ref_nearest_seg_hint = 0
            return

        ref_pts: List[Tuple[float, float, float]] = []

        v0 = self.wp_v[0]
        a0 = self.wp_a[0]

        for i0 in range(n - 1):
            i1 = i0 + 1
            x0, y0, z0 = self.waypoints[i0]
            x1, y1, z1 = self.waypoints[i1]

            T_raw = self.seg_T_est[i0] if i0 < len(self.seg_T_est) else self.T_min
            T = max(0.05, T_raw * getattr(self, "speed_scale", 1.0))

            if i1 == n - 1:
                v1 = (0.0, 0.0, 0.0)
                a1 = (0.0, 0.0, 0.0)
            else:
                v1 = self.wp_v[i1]
                a1 = self.wp_a[i1]

            cx = quintic_coeff(x0, v0[0], a0[0], x1, v1[0], a1[0], T)
            cy = quintic_coeff(y0, v0[1], a0[1], y1, v1[1], a1[1], T)
            cz = quintic_coeff(z0, v0[2], a0[2], z1, v1[2], a1[2], T)

            xs = sample_quintic_segment_1d(cx, T, dt)
            ys = sample_quintic_segment_1d(cy, T, dt)
            zs = sample_quintic_segment_1d(cz, T, dt)

            seg_pts = list(zip(xs, ys, zs))
            if ref_pts and seg_pts:
                seg_pts = seg_pts[1:]
            ref_pts.extend(seg_pts)

            v0 = (
                eval_quintic_derivative(cx, T),
                eval_quintic_derivative(cy, T),
                eval_quintic_derivative(cz, T),
            )
            a0 = (
                eval_quintic_second_derivative(cx, T),
                eval_quintic_second_derivative(cy, T),
                eval_quintic_second_derivative(cz, T),
            )

        self.ref_traj_pts = ref_pts
        self._ref_nearest_seg_hint = 0
        self.get_logger().info(f"Referencia muestreada: {len(self.ref_traj_pts)} puntos (dt={dt:.3f}s)")

    # Utilities
    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cb_pose(self, msg: Pose):
        self.current_pose = msg

    def _publish_posctrl(self, enabled: bool):
        m = Bool()
        m.data = enabled
        self.pub_posctrl.publish(m)

    def _publish_goal(self, x: float, y: float, z: float):
        t = Twist()
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.linear.z = float(z)
        t.angular.z = 0.0
        self.pub_cmd.publish(t)

    def _log_error_row(self):
        if self.current_pose is None:
            return

        px = float(self.current_pose.position.x)
        py = float(self.current_pose.position.y)
        pz = float(self.current_pose.position.z)

        pts_ref = getattr(self, "ref_traj_pts", None) or self.waypoints
        q, seg_i, seg_t, dist = closest_point_on_polyline_3d(
            (px, py, pz),
            pts_ref,
            hint_i=getattr(self, "_ref_nearest_seg_hint", 0),
            window=400
        )
        self._ref_nearest_seg_hint = seg_i

        qx, qy, qz = q
        ex = px - qx
        ey = py - qy
        ez = pz - qz
        en = dist

        t_ns = self.get_clock().now().nanoseconds
        t_sec = (t_ns - self._t0_ns) * 1e-9

        self._csv.writerow([t_sec, px, py, pz, qx, qy, qz, ex, ey, ez, en, seg_i, seg_t])
        self._csv_fh.flush()

    # RViz publishing
    def _publish_planned_trajectory(self):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = self.frame_id
        marker = Marker()
        marker.header.stamp = now
        marker.header.frame_id = self.frame_id
        marker.ns = "traj_planned"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for (x, y, z) in self.waypoints:
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
            p = Point()
            p.x, p.y, p.z = x, y, z
            marker.points.append(p)
        self.pub_path.publish(path)
        self.pub_marker.publish(marker)
    
    def _publish_mode(self):
        msg = String()
        msg.data = str(self.mode)
        self.pub_mode.publish(msg)

    def _publish_reference(self, x: float, y: float, z:float, vx: float, vy: float, vz: float, ax: float, ay: float, az: float):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)

        msg.pose.pose.orientation.w = 1.0

        # v_ref in twist.linear (world)
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = float(vz)

        # a_ref (TEMPORAL)
        msg.twist.twist.angular.x = float(ax)
        msg.twist.twist.angular.y = float(ay)
        msg.twist.twist.angular.z = float(az)

        self.pub_ref.publish(msg)
        
    def _on_params(self, params):
        for p in params:
            if p.name == "mode":
                if p.value not in ("pid", "lqr"):
                    return SetParametersResult(successful=False, reason="mode must be 'pid' or 'lqr'")
                if p.value != self.mode:
                    self.mode = p.value
                    self._publish_posctrl(self.mode == "pid")
                    self._publish_mode()
                    self.get_logger().info(f"Mode changed to: {self.mode}")
        return SetParametersResult(successful=True)

    def _append_executed_point(self, x: float, y: float, z: float):
        now = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        self.path_msg.header.stamp = now
        self.path_msg.poses.append(ps)
        self.pub_path.publish(self.path_msg)
        p = Point()
        p.x, p.y, p.z = x, y, z
        self.marker_msg.header.stamp = now
        self.marker_msg.points.append(p)
        self.pub_marker.publish(self.marker_msg)

    # Precompute times and waypoint derivatives
    def _precompute_segment_times_and_waypoint_derivatives(self, v_scale: float = 1.0, alpha_T_filter: float = 0.0):
        n = len(self.waypoints)
        if n < 2:
            self.seg_T_est = []
            self.wp_v = [(0.0, 0.0, 0.0) for _ in range(n)]
            self.wp_a = [(0.0, 0.0, 0.0) for _ in range(n)]
            return

        self.seg_T_est = []
        for i in range(n - 1):
            x0, y0, z0 = self.waypoints[i]
            x1, y1, z1 = self.waypoints[i + 1]
            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            T = max(self.T_min, dist / max(1e-6, self.v_ref))
            self.seg_T_est.append(T)

        if alpha_T_filter > 0.0:
            for i in range(1, len(self.seg_T_est)):
                self.seg_T_est[i] = alpha_T_filter * self.seg_T_est[i - 1] + (1.0 - alpha_T_filter) * self.seg_T_est[i]

        self.wp_a = [(0.0, 0.0, 0.0) for _ in range(n)]
        self.wp_v = [(0.0, 0.0, 0.0) for _ in range(n)]
        for i in range(1, n - 1):
            xm, ym, zm = self.waypoints[i - 1]
            xp, yp, zp = self.waypoints[i + 1]
            T_prev = self.seg_T_est[i - 1]
            T_next = self.seg_T_est[i]
            denom = max(1e-6, (T_prev + T_next))
            vx = v_scale * (xp - xm) / denom
            vy = v_scale * (yp - ym) / denom
            vz = v_scale * (zp - zm) / denom
            self.wp_v[i] = (vx, vy, vz)

        self.get_logger().info(f"Derivadas: v=a=0 en extremos; v intermedias con v_scale={v_scale:.2f}")

    # Start segment with speed_scale applied and safety at end
    def _start_segment(self, i: int, init_v: Optional[Tuple[float, float, float]] = None,
                       init_a: Optional[Tuple[float, float, float]] = None):
        n = len(self.waypoints)
        i0 = i
        i1 = i0 + 1
        if i1 >= n:
            self._ended = True
            self.get_logger().info("Trayectoria finalizada (no hay segmento siguiente).")
            return

        x0, y0, z0 = self.waypoints[i0]
        x1, y1, z1 = self.waypoints[i1]
        dx, dy, dz = (x1 - x0), (y1 - y0), (z1 - z0)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        T_raw = max(self.T_min, dist / max(1e-6, self.v_ref))
        T = max(0.05, T_raw * getattr(self, "speed_scale", 1.0))

        self.seg_i = i0
        self.seg_T = T
        self.seg_t0 = self._now_sec()

        if init_v is not None:
            vx0, vy0, vz0 = init_v
        else:
            vx0, vy0, vz0 = self.wp_v[i0]

        if init_a is not None:
            ax0, ay0, az0 = init_a
        else:
            ax0, ay0, az0 = self.wp_a[i0]

        if i1 == n - 1:
            vx1, vy1, vz1 = (0.0, 0.0, 0.0)
            ax1, ay1, az1 = (0.0, 0.0, 0.0)
        else:
            vx1, vy1, vz1 = self.wp_v[i1]
            ax1, ay1, az1 = self.wp_a[i1]

        self.cx = quintic_coeff(x0, vx0, ax0, x1, vx1, ax1, T)
        self.cy = quintic_coeff(y0, vy0, ay0, y1, vy1, ay1, T)
        self.cz = quintic_coeff(z0, vz0, az0, z1, vz1, az1, T)

        self.get_logger().info(f"Nuevo segmento {i0}->{i1} (T_raw={T_raw:.2f}s, T_scaled={T:.2f}s, dist={dist:.2f}m)")

    # Control loop continuous
    def _tick(self):

        if self.current_pose is None:
            self.get_logger().info("Aún no llega /simple_drone/gt_pose...", throttle_duration_sec=1.0)
            return

        if getattr(self, "_ended", False):
            xL, yL, zL = self.waypoints[-1]
            if self.mode == "pid":
                self._publish_goal(xL, yL, zL)
            else:
                self._publish_reference(xL, yL, zL, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            
            self._log_error_row()
            self._append_executed_point(xL, yL, zL)
            if self.land_at_end:
                self._end_hold_counter += 1
                if self._end_hold_counter >= self._end_hold_cycles:
                    try:
                        self.pub_land.publish(Empty())
                    except Exception:
                        pass
                    self.land_at_end = False
            return

        n = len(self.waypoints)
        i0 = self.seg_i
        i1 = i0 + 1
        if i1 >= n:
            self._ended = True
            self.get_logger().info("No hay siguiente segmento: marcando trayectoria finalizada.")
            return

        x1, y1, z1 = self.waypoints[i1]
        t = self._now_sec() - self.seg_t0

        if t >= self.seg_T:
        
            if self.mode == "pid":
                self._publish_goal(x1, y1, z1)
            else:
                self._publish_reference(x1, y1, z1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            
            self._log_error_row()
            self._append_executed_point(x1, y1, z1)

            if i1 == n - 1:
                self._ended = True
                self.get_logger().info("Último segmento completado. Manteniendo posición final.")
                return

            v_end_x = eval_quintic_derivative(self.cx, self.seg_T)
            v_end_y = eval_quintic_derivative(self.cy, self.seg_T)
            v_end_z = eval_quintic_derivative(self.cz, self.seg_T)
            a_end_x = eval_quintic_second_derivative(self.cx, self.seg_T)
            a_end_y = eval_quintic_second_derivative(self.cy, self.seg_T)
            a_end_z = eval_quintic_second_derivative(self.cz, self.seg_T)
            self._start_segment(i1, init_v=(v_end_x, v_end_y, v_end_z), init_a=(a_end_x, a_end_y, a_end_z))
            return

        tx = eval_quintic(self.cx, t)
        ty = eval_quintic(self.cy, t)
        tz = eval_quintic(self.cz, t)

        if self.mode == "pid":
            self._publish_goal(tx, ty, tz)
        else:
            # Referencia LQR
            vx = eval_quintic_derivative(self.cx, t)
            vy = eval_quintic_derivative(self.cy, t)
            vz = eval_quintic_derivative(self.cz, t)

            ax = eval_quintic_second_derivative(self.cx, t)
            ay = eval_quintic_second_derivative(self.cy, t)
            az = eval_quintic_second_derivative(self.cz, t)

            self._publish_reference(tx, ty, tz, vx, vy, vz, ax, ay, az)

        self._log_error_row()
        self._append_executed_point(tx, ty, tz)

    def destroy_node(self):
        try:
            if getattr(self, "_csv_fh", None) is not None:
                self._csv_fh.flush()
                self._csv_fh.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDroneTrajectoryFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub_land.publish(Empty())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
