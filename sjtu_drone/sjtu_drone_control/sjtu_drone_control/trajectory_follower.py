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


def quintic_coeff(p0, v0, a0, p1, v1, a1, T):
    """Coeficientes para p(t)=c0+c1 t+c2 t^2+c3 t^3+c4 t^4+c5 t^5 con BC en 0 y T."""
    c0 = p0
    c1 = v0
    c2 = a0 / 2.0
    T2 = T * T
    T3 = T2 * T
    T4 = T3 * T
    T5 = T4 * T

    c3 = (20*(p1-p0) - (8*v1 + 12*v0)*T - (3*a0 - a1)*T2) / (2*T3)
    c4 = (30*(p0-p1) + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T2) / (2*T4)
    c5 = (12*(p1-p0) - (6*v1 + 6*v0)*T - (a0 - a1)*T2) / (2*T5)
    return (c0, c1, c2, c3, c4, c5)


def eval_quintic(c, t):
    c0, c1, c2, c3, c4, c5 = c
    return c0 + c1*t + c2*t*t + c3*t**3 + c4*t**4 + c5*t**5


class SimpleDroneTrajectoryFollower(Node):
    """
    sjtu_drone (modo posctrl):
      - /simple_drone/posctrl (Bool) True
      - /simple_drone/cmd_vel (Twist): linear.{x,y,z} = objetivo de posición, angular.z = yaw objetivo (si lo usas)
      - /simple_drone/gt_pose (Pose): real

    Esta versión interpola con polinomios de 5º grado (minimum jerk) entre waypoints.
    """

    def __init__(self):
        super().__init__("trajectory_follower_quintic")

        # Publishers
        self.pub_posctrl = self.create_publisher(Bool, "/simple_drone/posctrl", 10)
        self.pub_cmd = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)
        self.pub_takeoff = self.create_publisher(Empty, "/simple_drone/takeoff", 10)
        self.pub_land = self.create_publisher(Empty, "/simple_drone/land", 10)

        # Subscriber
        self.sub_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self._cb_pose, 10)
        self.current_pose: Optional[Pose] = None

        # RViz
        self.pub_path = self.create_publisher(Path, "/simple_drone/trajectory_path", 10)
        self.pub_marker = self.create_publisher(Marker, "/simple_drone/trajectory_marker", 10)

        # Parámetros (recomendación: más tasa para setpoints suaves)
        self.rate_hz = 50.0

        # “Velocidad” de referencia para fijar duración del segmento (m/s aprox)
        self.v_ref = 1.5
        self.T_min = 1.0

        # Hold opcional al final de cada segmento
        self.hold_seconds = 0.3
        self.hold_cycles = max(1, int(self.hold_seconds * self.rate_hz))
        self._holding = False
        self._hold_counter = 0

        # Waypoints
        pA = (-3.397180, -10.136400, 7.0)
        pB = (3.0, 5.0, 4.0)
        pC = (6.0, 0., 9.0)
        pD = (-9.971850, 3.162810, 7.0)
        pE = (-13.0, 5.0, 8.0)
        pF = (0.0, 0.0, 5.0)

        self.waypoints: List[Tuple[float, float, float]] = [pA, pB, pC, pD, pE, pF]

        # Segmento actual: i -> i+1
        self.seg_i = 0
        self.seg_T = 2.0
        self.seg_t0 = self._now_sec()
        self.cx = self.cy = self.cz = None

        # CSV logging
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.csv_path = os.path.join(script_dir, "trajectory_errors.csv")
        self._csv_fh = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._csv = csv.writer(self._csv_fh)
        self._csv.writerow([
            "t_sec",
            "seg_i",
            "target_x", "target_y", "target_z",
            "pose_x", "pose_y", "pose_z",
            "err_x", "err_y", "err_z",
            "err_norm",
        ])
        self._t0_ns = self.get_clock().now().nanoseconds

        # RViz msgs base
        self.frame_id = "world"
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = self.frame_id
        self.marker_msg.ns = "traj_exec"
        self.marker_msg.id = 0
        self.marker_msg.type = Marker.LINE_STRIP
        self.marker_msg.action = Marker.ADD
        self.marker_msg.scale.x = 0.05
        self.marker_msg.color.r = 1.0
        self.marker_msg.color.g = 0.2
        self.marker_msg.color.b = 0.2
        self.marker_msg.color.a = 1.0
        self.marker_msg.pose.orientation.w = 1.0

        # Activa posctrl y despega
        self._publish_posctrl(True)
        self.pub_takeoff.publish(Empty())

        # Publica “plan” (solo waypoints)
        self._publish_planned_trajectory()

        # Inicializa primer segmento
        self._start_segment(0)

        # Timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            f"Quintic follower listo. rate={self.rate_hz}Hz, v_ref={self.v_ref}m/s, "
            f"CSV: {self.csv_path}"
        )

    # ---------- Utils ----------
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
        # Congela yaw:
        t.angular.z = 0.0
        self.pub_cmd.publish(t)

    def _log_error_row(self, seg_i: int, tx: float, ty: float, tz: float):
        if self.current_pose is None:
            return

        px = float(self.current_pose.position.x)
        py = float(self.current_pose.position.y)
        pz = float(self.current_pose.position.z)

        ex = px - float(tx)
        ey = py - float(ty)
        ez = pz - float(tz)
        en = math.sqrt(ex * ex + ey * ey + ez * ez)

        t_ns = self.get_clock().now().nanoseconds
        t_sec = (t_ns - self._t0_ns) * 1e-9

        self._csv.writerow([t_sec, seg_i, tx, ty, tz, px, py, pz, ex, ey, ez, en])
        self._csv_fh.flush()

    # ---------- RViz ----------
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
        marker.scale.x = 0.03
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

    # ---------- Quintic segments ----------
    def _start_segment(self, i: int):
        n = len(self.waypoints)
        i0 = i % n
        i1 = (i0 + 1) % n

        x0, y0, z0 = self.waypoints[i0]
        x1, y1, z1 = self.waypoints[i1]

        dx, dy, dz = (x1 - x0), (y1 - y0), (z1 - z0)
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        T = max(self.T_min, dist / max(1e-6, self.v_ref))
        self.seg_i = i0
        self.seg_T = T
        self.seg_t0 = self._now_sec()

        # BC “suaves”: v=a=0 en extremos
        self.cx = quintic_coeff(x0, 0.0, 0.0, x1, 0.0, 0.0, T)
        self.cy = quintic_coeff(y0, 0.0, 0.0, y1, 0.0, 0.0, T)
        self.cz = quintic_coeff(z0, 0.0, 0.0, z1, 0.0, 0.0, T)

        self.get_logger().info(
            f"Nuevo segmento {i0}->{i1} (T={T:.2f}s, dist={dist:.2f}m)"
        )

    # ---------- Control loop ----------
    def _tick(self):
        # Mantén posctrl
        self._publish_posctrl(True)

        if self.current_pose is None:
            self.get_logger().info("Aún no llega /simple_drone/gt_pose...", throttle_duration_sec=1.0)
            return

        n = len(self.waypoints)
        i0 = self.seg_i
        i1 = (i0 + 1) % n
        x1, y1, z1 = self.waypoints[i1]

        # Si estamos en hold al final del segmento
        if self._holding:
            self._publish_goal(x1, y1, z1)
            self._log_error_row(i0, x1, y1, z1)
            self._append_executed_point(x1, y1, z1)

            self._hold_counter += 1
            if self._hold_counter >= self.hold_cycles:
                self._holding = False
                self._hold_counter = 0
                self._start_segment(i1)
            return

        # Evalúa quintic en el tiempo del segmento
        t = self._now_sec() - self.seg_t0

        if t >= self.seg_T:
            # Clava final y entra en hold
            self._publish_goal(x1, y1, z1)
            self._log_error_row(i0, x1, y1, z1)
            self._append_executed_point(x1, y1, z1)
            self._holding = True
            self._hold_counter = 0
            return

        tx = eval_quintic(self.cx, t)
        ty = eval_quintic(self.cy, t)
        tz = eval_quintic(self.cz, t)

        self._publish_goal(tx, ty, tz)
        self._log_error_row(i0, tx, ty, tz)
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

