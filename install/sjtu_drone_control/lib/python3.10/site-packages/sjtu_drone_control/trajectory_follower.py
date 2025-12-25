#!/usr/bin/env python3
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import Twist, Pose, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class SimpleDroneTrajectoryFollower(Node):
    """
    Publica:
      - /simple_drone/posctrl  (std_msgs/Bool)  -> True
      - /simple_drone/cmd_vel  (geometry_msgs/Twist) con linear.{x,y,z} = objetivo (modo posctrl)
    y usa:
      - /simple_drone/gt_pose (geometry_msgs/Pose) para decidir cuándo pasar al siguiente punto.

    Nota (sjtu_drone):
      - posctrl=True: cmd_vel se interpreta como "pose target" (x,y,z)
      - posctrl=False: cmd_vel se interpreta como control normal (velocidad/tilt según modo)
    """

    def __init__(self):
        super().__init__("trajectory_follower")

        # ---- Publishers (tópicos del dron)
        self.pub_posctrl = self.create_publisher(Bool, "/simple_drone/posctrl", 10)
        self.pub_cmd = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)

        # Opcional: despegar/aterrizar
        self.pub_takeoff = self.create_publisher(Empty, "/simple_drone/takeoff", 10)
        self.pub_land = self.create_publisher(Empty, "/simple_drone/land", 10)

        # ---- Subscriber de pose
        self.sub_pose = self.create_subscription(Pose, "/simple_drone/gt_pose", self._cb_pose, 10)
        self.current_pose: Optional[Pose] = None

        # ---- RViz publishers
        self.pub_path = self.create_publisher(Path, "/simple_drone/trajectory_path", 10)
        self.pub_marker = self.create_publisher(Marker, "/simple_drone/trajectory_marker", 10)

        # ---- Parámetros de seguimiento
        self.rate_hz = 10.0
        self.arrival_tol = 0.35          # metros para considerar waypoint alcanzado
        self.hold_seconds = 0.5          # “quieto” en el waypoint alcanzado
        self.hold_cycles_at_wp = max(1, int(self.hold_seconds * self.rate_hz))
        self._hold_counter = 0

        # ---- Trayectoria: A -> ... -> B -> ... -> A (puntos intermedios)
        pA = (18.397180, 5.136400, 5.0)
        pB = (13.971850,   3.162810, 5.0)

        self.step_m = 0.75  # 0.5–1.0 suele ir bien
        self.waypoints: List[Tuple[float, float, float]] = self._build_line_waypoints(
            pA, pB, step_m=self.step_m, loop=True
        )
        self.wp_idx = 0

        # ---- Inicializa RViz msgs
        self.frame_id = "map"
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

        self.marker_msg = Marker()
        self.marker_msg.header.frame_id = self.frame_id
        self.marker_msg.ns = "traj"
        self.marker_msg.id = 0
        self.marker_msg.type = Marker.LINE_STRIP
        self.marker_msg.action = Marker.ADD
        self.marker_msg.scale.x = 0.05
        self.marker_msg.color.r = 1.0
        self.marker_msg.color.g = 0.2
        self.marker_msg.color.b = 0.2
        self.marker_msg.color.a = 1.0
        self.marker_msg.pose.orientation.w = 1.0  # identidad

        # ---- Activa posctrl y despega
        self._publish_posctrl(True)
        self._try_takeoff_once()

        # ---- Publica una vez la trayectoria “planeada” para RViz
        self._publish_planned_trajectory()

        # Timer principal
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)

        self.get_logger().info(
            "Trajectory follower listo: trayectoria 2 puntos "
            f"A=({pA[0]:.3f},{pA[1]:.3f},{pA[2]:.1f}) -> "
            f"B=({pB[0]:.3f},{pB[1]:.3f},{pB[2]:.1f}). "
            "Esperando /simple_drone/gt_pose..."
        )

    # -------------------- Trayectoria (A<->B con discretización) --------------------
    def _build_line_waypoints(
        self,
        p0: Tuple[float, float, float],
        p1: Tuple[float, float, float],
        step_m: float = 1.0,
        loop: bool = True,
    ) -> List[Tuple[float, float, float]]:
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        dx, dy, dz = (x1 - x0), (y1 - y0), (z1 - z0)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < 1e-6:
            return [p0]

        step_m = max(step_m, 1e-3)
        n = max(1, int(math.ceil(dist / step_m)))

        wps: List[Tuple[float, float, float]] = []
        # A -> B
        for i in range(n + 1):
            t = i / n
            wps.append((x0 + t * dx, y0 + t * dy, z0 + t * dz))

        if loop:
            # B -> A sin duplicar extremos
            for i in range(1, n):
                t = i / n
                wps.append((x1 - t * dx, y1 - t * dy, z1 - t * dz))

        return wps

    # -------------------- Callbacks / Publicación --------------------
    def _cb_pose(self, msg: Pose):
        self.current_pose = msg

    def _publish_posctrl(self, enabled: bool):
        m = Bool()
        m.data = enabled
        self.pub_posctrl.publish(m)

    def _try_takeoff_once(self):
        self.pub_takeoff.publish(Empty())

    def _publish_goal(self, x: float, y: float, z: float):
        t = Twist()
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.linear.z = float(z)
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = 0.0
        self.pub_cmd.publish(t)

    def _dist_to(self, x: float, y: float, z: float) -> float:
        assert self.current_pose is not None
        dx = self.current_pose.position.x - x
        dy = self.current_pose.position.y - y
        dz = self.current_pose.position.z - z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # -------------------- RViz helpers --------------------
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

    # -------------------- Control loop --------------------
    def _tick(self):
        # Re-publica posctrl (robustez)
        self._publish_posctrl(True)

        if self.current_pose is None:
            self.get_logger().info("Aún no llega /simple_drone/gt_pose...", throttle_duration_sec=1.0)
            return

        x, y, z = self.waypoints[self.wp_idx]
        d = self._dist_to(x, y, z)

        if d <= self.arrival_tol:
            if self._hold_counter < self.hold_cycles_at_wp:
                self._hold_counter += 1
                self._publish_goal(x, y, z)
                self._append_executed_point(x, y, z)
                return

            self._hold_counter = 0
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            nx, ny, nz = self.waypoints[self.wp_idx]
            self.get_logger().info(
                f"Waypoint alcanzado. Siguiente -> idx={self.wp_idx} (x={nx:.2f}, y={ny:.2f}, z={nz:.2f})"
            )

        self._publish_goal(x, y, z)
        self._append_executed_point(x, y, z)
        self.get_logger().info(
            f"Objetivo idx={self.wp_idx} (x={x:.2f}, y={y:.2f}, z={z:.2f}) d={d:.2f}m",
            throttle_duration_sec=1.0,
        )


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
