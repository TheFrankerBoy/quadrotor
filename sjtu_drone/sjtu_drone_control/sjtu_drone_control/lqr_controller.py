#!/usr/bin/env python3

import math
from typing import Optional, Tuple
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def integrate_antiwindup(v: float, a: float, dt: float, v_lim: float) -> float:

    if abs(v) >= v_lim - 1e-9:
        # Si a empuja hacia el mismo signo que v (más saturación), bloquea
        if (v > 0.0 and a > 0.0) or (v < 0.0 and a < 0.0):
            return v
    return v + a * dt


def yaw_from_quat(q) -> float:
    # yaw de cuaternion (z-y-x)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_rotate(q, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    # Rotar vector v con cuaternion q (world = q * v * q^{-1})
    # q es geometry_msgs/Quaternion con los campos x,y,z,w
    x, y, z = q.x, q.y, q.z
    w = q.w
    vx, vy, vz = v

    # t = 2 * cross(q_vec, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    # v' = v + w*t + cross(q_vec, t)
    vpx = vx + w * tx + (y * tz - z * ty)
    vpy = vy + w * ty + (z * tx - x * tz)
    vpz = vz + w * tz + (x * ty - y * tx)
    return (vpx, vpy, vpz)


def world_to_heading_xy(vx_w: float, vy_w: float, yaw: float) -> Tuple[float, float]:
    # Aplicar Rz(yaw)^T : world -> heading frame
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    return (cy * vx_w + sy * vy_w, -sy * vx_w + cy * vy_w)


class LQRController(Node):

    def __init__(self):
        super().__init__("lqr_controller")

        # Control LQR deshabilitado al inicio
        self.enabled = False
        
        # Topics
        self.mode_topic = "/simple_drone/control_mode"
        self.odom_topic = "/simple_drone/odom"
        self.ref_topic = "/simple_drone/ref_odom"
        self.cmd_topic = "/simple_drone/cmd_vel"
        
        self.prev_pr = None
        self.jump_pos_m = 1.0      # si la referencia salta >1m en un tick, es discontinuidad
        self.jump_v_ms = 2.0       # si v_ref salta >2m/s

        # Control rate (node loop)
        self.rate_hz = 100.0

        # ganancias LQR
        # a_des = a_ref - K*[ep; ev]
        self.K = [
            0.9586, 0.0, 0.0, 1.5522, 0.0, 0.0,
            0.0, 0.9586, 0.0, 0.0, 1.5522, 0.0,
            0.0, 0.0, 1.9089, 0.0, 0.0, 2.4021,
        ]

        # Limites
        self.a_xy_max = 4.0   # m/s^2
        self.a_z_max = 4.0
        self.v_xy_max = 2.5   # m/s
        self.v_z_max = 2.0

        # Estado interno del comando de velocidad
        # XY almacenados en heading frame; Z en world frame
        self.vcmd_hx = 0.0
        self.vcmd_hy = 0.0
        self.vcmd_z = 0.0

        # Mantenedores del estado
        self.odom: Optional[Odometry] = None
        self.ref: Optional[Odometry] = None
        self.last_t: Optional[float] = None
        self.initialized_cmd = False

        # Subscriptions/Publishers
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._cb_odom, 10)
        self.sub_ref = self.create_subscription(Odometry, self.ref_topic, self._cb_ref, 10)
        self.sub_mode = self.create_subscription(String, self.mode_topic, self._cb_mode, 10)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info("LQR node")

    def _cb_odom(self, msg: Odometry):
        self.odom = msg

    def _cb_ref(self, msg: Odometry):
        self.ref = msg
        
    def _cb_mode(self, msg: String):
        if msg.data == "lqr":
            if not self.enabled:
                self.get_logger().info("LQR ENABLED")
            self.enabled = True
        else:
            if self.enabled:
                self.get_logger().info("LQR DISABLED")
            self.enabled = False
            self.initialized_cmd = False
            self.last_t = None

    def _tick(self):
    
        if not self.enabled:
            return

        if self.odom is None or self.ref is None:
            return

        # dt del nodo clock
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.last_t is None:
            self.last_t = t
            return
        dt = t - self.last_t
        self.last_t = t
        
        if dt <= 1e-6:
            return

        # --- Estado actual ---
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = yaw_from_quat(q)

        # Velocidad en child frame (base_footprint)
        v_child = self.odom.twist.twist.linear
        # Convertir a world usando cuaternion
        v_wx, v_wy, v_wz = quat_rotate(q, (v_child.x, v_child.y, v_child.z))
        # Convertir world XY al heading frame
        v_hx, v_hy = world_to_heading_xy(v_wx, v_wy, yaw)

        # --- Referencias (asumiendo world frame) ---
        pr = self.ref.pose.pose.position
        vr = self.ref.twist.twist.linear
        ar = self.ref.twist.twist.angular  # usado como acc_ref
        
        if self.prev_pr is not None:
            dx = pr.x - self.prev_pr[0]
            dy = pr.y - self.prev_pr[1]
            dz = pr.z - self.prev_pr[2]
            if math.sqrt(dx*dx + dy*dy + dz*dz) > self.jump_pos_m:
                # discontinuidad en referencia -> reset suave
                self.get_logger().warn("Reference position jump -> reset vcmd to v_ref")
                self.vcmd_hx, self.vcmd_hy, self.vcmd_z = vr_hx, vr_hy, vr.z
        self.prev_pr = (pr.x, pr.y, pr.z)

        # Convertir referencias del world XY al heading frame
        vr_hx, vr_hy = world_to_heading_xy(vr.x, vr.y, yaw)
        ar_hx, ar_hy = world_to_heading_xy(ar.x, ar.y, yaw)

        # Error de posicion en el heading frame
        ep_hx, ep_hy = world_to_heading_xy(p.x - pr.x, p.y - pr.y, yaw)
        ep_z = p.z - pr.z

        # Error de velocidad
        ev_hx = v_hx - vr_hx
        ev_hy = v_hy - vr_hy
        ev_z = v_wz - vr.z

        # --- LQR: a_des = a_ref - K * [ep; ev] ---
        K = self.K
        ax = ar_hx - (K[0]*ep_hx + K[1]*ep_hy + K[2]*ep_z + K[3]*ev_hx + K[4]*ev_hy + K[5]*ev_z)
        ay = ar_hy - (K[6]*ep_hx + K[7]*ep_hy + K[8]*ep_z + K[9]*ev_hx + K[10]*ev_hy + K[11]*ev_z)
        az = ar.z  - (K[12]*ep_hx + K[13]*ep_hy + K[14]*ep_z + K[15]*ev_hx + K[16]*ev_hy + K[17]*ev_z)

        ax = clamp(ax, -self.a_xy_max, self.a_xy_max)
        ay = clamp(ay, -self.a_xy_max, self.a_xy_max)
        az = clamp(az, -self.a_z_max,  self.a_z_max)

        # Inicializacion de los comandos a la referencia de velocidad la primera vez (para evitar saltos)
        if not self.initialized_cmd:
            self.vcmd_hx = vr_hx
            self.vcmd_hy = vr_hy
            self.vcmd_z = vr.z
            self.initialized_cmd = True

        # --- Integración con "fuga" hacia v_ref (evita deriva/windup) ---
        tau_track = 0.4  # segundos. Más bajo = más "pegado" a v_ref
        alpha = clamp(dt / tau_track, 0.0, 1.0)

        self.vcmd_hx = integrate_antiwindup(self.vcmd_hx, ax, dt, self.v_xy_max)
        self.vcmd_hy = integrate_antiwindup(self.vcmd_hy, ay, dt, self.v_xy_max)
        self.vcmd_z  = integrate_antiwindup(self.vcmd_z,  az, dt, self.v_z_max)

        # fuga / "reset suave" hacia la referencia
        self.vcmd_hx += alpha * (vr_hx - self.vcmd_hx)
        self.vcmd_hy += alpha * (vr_hy - self.vcmd_hy)
        self.vcmd_z  += alpha * (vr.z  - self.vcmd_z)

        
        # Saturaciones del comando de velocidad
        self.vcmd_hx = clamp(self.vcmd_hx, -self.v_xy_max, self.v_xy_max)
        self.vcmd_hy = clamp(self.vcmd_hy, -self.v_xy_max, self.v_xy_max)
        self.vcmd_z  = clamp(self.vcmd_z,  -self.v_z_max,  self.v_z_max)

        # Publicar Twist: plugin espera cmd_vel.linear.x/y en heading frame; z en world vertical
        cmd = Twist()
        cmd.linear.x = float(self.vcmd_hx)
        cmd.linear.y = float(self.vcmd_hy)
        cmd.linear.z = float(self.vcmd_z)
        cmd.angular.z = 0.0
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = LQRController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

