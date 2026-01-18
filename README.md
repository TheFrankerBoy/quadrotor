# Control a bajo nivel

Simulación de un **quadrotor** basada en ROS 2 y Gazebo.

El acrónimo **sjtu** proviene de *Shanghai Jiao Tong University*. Este repositorio
incluye el modelo del dron, sensores, un plugin de control en cascada basado en
PID y extensiones para **seguimiento de trayectorias 3D** y **control LQR**.

---

## Requisitos

Probado con:
- Ubuntu 22.04
- ROS 2 Humble
- Gazebo 11

---

## Ejecución de la simulación

Control exclusivamente mediante PIDs:

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
ros2 run sjtu_drone_control trajectory_follower --ros-args -p mode:=pid
```

Control mediante LQR (outer loop) y PIDs (inner loop):

```bash
ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py
ros2 run sjtu_drone_controller lqr_controller
ros2 run sjtu_drone_controller trajectory_follower --ros-args -p mode:=lqr
```

## Nodos

Los dos nodos desarrollados son **trajectory_follower** y **lqr_controller**.

### trajectory_follower

Este nodo implementa un **seguidor de trayectorias 3D** para el dron del paquete `sjtu_drone`, generando una referencia temporal suave y publicando consignas compatibles con dos modos de control.

En cada instante se calcula:
- p_d(t): posición deseada
- v_d(t): velocidad deseada
- a_d(t): aceleración deseada

### lqr_controller

El controlador LQR calcula aceleraciones deseadas a partir de:
- error de posición = posición actual menos posición deseada
- error de velocidad = velocidad actual menos velocidad deseada

La ley de control aplicada es:
- aceleración deseada = aceleración de referencia menos K por el vector de errores
