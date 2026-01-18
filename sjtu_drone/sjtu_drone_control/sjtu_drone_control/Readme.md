1. **drone_utils/drone_object.py**: This file defines the `DroneObject` class. The class includes methods for controlling the drone, including taking off, landing, and moving in various directions.

2. **open_loop_control.py**: This script implements an open-loop control system for the drone. Open-loop control refers to sending commands to the drone without any feedback mechanism to adjust the commands based on the drone's response. The file provides functions to control the drone's movement, such as moving forward, turning, ascending, or descending. The drone's movement is controlled by adjusting the drone's roll, pitch, yaw, and vertical velocity. To run the script, use the following command:

   ```bash
   ros2 run sjtu_drone_control open_loop_control.py --task <'square', 'triangle', 'forward', 'backward', 'left', 'right', 'up', 'down'> --distance <float [m]> --number <int>
   ```


3. **drone_position_control.py**: This file contains the implementation of a position control system for the drone. Unlike open-loop control, position control utilizes the gazebo plugin and PID controllers to move the drone to the desired position. To run the script, use the following command:

   ```bash
   ros2 run sjtu_drone_control drone_position_control.py
   ```

   # SimpleDrone Trajectory Follower (ROS 2)

Este nodo implementa un **seguidor de trayectorias 3D** para el dron del paquete `sjtu_drone`, generando una referencia temporal suave y publicando consignas compatibles con dos modos de control:

- **PID (posición)**: publica objetivos de posición como `Twist.linear = (x,y,z)` en `/simple_drone/cmd_vel`.
- **LQR (referencia completa)**: publica una referencia de **posición, velocidad y aceleración** en `/simple_drone/ref_odom` para que el controlador LQR externo calcule el mando.

Además, el nodo publica la trayectoria para visualizarla en RViz y registra el error de seguimiento en un CSV.

---

## Funcionalidades principales

### 1) Generación de trayectoria 3D
- Define un conjunto de *waypoints* base.
- Suaviza la ruta con **Catmull–Rom** para obtener una curva continua.
- Inserta un tramo especial tipo **hélice** entre dos puntos para generar un segmento con curvatura y variación vertical.
- Densifica los puntos para limitar la longitud máxima de segmento.

### 2) Parametrización temporal y suavizado dinámico
Para cada segmento entre waypoints:
- Calcula un tiempo de ejecución en función de una velocidad deseada (`v_ref`) y un tiempo mínimo (`T_min`).
- Precalcula derivadas aproximadas en los waypoints (velocidad) para evitar discontinuidades.
- Construye un perfil **quintico (polinomio de quinto grado)** por eje (x,y,z), que garantiza una trayectoria suave.

En cada instante `t`, el nodo evalúa:
- Posición: \( \mathbf{p}_d(t) \)
- Velocidad: \( \mathbf{v}_d(t) \)
- Aceleración: \( \mathbf{a}_d(t) \)

### 3) Publicación de referencias según modo
El nodo funciona en dos modos seleccionables:

#### **Modo `pid`**
- Activa el control de posición del plugin:
  - Publica `Bool` en `/simple_drone/posctrl`
- Publica el objetivo (posición) en:
  - `/simple_drone/cmd_vel` (`geometry_msgs/Twist`)
  - usando `Twist.linear.x/y/z = (x,y,z)`.

#### **Modo `lqr`**
- Desactiva el control de posición del plugin (para que el LQR sea el lazo externo).
- Publica referencia completa en:
  - `/simple_drone/ref_odom` (`nav_msgs/Odometry`), con la convención:
    - `pose.pose.position` → \( \mathbf{p}_d \) (mundo)
    - `twist.twist.linear` → \( \mathbf{v}_d \) (mundo)
    - `twist.twist.angular` → \( \mathbf{a}_d \) (mundo, usado como contenedor temporal)

### 4) Cambio de modo en caliente (sin relanzar)
El parámetro ROS 2 `mode` puede cambiarse en ejecución (`pid` ↔ `lqr`) mediante callback de parámetros. Al cambiar:
- Se actualiza `/simple_drone/posctrl`
- Se publica el modo actual en `/simple_drone/control_mode` (String), para que otros nodos (ej. `lqr_controller`) activen/desactiven su lógica.

### 5) Visualización en RViz
Publica:
- `/simple_drone/trajectory_path` (`nav_msgs/Path`): trayectoria planificada y puntos ejecutados.
- `/simple_drone/trajectory_marker` (`visualization_msgs/Marker`): líneas en RViz (planned + executed).

### 6) Logging de error a CSV
En cada iteración:
- Lee la pose actual desde `/simple_drone/gt_pose`.
- Calcula el punto más cercano sobre la trayectoria (polilínea de referencia).
- Guarda en `trajectory_errors.csv`:
  - posición actual
  - punto más cercano
  - error (x,y,z) y norma del error
  - índice del segmento más cercano

---

## Tópicos

### Publicados
- `/simple_drone/takeoff` (`std_msgs/Empty`)
- `/simple_drone/land` (`std_msgs/Empty`)
- `/simple_drone/posctrl` (`std_msgs/Bool`)
- `/simple_drone/control_mode` (`std_msgs/String`)
- `/simple_drone/cmd_vel` (`geometry_msgs/Twist`) *(modo pid)*
- `/simple_drone/ref_odom` (`nav_msgs/Odometry`) *(modo lqr)*
- `/simple_drone/trajectory_path` (`nav_msgs/Path`)
- `/simple_drone/trajectory_marker` (`visualization_msgs/Marker`)

### Suscritos
- `/simple_drone/gt_pose` (`geometry_msgs/Pose`)

---

## Parámetros

- `mode` (string): `"pid"` o `"lqr"`  
  Por defecto: `"lqr"`

Ejemplo de cambio en caliente:
```bash
ros2 param set /trajectory_follower mode pid
ros2 param set /trajectory_follower mode lqr
