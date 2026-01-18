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

Este nodo implementa un **seguidor de trayectorias 3D** para el dron del paquete `sjtu_drone`, generando una referencia temporal suave y publicando consignas compatibles con dos modos de control:

- **PID (posición)**: publica objetivos de posición como `Twist.linear = (x,y,z)` en `/simple_drone/cmd_vel`.
- **LQR (referencia completa)**: publica una referencia de **posición, velocidad y aceleración** en `/simple_drone/ref_odom` para que el controlador LQR externo calcule el mando.

Además, el nodo publica la trayectoria para visualizarla en RViz y registra el error de seguimiento en un CSV.

---

#### Funcionalidades principales

##### 1) Generación de trayectoria 3D
- Define un conjunto de *waypoints* base.
- Suaviza la ruta con **Catmull–Rom** para obtener una curva continua.
- Inserta un tramo especial tipo **hélice** entre dos puntos para generar un segmento con curvatura y variación vertical.
- Densifica los puntos para limitar la longitud máxima de segmento.

##### 2) Parametrización temporal y suavizado dinámico
Para cada segmento entre waypoints:
- Calcula un tiempo de ejecución en función de una velocidad deseada (`v_ref`) y un tiempo mínimo (`T_min`).
- Precalcula derivadas aproximadas en los waypoints (velocidad) para evitar discontinuidades.
- Construye un perfil **quintico (polinomio de quinto grado)** por eje (x,y,z), que garantiza una trayectoria suave.

En cada instante `t`, el nodo evalúa:
- Posición: \( \mathbf{p}_d(t) \)
- Velocidad: \( \mathbf{v}_d(t) \)
- Aceleración: \( \mathbf{a}_d(t) \)

##### 3) Publicación de referencias según modo
El nodo funciona en dos modos seleccionables:

###### **Modo `pid`**
- Activa el control de posición del plugin:
  - Publica `Bool` en `/simple_drone/posctrl`
- Publica el objetivo (posición) en:
  - `/simple_drone/cmd_vel` (`geometry_msgs/Twist`)
  - usando `Twist.linear.x/y/z = (x,y,z)`.

###### **Modo `lqr`**
- Desactiva el control de posición del plugin (para que el LQR sea el lazo externo).
- Publica referencia completa en:
  - `/simple_drone/ref_odom` (`nav_msgs/Odometry`), con la convención:
    - `pose.pose.position` → \( \mathbf{p}_d \) (mundo)
    - `twist.twist.linear` → \( \mathbf{v}_d \) (mundo)
    - `twist.twist.angular` → \( \mathbf{a}_d \) (mundo, usado como contenedor temporal)

##### 4) Cambio de modo en caliente (sin relanzar)
El parámetro ROS 2 `mode` puede cambiarse en ejecución (`pid` ↔ `lqr`) mediante callback de parámetros. Al cambiar:
- Se actualiza `/simple_drone/posctrl`
- Se publica el modo actual en `/simple_drone/control_mode` (String), para que otros nodos (ej. `lqr_controller`) activen/desactiven su lógica.

##### 5) Visualización en RViz
Publica:
- `/simple_drone/trajectory_path` (`nav_msgs/Path`): trayectoria planificada y puntos ejecutados.
- `/simple_drone/trajectory_marker` (`visualization_msgs/Marker`): líneas en RViz (planned + executed).

##### 6) Logging de error a CSV
En cada iteración:
- Lee la pose actual desde `/simple_drone/gt_pose`.
- Calcula el punto más cercano sobre la trayectoria (polilínea de referencia).
- Guarda en `trajectory_errors.csv`:
  - posición actual
  - punto más cercano
  - error (x,y,z) y norma del error
  - índice del segmento más cercano

---

#### Tópicos

##### Publicados
- `/simple_drone/takeoff` (`std_msgs/Empty`)
- `/simple_drone/land` (`std_msgs/Empty`)
- `/simple_drone/posctrl` (`std_msgs/Bool`)
- `/simple_drone/control_mode` (`std_msgs/String`)
- `/simple_drone/cmd_vel` (`geometry_msgs/Twist`) *(modo pid)*
- `/simple_drone/ref_odom` (`nav_msgs/Odometry`) *(modo lqr)*
- `/simple_drone/trajectory_path` (`nav_msgs/Path`)
- `/simple_drone/trajectory_marker` (`visualization_msgs/Marker`)

##### Suscritos
- `/simple_drone/gt_pose` (`geometry_msgs/Pose`)

---

#### Parámetros

- `mode` (string): `"pid"` o `"lqr"`  
  Por defecto: `"lqr"`

Ejemplo de cambio en caliente:
```bash
ros2 param set /trajectory_follower mode pid
ros2 param set /trajectory_follower mode lqr
```

### lqr_controller

Este nodo implementa un **controlador LQR en espacio de estados** que actúa como **lazo externo de aceleración/velocidad**, cerrando el seguimiento de una referencia completa \((\mathbf{p}_d, \mathbf{v}_d, \mathbf{a}_d)\) generada por el nodo `trajectory_follower` en modo `lqr`.

El controlador calcula aceleraciones deseadas a partir de los errores de posición y velocidad, y las **integra para generar comandos de velocidad** compatibles con el plugin del dron (`/simple_drone/cmd_vel`).

El nodo se **activa o desactiva automáticamente** en función del modo publicado en `/simple_drone/control_mode`.

---

#### Funcionalidades principales

##### 1) Activación automática según modo de control
- Escucha el tópico:
  - `/simple_drone/control_mode` (`std_msgs/String`)
- Estados:
  - `"lqr"` → **LQR habilitado**
  - cualquier otro valor → **LQR deshabilitado**
- Al deshabilitarse:
  - se resetea el estado interno del integrador
  - se evita la acumulación de error (*windup*)

Esto permite cambiar dinámicamente entre control PID puro y LQR sin relanzar nodos.

---

##### 2) Modelo de control LQR

El controlador implementa la ley:

\[
\mathbf{a}_{des} = \mathbf{a}_{ref} - K
\begin{bmatrix}
\mathbf{e}_p \\
\mathbf{e}_v
\end{bmatrix}
\]

donde:
- \(\mathbf{e}_p = \mathbf{p} - \mathbf{p}_d\)
- \(\mathbf{e}_v = \mathbf{v} - \mathbf{v}_d\)
- \(K \in \mathbb{R}^{3 \times 6}\) es la matriz de ganancias LQR precalculada

Las ganancias están fijadas en el nodo y separadas por ejes (x, y, z).

---

##### 3) Gestión de marcos de referencia

Para ser compatible con el plugin del dron:

- La **posición y velocidad de referencia** se asumen en **world frame**
- El control en **XY** se realiza en **heading frame** (marco alineado con el yaw del dron)
- El eje **Z** se controla directamente en **world frame**

Conversión realizada mediante:
- extracción de yaw desde el cuaternión
- rotación explícita de vectores velocidad y error

Esto desacopla el control horizontal de la orientación global del dron.

---

##### 4) Integración de aceleraciones con anti-windup

Las aceleraciones LQR se integran para generar comandos de velocidad:

- Integración explícita:
  - \(\dot{v} = a_{des}\)
- Saturaciones duras:
  - aceleración máxima
  - velocidad máxima
- **Anti-windup por bloqueo**:
  - si la velocidad está saturada y la aceleración empuja en el mismo sentido, la integración se bloquea

Esto evita acumulación de error cuando el sistema está limitado por saturación.

---

##### 5) Fuga suave hacia la referencia de velocidad

Para evitar deriva e inconsistencias numéricas:

- Se introduce una **fuga controlada** hacia la velocidad de referencia:
\[
v \leftarrow v + \alpha (v_{ref} - v)
\]

- \(\alpha\) depende del tiempo de muestreo y una constante `tau_track`
- Esto garantiza:
  - estabilidad a largo plazo
  - reenganche suave tras discontinuidades

---

##### 6) Gestión de discontinuidades en la referencia

El nodo detecta saltos grandes en la referencia de posición:
- Si el salto supera un umbral configurable:
  - se considera una discontinuidad
  - el comando de velocidad se resetea suavemente a la velocidad de referencia
- Esto evita picos de aceleración no realistas cuando la trayectoria cambia bruscamente.

---

##### 7) Publicación del comando de control

El comando final se publica como:

- `/simple_drone/cmd_vel` (`geometry_msgs/Twist`)
  - `linear.x`, `linear.y`: velocidades en **heading frame**
  - `linear.z`: velocidad vertical en **world frame**
  - `angular.z = 0` (yaw no controlado por este nodo)

---

#### Tópicos

##### Publicados
- `/simple_drone/cmd_vel` (`geometry_msgs/Twist`)

##### Suscritos
- `/simple_drone/odom` (`nav_msgs/Odometry`)
- `/simple_drone/ref_odom` (`nav_msgs/Odometry`)
- `/simple_drone/control_mode` (`std_msgs/String`)

---

#### Parámetros y constantes internas

Configurados directamente en el nodo:
- Frecuencia de control: `100 Hz`
- Límites:
  - aceleración XY / Z
  - velocidad XY / Z
- Umbrales de discontinuidad:
  - salto de posición
  - salto de velocidad
- Ganancias LQR fijas
