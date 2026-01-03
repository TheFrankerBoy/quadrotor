# 🚁 PX4 SITL + ROS2 + Gazebo (tmux launcher)

Proyecto de simulación de un **dron multirrotor** usando **PX4 en SITL**, **Gazebo (gz)** y **ROS2**, con control **offboard** y visualización en **QGroundControl**.

El objetivo de este repositorio es proporcionar una forma **reproducible y automatizada** de lanzar todo el entorno de simulación con **un solo comando**, usando `tmux` para gestionar los distintos procesos.

---

## 🧠 Arquitectura general

El sistema lanza y conecta automáticamente:

- **PX4 SITL** (autopiloto)
- **Gazebo (gz)** con el modelo `x500`
- **Micro XRCE-DDS Agent** (puente PX4 ↔ ROS2)
- **ROS2 nodes**:
  - Listener de sensores (`sensor_combined`)
  - Nodo de control **offboard**
- **QGroundControl** (interfaz de operación y monitorización)

Todo el sistema se ejecuta en local (SITL) y se organiza en una **sesión tmux** con múltiples panes.

---

## 📂 Estructura esperada de carpetas

Este repositorio **NO contiene PX4 ni las dependencias pesadas**.  
Se asume que todo el entorno de simulación está en una carpeta externa con la siguiente estructura:

```text
~/CPR_PX4/
├── PX4-Autopilot
├── Micro-XRCE-DDS-Agent
├── ws_sensor_combined
└── ws_offboard_control
``````
## 🖥️ Gestión de la simulación (tmux)

La simulación se ejecuta dentro de una **sesión tmux** llamada `px4_sim`, que permite
gestionar múltiples procesos (PX4, ROS2, XRCE Agent, etc.) de forma ordenada.

> ⚠️ **No es necesario conocer tmux** para usar este proyecto.  
> Los siguientes comandos básicos son suficientes.

### Salir sin detener la simulación (desde tmux)
```text
Ctrl + b → d
```

### Salir sin detener la simulación (n (desde cualquier terminal)
``````text
tmux attach -t px4_sim
``````



### Detener TODO el sistema (desde cualquier terminal)
``````text
tmux kill-session -t px4_sim
``````
💡 Alternativamente, el script de lanzamiento incluye una opción recomendada (desde la carpeta raíz del repositorio):
```text
./scripts/run_sim.sh --kill
```
