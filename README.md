# Control y Navegación de un Quadrotor en Simulación

Este repositorio recoge el trabajo realizado para la asignatura **Control y Programación de Robots**, centrado en el estudio del **control y la navegación de un quadrotor en entornos de simulación**.

Con el objetivo de separar claramente los enfoques abordados, el proyecto se ha dividido en **dos ramas principales**, cada una dedicada a un nivel distinto del problema:

---

## 📌 Estructura del repositorio

### 🔧 Rama `control_bajo_nivel`
Esta rama contiene el desarrollo del **control a bajo nivel** del quadrotor, incluyendo:
- Modelado y simulación del sistema.
- Seguimiento de trayectorias.
- Controladores PID y LQR.
- Análisis del error y ajuste de parámetros.

Está orientada a comprender en profundidad la dinámica del vehículo y las técnicas clásicas de control.

👉 Consultar el `README.md` de esta rama para más detalles.

---

### 🚁 Rama `PX4_SITL`
Esta rama aborda la **navegación y operación autónoma** del quadrotor utilizando **PX4 en modo SITL**, integrando:
- Simulación en Gazebo.
- Programación de misiones con QGroundControl.
- Evitación de obstáculos mediante LiDAR.
- Ejecución centralizada del sistema.

Está enfocada a un nivel de abstracción más alto, similar al flujo de trabajo en aplicaciones reales.

👉 Consultar el `README.md` de esta rama para la descripción completa.

---

## ℹ️ Nota
La rama `main` actúa únicamente como **punto de entrada y organización del proyecto**.  
Todo el código y la documentación detallada se encuentran en las ramas indicadas.

---

📁 **Ramas principales del proyecto**:
- `control_bajo_nivel`
- `PX4_SITL`
