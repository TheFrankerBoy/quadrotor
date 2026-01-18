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

### 🤝 Reparto de tareas

Aunque cada integrante ha tenido un mayor peso en una de las partes, todos han participado en ambos bloques para garantizar una visión global del sistema.

| Integrante | PX4 / Navegación (SITL) | Control a bajo nivel |
|----------|--------------------------|----------------------|
| **Antonio Lago Solís** | Configuración de PX4 SITL, integración con Gazebo, programación de misiones en QGroundControl, evitación de obstáculos con LiDAR, ejecución centralizada mediante scripts | Apoyo en análisis de resultados e integración conceptual con el control |
| **Heliodoro Tejada Rodríguez** | Estudio de la arquitectura de PX4, análisis del control en cascada, validación de misiones y análisis de logs con PX4 Flight Review | Apoyo en la interpretación del comportamiento dinámico |
| **Francisco Fuentes Campos** | Apoyo en ejecución de simulaciones y análisis comparativo | Implementación y ajuste de controladores, seguimiento de trayectorias, análisis de estabilidad |
| **Javier Santos Martínez** | Apoyo en validación de trayectorias y flujo de misión | Modelado del sistema, diseño de estrategias de control y evaluación del rendimiento |
| **Daniel López Rubio** | Apoyo en simulaciones PX4 y supervisión de misiones | Implementación de algoritmos de control, simulación y análisis comparativo |

De forma transversal, todos los miembros han participado en la discusión técnica, análisis de resultados, elaboración de la memoria y preparación de la presentación final.

## ℹ️ Nota
La rama `main` actúa únicamente como **punto de entrada y organización del proyecto**.  
Todo el código y la documentación detallada se encuentran en las ramas indicadas.

---

📁 **Ramas principales del proyecto**:
- `control_bajo_nivel`
- `PX4_SITL`
