# ACC Development — QCar2 Autonomous Driving

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-orange.svg)](LICENSE)

Sistema de navegación autónoma para **Quanser QCar 2** en **Quanser Interactive Labs (QLabs)**, con soporte para grabación de rutas, waypoints en RViz/GUI, controladores Pure Pursuit y Stanley, y SLAM con Cartographer.

---

## Tabla de contenidos

- [Características](#-características)
- [Requisitos](#-requisitos)
- [Estructura del repositorio](#-estructura-del-repositorio)
- [Instalación y entorno](#-instalación-y-entorno)
- [Uso rápido](#-uso-rápido)
- [Modos de operación](#-modos-de-operación)
- [Documentación adicional](#-documentación-adicional)
- [Licencia](#-licencia)

---

## Características

| Funcionalidad | Descripción |
|--------------|-------------|
| **Drive & Record** | Graba rutas conduciendo con teclado y reprodúcelas de forma autónoma. |
| **Waypoints en RViz** | Define rutas con clics en el mapa (fondo = pista), calibración automática del QCar. |
| **Waypoints en GUI** | Alternativa con ventana Python y imagen del circuito. |
| **Pure Pursuit** | Controlador de seguimiento de trayectoria (estilo Smart Mobility / PythonRobotics). |
| **Stanley** | Controlador lateral tipo Stanley como alternativa. |
| **SLAM** | Cartographer para mapa y localización (map → base_link). |
| **Rutas grabadas** | Carga rutas JSON, modo “calca” con guía visual en verde. |

---

## Requisitos

- **Sistema:** Linux (Ubuntu 22.04 recomendado)
- **ROS 2:** Humble
- **Entorno:** Contenedor de desarrollo (Isaac ROS / Docker)
- **Simulación:** Quanser Interactive Labs con escenario *ACC Self Driving Car Competition*
- **Display (GUI):** `xhost +local:` en el host para ventanas gráficas

---

## Estructura del repositorio

```
ACC_Development/
├── README.md                    # Este archivo
├── run_t1.sh                    # Terminal 1: lanzar QLabs
├── run_t2.sh                    # Terminal 2: Cartographer (SLAM)
├── isaac_ros_common/            # Scripts y Docker para el contenedor
├── Development/
│   └── ros2/
│       ├── COMANDOS_WAYPOINTS.txt    # Comandos paso a paso (Drive & Record, RViz, etc.)
│       ├── record.sh                 # Grabar ruta
│       ├── navigate.sh               # Reproducir ruta
│       ├── routes/                   # Rutas grabadas (.json)
│       └── src/
│           ├── qcar2_autonomy/       # Nodos de autonomía (waypoints, Pure Pursuit, GUI, RViz)
│           ├── qcar2_nodes/          # Nodos QCar2 (TF, Cartographer, converter)
│           └── qcar2_interfaces/      # Mensajes e interfaces
└── ...
```

---

## Instalación y entorno

1. Clonar el repositorio (o descomprimir el código).
2. En el host, permitir ventanas X si usas GUI:
   ```bash
   xhost +local:
   ```
3. Entrar al contenedor de desarrollo:
   ```bash
   cd /ruta/a/ACC_Development
   ./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
   ```
4. Dentro del contenedor, compilar el workspace ROS 2:
   ```bash
   cd /workspaces/isaac_ros-dev/ros2
   colcon build --packages-up-to qcar2_autonomy
   source install/setup.bash
   ```

---

## Uso rápido

### Orden de terminales (desde la raíz del proyecto)

| Terminal | Comando | Función |
|----------|---------|--------|
| **T1** | `./run_t1.sh` | Lanza QLabs (simulación). |
| **T2** | `./run_t2.sh` | Lanza Cartographer (SLAM). |
| **T3** | Ver abajo | Waypoints, grabación o navegación. |

### Waypoints en RViz (recomendado)

1. Con T1 y T2 en marcha, en una tercera terminal:
   ```bash
   ./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
   cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash
   ros2 launch qcar2_autonomy waypoint_rviz_launch.py
   ```
2. En RViz: herramienta **Publish Point** → haz **un clic** donde está el QCar en QLabs (calibra solo).
3. Sigue añadiendo clics para waypoints; luego envía la ruta al coche:
   ```bash
   ros2 service call /waypoint_rviz_node/send_path std_srvs/srv/Empty
   ```
4. Borrar waypoints:  
   `ros2 service call /waypoint_rviz_node/clear_waypoints std_srvs/srv/Empty`

### Grabación y reproducción (Drive & Record)

- **Grabar:** T1 + T2 + (T3: `./record.sh nombre_ruta`) + (T4: teleop con teclado). Al terminar, Ctrl+C en T3; la ruta se guarda en `Development/ros2/routes/`.
- **Reproducir:** Con T1 y T2 activos, en T3:  
  `./navigate.sh routes/nombre_ruta.json`

Todos los comandos detallados están en **`Development/ros2/COMANDOS_WAYPOINTS.txt`**.

---

## Modos de operación

| Modo | Descripción |
|------|-------------|
| **Waypoints RViz** | Fondo = imagen de pista (`/map_circuit`), waypoints con Publish Point, Pure Pursuit. |
| **Waypoints GUI** | `ros2 launch qcar2_autonomy waypoint_map_gui_launch.py` (ventana Python con imagen del circuito). |
| **Grabar ruta** | `record.sh` + teleop; guarda JSON en `routes/`. |
| **Reproducir ruta** | `navigate.sh routes/archivo.json`. |
| **Modo híbrido** | Cargar ruta en RViz, editar waypoints y enviar con `send_path`. |
| **Modo calca** | Ruta de referencia en verde; waypoints amarillos encima; enviar con `send_path`. |

---

## Documentación adicional

- **`Development/ros2/COMANDOS_WAYPOINTS.txt`** — Comandos completos: grabación, reproducción, RViz, calibración, servicios.
- **`Development/ros2/src/qcar2_autonomy/config/CIRCUIT_README.txt`** — Uso de la imagen de pista y calibración en la GUI.

---

## Licencia

Este proyecto utiliza licencia **Apache 2.0** (o la indicada en el repositorio). Compatible con el ecosistema ROS 2 y con recursos académicos de Quanser.

---

## Agradecimientos

- [Quanser](https://www.quanser.com/) — QCar 2 e Interactive Labs.
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/) — Entorno de desarrollo en contenedor.
- Referencias de control: [Smart Mobility 2025](https://github.com/abrahammorohdez19/smart_mobility_2025), [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) (Pure Pursuit).
