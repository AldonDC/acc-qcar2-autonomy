# ACC Development — QCar2 Autonomous Driving

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-Apache%202.0-orange.svg)](LICENSE)

Sistema de navegación autónoma para **Quanser QCar 2** en **Quanser Interactive Labs (QLabs)**: desde teleoperación y SLAM hasta trayectorias planificadas, waypoints en RViz/GUI, control Pure Pursuit/Stanley, grabación y reproducción de rutas.

---

## Tabla de contenidos

- [Características](#-características)
- [Requisitos](#-requisitos)
- [Comandos paso a paso (Drive & Record)](#-comandos-paso-a-paso-drive--record)
- [Fase 1: Grabar una ruta](#-fase-1-grabar-una-ruta-recording)
- [Fase 2: Reproducir ruta](#-fase-2-reproducir-ruta-navigation)
- [Modo híbrido: Cargar ruta + editar en RViz](#-modo-híbrido-cargar-ruta--editar-en-rviz)
- [Modo calca: Grabación como guía](#-modo-calca-usar-grabación-como-guía-trace-mode)
- [Waypoints en RViz (fondo pista)](#-waypoints-en-rviz-fondo-pista)
- [Dudas y trucos](#-dudas-y-trucos)
- [Próximos pasos: Visión y homografía](#-próximos-pasos-visión-y-homografía)
- [Estructura del repositorio](#-estructura-del-repositorio)
- [Contribuir](#-contribuir)
- [Referencias](#-referencias)
- [Licencia](#-licencia)

---

## Características

| Funcionalidad | Descripción |
|--------------|-------------|
| **Drive & Record** | Graba rutas conduciendo con teclado y reprodúcelas de forma autónoma. |
| **Waypoints en RViz** | Define rutas con clics en el mapa (fondo = pista), calibración automática del QCar. |
| **Waypoints en GUI** | Alternativa con ventana Python e imagen del circuito. |
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

**Requisito previo en el host (para GUI/RViz):**

```bash
xhost +local:
```

---

## Comandos paso a paso (Drive & Record)

Todos los comandos se ejecutan **desde la raíz del proyecto** en el host, salvo los que indican “dentro del contenedor”. Orden recomendado: **Terminal 1 → Terminal 2 → Terminal 3** (y Terminal 4 solo para grabar).

---

## Fase 1: Grabar una ruta (Recording)

| Terminal | Acción | Comandos |
|----------|--------|----------|
| **T1** | Lanzar QLabs | `./run_t1.sh` |
| **T2** | Lanzar Cartographer (SLAM) | `./run_t2.sh` |
| **T3** | Entrar al contenedor y grabar | Ver bloque siguiente |
| **T4** | Control con teclado (teleop) | Ver bloque siguiente |

**Terminal 3 — Nodo grabador (dentro del contenedor):**

```bash
./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash
./record.sh vuelta_pro_3
```

*(Cambia `vuelta_pro_3` por el nombre que quieras para la ruta.)*

**Terminal 4 — Teleop (dentro del contenedor):**

```bash
./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
cd /workspaces/isaac_ros-dev/ros2
colcon build --packages-select qcar2_autonomy
source install/setup.bash
ros2 run qcar2_autonomy simple_teleop_keyboard
```

**Finalizar grabación:** En la **Terminal 3** pulsa **Ctrl+C**. La ruta se guarda en `Development/ros2/routes/vuelta_pro_3.json` (o el nombre que hayas usado).

---

## Fase 2: Reproducir ruta (Navigation)

Con **T1** y **T2** en marcha, en **Terminal 3** (dentro del contenedor):

```bash
./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
cd /workspaces/isaac_ros-dev/ros2
colcon build --packages-select qcar2_autonomy && source install/setup.bash
```

Luego elige una opción:

| Opción | Comando | Descripción |
|--------|---------|-------------|
| **A** | `./navigate.sh routes/vuelta_pro_3.json` | Reproducción estándar. |
| **B** | `./navigate.sh routes/vuelta_pro_1.json 5` | Curvas más suaves (usa 1 de cada 5 puntos). |
| **C** | `./navigate.sh routes/vuelta_pro_1.json 1 true` | Loop infinito (vueltas sin parar). |

---

## Modo híbrido: Cargar ruta + editar en RViz

Carga una ruta grabada en RViz, añade o quita puntos con **Publish Point** y envía la ruta al coche.

**Terminal 3 (dentro del contenedor):**

```bash
cd /workspaces/isaac_ros-dev/ros2
colcon build --packages-up-to qcar2_autonomy && source install/setup.bash
ros2 launch qcar2_autonomy waypoint_rviz_launch.py \
  load_file:=/workspaces/isaac_ros-dev/ros2/routes/vuelta1_waypoints.json
```

**En RViz:**

1. Los puntos de la grabación aparecen en **amarillo**.
2. Usa **Publish Point** para añadir más puntos o ajustar la ruta.
3. Envía la ruta al coche:
   ```bash
   ros2 service call /send_path std_srvs/srv/Empty
   ```

---

## Modo calca: Usar grabación como guía (Trace Mode)

Ves la grabación en **verde** (guía) y colocas waypoints **amarillos** encima para definir la trayectoria final.

**Terminal 3 (dentro del contenedor):**

```bash
ros2 launch qcar2_autonomy waypoint_rviz_launch.py \
  reference_file:=/workspaces/isaac_ros-dev/ros2/routes/vuelta_pro_1.json
```

**En RViz:**

1. Estela de la grabación en **verde** traslúcido.
2. Añade waypoints **amarillos** con **Publish Point** siguiendo la guía.
3. Envía al coche: `ros2 service call /send_path std_srvs/srv/Empty`
4. El coche se detiene al llegar al último waypoint.

---

## Waypoints en RViz (fondo pista)

Sin cargar ruta previa: fondo = imagen de la pista, waypoints con clics, calibración automática del QCar.

**Terminal 3 (dentro del contenedor):**

```bash
cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash
ros2 launch qcar2_autonomy waypoint_rviz_launch.py
```

- **Calibración:** Un clic con **Publish Point** donde está el QCar en QLabs; se calibra solo.
- **Añadir waypoints:** Más clics en el mapa.
- **Enviar ruta:** `ros2 service call /waypoint_rviz_node/send_path std_srvs/srv/Empty`
- **Borrar waypoints:** `ros2 service call /waypoint_rviz_node/clear_waypoints std_srvs/srv/Empty`

---

## Dudas y trucos

**¿Qué se ve en RViz?**  
- Fondo con rejilla (o imagen de pista si usas waypoint_rviz_launch).  
- Guía de referencia: línea **verde** con esferas y flechas.  
- Waypoints activos: esferas **ámbar** con línea amarilla.  
- Marcador de inicio: cilindro **azul**; meta: cilindro **rojo**.

**¿Borrar waypoints sin cerrar todo?**  
```bash
ros2 service call /clear_waypoints std_srvs/srv/Empty
```

**¿Dónde están las rutas grabadas?**  
En `Development/ros2/routes/` (archivos `.json`).

**Recompilar tras cambiar código:**  
```bash
colcon build --packages-up-to qcar2_autonomy && source install/setup.bash
```

---

## Próximos pasos: Visión y homografía

En desarrollo: **control del robot mediante visión** usando **homografía** para relacionar la vista de la cámara con el plano del mapa o del circuito (p. ej. puntos 7 y 8 de una libreta de control por visión).

Recursos de referencia:

| Tema | Enlace |
|------|--------|
| Homography vs Perspective Transform | [Substack — findHomography vs getPerspectiveTransform](https://shravankumar147.substack.com/p/homography-vs-perspective-transform) |
| Feature Matching + Homography (OpenCV) | [OpenCV Tutorial — find objects](https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html) |
| Object tracking con homografía | [GeeksforGeeks — OpenCV Object Tracking using Homography](https://www.geeksforgeeks.org/python/python-opencv-object-tracking-using-homography/) |

- **`getPerspectiveTransform`:** 4 puntos conocidos, transformación proyectiva fija (documentos, corrección de perspectiva).  
- **`findHomography`:** Múltiples puntos, RANSAC, robusto a outliers (stitching, AR, tracking de objetos planos).

---

## Estructura del repositorio

```
ACC_Development/
├── README.md
├── run_t1.sh                    # T1: QLabs
├── run_t2.sh                    # T2: Cartographer
├── isaac_ros_common/            # Contenedor de desarrollo
├── Development/
│   └── ros2/
│       ├── COMANDOS_WAYPOINTS.txt    # Comandos completos (este README los resume)
│       ├── record.sh                 # Grabar ruta
│       ├── navigate.sh               # Reproducir ruta
│       ├── routes/                   # Rutas grabadas (.json)
│       └── src/
│           ├── qcar2_autonomy/       # Waypoints, Pure Pursuit, Stanley, GUI, RViz
│           ├── qcar2_nodes/          # TF, Cartographer, converter
│           └── qcar2_interfaces/
└── ...
```

---

## Contribuir

Si quieres proponer cambios o contribuir al proyecto, hazlo mediante **Pull Request (PR)** para mantener un historial claro y revisar el código antes de integrarlo.

### Revisar issues

Antes de meter código, revisa los **Issues** del repo: qué está abierto, qué le toca a cada uno (documentación, autonomía, RViz, visión, etc.) y si hay alguno asignado o que quieras tomar. Si vas a trabajar en **cosas distintas**, usa **una rama por issue o por tema** (p. ej. `fix/issue-5-rviz` o `feature/issue-12-homografia`), haz ahí tus commits y, cuando termines ese issue, abre un PR con todo lo que hiciste para ese tema. Así se revisa por bloques y luego se hace merge a `main`.

### Cómo contribuir

1. **Fork** del repositorio y clona tu copia en local.
2. Revisa los **Issues** y decide en qué trabajar; si son tareas diferentes, **crea una rama por cada una** (p. ej. `feature/nombre-funcionalidad` o `fix/issue-N-descripcion`).
3. Trabaja en esa rama: **commits** con mensajes descriptivos.
4. Cuando hayas terminado el issue o la funcionalidad, abre un **Pull Request** contra `main` describiendo qué hiciste para ese issue.
5. Tras la revisión, el maintainer hará **merge a `main`** con tus cambios.

### Qué incluir en el PR

Para que la revisión sea ágil y el proyecto se mantenga ordenado, en la descripción del PR indica:

| Campo | Qué poner |
|-------|-----------|
| **Resumen** | Qué problema resuelves o qué funcionalidad añades. |
| **Cambios realizados** | Lista de los cambios concretos (archivos o comportamientos). |
| **Secciones/áreas modificadas** | Qué partes del proyecto tocas: documentación (README, COMANDOS_WAYPOINTS), paquetes ROS 2 (qcar2_autonomy, qcar2_nodes), scripts, launch, config RViz, etc. |
| **Motivo** | Por qué se hace el cambio (bug, mejora, nueva feature). |

Los maintainers revisarán el PR y podrán pedir ajustes. Una vez aprobado, se hace **merge a `main`** y tus commits quedan integrados. Si el PR cierra un issue, incluye en la descripción *Closes #N* (sustituye N por el número del issue) para que se cierre automáticamente al hacer merge.

---

## Referencias

- [Quanser](https://www.quanser.com/) — QCar 2 e Interactive Labs.  
- [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/) — Entorno de desarrollo en contenedor.  
- [Smart Mobility 2025](https://github.com/abrahammorohdez19/smart_mobility_2025) — Referencia Pure Pursuit / QCar.  
- [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics) — Pure Pursuit y path tracking.  
- **Homografía y visión:** [Substack](https://shravankumar147.substack.com/p/homography-vs-perspective-transform) · [OpenCV Feature Homography](https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html) · [GeeksforGeeks Object Tracking](https://www.geeksforgeeks.org/python/python-opencv-object-tracking-using-homography/).

---

## Licencia

Este proyecto utiliza licencia **Apache 2.0**. Compatible con el ecosistema ROS 2 y con recursos académicos de Quanser.
