# Comandos QCar2 – Un solo flujo (Terminal 2 fija)

**Desde la raíz del proyecto** (`ACC_Development/`):

| Terminal | Un solo comando (en el host) |
|----------|-----------------------------|
| **Terminal 1** | `./run_t1.sh` |
| **Terminal 2** | `./run_t2.sh` |
| **Terminal 3** | Entrar al contenedor y lanzar lo que toque (ver abajo) |

Detalle:
- **T1** (`run_t1.sh`): arranca el contenedor virtual QCar y ejecuta el script del mapa. No hace falta escribir nada más dentro del contenedor.
- **T2** (`run_t2.sh`): arranca el contenedor de desarrollo y dentro lanza build + `run_qcar2_with_map.sh`. Todo en un solo comando.
- **T3**: otra sesión del contenedor para demo, teleop, waypoints, RViz, etc.

| Paso | Dónde | Comando |
|------|--------|---------|
| **Terminal 1** | Host (desde `ACC_Development/`) | **`./run_t1.sh`** |
| **Terminal 2** | Host (desde `ACC_Development/`) | **`./run_t2.sh`** |
| **Terminal 3** | Host | `./isaac_ros_common/scripts/run_dev.sh ./Development -- bash` |
| | Dentro | `cd /workspaces/isaac_ros-dev/ros2` → `source install/setup.bash` → **lo que quieras** (ver abajo) |

---

## Por qué “se muere todo” al abrir RViz

1. **RViz en la misma terminal que el launch**  
   Si lanzas RViz en la misma terminal donde corre `run_qcar2_with_map.sh`, al cerrar RViz a veces se cierra toda la sesión.  
   **Solución:** Abre RViz en **otra terminal** (otra pestaña/ventana del contenedor, Terminal 3 o 4).

2. **Permiso de display (ventanas) – `xhost +local:`**  
   Para que RViz (y otras ventanas del contenedor) se vean en tu pantalla, en el **host** ejecuta **una vez por sesión** (o una vez después de reiniciar):
   ```bash
   xhost +local:
   ```
   No hace falta repetirlo en cada terminal; con una vez es suficiente hasta que cierres sesión o reinicies.  
   Si quieres que se ejecute siempre al abrir una terminal, añádelo al final de tu `~/.bashrc` en el host.

3. **Uso de memoria**  
   Cartographer + RViz + simulación consumen bastante RAM. Cierra otras aplicaciones pesadas si la máquina va justa.

4. **Si el GUI de waypoint_map_gui no se abre (Terminal 4)**  
   - En el **host**, en una terminal que tenga pantalla (no SSH sin X):
     ```bash
     xhost +local:
     export DISPLAY=${DISPLAY:-:0}
     cd /home/alfonsd/Documents/ACC_Development
     ./isaac_ros_common/scripts/run_dev.sh ./Development -- bash
     ```
   - Dentro del contenedor, comprueba que DISPLAY esté definido: `echo $DISPLAY` (debe salir `:0` o similar).
   - **python3-tk** ya está en la imagen (se añadió en Dockerfile.user). **Reconstruye la imagen una vez**: cierra todas las sesiones del contenedor (`exit`), en el host ejecuta `cd /home/alfonsd/Documents/ACC_Development && ./isaac_ros_common/scripts/run_dev.sh ./Development -- bash`; se reconstruirá la imagen con python3-tk. Desde entonces no tendrás que usar `docker exec ... apt-get install python3-tk`.
   - Si aun así falta tkinter: desde el HOST, `docker exec -u root isaac_ros_dev-x86_64-container bash -c "apt-get update && apt-get install -y python3-tk"`. Luego dentro del contenedor: `ros2 run qcar2_autonomy waypoint_map_gui`.
   - Luego: `cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash && ros2 run qcar2_autonomy waypoint_map_gui`.  
   Si aun así no se abre la ventana, el programa pasa a **modo terminal** (escribe `x y` y `enviar`).

---

## Terminal 3: qué ejecutar (solo uno cada vez)

```bash
cd /workspaces/isaac_ros-dev/ros2
source install/setup.bash
```

Luego **una** de estas opciones:

| Quieres | Comando |
|--------|---------|
| Demo automática | `ros2 run qcar2_autonomy simple_demo_nav` |
| Conducir con teclado | `ros2 run qcar2_autonomy simple_teleop_keyboard` |
| Waypoints por parámetro | `ros2 run qcar2_autonomy waypoint_follower_controller --ros-args -p waypoints:="[1,0, 1,1, 0,1, 0,0]"` |
| Evitar obstáculos (láser) | `ros2 run qcar2_autonomy obstacle_avoider` |
| **Waypoints en RViz** | Ver sección siguiente |
| **Demo de entrega (cuadrado)** | Ver sección "Demo de entrega" |

---

## Demo de entrega (cuadrado automático)

Demo sencilla para entregar: el QCar sigue una ruta fija en forma de **cuadrado de 1,5 m** y se ve claro en RViz (línea verde + esferas).

**Requisito:** El robot debe estar cerca del **origen del mapa (0, 0)** al iniciar (en simulación suele empezar ahí).

| Paso | Terminal | Comando |
|------|----------|---------|
| 1 | **T1** (host) | `./run_t1.sh` |
| 2 | **T2** (host) | `./run_t2.sh` |
| 3 | **T3** (entrar al contenedor) | `./isaac_ros_common/scripts/run_dev.sh ./Development -- bash` |
| 4 | T3 (dentro) | `cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash` |
| 5 | T3 (dentro) | `ros2 launch qcar2_autonomy demo_entrega_launch.py` |
| 6 | **T4** (otra sesión del contenedor) | `cd /workspaces/isaac_ros-dev/ros2 && source install/setup.bash` |
| 7 | T4 (dentro) | `rviz2 -d src/qcar2_autonomy/config/qcar2_waypoints_light.rviz` |

En RViz verás:
- **Map** (mapa de Cartographer)
- **WaypointsPath** (línea verde = ruta que sigue el coche)
- **WaypointMarkers** (esferas verdes en cada vértice)
- **TF** (ejes del robot)
- **LaserScan** (láser del QCar)

A los ~2 s se publica la ruta y el coche empieza a moverse. Para recompilar tras cambiar código:  
`colcon build --packages-select qcar2_autonomy && source install/setup.bash`

---

## Waypoints visuales en RViz

1. En **Terminal 3**:  
   `ros2 launch qcar2_autonomy waypoint_visual_launch.py`

2. En **otra terminal** (Terminal 4), mismo contenedor, con `source install/setup.bash`:
   ```bash
   rviz2 -d $(ros2 pkg prefix qcar2_autonomy)/share/qcar2_autonomy/config/qcar2_waypoints.rviz
   ```
   Si sale **std::bad_alloc** o RViz se cierra por memoria: usa la config ligera (sin mapa):  
   `rviz2 -d $(ros2 pkg prefix qcar2_autonomy)/share/qcar2_autonomy/config/qcar2_waypoints_light.rviz`
   Así RViz no comparte terminal con el launch y es menos probable que “se muera todo”.

3. Marca waypoints en RViz con **Publish Point** (clic en el mapa). Para enviar la ruta al coche puedes:
   - **Opción A (interfaz):** En otra terminal del contenedor: `ros2 run qcar2_autonomy waypoint_send_gui` → se abre una ventana con el botón **"Enviar ruta al coche"**.
   - **Opción B (terminal):** En la terminal donde corre `waypoint_visual_launch`, pulsa **Enter**.

---

## Waypoints desde archivo (bridge Python/YAML)

Si prefieres **definir los puntos en un archivo o en código** en lugar de hacer clic en RViz (el mapa lo sigues viendo en RViz):

1. Copia y edita el archivo de ejemplo:
   ```bash
   # En el contenedor, ruta típica:
   # src/qcar2_autonomy/config/waypoints_example.yaml
   ```
   Formato YAML:
   ```yaml
   waypoints:
     - [0.0, 0.0]
     - [2.0, 0.0]
     - [2.0, 2.0]
     - [0.0, 0.0]
   ```
   (Coordenadas en el frame `map`, en metros. Puedes obtenerlas haciendo un clic en RViz con Publish Point y leyendo la posición que imprime `waypoint_visual_node`.)

2. Con el **waypoint_follower_controller** y RViz ya corriendo, en otra terminal:
   ```bash
   ros2 run qcar2_autonomy waypoint_publish_from_file --ros-args -p waypoints_file:=/ruta/completa/a/mis_waypoints.yaml
   ```
   Si no pasas `waypoints_file`, publica un cuadrado por defecto (1.5 m).

3. En RViz verás la **línea verde (Path)** y las **esferas (WaypointMarkers)**; el coche seguirá la ruta. Así tienes: mapa visual en RViz + waypoints definidos en archivo/Python.

---

## Mapa en Python + clic para waypoints (más fácil)

Puedes **ver el mismo mapa del escenario (QLabs/Cartographer) en una ventana Python** y colocar waypoints con clic, sin abrir RViz:

1. T1 y T2 corriendo (mapa en `/map`). En otra terminal del contenedor: **waypoint_follower_controller** (por ejemplo con `ros2 launch qcar2_autonomy demo_entrega_launch.py` pero sin el demo_entrega_node, o lanzando solo el follower).
2. En otra terminal del contenedor:
   ```bash
   ros2 run qcar2_autonomy waypoint_map_gui
   ```
3. Se abre una ventana con el **mapa** (el de Cartographer del escenario). **Clic** en el mapa = añadir waypoint (se dibujan en verde). Botón **"Enviar ruta al coche"** = publica en `/waypoints_path` y el coche sigue. **"Borrar"** = vacía la lista para una nueva ruta.

Requisitos en el contenedor:
- **Ver el mapa en ventana (clic visual):** instalar tkinter (paquete del sistema, no pip):
  ```bash
  sudo apt-get update && sudo apt-get install -y python3-tk
  ```
  Luego `ros2 run qcar2_autonomy waypoint_map_gui` abrirá una ventana con el mapa; haz clic para añadir waypoints y "Enviar ruta al coche".
- Si no instalas `python3-tk`, el mismo comando usa **modo terminal** (escribes `x y` y `enviar`).
- `pip install matplotlib` si no está (para dibujar el mapa).

---

## Resumen

- **Terminal 1:** `./run_t1.sh` (desde la raíz del proyecto). Un solo comando.
- **Terminal 2:** `./run_t2.sh` (desde la raíz del proyecto). Un solo comando.
- **Terminal 3:** entrar al contenedor, luego `source install/setup.bash` y el nodo o launch que toque.
- **RViz:** abrirlo en **otra terminal**, y en el host tener hecho `xhost +local:` si hace falta.
