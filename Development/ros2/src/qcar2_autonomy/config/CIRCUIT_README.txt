Cómo poner la PISTA REAL de QLabs como fondo en el GUI
=======================================================

El mapa que ves por defecto puede ser el de ocupación (láser). Para que el fondo
sea la pista del circuito ACC que usas en QLabs (la misma que en la simulación):

1. Abre Quanser Interactive Labs con el escenario "ACC Self Driving Car Competition".

2. Pon la cámara en vista superior (desde arriba) para ver todo el circuito:
   carreteras, rotonda, aparcamientos, paso de cebras, etc.

3. Haz una captura de pantalla de esa vista (todo el circuito visible).

4. (Opcional) Recorta la imagen para dejar solo el circuito y un poco de margen.

5. Guarda la imagen con este nombre exacto en esta carpeta:
   pista_qcar1.png

   Ruta completa en tu proyecto:
   Development/ros2/src/qcar2_autonomy/config/pista_qcar1.png

   (Puedes guardarla desde el escritorio o Descargas y luego copiarla ahí, o
   "Guardar como" directamente en esa ruta.)

6. Recompila para que el paquete instale la imagen (opcional si trabajas desde src):
   cd /workspaces/isaac_ros-dev/ros2
   colcon build --packages-select qcar2_autonomy
   source install/setup.bash

7. Vuelve a abrir el GUI:
   ros2 run qcar2_autonomy waypoint_map_gui

   Verás la pista de QLabs de fondo. Haz clic en la pista para añadir waypoints;
   la trayectoria se dibuja en verde en el GUI y al pulsar "Enviar ruta al coche"
   el QCar sigue esa ruta en QLabs.

8. Si la ORIENTACIÓN del circuito en el GUI no coincide con QLabs (pista girada),
   usa circuit_rotation_deg: 0, 90, 180 o 270 hasta que se vea igual que en QLabs.
   Así la posición del coche (triángulo rojo) y los waypoints cuadran con la simulación.
   Ejemplo: rotación 90° → -p circuit_rotation_deg:=90

9. Por defecto el GUI usa las MISMAS DIMENSIONES que el mapa del QCar (/map):
   circuit_extent_from_map:=true. Si quieres fijar tú el extent: circuit_extent_from_map:=false
   y -p circuit_extent:="[-4.0, 4.0, -1.0, 4.0]".

10. Para que el triángulo rojo coincida exactamente con el QCAR en QLabs:
    - Calibración fácil: pulsa "Calibrar posición (clic donde está el QCAR)", luego haz clic en el mapa exactamente
      donde ves el coche en QLabs. El GUI mostrará el comando con los offsets; cierra el GUI, reinicia con ese comando.
    - Por defecto: pose_offset_x:=1.58, pose_offset_y:=0.26. Si aun se desvía, usa el botón Calibrar o ajusta a mano.
    - Orientación: pose_offset_theta (rad), ej. -p pose_offset_theta:=0.22

Resumen: copia tu captura del circuito QLabs → config/pista_qcar1.png
         y la trayectoria (línea verde + puntos) se verá sobre esa pista en el GUI.
