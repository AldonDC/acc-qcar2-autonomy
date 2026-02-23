#!/usr/bin/env python3
"""
Prueba rápida de la homografía: carga calibración (pista_qcar2.png) y convierte
pixels -> mundo. Ejecutar tras abrir QLabs para comparar si las coordenadas cuadran.

  ros2 run qcar2_autonomy test_homography_tool
"""
import sys


def main():
    try:
        from autonomy.homography import (
            load_calibration,
            pixel_to_world,
            world_to_pixel,
            get_circuit_image_path,
            get_circuit_image_size,
        )
    except ImportError:
        print("Ejecuta desde el workspace con: source install/setup.bash")
        print("  ros2 run qcar2_autonomy test_homography_tool")
        sys.exit(1)

    print("=== Prueba homografía (pista_qcar2.png -> mundo) ===\n")

    path = get_circuit_image_path()
    size = get_circuit_image_size()
    print(f"Imagen circuito: {path or 'no encontrada'}")
    print(f"Tamaño (px):    {size or 'N/A'}\n")

    if not load_calibration():
        print("ERROR: No se pudo cargar homography_calibration.yaml")
        sys.exit(1)
    print("Calibración cargada OK.\n")

    # Esquinas y centro (pixels) -> mundo (metros)
    puntos = [
        ("Esquina sup. izq (0, 0)", 0, 0),
        ("Esquina sup. der (1152, 0)", 1152, 0),
        ("Esquina inf. der (1152, 896)", 1152, 896),
        ("Esquina inf. izq (0, 896)", 0, 896),
        ("Centro (576, 448)", 576, 448),
    ]
    print("Pixel -> Mundo (metros, frame map/QLabs):")
    print("-" * 50)
    for nombre, px, py in puntos:
        wx, wy = pixel_to_world(px, py)
        print(f"  {nombre}")
        print(f"    -> world = ({wx:.3f}, {wy:.3f}) m")
    print()

    # Redondo: mundo -> pixel
    print("Redondo (world -> pixel): centro (0, 2.19) m -> pixel")
    px, py = world_to_pixel(0.0, 2.19)
    print(f"  -> pixel = ({px:.1f}, {py:.1f})")
    print()
    print("Compara estos valores con la posición del circuito en QLabs.")
    print("Si no cuadran, ajusta los 'world' en config/homography_calibration.yaml")
    return 0


if __name__ == "__main__":
    sys.exit(main())
