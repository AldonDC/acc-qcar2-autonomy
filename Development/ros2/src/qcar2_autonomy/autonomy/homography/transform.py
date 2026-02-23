"""
Transformación homográfica: pixels (imagen del circuito) <-> coordenadas cartesianas (mapa / QLabs).

Imagen de referencia: config/pista_qcar2.png (1152 x 896 px).

- getPerspectiveTransform: 4 puntos exactos (imagen ↔ mundo) -> matriz 3x3.
  Ver: https://shravankumar147.substack.com/p/homography-vs-perspective-transform
- findHomography: N>=4 puntos, RANSAC para outliers.
  Ver: https://docs.opencv.org/3.4/d1/de0/tutorial_py_feature_homography.html
- perspectiveTransform: aplicar H a puntos.
  Ver: https://www.geeksforgeeks.org/python/python-opencv-object-tracking-using-homography/
"""

import os
from typing import List, Tuple, Optional

import cv2
import numpy as np
import yaml

# Imagen del circuito usada para la homografía (misma que en config y circuit_map_publisher)
CIRCUIT_IMAGE_FILENAME = "pista_qcar2.png"

# Ruta por defecto al YAML de calibración (se puede cambiar con set_calibration_path)
_CALIBRATION_PATH: Optional[str] = None

# Matrices 3x3: pixel -> world y world -> pixel (tras load_calibration)
_H_pixel_to_world: Optional[np.ndarray] = None
_H_world_to_pixel: Optional[np.ndarray] = None


def set_calibration_path(path: str) -> None:
    """Establece la ruta al archivo YAML de calibración."""
    global _CALIBRATION_PATH
    _CALIBRATION_PATH = path


def get_circuit_image_path() -> Optional[str]:
    """
    Devuelve la ruta absoluta a la imagen del circuito (pista_qcar2.png).
    Busca en: calibration path dir, package share config, cwd/config.
    """
    candidates = []
    if _CALIBRATION_PATH and os.path.isfile(_CALIBRATION_PATH):
        base = os.path.dirname(_CALIBRATION_PATH)
        candidates.append(os.path.join(base, CIRCUIT_IMAGE_FILENAME))
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg = get_package_share_directory("qcar2_autonomy")
        candidates.append(os.path.join(pkg, "config", CIRCUIT_IMAGE_FILENAME))
    except Exception:
        pass
    for p in [os.getcwd(), os.path.join(os.getcwd(), "config")]:
        candidates.append(os.path.join(p, CIRCUIT_IMAGE_FILENAME))
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return None


def get_circuit_image_size() -> Optional[Tuple[int, int]]:
    """
    Carga pista_qcar2.png y devuelve (ancho, alto) en pixels.
    Útil para validar que los puntos de calibración están dentro de la imagen.
    """
    path = get_circuit_image_path()
    if not path:
        return None
    img = cv2.imread(path)
    if img is None:
        return None
    h, w = img.shape[:2]
    return (w, h)


def _ensure_float32(points: List[Tuple[float, float]]) -> np.ndarray:
    """Convierte lista de (x,y) en array (N,1,2) float32 para OpenCV."""
    arr = np.array(points, dtype=np.float32)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 2)
    return arr.reshape(-1, 1, 2)


def load_calibration(
    path: Optional[str] = None,
    use_ransac: bool = False,
    ransac_threshold: float = 5.0,
) -> bool:
    """
    Carga el archivo YAML de calibración y calcula la homografía.

    El YAML debe tener una lista de puntos con 'pixel' [x,y] y 'world' [x,y] (metros).
    Mínimo 4 puntos. Con 4 puntos se usa getPerspectiveTransform; con más, findHomography (RANSAC).

    Args:
        path: Ruta al YAML. Si no se pasa, usa la de set_calibration_path o la por defecto del paquete.
        use_ransac: Si True, siempre usa findHomography con RANSAC (útil si hay más de 4 puntos u outliers).
        ransac_threshold: Umbral en pixels para RANSAC (solo si use_ransac=True).

    Returns:
        True si se cargó y calculó H correctamente, False en caso contrario.
    """
    global _H_pixel_to_world, _H_world_to_pixel

    if path is None:
        path = _CALIBRATION_PATH
    if not path or not os.path.isfile(path):
        # Intentar ruta por defecto en config del paquete
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory("qcar2_autonomy")
            path = os.path.join(pkg_share, "config", "homography_calibration.yaml")
        except Exception:
            return False
        if not os.path.isfile(path):
            return False

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    points = data.get("points") or data.get("calibration_points") or []
    if len(points) < 4:
        return False

    pixels = _ensure_float32([p["pixel"] for p in points])
    world = _ensure_float32([p["world"] for p in points])

    if use_ransac or len(points) > 4:
        # findHomography: múltiples puntos, RANSAC (OpenCV tutorial / GeeksforGeeks)
        H, _ = cv2.findHomography(pixels, world, cv2.RANSAC, ransac_threshold)
        if H is None:
            return False
        _H_pixel_to_world = H
    else:
        # getPerspectiveTransform: exactamente 4 puntos (Substack)
        if len(points) != 4:
            # tomar los 4 primeros si hay más
            pixels = pixels[:4]
            world = world[:4]
        _H_pixel_to_world = cv2.getPerspectiveTransform(pixels, world)

    # Inversa para world -> pixel
    _H_world_to_pixel = np.linalg.inv(_H_pixel_to_world)
    return True


def pixel_to_world(px: float, py: float) -> Tuple[float, float]:
    """
    Convierte un punto de la imagen (pixels) a coordenadas del mundo (metros, frame del mapa).

    Debe haberse llamado load_calibration antes.
    """
    if _H_pixel_to_world is None:
        raise RuntimeError("Homografía no cargada. Llama a load_calibration() antes.")
    pt = np.array([[[px, py]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, _H_pixel_to_world)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


def world_to_pixel(wx: float, wy: float) -> Tuple[float, float]:
    """
    Convierte coordenadas del mundo (metros) a pixels en la imagen del circuito.
    """
    if _H_world_to_pixel is None:
        raise RuntimeError("Homografía no cargada. Llama a load_calibration() antes.")
    pt = np.array([[[wx, wy]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, _H_world_to_pixel)
    return float(out[0, 0, 0]), float(out[0, 0, 1])
