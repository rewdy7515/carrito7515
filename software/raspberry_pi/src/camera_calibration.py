"""Proyección geométrica de puntos de imagen al suelo.

Convención:

    u: píxel horizontal, izquierda -> derecha
    v: píxel vertical, arriba -> abajo
    X: centímetros hacia delante del carro
    Y: centímetros hacia la derecha del carro
    pitch positivo: cámara inclinada hacia abajo

El archivo de configuración contiene una lista de puntos ``(u, v)``. El punto
``angle`` central calcula el pitch; los puntos ``vertical`` ajustan ``fy`` /
``FOV vertical`` y los puntos ``horizontal`` ajustan ``fx`` / ``FOV
horizontal``. Los puntos ``distance`` solo sirven para comprobar el resultado.
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2


DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / "config" / "camera_calibration.json"
DEFAULT_POINTS = [
    {"name": "angle_center", "role": "angle", "u": 160, "v": 120},
    {"name": "horizontal_1", "role": "horizontal", "u": 80, "v": 120},
    {"name": "horizontal_2", "role": "horizontal", "u": 120, "v": 120},
    {"name": "horizontal_3", "role": "horizontal", "u": 200, "v": 120},
    {"name": "horizontal_4", "role": "horizontal", "u": 240, "v": 120},
    {"name": "vertical_1", "role": "vertical", "u": 160, "v": 80},
    {"name": "vertical_2", "role": "vertical", "u": 160, "v": 100},
    {"name": "vertical_3", "role": "vertical", "u": 160, "v": 140},
    {"name": "vertical_4", "role": "vertical", "u": 160, "v": 160},
]


@dataclass(frozen=True)
class CameraGeometry:
    width_px: int = 320
    height_px: int = 240
    horizontal_fov_deg: float = 70.4
    vertical_fov_deg: float = 55.0
    height_cm: float = 8.8
    pitch_deg: float = 20.0
    camera_forward_offset_cm: float = 0.0
    camera_lateral_offset_cm: float = 0.0
    focus_value: float | None = None

    @property
    def focal_x_px(self) -> float:
        return self.width_px / (2 * math.tan(math.radians(self.horizontal_fov_deg) / 2))

    @property
    def focal_y_px(self) -> float:
        return self.height_px / (2 * math.tan(math.radians(self.vertical_fov_deg) / 2))

    @property
    def center_x_px(self) -> float:
        return (self.width_px - 1) / 2

    @property
    def center_y_px(self) -> float:
        return (self.height_px - 1) / 2


def pixel_to_ground(
    u: float, v: float, camera: CameraGeometry, focal_y_px: float | None = None
) -> tuple[float, float] | None:
    """Convierte un píxel en ``(X, Y)`` sobre el suelo.

    El rayo vertical se obtiene del FOV vertical y del píxel ``v``. El pitch
    se suma al ángulo del rayo. Si el rayo no apunta al suelo, el punto queda
    fuera de la zona proyectable y se devuelve ``None``.
    """
    focal_y = camera.focal_y_px if focal_y_px is None else focal_y_px
    horizontal_ray = (u - camera.center_x_px) / camera.focal_x_px
    vertical_ray_angle = math.atan2(
        v - camera.center_y_px,
        focal_y,
    )
    down_angle = math.radians(camera.pitch_deg) + vertical_ray_angle
    if down_angle <= 0:
        return None
    x_cm = camera.height_cm / math.tan(down_angle)
    y_cm = x_cm * horizontal_ray
    return x_cm, y_cm


def image_to_ground(u: float, v: float, camera: CameraGeometry) -> tuple[float, float] | None:
    return pixel_to_ground(u, v, camera)


def camera_with_focal_y(camera: CameraGeometry, focal_y_px: float) -> CameraGeometry:
    vertical_fov_deg = math.degrees(
        2.0 * math.atan(camera.height_px / (2.0 * focal_y_px))
    )
    return CameraGeometry(
        **{**camera.__dict__, "vertical_fov_deg": vertical_fov_deg}
    )


def camera_with_focal_x(camera: CameraGeometry, focal_x_px: float) -> CameraGeometry:
    horizontal_fov_deg = math.degrees(
        2.0 * math.atan(camera.width_px / (2.0 * focal_x_px))
    )
    return CameraGeometry(
        **{**camera.__dict__, "horizontal_fov_deg": horizontal_fov_deg}
    )


def set_capture_focus(capture, focus_value: float | None) -> float | None:
    """Desactiva autofocus y aplica el valor UVC de enfoque, si es compatible."""
    if focus_value is None:
        return None
    value = max(0.0, min(255.0, float(focus_value)))
    try:
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        if not capture.set(cv2.CAP_PROP_FOCUS, value):
            return None
        actual = capture.get(cv2.CAP_PROP_FOCUS)
        return value if actual is None or actual < 0 else float(actual)
    except (AttributeError, TypeError, ValueError):
        return None


def project_points(points: list[dict], camera: CameraGeometry) -> list[dict]:
    projected = []
    for point in points:
        u, v = float(point["u"]), float(point["v"])
        ground = image_to_ground(u, v, camera)
        ground_distance = None if ground is None else math.hypot(*ground)
        projected.append({
            "name": point.get("name", "point"),
            "number": point.get("number"),
            "role": point.get("role", "distance"),
            "u": u,
            "v": v,
            "known_distance_cm": point.get("known_distance_cm"),
            "X_cm": None if ground is None else round(ground[0], 3),
            "Y_cm": None if ground is None else round(ground[1], 3),
            "distance_cm": None if ground_distance is None else round(ground_distance, 3),
        })
    return projected


def load_config(path: Path) -> tuple[CameraGeometry, list[dict]]:
    if not path.exists():
        camera = CameraGeometry()
        return camera, [{**point, "number": index} for index, point in enumerate(DEFAULT_POINTS, 1)]
    data = json.loads(path.read_text(encoding="utf-8"))
    camera_data = data.get("camera", data)
    camera = CameraGeometry(
        width_px=int(camera_data.get("width_px", 320)),
        height_px=int(camera_data.get("height_px", 240)),
        horizontal_fov_deg=float(camera_data.get("horizontal_fov_deg", 70.4)),
        vertical_fov_deg=float(camera_data.get("vertical_fov_deg", 55.0)),
        height_cm=float(camera_data.get("height_cm", 8.8)),
        pitch_deg=float(camera_data.get("pitch_deg", 20.0)),
        camera_forward_offset_cm=float(camera_data.get("camera_forward_offset_cm", 0.0)),
        camera_lateral_offset_cm=float(camera_data.get("camera_lateral_offset_cm", 0.0)),
        focus_value=(None if camera_data.get("focus_value") in (None, "") else float(camera_data["focus_value"])),
    )
    points = data.get("points", DEFAULT_POINTS.copy())
    normalized = []
    for point in points:
        item = dict(point)
        # Migra el punto central creado por la versión anterior: estaba en
        # v=180 y no representaba el centro óptico de una imagen 320x240.
        if item.get("name") == "center" and item.get("u") == 160 and item.get("v") == 180:
            item.update(name="angle_center", role="angle", v=120)
        elif item.get("name") == "angle_center":
            item["role"] = "angle"
        else:
            item.setdefault("role", "distance")
        normalized.append(item)
    # Mantiene puntos personalizados y completa la cuadrícula mínima de cuatro
    # puntos horizontales y cuatro verticales cuando se usa una configuración
    # anterior.
    horizontal_u = (80, 120, 200, 240)
    vertical_v = (80, 100, 140, 160)
    for index, u in enumerate(horizontal_u, 1):
        if sum(point.get("role") == "horizontal" for point in normalized) < 4:
            normalized.append({"name": f"horizontal_{index}", "role": "horizontal", "u": u, "v": 120})
    for index, v in enumerate(vertical_v, 1):
        if sum(point.get("role") == "vertical" for point in normalized) < 4:
            normalized.append({"name": f"vertical_{index}", "role": "vertical", "u": 160, "v": v})
    for number, point in enumerate(normalized, 1):
        point["number"] = number
    return camera, normalized


def save_config(path: Path, camera: CameraGeometry, points: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "camera": {
            "width_px": camera.width_px,
            "height_px": camera.height_px,
            "horizontal_fov_deg": camera.horizontal_fov_deg,
            "vertical_fov_deg": camera.vertical_fov_deg,
            "height_cm": camera.height_cm,
            "pitch_deg": camera.pitch_deg,
            "camera_forward_offset_cm": camera.camera_forward_offset_cm,
            "camera_lateral_offset_cm": camera.camera_lateral_offset_cm,
            "focal_x_px": round(camera.focal_x_px, 3),
            "focal_y_px": round(camera.focal_y_px, 3),
            "focus_value": camera.focus_value,
        },
        "points": points,
        "projected_points": project_points(points, camera),
    }
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def draw_points(frame, points: list[dict], camera: CameraGeometry) -> None:
    for point in project_points(points, camera):
        u, v = round(point["u"]), round(point["v"])
        color = (0, 255, 0) if point.get("role") == "angle" else (0, 220, 255)
        cv2.drawMarker(frame, (u, v), color, cv2.MARKER_CROSS, 12, 1)
        label = f"P{point.get('number', '?')}"
        cv2.putText(frame, label, (u + 7, v - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)


def camera_ground_from_car_distance(
    u: float, distance_cm: float, camera: CameraGeometry
) -> tuple[float, float]:
    """Obtiene el punto en coordenadas de cámara desde una distancia del carro."""
    horizontal_ray = (u - camera.center_x_px) / camera.focal_x_px
    offset_forward = camera.camera_forward_offset_cm
    offset_lateral = camera.camera_lateral_offset_cm
    a = 1.0 + horizontal_ray * horizontal_ray
    b = 2.0 * (offset_forward + horizontal_ray * offset_lateral)
    c = offset_forward * offset_forward + offset_lateral * offset_lateral - distance_cm * distance_cm
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0:
        raise ValueError("La distancia no coincide con el desplazamiento de la cámara")
    roots = ((-b + math.sqrt(discriminant)) / (2.0 * a), (-b - math.sqrt(discriminant)) / (2.0 * a))
    forward = next((value for value in roots if value > 0), None)
    if forward is None:
        raise ValueError("La distancia conocida queda detrás de la cámara")
    return forward, forward * horizontal_ray


def ground_distance_from_measurement(distance_cm: float) -> float:
    """Valida una distancia medida desde la base del centro del carro."""
    if distance_cm <= 0:
        raise ValueError("La distancia medida debe ser positiva")
    return distance_cm


def fit_vertical_focal_y(
    points: list[dict], camera: CameraGeometry, pitch_deg: float, height_cm: float
) -> tuple[float, list[dict]]:
    samples = []
    for point in points:
        if point.get("role") != "vertical" or point.get("known_distance_cm") in (None, ""):
            continue
        ground_distance = ground_distance_from_measurement(float(point["known_distance_cm"]))
        samples.append((point, ground_distance))
    if not samples:
        return camera.focal_y_px, []

    def error(focal_y_px: float) -> float:
        candidate = camera_with_focal_y(camera, focal_y_px)
        candidate = CameraGeometry(**{**candidate.__dict__, "pitch_deg": pitch_deg})
        total = 0.0
        for point, expected in samples:
            projected = pixel_to_ground(float(point["u"]), float(point["v"]), candidate, focal_y_px)
            if projected is None:
                return float("inf")
            total += (math.hypot(*projected) - expected) ** 2
        return total

    low, high = 50.0, 1000.0
    step = (high - low) / 190.0
    best = min(
        (low + index * step for index in range(191)),
        key=error,
    )
    radius = step
    for _ in range(32):
        left = max(20.0, best - radius)
        right = min(2000.0, best + radius)
        candidates = (left, (2 * left + right) / 3, (left + 2 * right) / 3, right)
        best = min(candidates, key=error)
        radius *= 0.55
    return best, [{"name": point["name"], "ground_distance_cm": round(expected, 3)} for point, expected in samples]


def fit_horizontal_focal_x(
    points: list[dict], camera: CameraGeometry, pitch_deg: float, height_cm: float
) -> float:
    samples = []
    for point in points:
        if point.get("role") != "horizontal" or point.get("known_distance_cm") in (None, ""):
            continue
        samples.append((point, ground_distance_from_measurement(float(point["known_distance_cm"]))))
    if not samples:
        return camera.focal_x_px

    def error(focal_x_px: float) -> float:
        candidate = camera_with_focal_x(camera, focal_x_px)
        candidate = CameraGeometry(**{**candidate.__dict__, "pitch_deg": pitch_deg})
        total = 0.0
        for point, expected in samples:
            projected = pixel_to_ground(float(point["u"]), float(point["v"]), candidate)
            if projected is None:
                return float("inf")
            total += (math.hypot(*projected) - expected) ** 2
        return total

    low, high = 50.0, 1000.0
    step = (high - low) / 190.0
    best = min((low + index * step for index in range(191)), key=error)
    radius = step
    for _ in range(32):
        left = max(20.0, best - radius)
        right = min(2000.0, best + radius)
        candidates = (left, (2 * left + right) / 3, (left + 2 * right) / 3, right)
        best = min(candidates, key=error)
        radius *= 0.55
    return best


class WebCalibrationState:
    def __init__(self, camera: CameraGeometry, points: list[dict], config_path: Path, capture):
        self.camera = camera
        self.points = points
        self.config_path = config_path
        self.capture = capture
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.latest_jpeg: bytes | None = None
        self.version = 0
        self.running = True

    def snapshot(self) -> tuple[CameraGeometry, list[dict]]:
        with self.lock:
            return self.camera, [dict(point) for point in self.points]

    def update_pitch(self, delta: float) -> CameraGeometry:
        with self.lock:
            self.camera = CameraGeometry(
                **{**self.camera.__dict__, "pitch_deg": self.camera.pitch_deg + delta}
            )
            return self.camera

    def update_focus(self, delta: float) -> CameraGeometry:
        with self.lock:
            current = self.camera.focus_value
            if current is None:
                detected = self.capture.get(cv2.CAP_PROP_FOCUS)
                current = 0.0 if detected is None or detected < 0 else float(detected)
            applied = set_capture_focus(self.capture, current + delta)
            if applied is None:
                raise ValueError("La cámara no permite controlar el enfoque desde OpenCV")
            self.camera = CameraGeometry(
                **{**self.camera.__dict__, "focus_value": applied}
            )
            return self.camera

    def calculate_pitch(
        self, height_cm: float, distance_cm: float, point_name: str = "angle_center"
    ) -> CameraGeometry:
        if height_cm <= 0 or distance_cm <= 0:
            raise ValueError("La altura y la distancia deben ser positivas")
        with self.lock:
            point = next((item for item in self.points if item.get("name") == point_name), None)
            if point is None:
                raise ValueError(f"No existe el punto de calibración: {point_name}")
            vertical_ray_angle = math.atan2(
                float(point["v"]) - self.camera.center_y_px,
                self.camera.focal_y_px,
            )
            self.camera = CameraGeometry(
                **{
                    **self.camera.__dict__,
                    "height_cm": height_cm,
                    "pitch_deg": math.degrees(math.atan2(height_cm, distance_cm))
                    - math.degrees(vertical_ray_angle),
                }
            )
            return self.camera

    def update_points(self, points: list[dict]) -> list[dict]:
        if not isinstance(points, list) or not points:
            raise ValueError("Debe existir al menos un punto")
        normalized = []
        for point in points:
            if not isinstance(point, dict):
                raise ValueError("Punto inválido")
            name = str(point.get("name", "point")).strip() or "point"
            role = str(point.get("role", "distance"))
            if role not in {"angle", "distance", "vertical", "horizontal"}:
                raise ValueError("role debe ser angle, distance, vertical u horizontal")
            u = float(point["u"])
            v = float(point["v"])
            if not 0 <= u < self.camera.width_px or not 0 <= v < self.camera.height_px:
                raise ValueError("Las coordenadas deben estar dentro de la imagen")
            known_distance = point.get("known_distance_cm")
            item = {"name": name, "role": role, "u": u, "v": v}
            if known_distance not in (None, ""):
                known_distance = float(known_distance)
                if known_distance <= 0:
                    raise ValueError("La distancia conocida debe ser positiva")
                item["known_distance_cm"] = known_distance
            normalized.append(item)
        for number, item in enumerate(normalized, 1):
            item["number"] = number
        with self.lock:
            self.points = normalized
            return [dict(point) for point in self.points]

    def calculate_pitch_from_points(
        self, height_cm: float, forward_offset_cm: float, lateral_offset_cm: float
    ) -> tuple[CameraGeometry, list[dict]]:
        if height_cm <= 0:
            raise ValueError("La altura de la cámara debe ser positiva")
        with self.lock:
            self.camera = CameraGeometry(
                **{
                    **self.camera.__dict__,
                    "height_cm": height_cm,
                    "camera_forward_offset_cm": 0.0,
                    "camera_lateral_offset_cm": 0.0,
                }
            )
            central = next((item for item in self.points if item.get("role") == "angle"), None)
            if central is None or central.get("known_distance_cm") in (None, ""):
                raise ValueError("Registra la distancia desde la base del centro del carro al punto central")
            central_ground = ground_distance_from_measurement(float(central["known_distance_cm"]))
            forward_distance, _ = camera_ground_from_car_distance(
                float(central["u"]), central_ground, self.camera
            )
            vertical_ray_angle = math.atan2(
                float(central["v"]) - self.camera.center_y_px,
                self.camera.focal_y_px,
            )
            pitch = math.degrees(math.atan2(height_cm, forward_distance))
            pitch -= math.degrees(vertical_ray_angle)
            self.camera = CameraGeometry(**{**self.camera.__dict__, "pitch_deg": pitch})
            fitted_focal_y, _ = fit_vertical_focal_y(
                self.points, self.camera, pitch, height_cm
            )
            self.camera = camera_with_focal_y(self.camera, fitted_focal_y)
            fitted_focal_x = fit_horizontal_focal_x(
                self.points, self.camera, pitch, height_cm
            )
            self.camera = camera_with_focal_x(self.camera, fitted_focal_x)
            results = []
            for point in self.points:
                diagonal = point.get("known_distance_cm")
                if diagonal in (None, ""):
                    continue
                expected_ground = ground_distance_from_measurement(float(diagonal))
                projected = pixel_to_ground(
                    float(point["u"]), float(point["v"]), self.camera
                )
                calculated_ground = None if projected is None else math.hypot(*projected)
                results.append({
                    "name": point["name"],
                    "role": point.get("role", "distance"),
                    "measured_ground_cm": round(float(diagonal), 3),
                    "ground_distance_cm": round(expected_ground, 3),
                    "calculated_ground_cm": None if calculated_ground is None else round(calculated_ground, 3),
                    "error_cm": None if calculated_ground is None else round(calculated_ground - expected_ground, 3),
                })
            return self.camera, results

    def save(self) -> None:
        camera, points = self.snapshot()
        save_config(self.config_path, camera, points)

    def update_frame(self, jpeg: bytes) -> None:
        with self.condition:
            self.latest_jpeg = jpeg
            self.version += 1
            self.condition.notify_all()

    def wait_for_frame(self, version: int) -> tuple[int, bytes]:
        with self.condition:
            while self.running and self.version <= version:
                self.condition.wait(timeout=1.0)
            if self.latest_jpeg is None:
                raise RuntimeError("No hay imagen de cámara")
            return self.version, self.latest_jpeg


class WebCalibrationHandler(BaseHTTPRequestHandler):
    state: WebCalibrationState
    server: ThreadingHTTPServer

    def send_body(self, body: bytes, content_type: str = "application/json; charset=utf-8") -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - nombre requerido por BaseHTTPRequestHandler
        if self.path == "/":
            page = """
            <!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
            <title>Calibración de cámara</title><style>
            body{margin:0;background:#202124;color:#eee;font-family:Arial;text-align:center}
            main{max-width:900px;margin:auto;padding:16px}.view{background:#111;border:2px solid #555}
            img{display:block;width:100%;height:auto}.controls{display:flex;gap:10px;justify-content:center;flex-wrap:wrap;margin:16px 0}
            button{font-size:20px;padding:10px 20px;border:0;border-radius:5px;background:#8ab4f8;cursor:pointer}
            #save{background:#9be49b}#status{min-height:24px;color:#9be49b}.note{color:#bbb}
            .points{margin-top:16px;text-align:left;overflow:auto}table{width:100%;border-collapse:collapse;background:#303134}
            th,td{padding:6px;border:1px solid #555;text-align:center}td input,td select{max-width:130px;width:100%;box-sizing:border-box}
            .distances{margin:16px 0;text-align:left;background:#303134;padding:12px;border-radius:5px}.distance-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:8px}.distance-grid label{display:flex;align-items:center;gap:6px}.distance-grid input{width:90px;box-sizing:border-box}
            .focus{background:#303134;padding:12px;margin:16px 0}.focus input{width:90px;font-size:18px}
            tr.selected{background:#455a64}
            </style></head><body><main><h1>Calibración de cámara</h1>
            <p class="note">La cámara muestra solo los puntos. Usa el mismo <b>u</b> central para puntos verticales y el mismo <b>v</b> central para puntos horizontales.</p>
            <section class="view"><img id="camera" src="/stream" onclick="selectNearest(event)"></section>
            <section class="focus"><h2>Enfoque de cámara</h2><button onclick="changeFocus(-5)">−</button><input id="focus" type="number" step="1" min="0" max="255" value="0"><button onclick="changeFocus(5)">+</button><button id="saveFocus" onclick="saveConfig()">Guardar enfoque</button><p class="note">El valor se guarda en <code>camera_calibration.json</code> y el robot lo aplica al iniciar. Si la cámara no admite control manual, aparecerá un error.</p></section>
            <section class="distances"><h2>Distancias reales por punto</h2><p class="note">Introduce la distancia horizontal real, medida sobre el suelo, desde la base del centro del carro hasta cada punto.</p><div id="distance-inputs" class="distance-grid">Cargando puntos...</div></section>
            <div class="controls"><button onclick="changePitch(-0.1)">− 0.1°</button>
            <button onclick="changePitch(0.1)">+ 0.1°</button><label>Altura cámara (cm)<input id="height" type="number" step="0.1" value="8.8"></label>
            <button onclick="calculatePitch()">Calcular pitch con puntos registrados</button><button id="save" onclick="saveConfig()">Guardar</button></div>
            <section class="points"><h2>Puntos de calibración</h2><div id="points-table">Cargando puntos...</div><button onclick="applyPoints()">Aplicar puntos</button></section>
            <div id="status">Cargando...</div><p class="note">Archivo: config/camera_calibration.json</p></main>
            <script>
            const status=document.getElementById('status');
            function show(v){status.textContent='pitch = '+Number(v.camera.pitch_deg).toFixed(2)+'°';}
            function esc(v){return String(v).replace(/[&<>\"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;',"'":'&#39;'}[c]))}
            function renderPoints(points,validation={}){document.getElementById('points-table').innerHTML='<table><tr><th>#</th><th>Nombre</th><th>Tipo</th><th>u</th><th>v</th><th>Suelo esperado</th><th>Suelo calculado</th><th>Error</th></tr>'+points.map((p,i)=>{const q=validation[p.name]||{};return '<tr data-i="'+i+'"><td>P'+(p.number??i+1)+'</td><td><input class="name" value="'+esc(p.name)+'"></td><td><select class="role"><option value="angle" '+(p.role==='angle'?'selected':'')+'>angle (pitch)</option><option value="vertical" '+(p.role==='vertical'?'selected':'')+'>vertical (fy)</option><option value="horizontal" '+(p.role==='horizontal'?'selected':'')+'>horizontal (fx)</option><option value="distance" '+(p.role==='distance'?'selected':'')+'>distance (validación)</option></select></td><td><input class="u" type="number" step="1" value="'+p.u+'"></td><td><input class="v" type="number" step="1" value="'+p.v+'"></td><td>'+(q.ground_distance_cm??'—')+' cm</td><td>'+(q.calculated_ground_cm??p.distance_cm??'—')+' cm</td><td>'+(q.error_cm??'—')+' cm</td></tr>'}).join('')+'</table>';document.getElementById('distance-inputs').innerHTML=points.map((p,i)=>'<label>P'+(p.number??i+1)+' '+esc(p.name)+'<input class="distance-known" data-i="'+i+'" type="number" step="0.1" min="0" placeholder="cm" value="'+(p.known_distance_cm??'')+'"></label>').join('')}
            async function refresh(){const data=await (await fetch('/api/calibration')).json();show(data);document.getElementById('height').value=data.camera.height_cm;document.getElementById('focus').value=data.camera.focus_value??'';renderPoints(data.points)}
            async function changePitch(delta){const r=await fetch('/api/pitch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({delta})});show(await r.json())}
            async function changeFocus(delta){const r=await fetch('/api/focus',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({delta})});const v=await r.json();if(v.camera){document.getElementById('focus').value=v.camera.focus_value;status.textContent='Enfoque = '+Number(v.camera.focus_value).toFixed(0);}else status.textContent=v.error||'La cámara no permite ajustar el enfoque'}
            async function calculatePitch(){await applyPoints();const r=await fetch('/api/calculate-pitch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({height_cm:+document.getElementById('height').value,forward_offset_cm:0,lateral_offset_cm:0})});const v=await r.json();if(v.camera){const validation={};(v.point_results||[]).forEach(q=>validation[q.name]=q);const data=await (await fetch('/api/calibration')).json();renderPoints(data.points,validation);status.textContent='pitch = '+Number(v.camera.pitch_deg).toFixed(2)+'° | fx = '+Number(v.camera.focal_x_px).toFixed(2)+' px | FOV H = '+Number(v.camera.horizontal_fov_deg).toFixed(2)+'° | fy = '+Number(v.camera.focal_y_px).toFixed(2)+' px | FOV V = '+Number(v.camera.vertical_fov_deg).toFixed(2)+'°';}else status.textContent=v.error||'Error al calcular'}
            function currentPoints(){const distances=[...document.querySelectorAll('.distance-known')];return [...document.querySelectorAll('#points-table tr[data-i]')].map(row=>({name:row.querySelector('.name').value,role:row.querySelector('.role').value,u:+row.querySelector('.u').value,v:+row.querySelector('.v').value,known_distance_cm:distances[Number(row.dataset.i)]?.value||''}))}
            async function applyPoints(){const r=await fetch('/api/points',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({points:currentPoints()})});const v=await r.json();if(v.points){renderPoints(v.points);status.textContent='Puntos aplicados';}else status.textContent=v.error||'Error al aplicar puntos'}
            function selectNearest(event){const img=event.currentTarget;const box=img.getBoundingClientRect();const u=(event.clientX-box.left)*320/box.width;const v=(event.clientY-box.top)*240/box.height;const rows=[...document.querySelectorAll('#points-table tr[data-i]')];let best=null;for(const row of rows){const du=u-Number(row.querySelector('.u').value),dv=v-Number(row.querySelector('.v').value),d=du*du+dv*dv;if(!best||d<best.d)best={row,d}}if(best&&best.d<=30*30){rows.forEach(row=>row.classList.remove('selected'));best.row.classList.add('selected');document.querySelector('.distance-known[data-i="'+best.row.dataset.i+'"]').focus()}}
            async function saveConfig(){const r=await fetch('/api/save',{method:'POST'});const v=await r.json();status.textContent=v.saved?'Guardado':'Error al guardar';setTimeout(refresh,1200)}
            refresh();
            </script></body></html>
            """.encode("utf-8")
            self.send_body(page, "text/html; charset=utf-8")
            return
        if self.path == "/api/calibration":
            camera, points = self.state.snapshot()
            camera_data = {
                **camera.__dict__,
                "focal_x_px": camera.focal_x_px,
                "focal_y_px": camera.focal_y_px,
                "horizontal_fov_deg": camera.horizontal_fov_deg,
                "vertical_fov_deg": camera.vertical_fov_deg,
            }
            self.send_body(json.dumps({"camera": camera_data, "points": project_points(points, camera)}).encode())
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        version = 0
        try:
            while self.state.running:
                version, jpeg = self.state.wait_for_frame(version)
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, RuntimeError):
            return

    def do_POST(self) -> None:  # noqa: N802 - nombre requerido por BaseHTTPRequestHandler
        try:
            if self.path == "/api/pitch":
                length = int(self.headers.get("Content-Length", "0"))
                values = json.loads(self.rfile.read(length))
                delta = float(values.get("delta", 0.0))
                if not -2.0 <= delta <= 2.0:
                    raise ValueError("delta fuera de rango")
                camera = self.state.update_pitch(delta)
                self.send_body(json.dumps({"camera": camera.__dict__}).encode())
                return
            if self.path == "/api/focus":
                length = int(self.headers.get("Content-Length", "0"))
                values = json.loads(self.rfile.read(length))
                delta = float(values.get("delta", 0.0))
                if not -25.0 <= delta <= 25.0:
                    raise ValueError("delta de enfoque fuera de rango")
                camera = self.state.update_focus(delta)
                self.send_body(json.dumps({"camera": camera.__dict__}).encode())
                return
            if self.path == "/api/calculate-pitch":
                length = int(self.headers.get("Content-Length", "0"))
                values = json.loads(self.rfile.read(length))
                camera, results = self.state.calculate_pitch_from_points(
                    float(values.get("height_cm", 0.0)),
                    float(values.get("forward_offset_cm", 0.0)),
                    float(values.get("lateral_offset_cm", 0.0)),
                )
                camera_data = {
                    **camera.__dict__,
                    "focal_x_px": camera.focal_x_px,
                    "focal_y_px": camera.focal_y_px,
                    "horizontal_fov_deg": camera.horizontal_fov_deg,
                    "vertical_fov_deg": camera.vertical_fov_deg,
                }
                self.send_body(json.dumps({"camera": camera_data, "used_points": len(results), "point_results": results}).encode())
                return
            if self.path == "/api/points":
                length = int(self.headers.get("Content-Length", "0"))
                values = json.loads(self.rfile.read(length))
                points = self.state.update_points(values.get("points"))
                camera, _ = self.state.snapshot()
                self.send_body(json.dumps({"points": project_points(points, camera)}).encode())
                return
            if self.path == "/api/save":
                self.state.save()
                self.send_body(b'{"saved":true}')
                return
        except (ValueError, TypeError, json.JSONDecodeError, OSError):
            self.send_error(400, "Solicitud de calibración inválida")
            return
        self.send_error(404)

    def log_message(self, format: str, *args: object) -> None:
        return


def run_web_camera(camera_source: int | str, config_path: Path, host: str, port: int) -> None:
    camera, points = load_config(config_path)
    capture = cv2.VideoCapture(camera_source)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera.width_px)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.height_px)
    if not capture.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara: {camera_source}")
    if camera.focus_value is not None:
        applied_focus = set_capture_focus(capture, camera.focus_value)
        if applied_focus is not None:
            camera = CameraGeometry(**{**camera.__dict__, "focus_value": applied_focus})

    state = WebCalibrationState(camera, points, config_path, capture)
    handler = type("BoundWebCalibrationHandler", (WebCalibrationHandler,), {"state": state})
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"Calibración web lista: http://pirobot.local:{server.server_port}")
    print("Usa +/− para pitch, Guardar para escribir camera_calibration.json y Ctrl+C para salir.")
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                time.sleep(0.01)
                continue
            current_camera, current_points = state.snapshot()
            draw_points(frame, current_points, current_camera)
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                state.update_frame(encoded.tobytes())
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        with state.condition:
            state.condition.notify_all()
        server.shutdown()
        capture.release()


def run_camera(camera_source: int | str, config_path: Path) -> None:
    camera, points = load_config(config_path)
    capture = cv2.VideoCapture(camera_source)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera.width_px)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.height_px)
    if not capture.isOpened():
        raise RuntimeError(f"No se pudo abrir la cámara: {camera_source}")
    if camera.focus_value is not None:
        applied_focus = set_capture_focus(capture, camera.focus_value)
        if applied_focus is not None:
            camera = CameraGeometry(**{**camera.__dict__, "focus_value": applied_focus})
    window = "Camera calibration"
    cv2.namedWindow(window)
    print("Edita los puntos en el JSON. +/- cambia pitch en vivo; S guarda; Q/Esc sale.")
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                continue
            draw_points(frame, points, camera)
            cv2.imshow(window, frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("+"), ord("=")):
                camera = CameraGeometry(**{**camera.__dict__, "pitch_deg": camera.pitch_deg + 0.1})
            elif key in (ord("-"), ord("_")):
                camera = CameraGeometry(**{**camera.__dict__, "pitch_deg": camera.pitch_deg - 0.1})
            elif key in (ord("s"), ord("S")):
                save_config(config_path, camera, points)
                print(f"Guardado: {config_path}")
            elif key in (27, ord("q"), ord("Q")):
                save_config(config_path, camera, points)
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", default="0")
    parser.add_argument("--config", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--print-only", action="store_true")
    parser.add_argument("--web", action="store_true", help="Ejecuta la calibración desde un navegador")
    parser.add_argument("--web-host", default="0.0.0.0")
    parser.add_argument("--web-port", type=int, default=8000)
    args = parser.parse_args()
    camera_source = int(args.camera) if str(args.camera).isdigit() else args.camera
    camera, points = load_config(args.config)
    if args.print_only:
        print(json.dumps(project_points(points, camera), indent=2))
        return
    if args.web:
        run_web_camera(camera_source, args.config, args.web_host, args.web_port)
        return
    run_camera(camera_source, args.config)


if __name__ == "__main__":
    main()
