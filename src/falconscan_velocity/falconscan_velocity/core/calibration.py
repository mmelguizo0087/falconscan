"""Calibración del montaje con el piso vacío (docs/maquina_estados_paso.md, sección 1).

Convención de instalación (equipo, 2026-10-21)
----------------------------------------------
- ``mount_height_m`` es la distancia piso -> centro óptico que mide el instalador y se toma como
  VERDADERA: la calibración no la cambia. Lo que el sensor ve de más o de menos es el **offset
  de rango** ``b`` (residual, después de la corrección propia de la unidad ``range_scale`` /
  ``range_bias_m`` de la prueba de pared, D11).
- La flecha de 0° del S2E se monta apuntando al piso: el rayo «abajo» NOMINAL es el del yaml
  (ROS 180° con sllidar_ros2). La calibración mide el **desvío de la flecha** respecto a la
  perpendicular al piso, α = ν − nominal, y lo informa siempre.
- Los valores del yaml son nominales y no se sobrescriben; los calibrados son «efectivos».

Modelo: los haces que caen en el piso cumplen ``r_i = H / cos(θ_i − ν) + b`` (ν = ángulo de la
perpendicular al piso). Primero se seleccionan con RANSAC + recta (``fit_floor``) y luego se
ajustan (ν, b) por Gauss-Newton con rechazo de residuos a 3σ (``fit_floor_offset``). Un offset
aditivo curva levemente los puntos (la componente perpendicular es H + b·cos φ), por eso no basta
la distancia de la recta.

Inclinación: sensor girado vs. piso con pendiente
-------------------------------------------------
En 2D, un sensor girado dentro de su plano y un piso con pendiente a lo largo del avance son
INDISTINGUIBLES mirando solo el piso. La rotación se mide respecto al piso (decidido): el camión
rueda sobre él, y una traslación del camión sigue siendo una traslación pura de h(x). Separarlas
(plomada) exige la pendiente medida aparte (``floor_slope_rad``); con ella, además, el piso queda
a H·cos(s) en perpendicular y ``b`` no absorbe la pendiente.

Lo que esta calibración NO ve
-----------------------------
- Guiñada (plano de escaneo no paralelo al avance): el piso se ve igual. Hace que el
  desplazamiento medido sea Δ/cos(ψ) (5° -> +0.4 %). Se detecta con un objeto de largo conocido
  o con desplazamientos conocidos (etapa 2).
- Alabeo (plano no vertical, giro alrededor del eje de avance): escala las alturas por
  1/cos(α) y corre lateralmente el punto de impacto. Pequeño para α de pocos grados.
"""
from __future__ import annotations

import math
import warnings
from contextlib import contextmanager
from dataclasses import dataclass, field

import numpy as np

from .geometry import MountGeometry, polar_to_xz


@contextmanager
def _sin_avisos():
    """Silencia «All-NaN slice» / «Degrees of freedom» de las columnas sin retorno."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        yield


def _ang_diff(a, b):
    """a − b envuelto a [−π, π)."""
    return (np.asarray(a) - b + math.pi) % (2 * math.pi) - math.pi


def offset_from_down(down_angle_rad: float, angle_direction: int) -> float:
    """angle_offset_rad tal que el rayo `down_angle_rad` quede en phi = −90° (en [−π, π))."""
    return float(_ang_diff(-math.pi / 2 - angle_direction * down_angle_rad, 0.0))


@dataclass(frozen=True)
class FloorFit:
    """Recta del piso en el marco del escaneo (ángulos del LaserScan)."""

    height_m: float              # distancia perpendicular del centro óptico a la recta
    down_angle_rad: float        # ángulo del escaneo de la normal hacia el piso, en [0, 2π)
    height_sigma_m: float        # incertidumbres 1σ aproximadas (residuos independientes)
    angle_sigma_rad: float
    rms_m: float                 # residuo RMS de los puntos del piso
    n_points: int
    inliers: np.ndarray = field(repr=False)   # máscara sobre los rayos de entrada


def fit_floor(
    ranges: np.ndarray,
    angles: np.ndarray,
    height_ref_m: float,
    down_guess_rad: float,
    *,
    height_tol_m: float = 0.25,
    max_tilt_rad: float = math.radians(25.0),
    inlier_tol_m: float = 0.02,
    max_incidence_rad: float = math.radians(65.0),
    trials: int = 400,
    seed: int = 0,
) -> FloorFit | None:
    """Ajusta la recta del piso: RANSAC con restricciones y luego mínimos cuadrados totales.

    Una recta candidata solo cuenta como piso si su normal está a menos de `max_tilt_rad` de
    `down_guess_rad` y su distancia a menos de `height_tol_m` de `height_ref_m`: así una pared o
    una mesa no se confunden con el piso. Se descartan los rayos con incidencia más rasante que
    `max_incidence_rad` (huella alargada, más ruido y sesgo). Los NaN se ignoran, nunca se rellenan.
    Devuelve None si no hay una recta que cumpla.
    """
    r = np.asarray(ranges, dtype=np.float64)
    a = np.asarray(angles, dtype=np.float64)
    valid = np.isfinite(r) & (r > 0)
    P = np.stack([r * np.cos(a), r * np.sin(a)], 1)
    ventana = valid & (np.abs(_ang_diff(a, down_guess_rad)) < max_tilt_rad + max_incidence_rad)
    idx = np.flatnonzero(ventana)
    if idx.size < 10:
        return None

    # --- RANSAC: pares de puntos -> recta (n, d) con n hacia el piso
    rng = np.random.default_rng(seed)
    i, j = idx[rng.integers(0, idx.size, trials)], idx[rng.integers(0, idx.size, trials)]
    t = P[j] - P[i]
    largo = np.hypot(t[:, 0], t[:, 1])
    ok = largo > 0.05
    n = np.stack([-t[ok, 1], t[ok, 0]], 1) / largo[ok, None]
    d = np.einsum("ij,ij->i", n, P[i[ok]])
    n[d < 0] *= -1
    d = np.abs(d)
    ok = (np.abs(_ang_diff(np.arctan2(n[:, 1], n[:, 0]), down_guess_rad)) < max_tilt_rad) \
        & (np.abs(d - height_ref_m) < height_tol_m)
    if not ok.any():
        return None
    n, d = n[ok], d[ok]
    Q = P[idx]
    dist = np.abs(Q @ n.T - d)                                         # puntos × candidatos
    # Incidencia: ángulo entre el rayo y la normal de cada candidato (cos = n·p/|p| = (n·p)/r)
    cos_inc = (Q @ n.T) / r[idx, None]
    puntaje = ((dist < inlier_tol_m) & (cos_inc > math.cos(max_incidence_rad))).sum(0)
    k = int(np.argmax(puntaje))
    if puntaje[k] < 10:
        return None
    nk, dk = n[k], d[k]

    # --- Refinamiento TLS sobre todos los rayos válidos (tramo de piso más largo = mejor ángulo)
    def seleccion(nv, dv, tol):
        e = P @ nv - dv
        with np.errstate(invalid="ignore"):
            return valid & (np.abs(e) < tol) & ((P @ nv) / np.where(valid, r, np.nan) > math.cos(max_incidence_rad))

    keep = seleccion(nk, dk, inlier_tol_m)
    for _ in range(4):
        if keep.sum() < 10:
            return None
        c = P[keep].mean(0)
        _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
        nk = vt[1] * (1.0 if c @ vt[1] > 0 else -1.0)
        dk = float(c @ nk)
        sigma = float((P[keep] @ nk - dk).std())
        nuevo = seleccion(nk, dk, min(inlier_tol_m, max(3 * sigma, 0.005)))
        if np.array_equal(nuevo, keep):
            break
        keep = nuevo
    if keep.sum() < 10:
        return None
    c = P[keep].mean(0)
    _, _, vt = np.linalg.svd(P[keep] - c, full_matrices=False)
    nk = vt[1] * (1.0 if c @ vt[1] > 0 else -1.0)
    tk = vt[0]
    dk = float(c @ nk)
    e = P[keep] @ nk - dk
    u = (P[keep] - c) @ tk
    m = int(keep.sum())
    sigma = float(e.std())
    s_ang = sigma / math.sqrt(max(float((u ** 2).sum()), 1e-12))
    s_d = math.sqrt(sigma ** 2 / m + (float(c @ tk) * s_ang) ** 2)   # el pie de la normal no es el centroide
    return FloorFit(height_m=dk, down_angle_rad=float(np.arctan2(nk[1], nk[0]) % (2 * math.pi)),
                    height_sigma_m=s_d, angle_sigma_rad=s_ang, rms_m=float(np.sqrt((e ** 2).mean())),
                    n_points=m, inliers=keep)


def fit_floor_offset(
    ranges: np.ndarray,
    angles: np.ndarray,
    height_m: float,
    fit: FloorFit,
    *,
    iterations: int = 15,
) -> tuple[float, float, float, float, float, int]:
    """Ajusta (ν, b) en ``r_i = H / cos(θ_i − ν) + b`` sobre los haces de piso de ``fit``.

    H queda fijo. Gauss-Newton partiendo de la recta, con dos rondas de rechazo de residuos a
    3σ (mín. 5 mm). Devuelve (ν, b, σ_ν, σ_b, rms, n); las σ suponen residuos independientes.
    """
    r = np.asarray(ranges, dtype=np.float64)
    a = np.asarray(angles, dtype=np.float64)
    ok = fit.inliers & np.isfinite(r)
    th, rr = a[ok], r[ok]
    nu, b = fit.down_angle_rad, fit.height_m - height_m
    for ronda in range(3):
        for _ in range(iterations):
            u = th - nu
            c = np.cos(u)
            pred = height_m / c + b
            J = np.stack([-height_m * np.tan(u) / c, np.ones_like(u)], 1)
            paso, *_ = np.linalg.lstsq(J, rr - pred, rcond=None)
            nu, b = nu + paso[0], b + paso[1]
            if abs(paso[0]) < 1e-9 and abs(paso[1]) < 1e-7:
                break
        e = rr - (height_m / np.cos(th - nu) + b)
        sigma = float(e.std())
        keep = np.abs(e) < max(3 * sigma, 0.005)
        if keep.all() or ronda == 2 or keep.sum() < 10:
            break
        th, rr = th[keep], rr[keep]
    u = th - nu
    J = np.stack([-height_m * np.tan(u) / np.cos(u), np.ones_like(u)], 1)
    e = rr - (height_m / np.cos(u) + b)
    sigma = float(e.std())
    cov = sigma ** 2 * np.linalg.inv(J.T @ J)
    return (float(nu % (2 * math.pi)), float(b), float(math.sqrt(cov[0, 0])), float(math.sqrt(cov[1, 1])),
            float(np.sqrt((e ** 2).mean())), int(th.size))


OK, CORREGIDO, FALLA = "OK", "CORREGIDO", "FALLA"


def _estado(valor: float, tol_ok: float, tol_max: float) -> str:
    return OK if abs(valor) <= tol_ok else (CORREGIDO if abs(valor) <= tol_max else FALLA)


@dataclass(frozen=True)
class MountCalibration:
    """Resultado de la calibración con el piso vacío (valores EFECTIVOS; el yaml no cambia)."""

    fit: FloorFit                    # recta: selección de los haces de piso y planitud
    angle_direction: int
    height_m: float                  # mount_height_m (cinta): la calibración no lo cambia
    perp_height_m: float             # H·cos(pendiente) si se dio la pendiente; si no, H
    range_offset_m: float            # b: offset residual sobre los rangos de entrada
    range_offset_sigma_m: float
    down_angle_rad: float            # ν: ángulo del escaneo de la perpendicular al piso
    down_angle_sigma_rad: float
    nominal_down_rad: float          # rayo «abajo» nominal (yaml; flecha 0° al piso)
    arrow_deviation_rad: float       # α = ν − nominal en el marco ROS (en la marca del S2E: −α)
    angle_offset_rad: float          # efectivo, con ν como «abajo»
    offset_status: str               # OK / CORREGIDO / FALLA
    angle_status: str
    model_rms_m: float               # residuo del modelo (ν, b)
    floor_slope_rad: float | None    # pendiente medida aparte (+ = sube hacia +x)
    plumb_angle_rad: float | None    # ángulo del escaneo de la vertical real (si hay pendiente)
    floor_x_range_m: tuple[float, float]   # tramo de piso visto en el ajuste (marco mundo)
    roi_coverage: float              # fracción de celdas de 10 cm de la ROI con piso visto
    roi_floor_dev_m: float           # máx |mediana de z| del piso por celda en la ROI (planitud)
    roi_object_x_m: tuple[float, float] | None   # tramo con algo por encima del piso en la ROI
    moving_fraction: float           # rayos del piso con variación temporal > 3 cm
    n_scans: int
    warnings: tuple[str, ...]        # avisos que no impiden operar
    failures: tuple[str, ...]        # causas de FALLA: solo REINTENTAR tras corregir el montaje

    @property
    def failed(self) -> bool:
        return bool(self.failures)

    @property
    def seen_height_m(self) -> float:
        """Distancia al piso que «ve» el sensor sin corregir el offset (H_visto = H + b)."""
        return self.perp_height_m + self.range_offset_m

    def range_bias_for(self, range_bias_m: float, range_scale: float) -> float:
        """range_bias_m efectivo: los rangos de entrada ya traían (r − range_bias_m)·range_scale."""
        return range_bias_m + self.range_offset_m / range_scale

    @property
    def geometry(self) -> MountGeometry:
        return MountGeometry(self.perp_height_m, self.angle_offset_rad, self.angle_direction)


def calibrate_mount(
    ranges: np.ndarray,
    angles: np.ndarray,
    height_ref_m: float,
    down_guess_rad: float,
    *,
    nominal_down_rad: float | None = None,
    angle_direction: int = 1,
    floor_slope_rad: float | None = None,
    roi_x_m: tuple[float, float] = (-3.0, 3.0),
    roi_z_max_m: float = 4.5,
    object_h_threshold_m: float = 0.4,
    offset_tol_ok_m: float = 0.03,
    offset_tol_max_m: float = 0.20,
    angle_tol_ok_rad: float = math.radians(0.5),
    angle_tol_max_rad: float = math.radians(5.0),
    **fit_kwargs,
) -> MountCalibration:
    """Calibra con escaneos del piso vacío (`ranges`: M rayos o N×M, ya corregidos en rango).

    ``height_ref_m`` = mount_height_m (cinta, fijo). ``nominal_down_rad`` = rayo «abajo» nominal
    contra el que se mide el desvío de la flecha (por defecto, ``down_guess_rad``). Con varios
    escaneos se ajusta la mediana temporal y se usa la dispersión por rayo para avisar si algo se
    movió. Lanza ValueError si no encuentra el piso.
    """
    pila = np.atleast_2d(np.asarray(ranges, dtype=np.float64))
    if pila.shape[0] > 1:
        with np.errstate(invalid="ignore"), _sin_avisos():
            r = np.nanmedian(pila, axis=0)
            dispersion = np.nanstd(pila, axis=0)
    else:
        r, dispersion = pila[0], np.zeros(pila.shape[1])
    a = np.asarray(angles, dtype=np.float64)
    nominal = down_guess_rad if nominal_down_rad is None else nominal_down_rad

    fit = fit_floor(r, a, height_ref_m, down_guess_rad, **fit_kwargs)
    if fit is None:
        raise ValueError(
            f"No encontré una recta de piso a {height_ref_m:.2f} m (±{fit_kwargs.get('height_tol_m', 0.25):.2f}) "
            f"cerca de la dirección «abajo» actual ({math.degrees(down_guess_rad) % 360:.1f}°). "
            "Revisa la altura medida, que el piso se vea y que el «rayo hacia abajo» aproximado sea correcto.")

    h_perp = height_ref_m * math.cos(floor_slope_rad) if floor_slope_rad is not None else height_ref_m
    nu, b, s_nu, s_b, rms_modelo, _ = fit_floor_offset(r, a, h_perp, fit)
    alfa = float(_ang_diff(nu, nominal))
    avisos: list[str] = []
    fallas: list[str] = []

    estado_b = _estado(b, offset_tol_ok_m, offset_tol_max_m)
    if estado_b == FALLA:
        fallas.append(f"El sensor ve el piso a {h_perp + b:.2f} m y el parámetro es {h_perp:.2f} m "
                      f"(offset {b * 100:+.0f} cm). Reinicie y revise el montaje del sensor y la altura medida.")
    estado_a = _estado(alfa, angle_tol_ok_rad, angle_tol_max_rad)
    if estado_a == FALLA:
        fallas.append(f"La flecha de 0° está desviada {math.degrees(-alfa):+.1f}° (en la marca del sensor) de la "
                      "perpendicular al piso: revise la fijación del sensor.")

    offset = offset_from_down(nu, angle_direction)
    g = MountGeometry(h_perp, offset, angle_direction)
    x, z = polar_to_xz(r - b, a, g)                              # z = altura sobre el piso calibrado

    plomada = None
    if floor_slope_rad is not None:
        # Abajo real en el marco del piso: phi = −90° − s  ->  theta = ν − dir·s
        plomada = float((nu - angle_direction * floor_slope_rad) % (2 * math.pi))

    # Tramo de piso visto y ROI
    xi = x[fit.inliers]
    rango_x = (float(xi.min()), float(xi.max()))
    x0, x1 = roi_x_m
    celdas = np.arange(x0, x1 + 1e-9, 0.1)
    en_roi = np.isfinite(x) & (x >= x0) & (x < x1)
    piso = en_roi & (np.abs(z) < 0.05)
    cubiertas = np.unique(np.floor((x[piso] - x0) / 0.1).astype(int))
    cobertura = cubiertas.size / max(celdas.size - 1, 1)
    dev = 0.0
    if piso.any():
        ic = np.floor((x[piso] - x0) / 0.1).astype(int)
        dev = max(abs(float(np.median(z[piso][ic == c]))) for c in cubiertas)
    if cobertura < 0.9:
        avisos.append(f"Solo se ve piso en el {cobertura * 100:.0f} % de la ROI: hay sombras o rayos sin "
                      "retorno (piso oscuro, mojado o muy rasante). Las celdas sin piso quedan NaN.")
    if dev > 0.02:
        avisos.append(f"El piso no es plano en la ROI: se aparta {dev * 100:.1f} cm de la recta. "
                      + ("Supera la mitad del umbral de objeto: ajusta el umbral o la ROI."
                         if dev > object_h_threshold_m / 2 else "Revisa rampas o cambios de pendiente."))
    alto = en_roi & (z > 0.10) & (z < roi_z_max_m)
    objeto = None
    if alto.sum() >= 5:
        objeto = (float(x[alto].min()), float(x[alto].max()))
        avisos.append(f"Hay algo sobre el piso dentro de la ROI (x = {objeto[0]:.2f} a {objeto[1]:.2f} m). "
                      "Si es fijo, la máquina de estados quedará OBSTRUIDA; si es temporal, retíralo y repite.")

    # Sobre todos los rayos que miran al piso, no solo los inliers: un rayo que cambió mucho
    # deja de estar sobre la recta de la mediana y no sería inlier
    cono = np.isfinite(r) & (np.abs(_ang_diff(a, nu)) < fit_kwargs.get("max_incidence_rad", math.radians(65.0)))
    moviles = float(np.mean(dispersion[cono] > 0.03)) if pila.shape[0] >= 3 and cono.any() else 0.0
    if moviles > 0.05:
        avisos.append(f"El {moviles * 100:.0f} % de los rayos del piso variaron más de 3 cm durante la "
                      "captura: algo se movió o el soporte vibra. Repite con la escena quieta.")

    return MountCalibration(
        fit=fit, angle_direction=angle_direction, height_m=height_ref_m, perp_height_m=h_perp,
        range_offset_m=b, range_offset_sigma_m=s_b, down_angle_rad=nu, down_angle_sigma_rad=s_nu,
        nominal_down_rad=nominal % (2 * math.pi), arrow_deviation_rad=alfa, angle_offset_rad=offset,
        offset_status=estado_b, angle_status=estado_a, model_rms_m=rms_modelo,
        floor_slope_rad=floor_slope_rad, plumb_angle_rad=plomada, floor_x_range_m=rango_x,
        roi_coverage=cobertura, roi_floor_dev_m=dev, roi_object_x_m=objeto, moving_fraction=moviles,
        n_scans=pila.shape[0], warnings=tuple(avisos), failures=tuple(fallas))


def detect_direction(
    ranges_empty: np.ndarray,
    ranges_object: np.ndarray,
    angles: np.ndarray,
    down_angle_rad: float,
    *,
    entry_side: int = -1,
    min_change_m: float = 0.10,
) -> tuple[int, float, int] | None:
    """angle_direction a partir de un objeto puesto del lado de ENTRADA del vehículo.

    Compara la mediana de escaneos con objeto contra el piso vacío: los rayos que acortan más
    de `min_change_m` son el objeto. Con angle_direction = +1, x = r·sin(theta − abajo); si el
    objeto queda del lado `entry_side` (−1: el vehículo entra por x negativo), el sentido es +1.
    Devuelve (angle_direction, x medio con sentido +1, n rayos) o None si no se ve el objeto.
    """
    def mediana(v):
        v = np.atleast_2d(np.asarray(v, dtype=np.float64))
        with np.errstate(invalid="ignore"), _sin_avisos():
            return np.nanmedian(v, axis=0)

    r0, r1 = mediana(ranges_empty), mediana(ranges_object)
    with np.errstate(invalid="ignore"):
        nuevo = np.isfinite(r1) & ~(r1 > r0 - min_change_m)     # más cerca, o donde antes no había retorno
    a = np.asarray(angles, dtype=np.float64)
    nuevo &= np.abs(_ang_diff(a, down_angle_rad)) < math.radians(80)
    n = int(nuevo.sum())
    if n < 5:
        return None
    x = r1[nuevo] * np.sin(a[nuevo] - down_angle_rad)
    xm = float(np.median(x))
    return (1 if np.sign(xm) == entry_side else -1), xm, n
