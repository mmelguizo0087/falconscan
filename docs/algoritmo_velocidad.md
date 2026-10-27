# Algoritmo de velocidad — decisiones de diseño y resultados en simulación

Basado en `docs/referencias/Reporte_Tecnico_Algoritmo_Velocidad_LiDAR_dToF.docx` (v1.0, 19-oct-2026).
Este documento registra **en qué se aparta la implementación del reporte y por qué**, y los
resultados de la matriz de ensayos sobre el simulador (`tools/evaluate_sim.py`).

> Los resultados son **sintéticos**: comparan variantes del algoritmo entre sí (ablación).
> Los criterios de aceptación reales se fijan con el sensor físico (reporte §8.5).

## Montaje asumido

LiDAR de velocidad en el pórtico, **plano de escaneo vertical que contiene el eje de avance**,
mirando hacia abajo. Marco mundo: `x` = avance, `z` = arriba, piso en `z = 0`.
`phi = angle_direction · theta + angle_offset`, `x = r cos(phi)`, `z = H + r sin(phi)`.

Sensor de referencia del simulador: RPLIDAR S2E (inferido del archivo `BOM_MVP_LiDAR_S2E.xlsx`),
10 Hz, 3200 muestras/vuelta (0.1125°), ruido de rango 1.5 cm. **Verificar con la hoja de datos real.**

## Decisiones

| # | Decisión | Reporte | Motivo |
|---|----------|---------|--------|
| D1 | Correlación sobre perfil **remuestreado en malla espacial h(x)**, no sobre h(θ) | §10 la deja abierta; §5.8 es "la etapa de mayor riesgo" | Una traslación del camión es traslación pura de h(x), no de h(θ). Así `Δs = k*·dx` sin ley de cosenos ni modelo calibrado F(k*). |
| D2 | Altura por celda = **máximo** de z | — | Desde arriba, la superficie superior caracteriza la carga; las caras verticales aportan puntos intermedios que no deben bajarla. |
| D3 | **Deskew por rayo** con `time_increment` (activo por defecto) | §5.1 lo menciona como condicional | Sin él, sesgo ∝ v²: +1 % a 3 m/s, +2.7 % a 5 m/s en simulación. Con él, < 0.3 %. |
| D4 | Comparar contra el escaneo de hace **N = 3** (`baseline_scans`) | §5.9 advierte el compromiso | Con Δt = 0.1 s el error de subcelda (~0.3 celdas) da σ_v ≈ 0.06 m/s. Con N = 3 baja ~3×. |
| D5 | Salida con timestamp en el **punto medio** del intervalo (`mid_stamp`) | — | v media de [t−Δt, t] corresponde a t−Δt/2. Con aceleración, usar el final da error sistemático (0.6 m/s² × 0.15 s ≈ 0.09 m/s). |
| D6 | Mediana con ventana incompleta: si los válidos discrepan > 0.15 m → inválido | §5.3 | `nanmedian` de 2 valores es un **promedio** y dejaba pasar medio outlier. |
| D7 | Segmentación exige tramos continuos ≥ 5 celdas | §5.5 (continuidad) | Outliers aislados se contaban como objeto. |
| D8 | Refinamiento subcelda parabólico sobre C(k) | no especificado | Sin él, la resolución de Δs es dx = 2 cm. |
| D9 | EMA solo para visualización | §5.10 | Con aceleración, el EMA empeora el RMSE de 0.026 a 0.12 m/s por retardo. Para integrar posición/volumen usar `velocity_mps` + `mid_stamp`. |
| D10 | Orden temporal de los rayos configurable (`scan_time_reversed`, `true` en el yaml) | — | `sllidar_ros2` (inverted:=false) publica `ranges` invertido: el índice 0 es el último rayo medido. Con el orden equivocado el deskew corrige al revés. Sesgo en simulación con escaneo invertido: sin el parámetro +1.0 / +2.6 / +6.0 % a 1.5 / 3 / 5 m/s (peor que sin deskew: +0.2 / +1.0 / +2.7 %); con el parámetro −0.6 / −0.3 / −0.2 %. |
| D11 | Corrección de rango `r_corr = (r − range_bias_m) · range_scale`, aplicada antes de la mediana | §6 fase 0 | Prueba de pared con la unidad 1 (20-oct-2026, sensor real, 20 s por distancia, recta ajustada a la pared): a 1.19 / 1.97 / 5.00 m (cinta) mide 1.1969 / 1.9822 / 5.0238 m. Ajuste lineal: `r_medido = 1.004265·d + 2.7 mm` (residuos ≤ 1.1 mm) → `range_scale = 0.995753`, `range_bias_m = 0.0027`. El offset de 2.7 mm no es significativo frente al error de la cinta (±3 mm); la escala sí. En simulación, sin corregir, ese error desplaza el sesgo de velocidad +0.5 / +0.4 / +0.2 % a 1.5 / 3 / 5 m/s. Valores por unidad: rehacer la prueba si se cambia de sensor. |
| D12 | Calibración de montaje en el **marco del piso** con la convención del equipo (2026-10-21): H = `mount_height_m` de la cinta (fijo), se ajustan el offset de rango `b` y el rayo perpendicular al piso ν en `r = H/cos(θ − ν) + b`; el desvío de la flecha 0° es α = ν − nominal (`core/calibration.py`, pestaña Calibración) | §6 | Con el piso solo, un sensor girado en su plano y un piso con pendiente son indistinguibles en 2D; no hace falta separarlos porque el camión avanza paralelo al piso (la rotación se mide respecto al piso, decidido). Con H fijo, un error δ de la cinta sale como `b ≈ −1.33 δ` y deja el piso curvado ±3–7 mm en la ROI para δ = 1–2 cm (sintético); α no se afecta. Un error de ángulo δ casi no afecta la velocidad (escala cos δ; NCC ignora el corrimiento vertical) pero levanta el piso en el borde de la ROI x·tan δ (5° → 26 cm a 3 m). Valores efectivos: el yaml guarda los nominales. No ve guiñada ni alabeo. Validado con escaneos sintéticos y el simulador. |
| D13 | **Ambos sentidos**: búsqueda simétrica `v_min_mps = −6` (antes −0.5), sentido por paso fijado en T5 por el borde de entrada, rapidez de avance `v_avance = sentido · v` y bandera `SENTIDO_INCONSISTENTE` (`pass_both_directions`, regla de `docs/maquina_estados_paso.md`) y referencia de signo solo a la salida (`invert_velocity_sign`). *Reemplazada en la integración del 2026-10-27 por D18 (búsqueda con `search_mode`), el PassTracker de Martín (siempre bidireccional) y D20 (signo solo para mostrar).* | — | El estimador ya era simétrico salvo el rango de búsqueda. `tools/evaluate_sim.py` (simulador, 2026-10-21), antes → después: hacia +x sin cambios (válidos, sesgo y RMSE iguales en 0.3–5 m/s); sentido contrario −1.5 / −3 / −5 m/s: 0 % → 73 / 71 / 67 % válidos, sesgo +0.008 / +0.010 / +0.009 m/s, RMSE 0.026 / 0.017 / 0.026; «frena 3→0.6 m/s» 73 → 92 % válidos (el límite de −0.5 m/s cortaba la búsqueda). Costo: ~5 → ~7.7 ms/escaneo. El signo del usuario no toca la geometría ni la ROI, por eso cambiarlo no reinicia nada. Sin validar con el sensor real. |
| D14 | El estimador **no se reinicia en los escaneos vacíos dentro de un paso** (ENTRANDO / EN_PASO / SALIENDO); se reinicia al salir (`core/pipeline.py`, `VelocityPipeline`). *Reemplazada el 2026-10-27 por D17 (`empty_reset_scans` + reinicio en FIN), que mide igual o mejor (ver D21).* | — | Decidido por el equipo (2026-10-21). `tools/evaluate_passes.py` (simulador, 6 camiones alternando sentido): sin escaneos vacíos dentro del paso, idéntico; con 5 / 10 / 20 % de escaneos «ciegos» (el camión no devuelve nada), pasos válidos 5 → 6 / 1 → 6 / 0 → 6 y escaneos OK 66 → 76 / 51 → 74 / 36 → 74 %; sin fragmentación en ningún caso. |
| D15 | Geometría nominal con la **flecha 0° del S2E al piso**: `angle_offset_rad = +π/2` (rayo ROS 180° al nadir); el simulador publica con la misma convención (`nadir_angle_deg = 180`) | — | sllidar_ros2 publica ROS = 180° − marca (prueba de pared). Con −π/2 (antes) el rayo al piso era la marca 180°. Cambiar simulador y nominal juntos deja `tools/evaluate_sim.py` idéntico (verificado). |
| D16 | **Abanico de interés** (`fan_angle_rad`, `fan_center_rad`): solo se usan los rayos a menos de la mitad de la apertura del centro, medido en el marco mundo desde la vertical hacia abajo (+ hacia +x); el resto queda NaN antes de la mediana. Lo usan el estimador, la calibración (centrado en el rayo «abajo» aproximado) y el visor. Por defecto 2π (sin filtro); el yaml usa 120° | — | Calibración real (unidad 1, H = 2.216 m, 21-oct-2026, `rosbags/calib_piso_2p216m_5s_r3`, 52 escaneos): con el giro completo, 193 rayos hacia arriba (ROS 278°–83°) chocan con el soporte a 0.2–0.43 m, quedan en z ≈ 2.27 m dentro de la ROI (`roi_z_max_m` 4.5) y dan «algo sobre el piso en x = −0.42…0.43 m»; en operación eso dejaría la máquina en OBSTRUIDA. Con 120°: sin avisos, b +4.0 → +4.4 mm, desvío de la flecha −1.305° → −1.275° (se excluyen los rayos más rasantes, |x| > 3.84 m). Al estar en el marco mundo sigue al nadir calibrado. Para cubrir la ROI hasta \|x\| = X hace falta apertura > 2·atan(X/H); la calibración avisa si no llega. |
| D17 | Un escaneo vacío **se salta sin borrar el historial**; se reinicia solo tras `empty_reset_scans` (5) vacíos seguidos (2026-10-24) | — | Antes cada escaneo vacío (dropout, hueco cabina–volco) reiniciaba y costaba ~0.4 s sin velocidad (vacío + 3 de `WARMING_UP`). Con 10 % de escaneos perdidos: válidos 47 % → 67 %, RMSE 0.023 → 0.020 m/s; el resto de la matriz no cambia. Un hueco largo lo acota `BAD_DT`. |
| D18 | Búsqueda de la correlación **simétrica** `[−v_max, v_max]` (`search_mode = "simetrica"`, 2026-10-24) | — | La búsqueda original `[−0.5, 6]` m/s no ve camiones en −x (0–4 % de válidos). Medido con estimador + PassTracker en los dos sentidos: "simetrica" y "sentido" (búsqueda según el sentido del paso) dan la misma exactitud (±0.01 m/s), pero "sentido" pierde la reversa más rápida que −0.5 m/s (caso "frena": 73 % de válidos frente a 92 %) y el volumen necesita medir la reversa para restarla. Costo: +1.2 ms/escaneo. Las tres opciones quedan en `search_mode` para ablación. |
| D19 | `object_extent` devuelve los **centros** de las celdas extremas (2026-10-24) | — | El borde real cae dentro de la celda extrema; con los bordes de celda el largo salía +1 celda (2 cm). Lo usa el volumen para ubicar frente y cola (+0.3 % de volumen). No cambia la velocidad. |
| D20 | Núcleo, nodos y **mensajes ROS en el marco mundo** (+x); `invert_velocity_sign` solo para mostrar (visor, HMI) con `with_sign` (2026-10-27, integración con el LiDAR de volumen) | — | Con D13 `velocity_node` publicaba `velocity_mps` con la referencia del usuario, pero `falconscan_volume/core/odometry.py` integra `velocity_mps` como marco x: con la referencia invertida el volumen habría integrado al revés. `PassEvent.v_*` es rapidez de avance (positiva en los dos sentidos) y `direction` va en el marco mundo. Decidido por Juan Manuel. |
| D21 | Integración de los dos `PassTracker` (2026-10-27): base de Martín (`core/pass_tracker.py`, el que usa el volumen) con dos correcciones: `n_scans`/`n_valid` cuentan solo escaneos **ocupados**, incluidos los de ENTRANDO (antes contaban los vacíos de SALIENDO y perdían las velocidades de la entrada), y el plazo de SIN_DATOS empieza en el primer `tick` si aún no llegó ningún escaneo | — | Con una secuencia limpia de 32 escaneos ocupados el tracker sin corregir daba 29 válidos de 34 (85 %) en vez de 32/32: la fracción bajaba artificialmente hacia `pass_min_valid_fraction` (60 %). `tools/evaluate_passes.py` (simulador, 6 camiones alternando sentido) con la integración frente a la versión anterior (D14): igual o mejor en todos los casos; p. ej. 30 % de pérdida de rayos 50 → 52 % de escaneos OK, 20 % de escaneos ciegos 74 → 76 %, 0 fragmentados y 6/6 sentidos correctos. `tools/evaluate_sim.py`: mismos válidos, sesgo y RMSE en +x que antes de integrar. Sin validar con el sensor real. |
| D22 | **Asignación de los S2E (2026-10-27):** longitudinal (velocidad) = 192.168.8.8 (MAC 52:50:69:86:46:38); transversal (volumen) = 192.168.11.2 (unidad 1, S/N 508BFA92…). La corrección de rango de la prueba de pared (D11) pasa a `volume_node`; `velocity_node` queda con `range_bias_m = 0`, `range_scale = 1` hasta hacerle su prueba de pared. Launch: `velocity_s2e` 8.8, `volume_lidar` 11.2, `portico` las dos. La RPi tiene 192.168.8.100/24 y 192.168.11.100/24 en eth0 | §6 fase 0 | La corrección es de la unidad, no del puesto. Prueba real del 2026-10-27: los dos drivers a la vez desde el visor, 10.01 / 10.02 Hz, 3240 rayos/vuelta. |
| D23 | Transversal con la misma convención que el longitudinal: **flecha 0° al piso = ROS 180°** (`VolumeParams.angle_offset_rad = +π/2`, yaml igual); el simulador del pórtico publica el escaneo transversal con el nadir en 180° (`vol_nadir_angle_deg`). `VolumeParams.range_scale` nuevo (mismo nombre y fórmula que D11). `SIM_VOLUME_OVERRIDES` (H del simulador, sin corrección de rango) en `volume_sim.launch.py` y el visor | — | Decisión del equipo del 2026-10-21 (flecha siempre al piso) aplicada a los dos sensores. Cambiar simulador y nominal juntos deja `tools/evaluate_volume_sim.py` idéntico en los casos de velocidad constante (simulación). |
| D24 | **Calibración de los dos LiDAR** (pestaña «Calibración» del visor): la misma rutina del piso (D12) para cada plano (x–z y y–z con la ROI lateral relativa a `y_offset_m`), captura simultánea; orientación del transversal con un objeto a la izquierda (+y = izquierda de un vehículo que va hacia +x, `detect_direction(entry_side=+1)`); **relación** con un objeto delgado en el cruce de los planos (`object_position`): el longitudinal mide `vol_plane_offset_m` (instalación: 0 cm), el transversal `vel_plane_y_m`, y los dos la altura del objeto (control cruzado). Tolerancias `cal_*` comunes a los dos S2E | §6 | Simulación (escenas quietas del pórtico): offset 0.0 cm y desvío 0.00° en los dos, cruce x = −0.2 cm / y = +0.1 cm, altura del poste 1.512 / 1.511 m. Sintético: transversal desplazado 0.8 m con giro de 1° y offset de 12 mm recuperados. Sin validar con los sensores montados. |

## Resultados (simulador, parámetros por defecto)

Errores en m/s. "válidos" = % de escaneos con estimación OK mientras la volqueta está en la ROI.
Desde el 2026-10-24 `tools/evaluate_sim.py` corre estimador + PassTracker (como el nodo) y
cada caso en los dos sentidos; la tabla muestra +x, y en −x los resultados son iguales salvo el
signo (p. ej. 1.5 m/s: 80 %, sesgo +0.008, RMSE 0.025). Cada caso produce exactamente un paso.

| caso | válidos | sesgo | RMSE | RMSE EMA | máx |
|------|--------:|------:|-----:|---------:|----:|
| constante 0.3 m/s | 88 % | −0.002 | 0.024 | 0.010 | 0.067 |
| constante 0.8 m/s | 82 % | −0.002 | 0.012 | 0.006 | 0.063 |
| constante 1.5 m/s | 78 % | −0.009 | 0.024 | 0.014 | 0.050 |
| constante 3.0 m/s | 78 % | −0.009 | 0.020 | 0.011 | 0.067 |
| constante 5.0 m/s | 75 % | −0.010 | 0.023 | 0.016 | 0.047 |
| acelera 0.5→3.2 m/s | 73 % | −0.014 | 0.026 | 0.123 | 0.068 |
| frena 3.0→0.6 m/s | 73 % | −0.012 | 0.024 | 0.074 | 0.055 |
| pérdida 15 % puntos | 64 % | −0.006 | 0.024 | 0.012 | 0.067 |
| escaneos perdidos 10 % | 67 % | −0.009 | 0.020 | 0.014 | 0.049 |
| outliers 1 % | 81 % | −0.006 | 0.033 | 0.017 | 0.200 |
| ruido rango 3 cm | 71 % | −0.015 | 0.033 | 0.021 | 0.097 |
| LiDAR 5 Hz | 78 % | −0.004 | 0.010 | 0.006 | 0.026 |

Versión "ingenua" (`baseline_scans=1`, `deskew_iterations=0`), como referencia de ablación:
RMSE 0.05–0.14 m/s y sesgo +0.12 m/s a 5 m/s.

Cómputo: ~3.5 ms/escaneo en el contenedor de desarrollo (x86). **Medir en la RPi 5.**

## Pendientes / riesgos abiertos

1. **Montaje real sin validar.** La invariancia del perfil depende de que el plano de escaneo
   contenga el eje de avance. Si queda girado (yaw), el perfil se deforma; ensayar la sensibilidad.
2. **Sesgo residual de −0.3 a −0.6 %** en simulación, sin explicar todavía (posible efecto de
   perspectiva en bordes verticales con `max` por celda). Investigar antes de fijar tolerancias.
3. **~20–25 % de escaneos no válidos**, sobre todo AMBIGUOUS a la entrada y salida de la volqueta
   (perfil parcial). Para el volumen se necesita posición continua: definir cómo se interpola
   en esos tramos (ver 5).
4. **Calibración** (FMEA RPN 252): offset de rango y desvío de la flecha ya tienen rutina (D12)
   en el visor, aún sin usar con el sensor montado; falta la máquina de calibración al encender
   (`docs/maquina_estados_paso.md`). Falta: guiñada del plano (objeto de largo conocido o
   desplazamientos conocidos, etapa 2) y vigilar la deriva del montaje (vibración) comparando
   el piso en cada estado VACÍO con la calibración guardada.
5. **Integración con volumen.** El volumen necesita la posición x(t) de cada perfil transversal,
   no la velocidad por intervalo. Siguiente paso: integrar Δs (no v·Δt filtrada) y estimar la
   deriva acumulada a lo largo de una pasada completa.
6. **Referencia independiente** para la validación física (reporte §8.1): encoder, cronometraje
   entre marcas o rueda odométrica. Sin eso, ningún número de campo es defendible.
