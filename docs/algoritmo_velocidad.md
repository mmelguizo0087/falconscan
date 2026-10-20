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

## Resultados (simulador, parámetros por defecto)

Errores en m/s. "válidos" = % de escaneos con estimación OK mientras la volqueta está en la ROI.

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
4. **Calibración** (FMEA RPN 252): procedimiento formal para H, `angle_offset` y `angle_direction`
   con objeto patrón; hoy son valores supuestos.
5. **Integración con volumen.** El volumen necesita la posición x(t) de cada perfil transversal,
   no la velocidad por intervalo. Siguiente paso: integrar Δs (no v·Δt filtrada) y estimar la
   deriva acumulada a lo largo de una pasada completa.
6. **Referencia independiente** para la validación física (reporte §8.1): encoder, cronometraje
   entre marcas o rueda odométrica. Sin eso, ningún número de campo es defendible.
