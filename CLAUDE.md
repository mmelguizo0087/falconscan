# Falcon Scan — Cainel S.A.S.

Sistema de medición volumétrica de carga en volquetas mediante LiDAR montado en pórtico,
para agregados, minería y construcción. El camión pasa sin detenerse y el sistema calcula
el volumen de material del volco.

**Foco actual: sprint técnico de desarrollo de producto (Fase 1 — MVP).** No es trabajo comercial.

## Arquitectura del producto

- **2 LiDAR en el pórtico:**
  - **LiDAR de velocidad** (implementado): plano de escaneo vertical que contiene el eje de
    avance. Estima cuánto avanzó el camión entre escaneos; el `PassTracker` arma los pasos.
  - **LiDAR de volumen** (implementado en simulación, 2026-10-24): perfiles transversales.
    Volumen de la ENVOLVENTE = ∫ área(perfil) dx, con x integrada de la velocidad. Frente y
    cola se ubican con el LiDAR de velocidad. **El error de velocidad se traslada 1:1 al
    volumen.** El volumen de material (separar el volco y restar la tara) es la etapa siguiente.
- Sensor de referencia: RPLIDAR S2E (dToF, 10 Hz), inferido del BOM. Verificar con la hoja de datos.
- Hardware MVP: Raspberry Pi 5 (4 GB), HMI Nextion, alimentación 110–220 V, exportación a SD.
- Software: ROS 2 (objetivo: Jazzy sobre Ubuntu 24.04), nodos en Python.

## Acuerdos del MVP (Notion, "acuerdos")

- Una entrada y una salida; un camión a la vez.
- Sin reconocimiento por visión ni tag (sin ID de vehículo).
- Sin pórtico definitivo en el primer MVP: el sensor se fija en un sitio adecuado para la prueba.
- La HMI muestra: estado del proceso, sistema listo/no listo y última medición; exporta a SD.
- Conectividad con servidor: **fuera del alcance de Fase 1**.

## Criterios de aceptación (Notion, "Criterios de éxito")

- Error de volumen ≤ 8 % frente a un método de referencia.
- Repetibilidad ≤ 5 % para la misma carga.
- Tiempo de medición compatible con paso sin detenerse (número aún por fijar).
- Cada medición válida genera un registro (volumen + identificación disponible).

## Prioridades del FMEA

1. Cálculo volumétrico incorrecto con datos válidos (RPN 400).
2. Segmentación carga/vehículo/entorno (RPN 378).
3. Transformación geométrica / calibración (RPN 252).
4. Datos LiDAR inválidos o incompletos (RPN 240).

## Estructura del repo

```
src/falconscan_velocity/        paquete ROS 2 (ament_python)
  falconscan_velocity/core/     algoritmo en Python puro + numpy; NO importa ROS
    scan.py                     validación de rangos, mediana con NaN
    geometry.py                 polar→xz, perfil h(x) en malla espacial, segmentación
    correlation.py              NCC con máscara, subcelda, ambigüedad
    estimator.py                VelocityParams, VelocityEstimator, Status
    pass_tracker.py             PassTracker (pasos de camión) y VelocityPipeline (estimador + tracker)
  falconscan_velocity/sim/      simulador 2D (ray-casting de volqueta en movimiento)
  falconscan_velocity/velocity_node.py   /scan -> /velocity_estimate, /pass_state, /pass_event
  falconscan_velocity/sim_publisher.py   publica /scan simulado
  config/falconscan.yaml        parámetros de LOS DOS nodos (velocity_node y volume_node)
  test/                         pytest; corre sin ROS
src/falconscan_volume/          paquete ROS 2 del LiDAR transversal (usa el núcleo de velocidad)
  falconscan_volume/core/       section, odometry, integration, checks, reconstruction, reconstructor
  falconscan_volume/sim/        simulador 3D del pórtico (los dos LiDAR, volquetas con volumen verdadero)
  falconscan_volume/volume_node.py       reconstrucción 3D + volumen; sim_portico.py simula los dos LiDAR
  launch/                       volume_sim (todo simulado + RViz), volume_lidar (A1/S2E real), portico
  rviz/volumen.rviz
src/falconscan_interfaces/      mensajes (VelocityEstimate, PassEvent, PassState, VolumeState, VolumeMeasurement)
tools/evaluate_sim.py           matriz de ensayos de velocidad en simulación (ablación)
tools/evaluate_volume_sim.py    matriz de ensayos de volumen (simulador 3D)
docs/algoritmo_velocidad.md     decisiones de diseño + resultados + pendientes
docs/maquina_estados_paso.md    máquinas de calibración (pendiente) y operación (PassTracker, implementada)
docs/maquina_estados_volumen.md reconstructor de volumen: estados, chequeos, parámetros, resultados
docs/referencias/               reporte técnico original del método
```

## Comandos

```bash
# Pruebas (sin ROS)
cd src/falconscan_velocity && python3 -m pytest -q test
cd src/falconscan_volume && python3 -m pytest -q test      # ~1–2 min (simulador 3D)

# Matriz de ensayos en simulación; --set para ablación
python3 tools/evaluate_sim.py
python3 tools/evaluate_sim.py --set baseline_scans=1 --set deskew_iterations=0
python3 tools/evaluate_volume_sim.py --reps 8              # volumen; --true-velocity, --plot rec.png

# Con ROS 2 (en la RPi o en un PC con ROS)
colcon build --symlink-install && source install/setup.bash
ros2 launch falconscan_velocity velocity.launch.py use_sim:=true truck_speed:=2.0
# Con el RPLIDAR S2E real (driver sllidar_ros2 + velocity_node); PC en 192.168.11.100/24
ros2 launch falconscan_velocity velocity_s2e.launch.py
ros2 topic echo /velocity_estimate
# Pórtico completo simulado (los dos LiDAR) con RViz; reconstrucción en /volume/cloud y /volume/surface
ros2 launch falconscan_volume volume_sim.launch.py speed_kmh:=7.5 direction:=0
ros2 topic echo /volume/measurement
# LiDAR de volumen real (A1 por serie o S2E por UDP) + volume_node
ros2 launch falconscan_volume volume_lidar.launch.py lidar:=a1 rviz:=true
```

## Convenciones (respetarlas)

- **Inválido = NaN, siempre.** Nunca reemplazar un rango inválido ni una celda vacía por 0 o r_max:
  una sombra no es piso.
- **El núcleo no depende de ROS.** Toda la lógica nueva va en `core/` con pruebas; los nodos solo
  convierten mensajes.
- **Parámetros:** un parámetro nuevo se agrega a `VelocityParams` (o `VolumeParams`) **y** a su
  sección de `config/falconscan.yaml` (hay pruebas que lo verifican, incluido el tipo: floats con
  punto decimal). Los dos LiDAR usan los mismos nombres de montaje/calibración.
- **Cada mejora se mide.** Toda variante (filtro, Hampel, Kalman, gradiente…) entra solo si
  `tools/evaluate_sim.py` (y después los rosbag reales) muestran una mejora cuantitativa.
  Introducir los cambios de a uno (reporte §12).
- **Para integrar posición o volumen** integrar `velocity_mps` en `mid_stamp` (header.stamp del
  mensaje), como `falconscan_volume/core/odometry.py`. **No sumar `displacement_m`**: con
  `baseline_scans = 3` los intervalos se solapan y se contaría tres veces. `velocity_filtered_mps`
  (EMA) tiene retardo y es solo para mostrar.
- Código, comentarios y documentación en español.
- Los resultados del simulador **no** son evidencia de desempeño de campo; decirlo siempre que
  se reporten.

## Equipo

- Martín Melguizo — CEO / Product Manager; mecánica, algoritmos de escaneo.
- Juan Manuel Marín — CTO; electrónica, software, GUI, IoT.
- David Rozo — consultor experto en LiDAR.
- Seguimiento académico: Universidad EIA.

## Siguiente trabajo técnico

Ver "Pendientes" en `docs/algoritmo_velocidad.md` y `docs/maquina_estados_volumen.md`. En orden:
segundo LiDAR conectado (IP del 2.º S2E) y rosbag reales de los dos → calibración (máquina
CAL_* compartida por los dos LiDAR) con objeto patrón → referencia independiente de velocidad →
volumen de material (segmentar el volco y restar la tara).
