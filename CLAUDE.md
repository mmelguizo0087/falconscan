# Falcon Scan — Cainel S.A.S.

Sistema de medición volumétrica de carga en volquetas mediante LiDAR montado en pórtico,
para agregados, minería y construcción. El camión pasa sin detenerse y el sistema calcula
el volumen de material del volco.

**Foco actual: sprint técnico de desarrollo de producto (Fase 1 — MVP).** No es trabajo comercial.

## Arquitectura del producto

- **2 LiDAR en el pórtico:**
  - **LiDAR de velocidad** (este repo, implementado): plano de escaneo vertical que contiene
    el eje de avance. Estima cuánto avanzó el camión entre escaneos.
  - **LiDAR de volumen** (pendiente): perfiles transversales de la carga. Volumen = Σ área(perfil) · Δx,
    con Δx sacado del LiDAR de velocidad. **El error de velocidad se traslada 1:1 al volumen.**
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
    pass_tracker.py             máquina de estados de paso (PassTracker, PassEvent, PassFlag)
    pipeline.py                 VelocityPipeline: estimador + tracker con la política de reinicio (nodo y visor)
    calibration.py              calibración con el piso: offset de rango y desvío de la flecha 0° (H = cinta)
  falconscan_velocity/sim/      simulador 2D (ray-casting de volqueta en movimiento)
  falconscan_velocity/velocity_node.py   /scan -> /velocity_estimate, /pass_state, /pass_event
  falconscan_velocity/sim_publisher.py   publica /scan simulado
  config/velocity.yaml          parámetros (1:1 con VelocityParams)
  test/                         pytest; corre sin ROS
src/falconscan_interfaces/      mensajes (VelocityEstimate, PassState, PassEvent)
tools/evaluate_sim.py           matriz de ensayos en simulación (ablación)
tools/evaluate_passes.py        máquina de paso en simulación: camiones en ambos sentidos, fragmentación
docs/algoritmo_velocidad.md     decisiones de diseño + resultados + pendientes
docs/maquina_estados_paso.md    máquinas de estados de calibración (al encender, propuesta) y operación (paso del camión, implementada)
tools/visor_lidar/              GUI de pruebas (PyQt6 + pyqtgraph): sensor en vivo, prueba de paso,
                                calibración de montaje, rosbags; `instalar.sh` crea el acceso directo del escritorio
docs/referencias/               reporte técnico original del método
```

## Comandos

```bash
# Pruebas (sin ROS)
cd src/falconscan_velocity && python3 -m pytest -q test

# Matriz de ensayos en simulación; --set para ablación
python3 tools/evaluate_sim.py
python3 tools/evaluate_sim.py --set baseline_scans=1 --set deskew_iterations=0
python3 tools/evaluate_passes.py                  # pasos; --reset-on-empty = política anterior

# Con ROS 2 (en la RPi o en un PC con ROS)
colcon build --symlink-install && source install/setup.bash
ros2 launch falconscan_velocity velocity.launch.py use_sim:=true truck_speed:=2.0
# Con el RPLIDAR S2E real (driver sllidar_ros2 + velocity_node); PC en 192.168.11.100/24
ros2 launch falconscan_velocity velocity_s2e.launch.py
ros2 topic echo /velocity_estimate
ros2 topic echo /pass_event      # INICIO/FIN de cada paso con banderas de calidad
ros2 topic echo /pass_state      # estado de la máquina de paso y `ready` para la HMI
```

## Convenciones (respetarlas)

- **Inválido = NaN, siempre.** Nunca reemplazar un rango inválido ni una celda vacía por 0 o r_max:
  una sombra no es piso.
- **El núcleo no depende de ROS.** Toda la lógica nueva va en `core/` con pruebas; los nodos solo
  convierten mensajes.
- **Parámetros:** un parámetro nuevo se agrega a `VelocityParams` **y** a `config/velocity.yaml`
  (hay una prueba que lo verifica, incluido el tipo: floats con punto decimal).
- **Cada mejora se mide.** Toda variante (filtro, Hampel, Kalman, gradiente…) entra solo si
  `tools/evaluate_sim.py` (y después los rosbag reales) muestran una mejora cuantitativa.
  Introducir los cambios de a uno (reporte §12).
- **Signo de la velocidad:** estimador y máquina de estados trabajan siempre en +x del marco mundo
  y miden ambos sentidos. La referencia del usuario (`invert_velocity_sign`) se aplica solo al
  publicar o mostrar (`with_sign`); no mezclar velocidades con y sin referencia en el núcleo.
- **Geometría nominal:** la flecha 0° del S2E apunta al piso = rayo ROS 180° (`angle_offset_rad =
  +π/2`), también en el simulador. `mount_height_m` es la cinta (verdadera); la calibración da
  valores efectivos (offset de rango, rayo hacia abajo) y no sobrescribe el yaml.
- **Para integrar posición o volumen** usar `advance_mps · dt_s` (rapidez de avance en el sentido del
  paso: no depende de la referencia de signo y una reversa resta) con `mid_stamp` (header.stamp del
  mensaje). `velocity_filtered_mps` (EMA) tiene retardo y es solo para mostrar.
- Código, comentarios y documentación en español.
- Los resultados del simulador **no** son evidencia de desempeño de campo; decirlo siempre que
  se reporten.

## Equipo

- Martín Melguizo — CEO / Product Manager; mecánica, algoritmos de escaneo.
- Juan Manuel Marín — CTO; electrónica, software, GUI, IoT.
- David Rozo — consultor experto en LiDAR.
- Seguimiento académico: Universidad EIA.

## Siguiente trabajo técnico

Ver "Pendientes" en `docs/algoritmo_velocidad.md`. En orden: grabar rosbag reales con el S2E →
calibración de montaje con objeto patrón → referencia independiente de velocidad → integración
de posición x(t) para el LiDAR de volumen.
