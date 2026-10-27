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
- Software: ROS 2 (objetivo: Jazzy sobre Ubuntu 24.04), nodos en Python. La RPi de desarrollo
  corre ROS 2 **Lyrical** sobre Ubuntu 26.04 (Python 3.14): compila igual con
  `colcon build --symlink-install`; el driver `sllidar_ros2` va en su propio workspace
  (`../lidar_ws`, con el CMakeLists ajustado a Lyrical).

## Estado del hardware (2026-10-27)

Los dos S2E del pórtico funcionan y están en el mismo switch (D22); la RPi tiene 192.168.8.100/24
y 192.168.11.100/24 en eth0 (perfil de NetworkManager «lidar»):
- **LONGITUDINAL / velocidad: 192.168.8.8** (MAC 52:50:69:86:46:38). Sin prueba de pared todavía:
  `velocity_node` sin corrección de escala.
- **TRANSVERSAL / volumen: 192.168.11.2** = unidad 1 (S/N 508BFA92…). Prueba de pared (D11: escala
  0.995753, sesgo 2.7 mm), ahora en `volume_node`.
- Los dos planos se cruzan en el mismo x: `vol_plane_offset_m = 0`.
- **RPLIDAR A1M8** (USB, banco del LiDAR de volumen): el motor gira pero no envía datos.
- `sllidar_ros2` con `inverted:=false` publica los rayos en orden inverso al de medición
  (`scan_time_reversed: true`, D10) y el ángulo ROS = 180° − marca del sensor.

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
    geometry.py                 polar→xz, abanico de interés, perfil h(x) en malla espacial, segmentación
    correlation.py              NCC con máscara, subcelda, ambigüedad
    estimator.py                VelocityParams, VelocityEstimator, Status
    pass_tracker.py             PassTracker (pasos de camión) y VelocityPipeline (estimador + tracker)
    calibration.py              calibración con el piso: offset de rango y desvío de la flecha 0° (H = cinta)
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
tools/evaluate_passes.py        pasos en simulación: camiones en ambos sentidos, escaneos ciegos, fragmentación
tools/evaluate_volume_sim.py    matriz de ensayos de volumen (simulador 3D)
tools/visor_lidar/              GUI de pruebas (PyQt6 + pyqtgraph; apt: python3-pyqt6 python3-pyqtgraph
                                python3-opengl). `instalar.sh` crea el acceso directo y avisa dependencias
  visor_lidar.py                ventana, fuentes (S2E / simulador / rosbag), pestaña «Sensor», grabación
  paso.py                       «Prueba de paso»: estimador + PassTracker en el visor; «Guardar en yaml»
                                escribe SOLO la sección velocity_node
  calibracion.py                «Calibración» de los DOS LiDAR: piso vacío con H de cinta, orientación de cada
                                plano y relación entre ellos (core/calibration.py, D24)
  portico.py                    drivers y tópicos de los dos LiDAR (los usan Calibración y Volumen 3D)
  volumen.py                    «Volumen 3D»: VelocityPipeline + VolumeReconstructor en el visor; nube y
                                superficie 3D, vista superior, A(x), perfil transversal, mediciones, PLY/JSON
  sim_volumen.py                genera (en paralelo) y guarda un paso del simulador 3D para «Volumen 3D»;
                                los escenarios son los de tools/evaluate_volume_sim.py
docs/algoritmo_velocidad.md     decisiones de diseño + resultados + pendientes
docs/maquina_estados_paso.md    máquinas de calibración (pendiente) y operación (PassTracker, implementada)
docs/maquina_estados_volumen.md reconstructor de volumen: estados, chequeos, parámetros, resultados
docs/referencias/               reporte técnico original del método
```

## Comandos

```bash
# Pruebas (sin ROS)
cd src/falconscan_velocity && python3 -m pytest -q test
cd src/falconscan_volume && python3 -m pytest -q test      # ~1–2 min en PC, ~4 min en la RPi

# Matriz de ensayos en simulación; --set para ablación
python3 tools/evaluate_sim.py
python3 tools/evaluate_sim.py --set baseline_scans=1 --set deskew_iterations=0
python3 tools/evaluate_passes.py                           # pasos; --set empty_reset_scans=1 = reiniciar en cada vacío
python3 tools/evaluate_volume_sim.py --reps 8              # volumen; --true-velocity, --plot rec.png

# Con ROS 2 (en la RPi o en un PC con ROS)
colcon build --symlink-install && source install/setup.bash
ros2 launch falconscan_velocity velocity.launch.py use_sim:=true truck_speed:=2.0
# Con el RPLIDAR S2E real (driver sllidar_ros2 + velocity_node); PC en 192.168.11.100/24
ros2 launch falconscan_velocity velocity_s2e.launch.py
ros2 topic echo /velocity_estimate
ros2 topic echo /pass_event      # INICIO/FIN de cada paso con banderas de calidad
ros2 topic echo /pass_state      # estado de la máquina de paso y `ready` para la HMI
# Pórtico completo simulado (los dos LiDAR) con RViz; reconstrucción en /volume/cloud y /volume/surface
ros2 launch falconscan_volume volume_sim.launch.py speed_kmh:=7.5 direction:=0
ros2 topic echo /volume/measurement
# LiDAR de volumen real (A1 por serie o S2E por UDP) + volume_node
ros2 launch falconscan_volume volume_lidar.launch.py lidar:=a1 rviz:=true
# Visor de pruebas (en la RPi también hay acceso directo en el escritorio)
tools/visor_lidar/visor_lidar.sh
# Paso del simulador 3D para el visor, sin GUI (escenarios: --lista)
python3 tools/visor_lidar/sim_volumen.py --caso "10 km/h +x" --salida /tmp/paso.npz
```

En la RPi el simulador 3D del pórtico cuesta ~1 s por escaneo: `volume_sim.launch.py` no llega a
10 Hz (se usa en un PC). Para ver el volumen en la RPi, la pestaña «Volumen 3D» genera el
escenario una vez (en caché) y lo reproduce. Si `colcon build` falla por un `velocity.yaml` que
ya no existe, borrar el enlace viejo en `build/falconscan_velocity/config/`.

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
- **Signo de la velocidad:** núcleo, nodos y mensajes ROS trabajan siempre en +x del marco mundo
  y miden ambos sentidos. `invert_velocity_sign` es **solo para mostrar** (visor, HMI) con
  `with_sign`; nunca aplicarlo a lo que se publica, porque el volumen integra `velocity_mps` (D20).
- **Geometría nominal:** la flecha 0° del S2E apunta al piso = rayo ROS 180° (`angle_offset_rad =
  +π/2`) en los DOS LiDAR, también en los simuladores (2D y pórtico 3D; D15, D23). `mount_height_m` es la cinta (verdadera);
  la calibración da valores efectivos (offset de rango, rayo hacia abajo) y no sobrescribe el yaml.
  El yaml es el del sensor FÍSICO (H de la cinta, rayos invertidos de sllidar_ros2, calibración de
  cada unidad); con los simuladores los launch aplican `SIM_VELOCITY_OVERRIDES` (`falconscan_velocity/sim`) y
  `SIM_VOLUME_OVERRIDES` (`falconscan_volume/sim`). La corrección de rango va con la UNIDAD, no con el puesto.
- **Pasos de camión:** `PassEvent.v_*` es la rapidez de avance (sentido · v, positiva en los dos
  sentidos); `flags` es una lista de nombres (vacía = válido); `n_scans`/`n_valid` cuentan solo
  escaneos ocupados, desde ENTRANDO (D21).
- **Decisiones de diseño:** cada una lleva un número D en `docs/algoritmo_velocidad.md`. Antes de
  numerar una nueva, hacer pull y tomar el siguiente libre (hoy D25): dos ramas ya chocaron en D10–D12.
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
montar los dos LiDAR y calibrarlos con la pestaña «Calibración» (cinta + piso vacío + cruce) →
prueba de pared del LiDAR de velocidad (192.168.8.8) → rosbag reales de los dos → máquina CAL_*
al encender (los nodos aún usan los nominales del yaml) con objeto patrón → referencia independiente
de velocidad → volumen de material (segmentar el volco y restar la tara).
