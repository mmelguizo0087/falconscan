# Falcon Scan

Medición volumétrica de carga en volquetas por LiDAR — Cainel S.A.S.

Estado: **Fase 1 (MVP)**. Implementado: estimación de velocidad de avance con el LiDAR longitudinal.

## Inicio rápido

```bash
# Pruebas y simulación (solo Python 3.10+ y numpy; no requiere ROS)
pip install numpy pytest pyyaml
cd src/falconscan_velocity && python3 -m pytest -q test && cd ../..
python3 tools/evaluate_sim.py
```

Con ROS 2 (Jazzy):

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch falconscan_velocity velocity.launch.py use_sim:=true truck_speed:=2.0   # simulado
ros2 launch falconscan_velocity velocity.launch.py scan_topic:=/scan               # sensor real
```

Tópicos:

| Tópico | Tipo | |
|---|---|---|
| `/scan` (entrada) | `sensor_msgs/LaserScan` | LiDAR de velocidad |
| `/velocity_estimate` | `falconscan_interfaces/VelocityEstimate` | velocidad, Δs, Δt, estado y diagnóstico de la correlación |
| `/height_profile` | `std_msgs/Float32MultiArray` | perfil h(x) para depurar (`publish_profile: true`) |

## Documentación

- `docs/algoritmo_velocidad.md` — método, decisiones de diseño, resultados en simulación y pendientes.
- `docs/referencias/` — reporte técnico original.
- `CLAUDE.md` — contexto del proyecto y convenciones de código.
