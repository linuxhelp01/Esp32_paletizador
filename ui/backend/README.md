# palletizer_ui_backend

Nodo ROS 2 en PC que conecta el paletizador ESP32-S3 con la interfaz React.

## Ejecutar sin instalar

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
cd /home/felipe/proyecto/Esp32_paletizador/ui/backend
python3 -m palletizer_ui_backend.server
```

Por defecto levanta WebSocket en:

```text
ws://127.0.0.1:8765
```

## Ejecutar como paquete ROS 2

Si copias `ui/backend` dentro de un workspace ROS 2:

```bash
colcon build --packages-select palletizer_ui_backend
source install/setup.bash
ros2 run palletizer_ui_backend palletizer_ui_backend
```

## Comandos WebSocket

El frontend envia JSON:

```json
{"type":"move_xyz","goal":{"x_mm":50,"y_mm":50,"z_mm":20,"use_a":true,"a_deg":90,"speed_mm_s":25,"accel_mm_s2":50,"angular_speed_deg_s":90,"angular_accel_deg_s2":180,"tolerance_mm":1,"angular_tolerance_deg":1,"timeout_ms":30000}}
{"type":"emergency_stop","data":true}
{"type":"refresh"}
```

Los comandos de servicios y actions secundarios estan implementados, pero
pueden responder `unavailable` si el firmware actual no los expone en el grafo
ROS.


## Documentacion acumulada

La documentacion consolidada del proyecto esta en `../../README.md` y `../../docs/`.
