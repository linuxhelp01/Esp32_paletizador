# Guia de interfaz React y backend ROS 2

Esta guia acumula la documentacion de la interfaz grafica, el backend ROS 2, el frontend React y el launch file del proyecto.

## Arquitectura

```text
ESP32-S3 /palletizer_controller
  micro-ROS serial USB
  micro_ros_agent
PC ROS 2 Humble
  /palletizer_ui_backend
  WebSocket JSON
React dashboard
```

El frontend React no se conecta directamente a ROS 2. Se conecta al backend por WebSocket. El backend es un nodo ROS 2 `rclpy` que escucha telemetria, envia goals de action y publica la parada de emergencia.

## Capacidades actuales de la UI

- Estado de conexion ROS, backend y nodo del ESP32-S3.
- Telemetria cartesiana en milimetros.
- `joint_states` en unidades ROS.
- RPM y estado por motor.
- Ventana deslizante de estados de motores con `online`, `enabled`, `stalled`, `enc31`, `raw35`, error angular, homing y movimiento.
- Parada de emergencia por `/palletizer/emergency_stop`.
- Envio de setpoint cartesiano por `/palletizer/move_xyz`.
- Gemelo digital 3D con plantilla procedural.
- Entrada por teclado para velocidad, aceleracion, tolerancia y timeout.
- Controles secundarios preparados, pero deshabilitados cuando el firmware no expone la entidad ROS correspondiente.

## Interaccion del gemelo 3D

Por defecto, el mouse orienta la visualizacion 3D. El setpoint solo se puede manipular despues de hacer click en la esfera amarilla que representa el objetivo. Cuando el modo de edicion esta activo, el plano de arrastre permite mover X/Y. La coordenada Z, velocidad, aceleracion, tolerancia y timeout se ingresan por teclado.

La esfera verde representa la posicion actual medida. La esfera amarilla representa el setpoint que se enviara si el usuario confirma.

## Arranque manual

Terminal 1: agente micro-ROS.

```bash
source /home/felipe/microros_ws/install/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial --dev /dev/ttyACM0 -v4
```

Terminal 2: backend ROS 2.

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
cd /home/felipe/proyecto/Esp32_paletizador/ui/backend
python3 -m palletizer_ui_backend.server
```

Terminal 3: frontend React.

```bash
cd /home/felipe/proyecto/Esp32_paletizador/ui/frontend
npm install
npm run dev
```

Abrir:

```text
http://localhost:5173
```

## Arranque con launch file

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py
```

Argumentos principales:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py \
  serial_dev:=/dev/ttyACM0 \
  ros_domain_id:=10 \
  frontend_port:=5173 \
  backend_port:=8765
```

Si el agente ya esta corriendo:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_agent:=false
```

Si no quieres arrancar React desde launch:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_frontend:=false
```

Para depurar comandos expandidos:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py show_commands:=true
```

## Backend WebSocket

Por defecto el backend levanta:

```text
ws://127.0.0.1:8765
```

Comandos JSON usados por el frontend:

```json
{"type":"move_xyz","goal":{"x_mm":50,"y_mm":50,"z_mm":20,"speed_mm_s":25,"accel_mm_s2":50,"tolerance_mm":1,"timeout_ms":30000}}
{"type":"emergency_stop","data":true}
{"type":"refresh"}
```

Los comandos de servicios y actions secundarios estan implementados en la UI/backend, pero pueden responder `unavailable` si el firmware actual no los expone.

## Node/NPM en WSL

Si usas WSL, ejecuta `npm install` y `npm run dev` desde Ubuntu/Linux dentro de `/home/felipe/...`. No uses Node de Windows sobre rutas `\\wsl.localhost\...`, porque paquetes como `esbuild` fallan con rutas UNC.

## Fuente original acumulada

Esta guia consolida el contenido de:

- `ui/README.md`
- `ui/frontend/README.md`
- `ui/backend/README.md`
- `ui/launch/palletizer_ui.launch.py`
