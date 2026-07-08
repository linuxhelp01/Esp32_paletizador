# Guia de inicio del sistema

Esta guia indica como levantar en una sola orden el agente micro-ROS, el backend ROS 2 de la interfaz y el visualizador React del paletizador.

## Requisitos previos

- ESP32-S3 flasheado con el firmware actual del proyecto.
- ESP32-S3 conectado por USB CDC nativo al computador.
- Dominio ROS del firmware: `ROS_DOMAIN_ID=10`.
- Paquete `palletizer_msgs` disponible en `/home/felipe/palletizer_msgs_ws`.
- micro-ROS agent disponible en `/home/felipe/microros_ws`.
- Dependencias del frontend instaladas en `ui/frontend`.

## Comando recomendado

Desde cualquier terminal:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py
```

Este comando inicia:

- `micro_ros_agent` para el ESP32-S3.
- Backend ROS 2 `/palletizer_ui_backend`.
- Servidor web React/Vite.

Abre el visualizador en:

```text
http://localhost:5173
```

## Comando con opciones explicitas

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py \
  ros_domain_id:=10 \
  serial_dev:=/dev/ttyACM0 \
  agent_verbose:=1 \
  backend_host:=127.0.0.1 \
  backend_port:=8765 \
  frontend_host:=0.0.0.0 \
  frontend_port:=5173 \
  backend_connection_stale_ms:=1500 \
  backend_availability_hold_ms:=1200 \
  enable_home_origin_actions:=false \
  enable_axis_services:=false \
  enable_extended_services:=false
```

## Opciones del launch file

| Opcion | Default | Uso |
|---|---:|---|
| `ros_domain_id` | `10` | Dominio ROS que debe coincidir con el firmware ESP32-S3. |
| `serial_dev` | `/dev/ttyACM0` | Puerto USB CDC del ESP32-S3 usado por el micro-ROS agent. |
| `agent_verbose` | `1` | Verbosidad del micro-ROS agent. Usar `4` solo para depuracion. |
| `backend_host` | `127.0.0.1` | Host WebSocket del backend. |
| `backend_port` | `8765` | Puerto WebSocket del backend. |
| `frontend_host` | `0.0.0.0` | Host del servidor Vite/React. |
| `frontend_port` | `5173` | Puerto del visualizador web. |
| `backend_connection_stale_ms` | `1500` | Tiempo maximo sin telemetria antes de marcar ESP32/ROS como desconectado. |
| `backend_availability_hold_ms` | `1200` | Retencion corta para evitar parpadeos de disponibilidad del grafo ROS. |
| `enable_home_origin_actions` | `false` | Crea clientes UI para `/palletizer/home_axis` y `/palletizer/go_origin` solo si el firmware los expone. |
| `enable_axis_services` | `false` | Crea clientes UI para `/palletizer/enable_axis` y `/palletizer/set_axis_limits` solo si el firmware los expone. |
| `enable_extended_services` | `false` | Crea clientes UI para diagnostico/homing extendido solo si el firmware los expone. |
| `ros_setup` | `/opt/ros/humble/setup.bash` | Setup base de ROS 2 Humble. |
| `microros_setup` | `/home/felipe/microros_ws/install/setup.bash` | Setup del workspace micro-ROS. |
| `palletizer_msgs_setup` | `/home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash` | Setup de interfaces custom. |
| `agent_executable` | `/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent` | Ejecutable del agente micro-ROS. |
| `start_agent` | `true` | Inicia o no el micro-ROS agent. |
| `start_backend` | `true` | Inicia o no el backend ROS 2/WebSocket. |
| `start_frontend` | `true` | Inicia o no React/Vite. |
| `show_commands` | `false` | Muestra los comandos expandidos generados por el launch. |

## Variantes utiles

### Usar otro puerto del ESP32-S3

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py serial_dev:=/dev/ttyACM1
```

Para identificar el puerto:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

### Iniciar sin agente micro-ROS

Usalo si ya tienes un agente corriendo manualmente:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_agent:=false
```

### Iniciar sin frontend

Usalo si quieres probar solo ROS/backend:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_frontend:=false
```

### Mostrar comandos internos del launch

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py show_commands:=true
```

### Depurar micro-ROS agent con mas salida

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py agent_verbose:=4
```

Para operacion normal conviene `agent_verbose:=1` porque reduce ruido y carga en terminal.

### Activar interfaces opcionales del backend

El firmware actual expone por defecto la action `/palletizer/move_xyz` y el servicio `/palletizer/set_gripper`. Las actions de homing/origen y servicios extendidos pueden existir en `palletizer_msgs`, pero el backend no crea clientes para ellas por defecto para evitar falsos positivos en `ros2 action list` y `ros2 service list`.

Solo si recompilas el firmware con esas interfaces activas, inicia el launch con:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py \
  enable_home_origin_actions:=true \
  enable_axis_services:=true \
  enable_extended_services:=true
```

## Verificacion despues de iniciar

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Ver nodos:

```bash
ros2 node list
```

Debe aparecer:

```text
/palletizer_controller
/palletizer_ui_backend
```

Ver interfaces del ESP32:

```bash
ros2 node info /palletizer_controller
```

Ver frecuencias de telemetria:

```bash
ros2 topic hz /joint_states
ros2 topic hz /palletizer/axis_position_mm
ros2 topic hz /palletizer/status
```

Esperado aproximado:

```text
/joint_states                 ~20 Hz
/palletizer/axis_position_mm  ~20 Hz
/palletizer/status            ~4 Hz
```

Ver action activa:

```bash
ros2 action list -t
```

Debe aparecer:

```text
/palletizer/move_xyz [palletizer_msgs/action/MoveXYZ]
```

Ver service activo:

```bash
ros2 service list -t | grep palletizer
```

Debe aparecer:

```text
/palletizer/set_gripper [palletizer_msgs/srv/SetGripper]
```

## Pruebas rapidas

### Jog relativo desde ROS

Este es el mismo camino que usan los botones manuales del visualizador:

```bash
ros2 topic pub --once /palletizer/jog_xyz_delta std_msgs/msg/Float32MultiArray "{data: [5.0, 0.0, 0.0, 25.0, 50.0]}"
```

### Mover por action

```bash
ros2 action send_goal /palletizer/move_xyz palletizer_msgs/action/MoveXYZ \
  "{x_mm: 50.0, y_mm: 50.0, z_mm: 20.0, use_a: false, a_deg: 0.0, speed_mm_s: 25.0, accel_mm_s2: 50.0, angular_speed_deg_s: 90.0, angular_accel_deg_s2: 180.0, tolerance_mm: 1.0, angular_tolerance_deg: 1.0, timeout_ms: 30000}" \
  --feedback
```

### Abrir/cerrar pinza

```bash
ros2 service call /palletizer/set_gripper palletizer_msgs/srv/SetGripper "{closed: true}"
ros2 service call /palletizer/set_gripper palletizer_msgs/srv/SetGripper "{closed: false}"
```

## Indicadores del visualizador

El visualizador muestra cuatro estados principales:

- `UI conectada`: WebSocket entre React y backend.
- `Conexion fisica ESP32`: telemetria fresca recibida desde el ESP32 por ROS.
- `Comunicacion ROS`: mensajes ROS activos desde el nodo `/palletizer_controller`.
- `Paletizador operativo`: todos los motores online, habilitados, sin stall y sin falla.

El panel `Nodo ESP32` dentro del visualizador lista cada topico, subscriber, service y action activa del nodo, mostrando si hay datos frescos o accionador disponible.

## Si no conecta

1. Verifica que no haya otro agente usando el puerto:

```bash
ps aux | grep -E "micro_ros_agent|palletizer_ui_backend|vite" | grep -v grep
```

2. Verifica puerto:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

3. Verifica dominio:

```bash
echo $ROS_DOMAIN_ID
```

Debe ser `10` para coincidir con el firmware actual.

4. Reinicia todo con comandos visibles:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py show_commands:=true
```

5. Si el nodo aparece pero no hay topicos, desconecta y conecta el ESP32-S3 y vuelve a lanzar. El firmware intenta reconectar al agente, pero el bus USB/CDC puede quedar retenido si otro proceso usa el puerto.
