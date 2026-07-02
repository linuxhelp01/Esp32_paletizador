# Interfaz grafica del paletizador

Esta carpeta contiene la interfaz de visualizacion y control del paletizador.

Arquitectura:

```text
ESP32-S3 /palletizer_controller
  micro-ROS serial USB
  micro_ros_agent
PC ROS 2 Humble
  /palletizer_ui_backend
  WebSocket JSON
React dashboard
```

El frontend React no se conecta directamente a ROS 2. Se conecta al backend por
WebSocket. El backend es un nodo ROS 2 `rclpy` que:

- escucha `/joint_states`
- escucha `/palletizer/axis_position_mm`
- escucha `/palletizer/motor_rpm`
- escucha `/palletizer/status`
- escucha `/palletizer/fault_state`
- publica `/palletizer/emergency_stop`
- envia goals a `/palletizer/move_xyz`
- deja preparados servicios y actions secundarios para cuando el firmware los exponga

## Arranque

Terminal 1: agente micro-ROS.

```bash
source /home/felipe/microros_ws/install/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial --dev /dev/ttyACM0 -v4
```

Terminal 2: backend.

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
cd /home/felipe/proyecto/Esp32_paletizador/ui/backend
python3 -m palletizer_ui_backend.server
```

Terminal 3: frontend.

```bash
cd /home/felipe/proyecto/Esp32_paletizador/ui/frontend
npm install
npm run dev
```

Abrir:

```text
http://localhost:5173
```

## Nota de estado

El firmware actual expone solo una parte del grafo ROS para mantener estabilidad
micro-ROS:

```text
activos: topics de telemetria, /palletizer/emergency_stop, /palletizer/move_xyz
desactivados: services, /palletizer/command, /palletizer/home_axis, /palletizer/go_origin
```

La UI muestra esos controles secundarios, pero los deshabilita si el backend no
detecta la entidad ROS correspondiente.
