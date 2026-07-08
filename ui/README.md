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
- envia goals XYZ + A a `/palletizer/move_xyz`
- deja preparados servicios y actions secundarios para cuando el firmware los exponga

La interfaz incluye un panel de gemelo digital 3D. El panel actual usa una geometria plantilla: por defecto el mouse orienta la camara, y solo despues de hacer click en la esfera amarilla se habilita el arrastre del setpoint X/Y. Z, velocidad/aceleracion lineal, A en grados, velocidad/aceleracion angular, tolerancias y timeout se ingresan con teclado; el envio se realiza por `/palletizer/move_xyz`. Tambien existe una ventana deslizante para estados detallados de motores.

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

El primer launch ejecuta `npm ci` automaticamente si faltan las dependencias
del frontend. El navegador conecta el WebSocket al mismo host con el que se
abrio la pagina, por lo que tambien funciona desde otra maquina usando
`http://IP_DEL_PC:5173`.


## Arranque con launch file

Tambien puedes iniciar todo desde un solo launch de ROS 2:

```bash
source /opt/ros/humble/setup.bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py
```

Esto inicia:

```text
micro_ros_agent
/palletizer_ui_backend
React/Vite en http://localhost:5173
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

Si solo quieres backend y agente, sin React:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_frontend:=false
```

Si React ya esta corriendo:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_frontend:=false
```

Para ver los comandos expandidos:

```bash
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py show_commands:=true
```

Antes de usar el launch por primera vez, instala dependencias del frontend desde
WSL/Linux, no con Node de Windows:

```bash
cd /home/felipe/proyecto/Esp32_paletizador/ui/frontend
npm install
```

## Nota de estado

El firmware actual expone solo una parte del grafo ROS para mantener estabilidad
micro-ROS:

```text
activos: topics de telemetria, /palletizer/emergency_stop, /palletizer/move_xyz
desactivados: services, /palletizer/command, /palletizer/fast_move_xyz, /palletizer/home_axis, /palletizer/go_origin
```

La UI muestra esos controles secundarios, pero los mantiene no disponibles
mientras sus interfaces opcionales no existan en el grafo ROS.

## Modelo 3D

La estructura para URDF/STL queda en:

```text
ui/frontend/public/robot_description/
```

La guia detallada esta en:

```text
ui/GUIA_MODELO_3D_REACT.md
docs/GUIA_MODELO_3D_REACT.md
```

La documentacion acumulada para GitHub esta en:

```text
README.md
docs/README.md
```


## Eje A y Servo PWM

La interfaz contempla un quinto driver fisico como eje rotatorio `A` con CAN ID `0x05`. En las tablas de telemetria aparece como motor `A`; en `joint_states` se publica como junta rotatoria en radianes.

En el panel de homing se puede seleccionar `A rotatorio` y presionar `Home`. Si la action `/palletizer/home_axis` no esta disponible, el backend envia el comando textual `HOME A` por `/palletizer/command`.

El panel `Servo PWM` controla el actuador auxiliar por angulo o pulso en microsegundos. El pin por defecto es `GPIO 18` y se ajusta en `src/config.h`.

Servicio de pinza/servo:

```text
/palletizer/set_gripper [palletizer_msgs/srv/SetGripper]
```

Request:

```text
bool closed   # true=tomar/cerrar, false=soltar/abrir
```

Response:

```text
bool success
string message
bool closed
float32 angle_deg
uint16 pulse_us
```

Mapeo actual del firmware:

```text
closed=true  -> SERVO 0 deg   -> tomar/cerrar
closed=false -> SERVO 180 deg -> soltar/abrir
```


La barra superior del visor muestra cuatro estados: `UI conectada`, `Conexion fisica ESP32`, `Comunicacion ROS` y `Paletizador operativo`. `UI conectada` solo confirma WebSocket con el backend. `Conexion fisica ESP32` se activa si el nodo `/palletizer_controller` aparece en el grafo o llega telemetria desde el ESP32-S3. `Comunicacion ROS` exige mensajes ROS frescos. `Paletizador operativo` exige comunicacion ROS, sin falla, y los 5 motores `online`, `enabled` y sin `stalled`.
