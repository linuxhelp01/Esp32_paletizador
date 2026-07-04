# ESP32-S3 Paletizador micro-ROS

Firmware y herramientas de control para un paletizador cartesiano con ESP32-S3, micro-ROS Humble, drivers MKS Servo57D por CAN/TWAI, nodo ROS 2 en PC y dashboard React con gemelo digital 3D.

Este README acumula la informacion de las guias del proyecto para que GitHub muestre una vista completa del estado actual. Las guias separadas quedan en `docs/`.

## Estado actual

El ESP32-S3 no ejecuta ROS 2 completo. Ejecuta un cliente micro-ROS y se comunica con el PC mediante `micro_ros_agent` por USB serial nativo CDC. El PC traduce esa conexion hacia el grafo ROS 2 Humble.

Nodo esperado del firmware:

```text
/palletizer_controller
```

Entidades ROS activas actualmente:

```text
Publishers:
  /joint_states
  /palletizer/axis_position_mm
  /palletizer/fault_state
  /palletizer/motor_rpm
  /palletizer/status

Subscribers:
  /palletizer/emergency_stop
  /palletizer/command

Action servers:
  /palletizer/move_xyz [palletizer_msgs/action/MoveXYZ]

Service servers:
  ninguno en el perfil estable actual
```

Funciones existentes en codigo o interfaces, pero desactivadas en el grafo ROS actual por estabilidad micro-ROS:

```text
/palletizer/enable_axis
/palletizer/set_axis_limits
/palletizer/set_zero
/palletizer/clear_fault
/palletizer/release_stall
/palletizer/get_driver_status
/palletizer/home_axis
/palletizer/go_origin
```

Banderas relevantes en `src/ros_bridge.cpp`:

```cpp
ROS_ENABLE_COMMAND_SUBSCRIBER = false;
ROS_ENABLE_SERVICE_SERVERS = false;
ROS_ENABLE_EXTENDED_SERVICE_SERVERS = false;
ROS_ENABLE_HOME_ORIGIN_ACTIONS = false;
```

## Arquitectura

```text
ESP32-S3
  Firmware PlatformIO + Arduino
  micro_ros_platformio
  USB CDC serial /dev/ttyACM0
  CAN/TWAI a 1 Mbit/s hacia drivers MKS Servo57D

PC ROS 2 Humble
  micro_ros_agent serial
  palletizer_msgs
  /palletizer_ui_backend

Frontend
  React + Vite
  WebSocket JSON hacia backend
  Gemelo digital 3D con Three.js
```

## Requisitos

- Ubuntu 22.04 o WSL Ubuntu 22.04.
- ROS 2 Humble.
- Workspace `palletizer_msgs_ws` con interfaces custom.
- Workspace `microros_ws` con `micro_ros_agent`.
- PlatformIO.
- Node/NPM dentro de Linux/WSL para la UI.

## Configuracion PlatformIO

Archivo principal:

```text
platformio.ini
```

Configuracion relevante:

```ini
[platformio]
default_envs = 4d_systems_esp32s3_gen4_r8n16_microros

[env]
platform = espressif32
board = 4d_systems_esp32s3_gen4_r8n16
framework = arduino
monitor_port = /dev/ttyACM0
upload_port = /dev/ttyACM0
upload_speed = 460800
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
    -DENABLE_MICRO_ROS=1

[env:4d_systems_esp32s3_gen4_r8n16_microros]
lib_deps =
    https://github.com/micro-ROS/micro_ros_platformio
board_microros_distro = humble
board_microros_transport = serial
board_microros_user_meta = palletizer.meta
```

`micro_ros_platformio` es la pieza que integra las librerias micro-ROS en el firmware del ESP32-S3. `palletizer.meta` define paquetes y tipos que deben entrar en el firmware.

## Dominio ROS

El firmware usa:

```text
MICRO_ROS_DOMAIN_ID = 10
```

Por lo tanto, el PC, el agente, las terminales y la UI deben usar:

```bash
export ROS_DOMAIN_ID=10
```

En cada terminal ROS:

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Verifica el tipo custom:

```bash
ros2 interface show palletizer_msgs/action/MoveXYZ
```

## Iniciar micro-ROS agent

Conecta el ESP32-S3 por USB y revisa el puerto:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Normalmente sera:

```text
/dev/ttyACM0
```

Inicia el agente:

```bash
source /home/felipe/microros_ws/install/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial --dev /dev/ttyACM0 -v4
```

Logs esperados:

```text
session established
participant created
publisher created
subscriber created
datawriter created
datareader created
```

## Verificacion ROS 2

En otra terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 node info /palletizer_controller
```

Debe aparecer:

```text
/palletizer_controller
```

Ver action:

```bash
ros2 action list -t
ros2 action info /palletizer/move_xyz
ros2 topic list --include-hidden-topics -t | grep move_xyz
ros2 service list --include-hidden-services -t | grep move_xyz
```

## Leer topicos

Posicion cartesiana en milimetros:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

Orden:

```text
[x_mm, y_mm, z_mm]
```

`joint_states` usa unidades ROS. Para ejes prismaticos, la posicion se publica en metros:

```bash
ros2 topic echo /joint_states
```

RPM:

```bash
ros2 topic echo /palletizer/motor_rpm
```

Estado general JSON:

```bash
ros2 topic echo /palletizer/status
```

Campos esperados en `/palletizer/status`:

```text
fault
reason
homing
online
enabled
stalled
enc31
raw35
mm
vel_mm_s
rpm
angleError
limits
moveStatus
home91
home3B
```

Falla resumida:

```bash
ros2 topic echo /palletizer/fault_state
```

## Enviar parada de emergencia

La emergencia local del ESP32-S3 tiene prioridad sobre ROS. ROS puede solicitar una parada, pero la logica local de seguridad sigue siendo la autoridad final.

```bash
ros2 topic pub --once /palletizer/emergency_stop std_msgs/msg/Bool "{data: true}"
```

## Enviar setpoint por action

Action:

```text
/palletizer/move_xyz
```

Tipo:

```text
palletizer_msgs/action/MoveXYZ
```

Campos del goal:

```text
x_mm                   posicion X absoluta en mm
y_mm                   posicion Y absoluta en mm
z_mm                   posicion Z absoluta en mm
use_a                  si es true, el action tambien mueve el eje A
a_deg                  posicion absoluta del eje A en grados
speed_mm_s             velocidad lineal XYZ en mm/s
accel_mm_s2            aceleracion lineal XYZ en mm/s2
angular_speed_deg_s    velocidad angular A en deg/s
angular_accel_deg_s2   aceleracion angular A en deg/s2
tolerance_mm           tolerancia de llegada XYZ en mm
angular_tolerance_deg  tolerancia de llegada A en grados
timeout_ms             tiempo maximo antes de abortar
```

Ejemplo:

```bash
ros2 action send_goal /palletizer/move_xyz palletizer_msgs/action/MoveXYZ \
"{x_mm: 50.0, y_mm: 50.0, z_mm: 20.0, use_a: true, a_deg: 90.0, speed_mm_s: 25.0, accel_mm_s2: 50.0, angular_speed_deg_s: 90.0, angular_accel_deg_s2: 180.0, tolerance_mm: 1.0, angular_tolerance_deg: 1.0, timeout_ms: 30000}" \
--feedback
```

Feedback esperado:

```text
current_x_mm
current_y_mm
current_z_mm
current_a_deg
error_x_mm
error_y_mm
error_z_mm
error_a_deg
progress
state
```

Estados comunes:

```text
accepted
queued
moving
waiting_for_encoders
arrived
fault
timeout
canceled
rejected_by_control
```

Resultado al llegar:

```text
success: true
message: target reached
final_x_mm
final_y_mm
final_z_mm
```

La condicion de llegada se basa en la posicion medida por el ESP32-S3 desde los encoders. El firmware exige que la posicion quede dentro de la tolerancia durante:

```text
ACTION_RESULT_POSITION_STABLE_MS = 100 ms
```

Periodo de feedback:

```text
ACTION_FEEDBACK_PERIOD_MS = 50 ms
```

## Telemetria CAN

El dato principal de posicion usado para control y confirmacion de llegada es `0x31`, que cuenta desde el homing.

Configuracion relevante:

```text
ENCODER_POLL_MS = 1
ENCODER_REQUESTS_PER_POLL = 2
POSITION_SAMPLE_MAX_AGE_MS = 4
```

Con 4 motores, el firmware busca actualizar cada motor cerca de 500 Hz.

Datos secundarios:

```text
0x32 RPM
0x35 encoder raw diagnostico
0x39 error angular
0x3A enable state
0x3B homing status
0x3E stall
```

Conversion a posicion lineal:

```text
ENCODER_COUNTS_PER_REV = 16384
LEADSCREW_MM_PER_REV = 8.0
ENCODER_COUNTS_PER_MM = 2048
```

## Eje A rotatorio y servo PWM

Se agrego un cuarto grado de libertad logico como eje `A`. Este eje usa un driver MKS Servo42D en la red CAN con:

```text
CAN ID A = 0x05
```

`A` es rotatorio y no usa limites lineales de software. Su posicion se calcula desde el encoder 0x31 en grados y se publica en:

```text
/joint_states        A en radianes
/palletizer/status   a_deg y arrays de 5 motores
/palletizer/motor_rpm 5 valores: X1, X2, Y, Z, A
```

El homing independiente del eje rotatorio se puede solicitar desde React con el boton de homing seleccionando `A rotatorio`. El backend usa la action `/palletizer/home_axis` si esta disponible; si no, publica el comando textual:

```text
HOME A
```

El servo auxiliar por PWM tradicional se configura en `src/config.h`:

```cpp
AUX_SERVO_PWM_PIN = GPIO_NUM_17
AUX_SERVO_MIN_US = 500
AUX_SERVO_MAX_US = 2500
AUX_SERVO_CENTER_US = 1500
```

El firmware usa la libreria `ESP32Servo` y escribe el pulso con `writeMicroseconds()`.

Comandos textuales disponibles:

```text
SERVO <0..180>
SERVO_US <500..2500>
SERVO OFF
```

La UI incluye un panel `Servo PWM` para enviar angulo, pulso en microsegundos o desactivar el PWM.


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

## Interfaz React

Arquitectura UI:

```text
ESP32-S3 /palletizer_controller
  micro-ROS serial USB
  micro_ros_agent
PC ROS 2 Humble
  /palletizer_ui_backend
  WebSocket JSON
React dashboard
```

El frontend React no se conecta directamente a ROS 2. Se conecta al backend por WebSocket. El backend escucha:

```text
/joint_states
/palletizer/axis_position_mm
/palletizer/motor_rpm
/palletizer/status
/palletizer/fault_state
```

Y controla:

```text
/palletizer/emergency_stop
/palletizer/move_xyz
```

La UI incluye:

- telemetria de posicion, RPM y estado
- controles de movimiento XYZ + A por action
- joystick digital para jog discreto XY/Z
- parada de emergencia
- panel de feedback/result de action
- pinza servo binaria por servicio `/palletizer/set_gripper`: Tomar y Soltar
- ventana deslizante de estado de motores
- gemelo digital 3D
- controles secundarios deshabilitados si no existen en ROS

## Gemelo digital 3D

El panel 3D usa `three`, `@react-three/fiber` y `@react-three/drei`. Actualmente usa geometria procedural como plantilla.

Interaccion actual:

- Por defecto, el mouse orienta la camara.
- Al hacer click en la esfera amarilla, se habilita la edicion del setpoint.
- En modo edicion, el usuario arrastra X/Y sobre el plano de trabajo.
- Z, velocidad/aceleracion lineal, A en grados, velocidad/aceleracion angular, tolerancias y timeout se ingresan por teclado.
- Al presionar `Enviar target 3D`, el frontend envia el comando `move_xyz` al backend.
- La esfera verde representa la posicion medida.
- La esfera amarilla representa el setpoint pendiente.

Archivos del modelo real:

```text
ui/frontend/public/robot_description/
|-- README.md
|-- urdf/
|   |-- README.md
|   |-- palletizer.template.urdf
|   `-- palletizer.urdf
`-- meshes/
    |-- stl/
    |   |-- README.md
    |   |-- base.stl
    |   |-- x_axis.stl
    |   |-- y_axis.stl
    |   |-- z_axis.stl
    |   `-- tool.stl
    |-- collision/
    `-- web/
```

Formato recomendado:

```text
URDF + STL
```

URDF trabaja en metros. Si el CAD exporta STL en mm, usa en el URDF:

```xml
<mesh filename="/robot_description/meshes/stl/base.stl" scale="0.001 0.001 0.001"/>
```

Para React/Vite convienen rutas web absolutas:

```xml
<mesh filename="/robot_description/meshes/stl/base.stl"/>
```

Evita rutas ROS `package://` dentro del URDF usado por el navegador.

Joints recomendados:

```text
x_axis_joint
y_axis_joint
z_axis_joint
```

Estructura logica:

```text
base_link
`-- x_axis_link       x_axis_joint, prismatic, axis 1 0 0
    `-- y_axis_link   y_axis_joint, prismatic, axis 0 1 0
        `-- z_axis_link  z_axis_joint, prismatic, axis 0 0 1
            `-- tool_link
```

Mapeo de telemetria:

```text
x_joint = x_mm / 1000
y_joint = y_mm / 1000
z_joint = z_mm / 1000
```

## Arranque UI manual

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

## Arranque UI con launch file

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

Opciones utiles:

```bash
# Si el agente ya esta corriendo
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_agent:=false

# Sin frontend React
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py start_frontend:=false

# Ver comandos expandidos
ros2 launch /home/felipe/proyecto/Esp32_paletizador/ui/launch/palletizer_ui.launch.py show_commands:=true
```

## Backend WebSocket

El backend levanta por defecto:

```text
ws://127.0.0.1:8765
```

Comandos JSON principales:

```json
{"type":"move_xyz","goal":{"x_mm":50,"y_mm":50,"z_mm":20,"use_a":true,"a_deg":90,"speed_mm_s":25,"accel_mm_s2":50,"angular_speed_deg_s":90,"angular_accel_deg_s2":180,"tolerance_mm":1,"angular_tolerance_deg":1,"timeout_ms":30000}}
{"type":"emergency_stop","data":true}
{"type":"refresh"}
```

## Compilar y cargar firmware

Compilar:

```bash
cd /home/felipe/proyecto/Esp32_paletizador
/home/felipe/.platformio/penv/bin/pio run
```

Cargar al ESP32-S3:

```bash
/home/felipe/.platformio/penv/bin/pio run -t upload
```

Si modificas `palletizer.meta`, limpia micro-ROS antes de compilar:

```bash
/home/felipe/.platformio/penv/bin/pio run -t clean_microros
/home/felipe/.platformio/penv/bin/pio run
```

## Problemas comunes

### `The passed action type is invalid`

Falta cargar `palletizer_msgs` en esa terminal.

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
ros2 interface show palletizer_msgs/action/MoveXYZ
```

### `ros2 action list` no muestra nada

Revisar:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
ros2 node list
ros2 node info /palletizer_controller
ros2 action list -t
```

Tambien confirma que el agente use el mismo dominio `ROS_DOMAIN_ID=10`.

### Aparecen topicos, pero no services

Es normal en el firmware actual. Los services estan desactivados por estabilidad micro-ROS.

### No aparece `/palletizer/command`

En el firmware actual deberia aparecer. Si no aparece, probablemente estas ejecutando firmware antiguo o el agente no reconstruyo entidades despues de cargar. Reinicia el agente y verifica `ros2 node info /palletizer_controller`. La interfaz recomendada para movimiento cartesiano sigue siendo `/palletizer/move_xyz`; `/palletizer/command` queda para comandos auxiliares como `HOME A` y `SERVO`.

### Diferencia entre setpoint y `/joint_states`

Los setpoints de la action estan en milimetros. En ROS, `joint_states.position` para ejes prismaticos esta en metros. Para comparar en milimetros usa:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

### El ESP32 no aparece como `/dev/ttyACM0`

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Usa el puerto que aparezca en el comando del agente.

### El puerto esta ocupado

```bash
fuser -v /dev/ttyACM0
```

Deten el agente con `Ctrl+C` antes de cargar firmware.

### Node falla en WSL con rutas UNC

Ejecuta `npm install` desde Ubuntu/WSL dentro de `/home/felipe/...`, no desde Windows sobre `\\wsl.localhost\...`.

## Documentacion

Las guias acumuladas quedan en:

```text
docs/README.md
docs/GUIA_MICRO_ROS.md
docs/GUIA_UI_REACT.md
docs/GUIA_MODELO_3D_REACT.md
```

Rutas historicas mantenidas:

```text
GUIA_MICRO_ROS.md
ui/README.md
ui/frontend/README.md
ui/backend/README.md
ui/GUIA_MODELO_3D_REACT.md
```


La barra superior del visor muestra cuatro estados: `UI conectada`, `Conexion fisica ESP32`, `Comunicacion ROS` y `Paletizador operativo`. `UI conectada` solo confirma WebSocket con el backend. `Conexion fisica ESP32` se activa si el nodo `/palletizer_controller` aparece en el grafo o llega telemetria desde el ESP32-S3. `Comunicacion ROS` exige mensajes ROS frescos. `Paletizador operativo` exige comunicacion ROS, sin falla, y los 5 motores `online`, `enabled` y sin `stalled`.
