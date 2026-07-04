# Guia basica micro-ROS ESP32-S3

Esta guia resume el estado actual del proyecto del paletizador con ESP32-S3,
micro-ROS Humble y drivers MKS por CAN/TWAI.

El ESP32-S3 no ejecuta ROS 2 completo. Ejecuta un cliente micro-ROS y se
comunica con el PC mediante `micro_ros_agent` por USB serial. El PC traduce esa
conexion hacia el grafo ROS 2.

## 1. Estado actual del firmware

El perfil actual prioriza estabilidad del nodo, telemetria, parada de
emergencia y la action de movimiento cartesiano.

Nodo esperado:

```text
/palletizer_controller
```

Topicos publicados actualmente:

```text
/joint_states
/palletizer/axis_position_mm
/palletizer/fault_state
/palletizer/motor_rpm
/palletizer/status
```

Subscribers activos actualmente:

```text
/palletizer/emergency_stop
/palletizer/command
```

Action activa actualmente:

```text
/palletizer/move_xyz [palletizer_msgs/action/MoveXYZ]
```

No hay services activos en el perfil actual. El subscriber `/palletizer/command` queda activo para comandos auxiliares de bajo costo como `HOME A` y control del servo PWM.

Estas funciones existen en codigo o en las interfaces, pero estan desactivadas
en el grafo ROS actual para evitar reconexiones e inestabilidad por exceso de
entidades micro-ROS:

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

Las banderas relevantes estan en `src/ros_bridge.cpp`:

```cpp
ROS_ENABLE_COMMAND_SUBSCRIBER = false;
ROS_ENABLE_SERVICE_SERVERS = false;
ROS_ENABLE_EXTENDED_SERVICE_SERVERS = false;
ROS_ENABLE_HOME_ORIGIN_ACTIONS = false;
```

## 2. Preparar terminal ROS 2

En cada terminal donde uses `ros2`, carga el workspace de mensajes y fija el
dominio:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Verifica que el tipo custom exista:

```bash
ros2 interface show palletizer_msgs/action/MoveXYZ
```

Si aparece `Unknown package 'palletizer_msgs'`, falta cargar el `source`
anterior.

El firmware usa:

```text
MICRO_ROS_DOMAIN_ID = 10
```

Por lo tanto el PC, el agente y las terminales ROS deben usar:

```text
ROS_DOMAIN_ID=10
```

## 3. Iniciar el agente micro-ROS

Conecta el ESP32-S3 por USB y revisa el puerto:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Normalmente aparece:

```text
/dev/ttyACM0
```

Levanta el agente en una terminal dedicada:

```bash
source /home/felipe/microros_ws/install/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial --dev /dev/ttyACM0 -v4
```

Deja esa terminal abierta.

En los logs deberias ver mensajes como:

```text
create_client
session established
participant created
publisher created
subscriber created
datawriter created
datareader created
```

Si aparece otro puerto, cambia `/dev/ttyACM0` por el puerto real.

## 4. Verificacion rapida del grafo ROS

En otra terminal:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Reinicia el daemon si ROS 2 queda con cache vieja:

```bash
ros2 daemon stop
ros2 daemon start
```

Ver nodos:

```bash
ros2 node list
```

Debe aparecer:

```text
/palletizer_controller
```

Ver detalle del nodo:

```bash
ros2 node info /palletizer_controller
```

Estado esperado actualmente:

```text
Subscribers:
  /palletizer/emergency_stop
  /palletizer/command

Publishers:
  /joint_states
  /palletizer/axis_position_mm
  /palletizer/fault_state
  /palletizer/motor_rpm
  /palletizer/status

Service Servers:
  ninguno

Action Servers:
  /palletizer/move_xyz
```

Ver action:

```bash
ros2 action list -t
ros2 action info /palletizer/move_xyz
```

Ver endpoints internos de la action:

```bash
ros2 topic list --include-hidden-topics -t | grep move_xyz
ros2 service list --include-hidden-services -t | grep move_xyz
```

## 5. Leer topicos

Leer posicion cartesiana directa en milimetros:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

Orden esperado:

```text
[x_mm, y_mm, z_mm]
```

Este es el topico mas directo para comparar contra los setpoints enviados por
la action.

Leer `joint_states`:

```bash
ros2 topic echo /joint_states
```

En `joint_states`, las posiciones prismaticas se expresan en metros. Por
ejemplo, 50 mm deberia verse como aproximadamente:

```text
0.05
```

Leer RPM de motores:

```bash
ros2 topic echo /palletizer/motor_rpm
```

Leer estado general:

```bash
ros2 topic echo /palletizer/status
```

`/palletizer/status` publica un JSON con datos como:

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

Leer estado de falla resumido:

```bash
ros2 topic echo /palletizer/fault_state
```

## 6. Enviar parada de emergencia

La emergencia local del ESP32-S3 tiene prioridad sobre ROS. ROS puede solicitar
una parada, pero la logica local de seguridad sigue siendo la autoridad final.

Enviar parada por ROS:

```bash
ros2 topic pub --once /palletizer/emergency_stop std_msgs/msg/Bool "{data: true}"
```

## 7. Enviar setpoint por action

Action disponible:

```text
/palletizer/move_xyz
```

Tipo:

```text
palletizer_msgs/action/MoveXYZ
```

Campos del goal:

```text
x_mm          posicion X absoluta en mm
y_mm          posicion Y absoluta en mm
z_mm          posicion Z absoluta en mm
speed_mm_s    velocidad lineal en mm/s
accel_mm_s2   aceleracion lineal en mm/s2
tolerance_mm  tolerancia de llegada en mm
timeout_ms    tiempo maximo antes de abortar
```

Ejemplo:

```bash
ros2 action send_goal /palletizer/move_xyz palletizer_msgs/action/MoveXYZ \
"{x_mm: 50.0, y_mm: 50.0, z_mm: 20.0, speed_mm_s: 25.0, accel_mm_s2: 50.0, tolerance_mm: 1.0, timeout_ms: 30000}" \
--feedback
```

Feedback esperado:

```text
current_x_mm
current_y_mm
current_z_mm
error_x_mm
error_y_mm
error_z_mm
progress
state
```

Estados comunes de `state`:

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

Resultado esperado al llegar:

```text
success: true
message: target reached
final_x_mm
final_y_mm
final_z_mm
```

La condicion de llegada se basa en la posicion medida por el ESP32-S3 desde los
encoders. Actualmente el firmware exige que la posicion quede dentro de la
tolerancia durante:

```text
ACTION_RESULT_POSITION_STABLE_MS = 100 ms
```

El feedback de la action esta limitado a:

```text
ACTION_FEEDBACK_PERIOD_MS = 50 ms
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
AUX_SERVO_PWM_PIN = GPIO_NUM_18
AUX_SERVO_MIN_US = 500
AUX_SERVO_MAX_US = 2500
AUX_SERVO_CENTER_US = 1500
```

Comandos textuales disponibles:

```text
SERVO <0..180>
SERVO_US <500..2500>
SERVO OFF
```

La UI incluye un panel `Servo PWM` para enviar angulo, pulso en microsegundos o desactivar el PWM.

## 8. Telemetria y muestreo CAN

El dato principal de posicion usado para control y confirmacion de llegada es
`0x31`, que cuenta desde el homing.

Configuracion relevante:

```text
ENCODER_POLL_MS = 1
ENCODER_REQUESTS_PER_POLL = 2
POSITION_SAMPLE_MAX_AGE_MS = 4
```

Con 4 motores, esto busca actualizar cada motor cerca de 500 Hz.

Datos secundarios:

```text
0x32 RPM
0x35 encoder raw diagnostico
0x39 error angular
0x3A enable state
0x3B homing status
0x3E stall
```

La conversion a posicion lineal usa:

```text
ENCODER_COUNTS_PER_REV = 16384
LEADSCREW_MM_PER_REV = 8.0
ENCODER_COUNTS_PER_MM = 2048
```

## 9. Latencia esperada y puntos de cuidado

Si el comando por action demora mas de un segundo, revisa primero estos puntos:

```bash
ros2 node info /palletizer_controller
ros2 action info /palletizer/move_xyz
```

Si el nodo aparece y desaparece, el problema suele ser reconexion o creacion de
entidades micro-ROS, no el movimiento en si.

El USB CDC del ESP32-S3 no se comporta igual que un UART fisico de 115200
baudios. `SERIAL_BAUD = 115200` no suele ser el cuello principal cuando usas
`/dev/ttyACM0` por USB nativo. El problema visto hasta ahora fue mas bien la
presion de entidades micro-ROS y telemetria.

Por eso el perfil actual mantiene desactivados los services y actions secundarias, pero deja activo `/palletizer/command` para comandos auxiliares de baja carga como `HOME A` y `SERVO`.

## 10. Problemas comunes

### `The passed action type is invalid`

La terminal no tiene cargado el paquete `palletizer_msgs`.

Solucion:

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

Tambien revisa que el agente este corriendo con el mismo dominio:

```bash
export ROS_DOMAIN_ID=10
```

### Aparecen topicos, pero no services

Es normal en el firmware actual. Los services estan desactivados por estabilidad
micro-ROS.

### No aparece `/palletizer/command`

En el firmware actual deberia aparecer. Si no aparece, probablemente estas ejecutando firmware antiguo o el agente no reconstruyo entidades despues de cargar. Reinicia el agente y verifica:

```bash
ros2 node info /palletizer_controller
```

La interfaz recomendada para movimiento cartesiano sigue siendo `/palletizer/move_xyz`; `/palletizer/command` queda para comandos auxiliares como `HOME A` y `SERVO`.

### Diferencia entre setpoint y `/joint_states`

Los setpoints de la action estan en milimetros. En ROS, `joint_states.position`
para ejes prismaticos esta en metros.

Para comparar en milimetros usa:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

### El ESP32 no aparece como `/dev/ttyACM0`

Revisar puertos:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Usa el puerto que aparezca en el comando del agente.

### El puerto esta ocupado

Busca procesos usando el puerto:

```bash
fuser -v /dev/ttyACM0
```

Deten el agente con `Ctrl+C` antes de cargar firmware.

## 11. Compilar y cargar firmware

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

## 12. Nota sobre funciones futuras

El codigo contiene base para:

```text
EnableAxis
SetAxisLimits
SetZero
ClearFault
ReleaseStall
GetDriverStatus
HomeAxis
GoOrigin
```

Pero reactivarlas aumenta la cantidad de entidades micro-ROS. Si se vuelven a
activar, hay que validar que el agente cree todas las entidades sin reconectar y
que `ros2 node info /palletizer_controller` muestre el grafo completo de forma
estable.

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
