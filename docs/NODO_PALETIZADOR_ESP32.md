# Nodo del paletizador ESP32-S3

Este documento catastra solo el nodo ROS 2 creado por el firmware del ESP32-S3. La fuente de verdad es `src/ros_bridge.cpp` y las interfaces custom estan en `extra_packages/palletizer_msgs`.

## Identidad del nodo

- Nodo: `/palletizer_controller`
- Dominio ROS usado por firmware: `ROS_DOMAIN_ID=10`
- Transporte micro-ROS: serial por USB CDC nativo del ESP32-S3
- MCU: ESP32-S3
- Funcion: puente entre ROS 2 y la logica local del paletizador, drivers MKS por CAN, emergencia local y servo PWM auxiliar.

## Resumen actual de interfaces activas

Con los flags actuales del firmware, el nodo expone:

| Tipo | Cantidad | Interfaces |
|---|---:|---|
| Publishers | 5 | `/joint_states`, `/palletizer/axis_position_mm`, `/palletizer/motor_rpm`, `/palletizer/status`, `/palletizer/fault_state` |
| Subscribers | 4 | `/palletizer/emergency_stop`, `/palletizer/jog_xyz_delta`, `/palletizer/command`, `/palletizer/fast_move_xyz` |
| Services | 1 | `/palletizer/set_gripper` |
| Actions | 1 | `/palletizer/move_xyz` |

Los servicios `enable_axis`, `set_axis_limits`, `set_zero`, `clear_fault`, `release_stall`, `get_driver_status` y las actions `home_axis`, `go_origin` estan definidos en el codigo y en `palletizer_msgs`, pero no estan activos en el nodo actual porque los flags `ROS_ENABLE_SERVICE_SERVERS`, `ROS_ENABLE_EXTENDED_SERVICE_SERVERS` y `ROS_ENABLE_HOME_ORIGIN_ACTIONS` estan en `false`.

## Publishers

### `/joint_states`

- Tipo: `sensor_msgs/msg/JointState`
- Frecuencia actual: `ROS_FAST_TELEMETRY_PERIOD_MS`, actualmente 50 ms, aprox. 20 Hz.
- Joints publicados: `X1`, `X2`, `Y`, `Z`, `A`.
- Posicion:
  - `X1`, `X2`, `Y`, `Z`: metros.
  - `A`: radianes.
- Velocidad:
  - `X1`, `X2`, `Y`, `Z`: m/s.
  - `A`: rad/s.
- `effort`: siempre `0.0`.
- Fuente: encoder `0x31` procesado localmente en el ESP32-S3.

Uso:

```bash
ros2 topic echo /joint_states
```

### `/palletizer/axis_position_mm`

- Tipo: `std_msgs/msg/Float32MultiArray`
- Frecuencia actual: 50 ms, aprox. 20 Hz.
- Datos: `[x_mm, y_mm, z_mm]`.
- Fuente:
  - `X`: calculado desde el par `X1/X2`.
  - `Y`: eje lineal Y.
  - `Z`: eje lineal Z.
- Objetivo: posicion cartesiana lineal usada por UI, feedback y visualizacion 3D.

Uso:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

### `/palletizer/motor_rpm`

- Tipo: `std_msgs/msg/Float32MultiArray`
- Frecuencia actual: `ROS_STATUS_TELEMETRY_PERIOD_MS`, actualmente 250 ms, aprox. 4 Hz.
- Datos: `[x1_rpm, x2_rpm, y_rpm, z_rpm, a_rpm]`.
- Fuente: respuesta CAN de velocidad/RPM del driver.
- Objetivo: telemetria secundaria de velocidad de motor.

Uso:

```bash
ros2 topic echo /palletizer/motor_rpm
```

### `/palletizer/status`

- Tipo: `std_msgs/msg/String`
- Frecuencia actual: 250 ms, aprox. 4 Hz.
- Formato: JSON en texto.
- Objetivo: estado consolidado del paletizador, motores, homing, diagnostico y servo auxiliar.

Campos principales del JSON:

| Campo | Tipo logico | Descripcion |
|---|---|---|
| `fault` | numero 0/1 | `1` si existe falla de seguridad local. |
| `reason` | string | Motivo de falla o `OK`. |
| `homing` | string | Estado interno de homing. |
| `online` | bool[5] | Motor visto por CAN dentro del timeout. |
| `enabledOk` | bool[5] | Estado enable leido correctamente. |
| `enabled` | bool[5] | Motor habilitado. |
| `stalled` | bool[5] | Driver reporta stall. |
| `raw35Ok` | bool[5] | Lectura diagnostica `0x35` valida. |
| `angleOk` | bool[5] | Error angular valido. |
| `enc31` | int[5] | Encoder `0x31`, cuenta desde homing/cero. |
| `raw35` | int[5] | Encoder diagnostico `0x35`. |
| `mm` | float[5] | Posicion por motor. En A representa grados aunque el nombre sea heredado. |
| `vel_mm_s` | float[5] | Velocidad por motor. En A representa deg/s. |
| `rpm` | int[5] | RPM por motor. |
| `lastAcc` | uint8[5] | Ultima aceleracion MKS comandada por motor. |
| `angleError` | int[5] | Error angular reportado por driver. |
| `units` | string[5] | `mm`, `mm`, `mm`, `mm`, `deg`. |
| `limits` | array | Limites configurados para X/Y/Z. |
| `moveStatus` | uint8[5] | Estado de movimiento reportado por driver. |
| `home91` | uint8[5] | Estado asociado a homing `0x91`. |
| `home3B` | array[5] | Estado homing `0x3B`: `[single_turn, origin]`. |
| `a_deg` | float | Posicion angular del eje A en grados. |
| `aux_servo` | object | Estado del servo PWM auxiliar. |

Orden de motores en arreglos de 5 elementos:

```text
[X1, X2, Y, Z, A]
```

Ejemplo de inspeccion:

```bash
ros2 topic echo /palletizer/status
```

### `/palletizer/fault_state`

- Tipo: `std_msgs/msg/String`
- Frecuencia actual: 250 ms, aprox. 4 Hz.
- Formato: `OK:<reason>` o `FAULT:<reason>`.
- Objetivo: indicador rapido de seguridad/falla.

Uso:

```bash
ros2 topic echo /palletizer/fault_state
```

## Subscribers

### `/palletizer/emergency_stop`

- Tipo: `std_msgs/msg/Bool`
- QoS: best effort.
- Campo usado: `data`.
- Efecto:
  - `true`: solicita parada de emergencia desde ROS.
  - La logica local de emergencia del ESP32-S3 mantiene prioridad superior a ROS.

Comando:

```bash
ros2 topic pub --once /palletizer/emergency_stop std_msgs/msg/Bool "{data: true}"
```

### `/palletizer/fast_move_xyz`

- Tipo: `std_msgs/msg/Float32MultiArray`
- QoS: best effort.
- Datos esperados:

```text
[x_mm, y_mm, z_mm, speed_mm_s, accel_mm_s2]
```

- `speed_mm_s` y `accel_mm_s2` son opcionales. Si no vienen, el firmware usa defaults.
- Objetivo: movimiento lineal XYZ rapido desde UI/backend sin pasar por action.
- No controla eje A.

Ejemplo:

```bash
ros2 topic pub --once /palletizer/fast_move_xyz std_msgs/msg/Float32MultiArray "{data: [50.0, 50.0, 20.0, 25.0, 50.0]}"
```

### `/palletizer/jog_xyz_delta`

- Tipo: `std_msgs/msg/Float32MultiArray`
- QoS: best effort.
- Datos esperados:

```text
[dx_mm, dy_mm, dz_mm, speed_mm_s, accel_mm_s2]
```

- `speed_mm_s` y `accel_mm_s2` son opcionales. Si no vienen, el firmware usa defaults.
- Objetivo: jog relativo de baja latencia desde la UI/backend.
- Diferencia clave respecto a `/palletizer/fast_move_xyz`: el ESP32-S3 calcula el objetivo absoluto usando su snapshot local de posicion, evitando que la interfaz mande setpoints basados en telemetria atrasada.
- No controla eje A.

Ejemplo:

```bash
ros2 topic pub --once /palletizer/jog_xyz_delta std_msgs/msg/Float32MultiArray "{data: [5.0, 0.0, 0.0, 25.0, 50.0]}"
```

### `/palletizer/command`

- Tipo: `std_msgs/msg/String`
- QoS: best effort.
- Objetivo: entrada de comandos textuales compatibles con el parser local del firmware.
- Uso recomendado: diagnostico, fallback y comandos operativos puntuales. Para movimiento principal usar preferentemente action o `fast_move_xyz` segun el caso.

Comandos relevantes soportados por el parser local:

```text
POSXYZ <x_mm> <y_mm> <z_mm> [mm_s] [mm_s2]
HOME <X|Y|Z|A|ALL> [min_mm max_mm] [fast_rpm slow_rpm]
ZERO <X|Y|Z> <min_mm> <max_mm>
ENABLE <X|Y|Z|A|ALL>
DISABLE <X|Y|Z|A|ALL>
SERVO <deg|OFF>
SERVO_US <us>
FAULT STATUS
FAULT RESET
FAULT TEST
```

Ejemplo:

```bash
ros2 topic pub --once /palletizer/command std_msgs/msg/String "{data: 'POSXYZ 50 50 20 25 50'}"
```

## Services activos

### `/palletizer/set_gripper`

- Tipo: `palletizer_msgs/srv/SetGripper`
- Funcion: accionar el servo PWM auxiliar como pinza binaria de lazo abierto.
- Mapeo actual:
  - `closed: true`: tomar/cerrar, angulo `0 deg`, pulso `AUX_SERVO_MIN_US`.
  - `closed: false`: soltar/abrir, angulo `180 deg`, pulso `AUX_SERVO_MAX_US`.
- Pin actual del servo: publicado en `/palletizer/status.aux_servo.pin`, actualmente `GPIO17`.

Request:

```text
bool closed
```

Response:

```text
bool success
string message
bool closed
float32 angle_deg
uint16 pulse_us
```

Comandos:

```bash
ros2 service call /palletizer/set_gripper palletizer_msgs/srv/SetGripper "{closed: true}"
ros2 service call /palletizer/set_gripper palletizer_msgs/srv/SetGripper "{closed: false}"
```

## Actions activas

### `/palletizer/move_xyz`

- Tipo: `palletizer_msgs/action/MoveXYZ`
- Funcion: enviar un setpoint absoluto XYZ y opcionalmente A.
- Criterio de finalizacion: el ESP32-S3 verifica posicion real medida por encoder/telemetria local y confirma resultado cuando entra en tolerancia y se mantiene estable.
- Solo puede haber una action activa a la vez.
- Si hay otra action activa, el goal se rechaza.

Goal:

```text
float32 x_mm
float32 y_mm
float32 z_mm
bool use_a
float32 a_deg
float32 speed_mm_s
float32 accel_mm_s2
float32 angular_speed_deg_s
float32 angular_accel_deg_s2
float32 tolerance_mm
float32 angular_tolerance_deg
uint32 timeout_ms
```

Result:

```text
bool success
string message
float32 final_x_mm
float32 final_y_mm
float32 final_z_mm
float32 final_a_deg
```

Feedback:

```text
float32 current_x_mm
float32 current_y_mm
float32 current_z_mm
float32 current_a_deg
float32 error_x_mm
float32 error_y_mm
float32 error_z_mm
float32 error_a_deg
float32 progress
string state
```

Ejemplo:

```bash
ros2 action send_goal /palletizer/move_xyz palletizer_msgs/action/MoveXYZ \
  "{x_mm: 50.0, y_mm: 50.0, z_mm: 20.0, use_a: false, a_deg: 0.0, speed_mm_s: 25.0, accel_mm_s2: 50.0, angular_speed_deg_s: 90.0, angular_accel_deg_s2: 180.0, tolerance_mm: 1.0, angular_tolerance_deg: 1.0, timeout_ms: 30000}" \
  --feedback
```

Ejemplo con eje A:

```bash
ros2 action send_goal /palletizer/move_xyz palletizer_msgs/action/MoveXYZ \
  "{x_mm: 50.0, y_mm: 50.0, z_mm: 20.0, use_a: true, a_deg: 90.0, speed_mm_s: 25.0, accel_mm_s2: 50.0, angular_speed_deg_s: 90.0, angular_accel_deg_s2: 180.0, tolerance_mm: 1.0, angular_tolerance_deg: 1.0, timeout_ms: 30000}" \
  --feedback
```

## Interfaces definidas pero no activas actualmente

Estas interfaces existen en `extra_packages/palletizer_msgs` y hay codigo asociado en `ros_bridge.cpp`, pero el firmware actual no las expone porque sus flags estan desactivados.

### Actions desactivadas

| Action | Tipo | Flag requerido | Uso previsto |
|---|---|---|---|
| `/palletizer/home_axis` | `palletizer_msgs/action/HomeAxis` | `ROS_ENABLE_HOME_ORIGIN_ACTIONS=true` | Homing de X/Y/Z/A/ALL con secuencia rapida/lenta. |
| `/palletizer/go_origin` | `palletizer_msgs/action/GoOrigin` | `ROS_ENABLE_HOME_ORIGIN_ACTIONS=true` | Ir al origen de un eje o todos. |

### Services desactivados

| Servicio | Tipo | Flag requerido | Uso previsto |
|---|---|---|---|
| `/palletizer/enable_axis` | `palletizer_msgs/srv/EnableAxis` | `ROS_ENABLE_SERVICE_SERVERS=true` | Habilitar/deshabilitar X/Y/Z/A/ALL/X1/X2. |
| `/palletizer/set_axis_limits` | `palletizer_msgs/srv/SetAxisLimits` | `ROS_ENABLE_SERVICE_SERVERS=true` | Configurar limites de software X/Y/Z. |
| `/palletizer/set_zero` | `palletizer_msgs/srv/SetZero` | `ROS_ENABLE_EXTENDED_SERVICE_SERVERS=true` | Setear cero de X/Y/Z y limites asociados. |
| `/palletizer/clear_fault` | `palletizer_msgs/srv/ClearFault` | `ROS_ENABLE_EXTENDED_SERVICE_SERVERS=true` | Limpiar falla de seguridad si la condicion local lo permite. |
| `/palletizer/release_stall` | `palletizer_msgs/srv/ReleaseStall` | `ROS_ENABLE_EXTENDED_SERVICE_SERVERS=true` | Liberar stall del driver. |
| `/palletizer/get_driver_status` | `palletizer_msgs/srv/GetDriverStatus` | `ROS_ENABLE_EXTENDED_SERVICE_SERVERS=true` | Snapshot completo de drivers, encoders, homing y limites. |

Nota: aunque estas interfaces esten desactivadas como servicios/actions, varias operaciones siguen disponibles por `/palletizer/command` como fallback textual.

## Verificacion desde ROS 2

Con el agente micro-ROS conectado y el dominio correcto:

```bash
export ROS_DOMAIN_ID=10
ros2 node list
ros2 node info /palletizer_controller
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

Resultado esperado con el firmware actual:

```text
Publishers:
  /joint_states: sensor_msgs/msg/JointState
  /palletizer/axis_position_mm: std_msgs/msg/Float32MultiArray
  /palletizer/motor_rpm: std_msgs/msg/Float32MultiArray
  /palletizer/status: std_msgs/msg/String
  /palletizer/fault_state: std_msgs/msg/String

Subscribers:
  /palletizer/emergency_stop: std_msgs/msg/Bool
  /palletizer/fast_move_xyz: std_msgs/msg/Float32MultiArray
  /palletizer/jog_xyz_delta: std_msgs/msg/Float32MultiArray
  /palletizer/command: std_msgs/msg/String

Service Servers:
  /palletizer/set_gripper: palletizer_msgs/srv/SetGripper

Action Servers:
  /palletizer/move_xyz: palletizer_msgs/action/MoveXYZ
```

## Relacion con la arquitectura local

El nodo ROS no ejecuta directamente el control critico. El flujo es:

```text
ROS 2 / UI
  -> micro-ROS serial
  -> /palletizer_controller
  -> cola de comandos del ESP32-S3
  -> tarea local de control en core 1
  -> scheduler CAN determinista
  -> drivers MKS
```

La emergencia local, el scheduler CAN, la lectura de encoder `0x31` y la validacion de llegada a destino viven en el ESP32-S3. ROS 2 entrega setpoints, recibe telemetria y muestra feedback, pero no debe considerarse el lazo duro de control.
