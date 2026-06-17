# Guia basica micro-ROS ESP32-S3

Esta guia resume el uso del ESP32-S3 como cliente micro-ROS para el
paletizador. El ESP32-S3 no ejecuta ROS 2 completo: ejecuta un cliente
micro-ROS y se comunica por USB serial con el `micro_ros_agent` en el PC.

## 1. Preparar una terminal ROS 2

Antes de usar los topicos o la action, carga el entorno del paquete de mensajes:

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

## 2. Crear o verificar el agente micro-ROS

El agente corre en el PC y traduce la comunicacion micro-ROS del ESP32-S3 hacia
el grafo ROS 2 Humble. Debe estar compilado para la misma distribucion ROS 2 que
estas usando: Humble.

### Opcion A: usar el agente ya instalado

Primero carga el entorno:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Verifica que ROS 2 encuentre el paquete:

```bash
ros2 pkg prefix micro_ros_agent
```

Si el comando muestra una ruta, el agente ya esta disponible.

### Opcion B: crear el agente desde `micro_ros_setup`

Usa esta opcion si `ros2 pkg prefix micro_ros_agent` no encuentra el paquete o
si necesitas reconstruir el agente.

Instala dependencias base:

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool git
```

Crea el workspace de micro-ROS:

```bash
mkdir -p /home/felipe/microros_ws/src
cd /home/felipe/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
```

Instala dependencias:

```bash
cd /home/felipe/microros_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Compila `micro_ros_setup`:

```bash
cd /home/felipe/microros_ws
colcon build
source install/setup.bash
```

Crea el workspace del agente:

```bash
ros2 run micro_ros_setup create_agent_ws.sh
```

Compila el agente:

```bash
ros2 run micro_ros_setup build_agent.sh
source install/setup.bash
```

Verifica:

```bash
ros2 pkg prefix micro_ros_agent
```

Si el proyecto usa mensajes custom como `palletizer_msgs/action/MoveXYZ`, carga
tambien el entorno del workspace de mensajes:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

El dominio usado por este firmware es:

```text
ROS_DOMAIN_ID=10
```

El PC y el ESP32-S3 deben usar el mismo dominio. En el firmware se configura con
`MICRO_ROS_DOMAIN_ID = 10`.

## 3. Conectar el ESP32-S3 al agente micro-ROS

Conecta el ESP32-S3 por USB y revisa que aparezca un puerto:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Normalmente aparece como:

```bash
/dev/ttyACM0
```

Levanta el agente en una terminal dedicada:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
ROS_DOMAIN_ID=10 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Deja esa terminal abierta. Si el ESP32 esta conectado correctamente, el agente
debe mostrar mensajes como `create_client`, `session established`,
`publisher created`, `subscriber created` y `replier created`.

Si el puerto aparece como `/dev/ttyUSB0`, cambia el comando:

```bash
ROS_DOMAIN_ID=10 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
```

Verifica que el agente quedo en el dominio correcto desde otra terminal:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
ros2 node list
```

Si el agente esta corriendo pero no aparecen nodos, revisa estas tres cosas:

```bash
echo $ROS_DOMAIN_ID
ls /dev/ttyACM* /dev/ttyUSB*
ros2 action list -t
```

El valor de `ROS_DOMAIN_ID` debe ser `10` en todas las terminales que usen ROS.

## 4. Ver nodos, topicos y action

En otra terminal:

```bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
```

Ver nodos:

```bash
ros2 node list
```

El nodo esperado es:

```bash
/palletizer_controller
```

Ver informacion del nodo:

```bash
ros2 node info /palletizer_controller
```

Topicos esperados:

```bash
/joint_states
/palletizer/axis_position_mm
/palletizer/fault_state
/palletizer/motor_rpm
/palletizer/status
```

Subscriber esperado:

```bash
/palletizer/emergency_stop
```

Action esperada:

```bash
/palletizer/move_xyz [palletizer_msgs/action/MoveXYZ]
```

Ver action:

```bash
ros2 action list -t
ros2 action info /palletizer/move_xyz
```

Los servicios y topicos internos de una action son ocultos. Para verlos:

```bash
ros2 service list --include-hidden-services -t | grep move_xyz
ros2 topic list --include-hidden-topics -t | grep move_xyz
```

## 5. Leer topicos

Leer posicion cartesiana directa en milimetros:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

Este es el topico mas directo para comparar contra los setpoints enviados por
la action. El orden esperado es:

```text
[x_mm, y_mm, z_mm]
```

Leer `joint_states`:

```bash
ros2 topic echo /joint_states
```

`/joint_states` usa convencion ROS. Si las articulaciones son prismaticas,
`position` se interpreta en metros. Por ejemplo, 50 mm deberia verse cerca de:

```text
0.05
```

Leer estado general:

```bash
ros2 topic echo /palletizer/status
```

Leer fallas:

```bash
ros2 topic echo /palletizer/fault_state
```

Leer RPM:

```bash
ros2 topic echo /palletizer/motor_rpm
```

## 6. Enviar parada de emergencia

Enviar parada:

```bash
ros2 topic pub --once /palletizer/emergency_stop std_msgs/msg/Bool "{data: true}"
```

## 7. Enviar setpoint por action

El action disponible es:

```bash
/palletizer/move_xyz
```

Tipo:

```bash
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

La action debe entregar feedback con:

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

Al llegar al destino, debe responder con resultado:

```text
success: true
message: target reached
final_x_mm
final_y_mm
final_z_mm
```

## 8. Problemas comunes

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
export ROS_DOMAIN_ID=10
ros2 node list
ros2 action list -t
```

Tambien revisar que el agente este corriendo y que use el mismo dominio:

```bash
ROS_DOMAIN_ID=10 ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

### El ESP32 no aparece como `/dev/ttyACM0`

Revisar puertos:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

Usar el puerto que aparezca en el comando del agente.

### Diferencia entre setpoint y `/joint_states`

Los setpoints de la action estan en milimetros. En ROS, `joint_states.position`
para ejes prismaticos esta en metros. Para comparar en milimetros usa:

```bash
ros2 topic echo /palletizer/axis_position_mm
```

## 9. Compilar y cargar firmware

Compilar:

```bash
cd /home/felipe/proyecto/Esp32_paletizador
/home/felipe/.platformio/penv/bin/pio run
```

Cargar al ESP32-S3:

```bash
/home/felipe/.platformio/penv/bin/pio run -t upload
```

Si el puerto esta ocupado por el agente, deten el agente con `Ctrl+C` y vuelve
a intentar la carga.

