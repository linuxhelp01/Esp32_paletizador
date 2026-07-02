# Palletizer Dashboard

Interfaz React para visualizar y controlar el paletizador mediante el nodo
`/palletizer_ui_backend`.

## Ejecutar

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

Terminal 3: React.

```bash
cd /home/felipe/proyecto/Esp32_paletizador/ui/frontend
npm install
npm run dev
```

Abrir:

```text
http://localhost:5173
```


## Gemelo digital 3D

El panel 3D usa `three`, `@react-three/fiber` y `@react-three/drei`.

El target del extremo se controla con el raton sobre el plano de trabajo. Z,
velocidad, aceleracion, tolerancia y timeout se ingresan con teclado. Al
confirmar, el frontend envia el mismo comando WebSocket `move_xyz` al backend.

Los archivos para el modelo real deben ir en:

```text
public/robot_description/
```

Guia detallada:

```text
../GUIA_MODELO_3D_REACT.md
```

Si estas en WSL, usa Node/NPM instalado dentro de Ubuntu. No uses el `npm` de
Windows sobre rutas `\\wsl.localhost\...`.
