# Guia para integrar el modelo 3D del paletizador en React

Esta guia describe como agregar un modelo 3D real del paletizador a la interfaz
React manteniendo la comunicacion actual con ROS 2 y el ESP32-S3.

## 1. Estado actual

La UI ya incluye un panel llamado **Gemelo digital**. Actualmente usa una
geometria plantilla generada por codigo para representar:

- zona de trabajo
- rieles principales
- posicion actual del extremo
- target interactivo
- marcador de altura Z

El usuario puede:

- orientar la camara 3D con el raton por defecto
- hacer click en la esfera amarilla del setpoint para habilitar edicion 3D
- arrastrar el target X/Y solo cuando el modo de edicion esta activo
- bloquear nuevamente el setpoint antes o despues de enviar el comando
- ingresar Z con teclado
- ingresar velocidad `speed_mm_s` con teclado
- ingresar aceleracion `accel_mm_s2` con teclado
- enviar el objetivo real mediante `/palletizer/move_xyz`

El panel mantiene las funcionalidades existentes del dashboard.

## 2. Carpeta para el modelo

Los archivos 3D deben quedar dentro del frontend, en una carpeta publica servida
por Vite:

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

`public/` es importante: cualquier archivo colocado ahi queda disponible desde
el navegador.

Por ejemplo:

```text
ui/frontend/public/robot_description/meshes/stl/base.stl
```

se accede desde React como:

```text
/robot_description/meshes/stl/base.stl
```

## 3. Formato recomendado

Para partir con URDF:

```text
URDF + STL
```

Recomendado:

- `palletizer.urdf` para la estructura cinematica
- STL separados por parte movil
- nombres simples y estables

Opcional futuro:

- GLB/GLTF en `meshes/web/` para una visualizacion mas bonita en web
- STL simplificados en `meshes/collision/` para colisiones o zonas seguras

## 4. Exportar desde CAD

Exporta piezas separadas, no todo el robot como una sola malla. Para un robot
cartesiano conviene separar:

```text
base.stl       estructura fija
x_axis.stl     carro o conjunto que se mueve en X
y_axis.stl     carro o conjunto que se mueve en Y
z_axis.stl     eje vertical
tool.stl       extremo, gripper o herramienta
```

Buenas practicas:

- manten los ejes del CAD coherentes con el robot real
- usa nombres funcionales, no nombres automaticos del CAD
- evita mallas demasiado pesadas
- si el STL queda enorme, reduce resolucion de exportacion
- verifica si el CAD exporta en mm o metros

URDF trabaja en metros. Si el STL exportado en mm aparece 1000 veces mas grande,
ajusta `scale="0.001 0.001 0.001"` en el URDF.

## 5. Rutas dentro del URDF

Para React/Vite, usa rutas web absolutas:

```xml
<mesh filename="/robot_description/meshes/stl/base.stl"/>
```

Evita rutas ROS tipo:

```xml
<mesh filename="package://palletizer_description/meshes/base.stl"/>
```

`package://` funciona bien en ROS/RViz, pero el navegador no sabe resolverlo sin
codigo adicional.

## 6. Joints recomendados

Usa estos nombres para que el codigo pueda mapear telemetria facilmente:

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

El firmware publica posiciones en milimetros:

```text
/palletizer/axis_position_mm = [x_mm, y_mm, z_mm]
```

URDF usa metros:

```text
x_joint = x_mm / 1000
y_joint = y_mm / 1000
z_joint = z_mm / 1000
```

## 7. Limites

Los limites del URDF deben coincidir con los limites de trabajo del firmware.
Ejemplo para 300 mm:

```xml
<limit lower="0.0" upper="0.3" effort="100" velocity="1.0"/>
```

La UI actual tambien lee limites desde `/palletizer/status` cuando existen. Si
el firmware no reporta limites configurados, usa valores fallback para la
plantilla.

## 8. Como activar un loader URDF luego

El panel actual esta en:

```text
ui/frontend/src/components/DigitalTwin/DigitalTwinPanel.tsx
```

Hoy usa geometria procedural en `RobotTemplate`. Cuando tengas el URDF/STL real,
la evolucion recomendada es:

1. instalar y mantener estas dependencias:

```bash
npm install three @react-three/fiber @react-three/drei urdf-loader
```

2. crear un componente `UrdfRobot.tsx`
3. cargar `/robot_description/urdf/palletizer.urdf`
4. resolver rutas STL desde `/robot_description/meshes/stl/`
5. aplicar joints desde la telemetria:

```ts
robot.joints.x_axis_joint.setJointValue(x_mm / 1000)
robot.joints.y_axis_joint.setJointValue(y_mm / 1000)
robot.joints.z_axis_joint.setJointValue(z_mm / 1000)
```

6. mantener el target interactivo separado del modelo real

El modelo real debe mostrar la posicion medida. El target amarillo debe mostrar
la orden que el usuario quiere enviar.

## 9. Flujo seguro de control

La UI no envia comandos continuamente mientras arrastras el mouse. El flujo es:

```text
arrastrar target -> previsualizar -> revisar velocidad/aceleracion -> enviar
```

Esto evita saturar micro-ROS y evita que un movimiento accidental del raton
mande comandos al robot real.

## 10. Prueba recomendada

1. Ejecuta agente micro-ROS.
2. Ejecuta backend.
3. Ejecuta frontend.
4. Verifica que el panel 3D muestre la posicion actual.
5. Haz click en la esfera amarilla para habilitar edicion y arrastra el target en X/Y.
6. Ingresa Z, velocidad y aceleracion.
7. Presiona `Enviar target 3D`.
8. Verifica feedback/result de `/palletizer/move_xyz`.

Comandos:

```bash
# backend
source /opt/ros/humble/setup.bash
source /home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash
export ROS_DOMAIN_ID=10
cd /home/felipe/proyecto/Esp32_paletizador/ui/backend
python3 -m palletizer_ui_backend.server
```

```bash
# frontend
cd /home/felipe/proyecto/Esp32_paletizador/ui/frontend
npm install
npm run dev
```
