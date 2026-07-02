# robot_description para la interfaz React

Esta carpeta es servida directamente por Vite. Todo lo que este aqui queda
accesible desde el navegador con rutas que parten en `/robot_description/...`.

Estructura recomendada:

```text
public/robot_description/
├── urdf/
│   ├── palletizer.urdf              # URDF final usado por la UI cuando se agregue loader
│   └── palletizer.template.urdf     # plantilla de referencia
└── meshes/
    ├── stl/                         # meshes visuales para URDF
    │   ├── base.stl
    │   ├── x_axis.stl
    │   ├── y_axis.stl
    │   ├── z_axis.stl
    │   └── tool.stl
    ├── collision/                   # meshes simplificados opcionales
    └── web/                         # GLB/GLTF opcional para visual web futura
```

Para React/Vite usa rutas absolutas web dentro del URDF:

```xml
<mesh filename="/robot_description/meshes/stl/base.stl"/>
```

Evita rutas `package://...` en el URDF que se carga en navegador, salvo que se
implemente un resolver en el loader.

Unidades:

- URDF usa metros.
- El firmware y la UI operativa usan milimetros.
- La escena plantilla actual usa milimetros para coincidir con `/palletizer/axis_position_mm`.
- Si se carga URDF real, el loader debe aplicar `axis_position_mm / 1000` a los joints prismaticos.

Nombres recomendados de joints:

```text
x_axis_joint
 y_axis_joint
 z_axis_joint
```

Si usas otros nombres, actualiza el mapeo en el componente 3D cuando se agregue
el loader URDF.
