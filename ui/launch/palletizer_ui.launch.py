from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _launch_setup(context, *args, **kwargs):
    launch_file = Path(__file__).resolve()
    ui_dir = launch_file.parents[1]
    project_root = ui_dir.parent
    frontend_dir = ui_dir / "frontend"
    backend_dir = ui_dir / "backend"

    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)
    serial_dev = LaunchConfiguration("serial_dev").perform(context)
    agent_verbose = LaunchConfiguration("agent_verbose").perform(context)
    backend_host = LaunchConfiguration("backend_host").perform(context)
    backend_port = LaunchConfiguration("backend_port").perform(context)
    backend_connection_stale_ms = LaunchConfiguration("backend_connection_stale_ms").perform(context)
    backend_availability_hold_ms = LaunchConfiguration("backend_availability_hold_ms").perform(context)
    enable_home_origin_actions = LaunchConfiguration("enable_home_origin_actions").perform(context)
    enable_axis_services = LaunchConfiguration("enable_axis_services").perform(context)
    enable_extended_services = LaunchConfiguration("enable_extended_services").perform(context)
    frontend_host = LaunchConfiguration("frontend_host").perform(context)
    frontend_port = LaunchConfiguration("frontend_port").perform(context)
    microros_setup = LaunchConfiguration("microros_setup").perform(context)
    palletizer_msgs_setup = LaunchConfiguration("palletizer_msgs_setup").perform(context)
    ros_setup = LaunchConfiguration("ros_setup").perform(context)
    agent_executable = LaunchConfiguration("agent_executable").perform(context)

    agent_cmd = (
        f"source {ros_setup} && "
        f"source {microros_setup} && "
        f"source {palletizer_msgs_setup} && "
        f"export ROS_DOMAIN_ID={ros_domain_id} && "
        f"{agent_executable} serial --dev {serial_dev} -v{agent_verbose}"
    )

    backend_cmd = (
        f"source {ros_setup} && "
        f"source {palletizer_msgs_setup} && "
        f"export ROS_DOMAIN_ID={ros_domain_id} && "
        f"export PALLETIZER_UI_HOST={backend_host} && "
        f"export PALLETIZER_UI_PORT={backend_port} && "
        f"export PALLETIZER_CONNECTION_STALE_MS={backend_connection_stale_ms} && "
        f"export PALLETIZER_AVAILABILITY_HOLD_MS={backend_availability_hold_ms} && "
        f"export PALLETIZER_ENABLE_HOME_ORIGIN_ACTIONS={enable_home_origin_actions} && "
        f"export PALLETIZER_ENABLE_AXIS_SERVICES={enable_axis_services} && "
        f"export PALLETIZER_ENABLE_EXTENDED_SERVICES={enable_extended_services} && "
        f"cd {backend_dir} && "
        f"python3 -m palletizer_ui_backend.server"
    )

    frontend_cmd = (
        f"cd {frontend_dir} && "
        f"VITE_PALLETIZER_WS_URL=ws://{backend_host}:{backend_port} "
        f"npm run dev -- --host {frontend_host} --port {frontend_port}"
    )

    actions = [
        LogInfo(msg=f"Proyecto paletizador: {project_root}"),
        LogInfo(msg=f"Frontend: http://localhost:{frontend_port}"),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("start_agent")),
            cmd=["bash", "-lc", agent_cmd],
            name="micro_ros_agent",
            output="screen",
            emulate_tty=True,
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("start_backend")),
            cmd=["bash", "-lc", backend_cmd],
            name="palletizer_ui_backend",
            output="screen",
            emulate_tty=True,
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration("start_frontend")),
            cmd=["bash", "-lc", frontend_cmd],
            name="palletizer_react_frontend",
            output="screen",
            emulate_tty=True,
        ),
    ]

    if _as_bool(LaunchConfiguration("show_commands").perform(context)):
        actions.extend([
            LogInfo(msg=f"agent_cmd: {agent_cmd}"),
            LogInfo(msg=f"backend_cmd: {backend_cmd}"),
            LogInfo(msg=f"frontend_cmd: {frontend_cmd}"),
        ])

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ros_domain_id", default_value="10", description="ROS_DOMAIN_ID usado por el ESP32-S3 y la UI."),
        DeclareLaunchArgument("serial_dev", default_value="/dev/ttyACM0", description="Puerto serial del ESP32-S3."),
        DeclareLaunchArgument("agent_verbose", default_value="1", description="Nivel de verbosidad del micro_ros_agent."),
        DeclareLaunchArgument("backend_host", default_value="127.0.0.1", description="Host WebSocket del backend."),
        DeclareLaunchArgument("backend_port", default_value="8765", description="Puerto WebSocket del backend."),
        DeclareLaunchArgument("backend_connection_stale_ms", default_value="1500", description="Tiempo sin telemetria para marcar ESP32/ROS como desconectado."),
        DeclareLaunchArgument("backend_availability_hold_ms", default_value="1200", description="Retencion de disponibilidad de acciones/servicios/topicos para evitar parpadeos del grafo ROS."),
        DeclareLaunchArgument("enable_home_origin_actions", default_value="false", description="Crear clientes UI para /palletizer/home_axis y /palletizer/go_origin solo si el firmware los expone."),
        DeclareLaunchArgument("enable_axis_services", default_value="false", description="Crear clientes UI para /palletizer/enable_axis y /palletizer/set_axis_limits solo si el firmware los expone."),
        DeclareLaunchArgument("enable_extended_services", default_value="false", description="Crear clientes UI para servicios extendidos de diagnostico/homing solo si el firmware los expone."),
        DeclareLaunchArgument("frontend_host", default_value="0.0.0.0", description="Host del servidor Vite."),
        DeclareLaunchArgument("frontend_port", default_value="5173", description="Puerto del servidor Vite."),
        DeclareLaunchArgument("ros_setup", default_value="/opt/ros/humble/setup.bash", description="Setup base de ROS 2 Humble."),
        DeclareLaunchArgument("microros_setup", default_value="/home/felipe/microros_ws/install/setup.bash", description="Setup del workspace micro-ROS."),
        DeclareLaunchArgument("palletizer_msgs_setup", default_value="/home/felipe/palletizer_msgs_ws/setup_palletizer_msgs.bash", description="Setup del workspace palletizer_msgs."),
        DeclareLaunchArgument("agent_executable", default_value="/home/felipe/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent", description="Ejecutable del micro_ros_agent."),
        DeclareLaunchArgument("start_agent", default_value="true", description="Arrancar micro_ros_agent."),
        DeclareLaunchArgument("start_backend", default_value="true", description="Arrancar nodo /palletizer_ui_backend."),
        DeclareLaunchArgument("start_frontend", default_value="true", description="Arrancar Vite/React."),
        DeclareLaunchArgument("show_commands", default_value="false", description="Mostrar comandos expandidos para depuracion."),
        OpaqueFunction(function=_launch_setup),
    ])
