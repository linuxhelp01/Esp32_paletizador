from pathlib import Path
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _as_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _quoted(value) -> str:
    return shlex.quote(str(value))


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
    frontend_host = LaunchConfiguration("frontend_host").perform(context)
    frontend_port = LaunchConfiguration("frontend_port").perform(context)
    install_frontend_dependencies = LaunchConfiguration("install_frontend_dependencies").perform(context)
    microros_setup = LaunchConfiguration("microros_setup").perform(context)
    palletizer_msgs_setup = LaunchConfiguration("palletizer_msgs_setup").perform(context)
    ros_setup = LaunchConfiguration("ros_setup").perform(context)
    agent_executable = LaunchConfiguration("agent_executable").perform(context)

    agent_cmd = (
        f"source {_quoted(ros_setup)} && "
        f"source {_quoted(microros_setup)} && "
        f"source {_quoted(palletizer_msgs_setup)} && "
        f"export ROS_DOMAIN_ID={_quoted(ros_domain_id)} && "
        f"{_quoted(agent_executable)} serial --dev {_quoted(serial_dev)} -v{_quoted(agent_verbose)}"
    )

    backend_cmd = (
        f"source {_quoted(ros_setup)} && "
        f"source {_quoted(palletizer_msgs_setup)} && "
        f"export ROS_DOMAIN_ID={_quoted(ros_domain_id)} && "
        f"export PALLETIZER_UI_HOST={_quoted(backend_host)} && "
        f"export PALLETIZER_UI_PORT={_quoted(backend_port)} && "
        f"cd {_quoted(backend_dir)} && "
        f"python3 -m palletizer_ui_backend.server"
    )

    dependency_cmd = ""
    if _as_bool(install_frontend_dependencies):
        dependency_cmd = (
            'if [ ! -x node_modules/.bin/vite ]; then '
            'echo "[frontend] Dependencias ausentes; ejecutando npm ci..."; '
            "npm ci || exit $?; "
            "fi && "
        )

    frontend_cmd = (
        f"cd {_quoted(frontend_dir)} && "
        f"{dependency_cmd}"
        f"VITE_PALLETIZER_WS_PORT={_quoted(backend_port)} "
        f"npm run dev -- --host {_quoted(frontend_host)} --port {_quoted(frontend_port)}"
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
        DeclareLaunchArgument("agent_verbose", default_value="4", description="Nivel de verbosidad del micro_ros_agent."),
        DeclareLaunchArgument("backend_host", default_value="0.0.0.0", description="Interfaz donde escucha el WebSocket del backend."),
        DeclareLaunchArgument("backend_port", default_value="8765", description="Puerto WebSocket del backend."),
        DeclareLaunchArgument("frontend_host", default_value="0.0.0.0", description="Host del servidor Vite."),
        DeclareLaunchArgument("frontend_port", default_value="5173", description="Puerto del servidor Vite."),
        DeclareLaunchArgument("install_frontend_dependencies", default_value="true", description="Ejecutar npm ci automaticamente si Vite no esta instalado."),
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
