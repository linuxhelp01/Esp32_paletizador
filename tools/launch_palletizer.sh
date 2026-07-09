#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIO_ENV="4d_systems_esp32s3_gen4_r8n16"
PIO_BIN="${PLATFORMIO_BIN:-$HOME/.platformio/penv/bin/platformio}"
PORT="${PALLETIZER_PORT:-}"
MODE="run"
START_UI=1

usage() {
  cat <<'EOF'
Uso:
  ./tools/launch_palletizer.sh [opciones]

Opciones:
  --upload          Compila, carga el firmware y abre la interfaz.
  --clean-upload    Limpia, compila, carga y abre la interfaz.
  --build           Solo compila el firmware.
  --no-ui           No abre la interfaz despues de cargar.
  --port DISPOSITIVO
                    Usa un puerto concreto, por ejemplo /dev/ttyACM0.
  -h, --help        Muestra esta ayuda.

Sin opciones abre la interfaz y conecta automaticamente el ESP32 detectado.
EOF
}

detect_port() {
  local candidate
  for candidate in /dev/ttyACM* /dev/ttyUSB*; do
    if [[ -c "$candidate" ]]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done
  return 1
}

wait_for_port() {
  local attempt
  for attempt in {1..20}; do
    if [[ -c "$PORT" ]]; then
      return 0
    fi
    sleep 0.5
  done
  return 1
}

while (($#)); do
  case "$1" in
    --upload)
      MODE="upload"
      ;;
    --clean-upload)
      MODE="clean-upload"
      ;;
    --build)
      MODE="build"
      START_UI=0
      ;;
    --no-ui)
      START_UI=0
      ;;
    --port)
      shift
      if (($# == 0)); then
        echo "ERROR: --port requiere un dispositivo." >&2
        exit 2
      fi
      PORT="$1"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: opcion desconocida: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

cd "$PROJECT_ROOT"

if [[ "$MODE" != "build" ]]; then
  if [[ -z "$PORT" ]]; then
    PORT="$(detect_port || true)"
  fi
  if [[ -z "$PORT" || ! -c "$PORT" ]]; then
    echo "ERROR: no se encontro el ESP32. Conectalo o usa --port /dev/ttyACM0." >&2
    exit 1
  fi
  if [[ ! -r "$PORT" || ! -w "$PORT" ]]; then
    echo "ERROR: sin permisos sobre $PORT. Verifica que el usuario pertenezca a dialout." >&2
    exit 1
  fi
  if fuser "$PORT" >/dev/null 2>&1; then
    echo "ERROR: $PORT esta ocupado. Cierra el monitor serial, agente o interfaz anterior." >&2
    fuser -v "$PORT" >&2 || true
    exit 1
  fi
fi

if [[ "$MODE" != "run" ]]; then
  if [[ ! -x "$PIO_BIN" ]]; then
    echo "ERROR: no se encontro PlatformIO en $PIO_BIN." >&2
    exit 1
  fi
  if [[ "$MODE" == "clean-upload" ]]; then
    "$PIO_BIN" run -e "$PIO_ENV" -t clean
  fi
  "$PIO_BIN" run -e "$PIO_ENV"
  if [[ "$MODE" == "upload" || "$MODE" == "clean-upload" ]]; then
    "$PIO_BIN" run -e "$PIO_ENV" -t upload --upload-port "$PORT"
    if ! wait_for_port; then
      echo "ERROR: $PORT no reaparecio despues de cargar el firmware." >&2
      exit 1
    fi
  fi
fi

if ((START_UI)); then
  python3 -c 'import serial, tkinter' 2>/dev/null || {
    echo "ERROR: faltan tkinter o pyserial para ejecutar la interfaz." >&2
    exit 1
  }
  export PALLETIZER_PORT="$PORT"
  export PALLETIZER_AUTOCONNECT=1
  exec python3 tools/paletizador_ui.py
fi

