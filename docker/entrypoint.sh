#!/usr/bin/env bash
set -euo pipefail

# Multi-app entrypoint for msensor_library containers.
# Usage: entrypoint.sh <app> [args...]
#        APP=<app> entrypoint.sh [args...]
# Installed apps: rplidar | mid360 | sim | icm20948

usage() {
  cat <<'EOF'
Usage: entrypoint.sh <app> [args...]
Available apps:
  rplidar   -> /usr/local/bin/rplidar_publisher
  mid360    -> /usr/local/bin/mid360_publisher
  sim       -> /usr/local/bin/sim_publisher
  icm20948  -> /usr/local/bin/icm20948_publisher

You can also set the APP environment variable instead of passing <app>.
Examples:
  entrypoint.sh rplidar
  entrypoint.sh mid360 /usr/local/etc/mid360_config.json 100 1 0
  APP=icm20948 entrypoint.sh 2
EOF
}

if [[ "${1:-}" =~ ^(-h|--help)$ ]]; then
  usage
  exit 0
fi

if [ $# -gt 0 ]; then
  APP_SEL="$1"
  shift
elif [ -n "${APP:-}" ]; then
  APP_SEL="$APP"
else
  usage
  exit 1
fi

case "$APP_SEL" in
  rplidar)
    exec /usr/local/bin/rplidar_publisher "$@"
    ;;
  mid360)
    CONFIG="${1:-/usr/local/etc/mid360_config.json}"
    if [ $# -gt 0 ]; then
      shift
    fi
    exec /usr/local/bin/mid360_publisher "$CONFIG" "$@"
    ;;
  sim)
    exec /usr/local/bin/sim_publisher "$@"
    ;;
  icm20948)
    ARG1="${1:-1}"
    if [ $# -gt 0 ]; then
      shift
    fi
    exec /usr/local/bin/icm20948_publisher "$ARG1" "$@"
    ;;
  *)
    if [ -x "/usr/local/bin/$APP_SEL" ]; then
      exec "/usr/local/bin/$APP_SEL" "$@"
    fi
    echo "Unknown app: $APP_SEL" >&2
    usage >&2
    exit 2
    ;;
 esac
