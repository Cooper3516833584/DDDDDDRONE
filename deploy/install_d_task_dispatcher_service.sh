#!/bin/sh
set -eu

REPO_ROOT=${1:-"$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"}
PYTHON_SDK_DIR=$(CDPATH= cd -- "$REPO_ROOT/python_sdk" && pwd)
TEMPLATE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)/d-task-drone-dispatcher.service.in
TARGET=/etc/systemd/system/d-task-drone-dispatcher.service

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

if pgrep -f '[s]erver_ros.py|[m]ission[12]_26.py|[f]c-server-watchdog.sh' >/dev/null 2>&1; then
    echo "existing server/task/watchdog process detected; stop safely and retry" >&2
    exit 1
fi

sed "s#@PYTHON_SDK_DIR@#$PYTHON_SDK_DIR#g" "$TEMPLATE" > "$TARGET"
systemctl daemon-reload
systemctl enable d-task-drone-dispatcher.service
echo "installed and enabled $TARGET; start it only after the aircraft is confirmed safe"
