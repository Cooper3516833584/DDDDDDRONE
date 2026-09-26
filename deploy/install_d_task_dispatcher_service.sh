#!/bin/sh
set -eu

REPO_ROOT=${1:-"$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"}
PYTHON_SDK_DIR=$(CDPATH= cd -- "$REPO_ROOT/python_sdk" && pwd)
DEPLOY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TEMPLATE=$DEPLOY_DIR/d-task-drone-dispatcher.service.in
TARGET=/etc/systemd/system/d-task-drone-dispatcher.service
UDEV_TEMPLATE=$DEPLOY_DIR/99-lx-flight-controller.rules
UDEV_TARGET=/etc/udev/rules.d/99-lx-flight-controller.rules
ENTRY=$PYTHON_SDK_DIR/d_task_dispatcher.py

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

# 入口必须存在，否则会装出一个 ExecStart 指向不存在文件的单元
if [ ! -f "$ENTRY" ]; then
    echo "mission entry not found: $ENTRY" >&2
    echo "upstream moved the dispatcher into python_sdk/former_code/; update this" >&2
    echo "installer and the unit template at the real entry before enabling it" >&2
    exit 1
fi

if pgrep -f '[s]erver_ros.py|[m]ission[12]_26.py|[f]c-server-watchdog.sh' >/dev/null 2>&1; then
    echo "existing server/task/watchdog process detected; stop safely and retry" >&2
    exit 1
fi

sed "s#@PYTHON_SDK_DIR@#$PYTHON_SDK_DIR#g" "$TEMPLATE" > "$TARGET"
install -m 0644 "$UDEV_TEMPLATE" "$UDEV_TARGET"
udevadm control --reload-rules
systemctl daemon-reload
systemctl enable d-task-drone-dispatcher.service
echo "installed and enabled $TARGET; start it only after the aircraft is confirmed safe"
