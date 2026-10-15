#!/bin/sh
set -eu

DEPLOY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$DEPLOY_DIR/.." && pwd)
PYTHON_SDK_DIR=$REPO_ROOT/python_sdk
TEMPLATE=$DEPLOY_DIR/rescue-drop-listener.service.in
ENTRY=$PYTHON_SDK_DIR/rescue_drop_2026.py
TARGET=/etc/systemd/system/rescue-drop-listener.service

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root: sudo sh $0 /absolute/path/to/python" >&2
    exit 1
fi
PYTHON_EXECUTABLE=${1:-}
if [ -z "$PYTHON_EXECUTABLE" ] || [ "${PYTHON_EXECUTABLE#/}" = "$PYTHON_EXECUTABLE" ]; then
    echo "pass the absolute path to the aircraft's existing Python environment" >&2
    exit 1
fi
case "$REPO_ROOT$PYTHON_EXECUTABLE" in
    *[!a-zA-Z0-9_./-]*)
        echo "repository and Python paths may contain only letters, digits, _, /, . or -" >&2
        exit 1
        ;;
esac
if [ ! -x "$PYTHON_EXECUTABLE" ]; then
    echo "Python interpreter is not executable: $PYTHON_EXECUTABLE" >&2
    exit 1
fi
if [ ! -f "$TEMPLATE" ] || [ ! -f "$ENTRY" ]; then
    echo "listener template or mission entry is missing" >&2
    exit 1
fi
if ! getent passwd fc >/dev/null; then
    echo "service user fc does not exist" >&2
    exit 1
fi
if ! grep -Fq -- '--car-control --confirm-flight' "$TEMPLATE"; then
    echo "listener unit must retain the confirmed car-control entry arguments" >&2
    exit 1
fi
if ! grep -Fq 'Conflicts=d-task-drone-dispatcher.service' "$TEMPLATE"; then
    echo "listener unit must conflict with the retired dispatcher" >&2
    exit 1
fi

# Validate the exact environment without opening a camera, ROS node or FC_Client.
if ! PYTHONPATH="$PYTHON_SDK_DIR${PYTHONPATH:+:$PYTHONPATH}" \
    "$PYTHON_EXECUTABLE" -c 'import rclpy, sensor_msgs; from FlightController import FC_Client, FC_Like; from FlightController.Solutions.Navigation import Navigation'; then
    echo "selected Python cannot import the required ROS2/FlightController modules" >&2
    exit 1
fi

RENDER_DIR=$(mktemp -d)
BACKUP_DIR=
trap 'rm -rf -- "$RENDER_DIR"' EXIT HUP INT TERM
RENDERED=$RENDER_DIR/rescue-drop-listener.service
sed -e "s|@PYTHON_SDK_DIR@|$PYTHON_SDK_DIR|g" \
    -e "s|@PYTHON_EXECUTABLE@|$PYTHON_EXECUTABLE|g" \
    "$TEMPLATE" > "$RENDERED"
systemd-analyze verify "$RENDERED"

if [ -e "$TARGET" ]; then
    BACKUP_DIR=$(mktemp -d /var/backups/rescue-drop-listener-XXXXXXXX)
    cp -a -- "$TARGET" "$BACKUP_DIR/"
    systemctl is-enabled rescue-drop-listener.service \
        > "$BACKUP_DIR/enabled-before" 2>/dev/null || true
fi
if ! cmp -s "$RENDERED" "$TARGET" 2>/dev/null; then
    install -m 0644 "$RENDERED" "$TARGET"
    echo "installed $TARGET"
else
    echo "already current: $TARGET"
fi
systemctl daemon-reload
systemd-analyze verify "$TARGET"

echo "installed unit:"
systemctl cat rescue-drop-listener.service
UNIT_USER=$(systemctl show rescue-drop-listener.service --property=User --value)
UNIT_WORKDIR=$(systemctl show rescue-drop-listener.service --property=WorkingDirectory --value)
UNIT_EXEC=$(systemctl show rescue-drop-listener.service --property=ExecStart --value)
echo "service user: $UNIT_USER"
echo "working directory: $UNIT_WORKDIR"
echo "exec start: $UNIT_EXEC"
if [ "$UNIT_USER" != fc ] || [ "$UNIT_WORKDIR" != "$PYTHON_SDK_DIR" ] || \
    ! printf '%s' "$UNIT_EXEC" | grep -Fq "$PYTHON_EXECUTABLE -u $ENTRY --car-control --confirm-flight"; then
    echo "installed listener unit does not match the selected user, paths or entry arguments" >&2
    exit 1
fi
echo "old dispatcher active: $(systemctl is-active d-task-drone-dispatcher.service 2>/dev/null || true)"
echo "new listener active: $(systemctl is-active rescue-drop-listener.service 2>/dev/null || true)"
echo "FC/task process owners (inspection only):"
pgrep -af '[s]erver_ros.py|[r]escue_drop_2026.py|[d]_task_dispatcher.py|[m]ission[0-9]+_26.py' || true
if [ -n "$BACKUP_DIR" ]; then
    echo "previous unit backup: $BACKUP_DIR"
fi
echo "installed only; no service was enabled, started, stopped or restarted"
echo "after confirming the aircraft is disarmed and no task is active: disable the old dispatcher and enable rescue-drop-listener.service"
