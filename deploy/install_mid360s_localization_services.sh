#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo bash $REPO_DIR/deploy/install_mid360s_localization_services.sh" >&2
    exit 1
fi
# The path is embedded in systemd and shell syntax; reject ambiguous paths.
if [[ ! "$REPO_DIR" =~ ^/[a-zA-Z0-9_./-]+$ ]]; then
    echo "Repository path must contain only letters, digits, _, /, . or -" >&2
    exit 1
fi
getent passwd fc >/dev/null
getent group fc >/dev/null
for required in /opt/ros/humble/setup.bash \
    "$REPO_DIR/ros2_ws/install/setup.bash" \
    "$REPO_DIR/ros2_ws/src/livox_ros_driver2/config/MID360s_config.json" \
    "$REPO_DIR/ros2_ws/src/FAST_LIO_ROS2/config/mid360s_drone.yaml"; do
    if [[ ! -r "$required" ]]; then
        echo "Missing required file: $required" >&2
        exit 1
    fi
done
for unit in mid360s-driver mid360s-fastlio; do
    if systemctl is-active --quiet "$unit.service"; then
        echo "Stop localization safely before reinstalling: $unit.service" >&2
        exit 1
    fi
done
if pgrep -f '/lib/(fast_lio/fastlio_mapping|livox_ros_driver2/livox_ros_driver2_node)( |$)' >/dev/null; then
    echo "Unmanaged localization node detected; stop its owner safely before installing" >&2
    exit 1
fi

render_dir="$(mktemp -d)"
trap 'rm -rf -- "$render_dir"' EXIT
for unit in mid360s-driver mid360s-fastlio; do
    sed "s|@REPO_DIR@|$REPO_DIR|g" "$REPO_DIR/deploy/$unit.service.in" > "$render_dir/$unit.service"
done
systemd-analyze verify "$render_dir/mid360s-driver.service" "$render_dir/mid360s-fastlio.service"

backup_dir="$(mktemp -d /var/backups/mid360s-localization-XXXXXXXX)"
for unit in mid360s-driver mid360s-fastlio; do
    target="/etc/systemd/system/$unit.service"
    if [[ -e "$target" ]]; then
        cp -a -- "$target" "$backup_dir/"
    fi
    systemctl is-enabled "$unit.service" > "$backup_dir/$unit.enabled-before" 2>/dev/null || true
    install -m 0644 "$render_dir/$unit.service" "$target"
done
systemctl daemon-reload
systemd-analyze verify /etc/systemd/system/mid360s-driver.service /etc/systemd/system/mid360s-fastlio.service
systemctl enable mid360s-driver.service mid360s-fastlio.service
echo "Backup: $backup_dir"
echo "Installed and enabled. No services were started."
echo "Start manually with:"
echo "sudo systemctl start mid360s-driver.service"
echo "sudo systemctl start mid360s-fastlio.service"
