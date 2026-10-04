#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo bash $REPO_DIR/deploy/install_localization_ipc_policy.sh" >&2
    exit 1
fi
source_file="$REPO_DIR/deploy/99-robocup-ipc.conf"
target_dir=/etc/systemd/logind.conf.d
target="$target_dir/99-robocup-ipc.conf"
if [[ ! -r "$source_file" || -L "$target_dir" || -L "$target" ]]; then
    echo "Missing policy or unexpected symlink; no configuration changed" >&2
    exit 1
fi
if [[ -f "$target" ]] && cmp -s "$source_file" "$target"; then
    echo "Policy already installed. No configuration changed."
else
    backup_dir="$(mktemp -d /var/backups/robocup-ipc-XXXXXXXX)"
    if [[ -e "$target" ]]; then
        cp -a -- "$target" "$backup_dir/99-robocup-ipc.conf"
    else
        touch "$backup_dir/previously-absent"
    fi
    install -d -m 0755 "$target_dir"
    install -m 0644 "$source_file" "$target"
    echo "Backup: $backup_dir"
fi
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B -c '
import sys
sys.path.insert(0, sys.argv[1])
from localization_startup import ipc_cleanup_disabled
if not ipc_cleanup_disabled():
    raise SystemExit("Another logind setting overrides RemoveIPC=no; reload refused")
' "$REPO_DIR/python_sdk"
# Ubuntu systemd 249 reloads logind configuration on SIGHUP. Do not restart
# logind, the FC bridge or either localization service from this installer.
systemctl kill --kill-who=main --signal=HUP systemd-logind.service
echo "Reload requested. Verify Config file reloaded in the logind journal."
echo "RemoveIPC=no applies to all ordinary users on this dedicated flight computer."
echo "No flight or localization program was started or restarted."
