#!/bin/sh
set -eu

DEPLOY_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
TEMPLATE=$DEPLOY_DIR/mid360s-fastlio-restart.sudoers
TARGET=/etc/sudoers.d/mid360s-fastlio-restart

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root: sudo sh $0" >&2
    exit 1
fi
if ! getent passwd fc >/dev/null; then
    echo "service user fc does not exist" >&2
    exit 1
fi
SYSTEMCTL_PATH=$(command -v systemctl)
RENDERED=$(mktemp)
trap 'rm -f -- "$RENDERED"' EXIT HUP INT TERM
sed "s|/usr/bin/systemctl|$SYSTEMCTL_PATH|g" "$TEMPLATE" > "$RENDERED"
chmod 0440 "$RENDERED"
visudo -cf "$RENDERED" >/dev/null

RULE="NOPASSWD: $SYSTEMCTL_PATH restart mid360s-fastlio.service"
if sudo -n -l -U fc 2>/dev/null | grep -Fq "$RULE"; then
    echo "fc already has noninteractive permission for exactly this FAST-LIO restart"
    exit 0
fi
if [ -e "$TARGET" ]; then
    if cmp -s "$RENDERED" "$TARGET"; then
        echo "$TARGET is already installed; check sudo -n -l -U fc"
        exit 0
    fi
    echo "refusing to overwrite existing $TARGET; inspect it and sudo -n -l -U fc" >&2
    exit 1
fi

install -m 0440 "$RENDERED" "$TARGET"
visudo -cf "$TARGET" >/dev/null
echo "installed $TARGET for $SYSTEMCTL_PATH (mode 0440)"
echo "verify with: sudo -n -l -U fc"
