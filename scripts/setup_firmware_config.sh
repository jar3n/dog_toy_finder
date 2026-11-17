#!/usr/bin/env bash
# Create the local Wi-Fi config by copying wifi_config.example.h to the
# git-ignored wifi_config.h. All other firmware config headers are committed
# and should be edited in place. Existing non-empty files are left alone.
#
# Usage:
#   ./scripts/setup_firmware_config.sh
#   ./scripts/setup_firmware_config.sh --force   (overwrite existing files)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIR="$REPO_ROOT/firmware/pico_microros/config"
FORCE=0

if [ "${1:-}" = "--force" ]; then
    FORCE=1
fi

if [ ! -d "$CONFIG_DIR" ]; then
    echo "Config directory not found: $CONFIG_DIR" >&2
    exit 1
fi

created=0
skipped=0
for example in "$CONFIG_DIR"/*.example.h; do
    [ -e "$example" ] || continue
    name="$(basename "$example" .example.h).h"
    target="$CONFIG_DIR/$name"
    size=0
    if [ -f "$target" ]; then
        size=$(wc -c < "$target")
    fi
    if [ "$size" -gt 0 ] && [ "$FORCE" -eq 0 ]; then
        echo "skip: $name (already present, non-empty)"
        skipped=$((skipped + 1))
        continue
    fi
    cp "$example" "$target"
    echo "created: $name (from $example)"
    created=$((created + 1))
done

echo ""
echo "Created $created, skipped $skipped."
echo "Edit config/wifi_config.h (SSID/password/log target) before building."
echo "The other config headers are committed; edit them in place."
