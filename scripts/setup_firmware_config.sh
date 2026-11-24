#!/usr/bin/env bash
# Create the local Wi-Fi configs by copying wifi_config.example.h to the
# git-ignored wifi_config.h in each firmware tree. All other firmware config
# headers are committed and should be edited in place. Existing non-empty files
# are left alone.
#
# Usage:
#   ./scripts/setup_firmware_config.sh
#   ./scripts/setup_firmware_config.sh --force   (overwrite existing files)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CONFIG_DIRS=(
    "$REPO_ROOT/firmware/pico_microros/config"
    "$REPO_ROOT/firmware/esp_cam"
)
FORCE=0

if [ "${1:-}" = "--force" ]; then
    FORCE=1
fi

created=0
skipped=0
for CONFIG_DIR in "${CONFIG_DIRS[@]}"; do
    if [ ! -d "$CONFIG_DIR" ]; then
        echo "Config directory not found: $CONFIG_DIR" >&2
        exit 1
    fi
    for example in "$CONFIG_DIR"/*.example.h; do
        [ -e "$example" ] || continue
        name="$(basename "$example" .example.h).h"
        target="$CONFIG_DIR/$name"
        size=0
        if [ -f "$target" ]; then
            size=$(wc -c < "$target")
        fi
        if [ "$size" -gt 0 ] && [ "$FORCE" -eq 0 ]; then
            echo "skip: $target (already present, non-empty)"
            skipped=$((skipped + 1))
            continue
        fi
        cp "$example" "$target"
        echo "created: $target (from $example)"
        created=$((created + 1))
    done
done

echo ""
echo "Created $created, skipped $skipped."
echo "Edit each wifi_config.h (SSID/password) before building its firmware."
echo "The other config headers are committed; edit them in place."
