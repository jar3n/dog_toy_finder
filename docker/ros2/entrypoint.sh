#!/bin/bash
# Start D-Bus and Avahi as root so mDNS (.local) names resolve, then drop to
# the host user for the requested command. Files written to the mounted
# /workspace keep the host user's ownership instead of becoming root-owned.
set -e

if [ "$(id -u)" -eq 0 ]; then
    if command -v dbus-daemon >/dev/null 2>&1; then
        mkdir -p /run/dbus
        # A stopped container can leave a stale pid/socket behind; dbus-daemon
        # then refuses to start and Avahi cannot resolve the ESP32-CAM .local
        # names.
        rm -f /run/dbus/pid /run/dbus/system_bus_socket
        dbus-daemon --system 2>/dev/null || true
    fi
    if command -v avahi-daemon >/dev/null 2>&1; then
        mkdir -p /var/run/avahi-daemon
        avahi-daemon --daemonize 2>/dev/null || true
    fi

    if command -v gosu >/dev/null 2>&1; then
        host_uid="${HOST_UID:-1000}"
        host_gid="${HOST_GID:-1000}"
        # Numeric gosu works even when HOST_UID/HOST_GID have no passwd entry
        # in the image. /tmp avoids a HOME owned by a different UID.
        exec gosu "${host_uid}:${host_gid}" env HOME=/tmp "$@"
    fi
fi

exec "$@"
