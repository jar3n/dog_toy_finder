#!/bin/bash
# Start D-Bus and Avahi so mDNS (.local) names resolve, then exec the command.
set -e
if command -v dbus-daemon >/dev/null 2>&1; then
    mkdir -p /run/dbus
    # A stopped container can leave a stale pid/socket behind; dbus-daemon then
    # refuses to start and Avahi cannot resolve the ESP32-CAM .local names.
    rm -f /run/dbus/pid /run/dbus/system_bus_socket
    dbus-daemon --system 2>/dev/null || true
fi
if command -v avahi-daemon >/dev/null 2>&1; then
    mkdir -p /var/run/avahi-daemon
    avahi-daemon --daemonize 2>/dev/null || true
fi
exec "$@"
