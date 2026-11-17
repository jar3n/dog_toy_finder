#!/bin/bash
# Start D-Bus and Avahi so mDNS (.local) names resolve, then exec the command.
set -e
if command -v dbus-daemon >/dev/null 2>&1; then
    mkdir -p /run/dbus
    if [ ! -S /run/dbus/system_bus_socket ]; then
        dbus-daemon --system 2>/dev/null || true
    fi
fi
if command -v avahi-daemon >/dev/null 2>&1; then
    mkdir -p /var/run/avahi-daemon
    avahi-daemon --daemonize 2>/dev/null || true
fi
exec "$@"
