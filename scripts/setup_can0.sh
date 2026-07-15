#!/usr/bin/env bash
# Brings up the can0 SocketCAN interface at the ADS-DV bus bitrate.
# Runs as root (via hydrakon-can0.service) before hydrakon-stack.service.
set -eo pipefail

IFACE=can0
BITRATE=500000

# The CAN adapter can take a moment to enumerate after boot; wait for it
# rather than failing immediately.
for i in $(seq 1 30); do
    if ip link show "$IFACE" &>/dev/null; then
        break
    fi
    sleep 1
done

ip link set "$IFACE" down 2>/dev/null || true
ip link set "$IFACE" type can bitrate "$BITRATE"
ip link set "$IFACE" up
