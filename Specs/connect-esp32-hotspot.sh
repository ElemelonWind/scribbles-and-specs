#!/usr/bin/env bash
set -euo pipefail

SSID="ESP32_Hotspot"
PASS="esp32pass"
IF="wlan0"
AP_IP="192.168.4.1"

echo "Starting ESP32 hotspot connection script..."

if ! command -v nmcli >/dev/null 2>&1; then
  echo "Error: nmcli not installed or not available."
  exit 1
fi

# Ensure WiFi radio is on (don't toggle off — it's disruptive)
nmcli radio wifi on
sleep 1

# Remove any stale connection profile for this SSID to avoid conflicts
if nmcli connection show "$SSID" &>/dev/null; then
  echo "Removing existing connection profile for '$SSID'..."
  nmcli connection delete "$SSID"
fi

echo "Scanning for SSID: $SSID on interface $IF..."
nmcli dev wifi rescan ifname "$IF" 2>/dev/null || true
sleep 3  # Give scan time to populate

# Confirm the SSID is visible before attempting connection
if ! nmcli dev wifi list ifname "$IF" | grep -q "$SSID"; then
  echo "ERROR: SSID '$SSID' not found in scan results. Is the ESP32 hotspot active?"
  nmcli dev wifi list ifname "$IF"
  exit 1
fi

echo "Connecting to '$SSID' on $IF..."
if ! nmcli dev wifi connect "$SSID" password "$PASS" ifname "$IF"; then
  echo "ERROR: Failed to connect to '$SSID'. Check password or ESP32 status."
  nmcli device status
  exit 1
fi

echo "Connection status:"
nmcli dev status

echo "IP address assigned to $IF:"
ip addr show "$IF" | grep "inet "

echo "Pinging AP at $AP_IP..."
if ping -c 4 "$AP_IP"; then
  echo "SUCCESS: Connected and AP is reachable."
  echo "You can now run: ros2 run specs socket_test"
else
  echo "WARNING: Connected but AP ping failed. Check ESP32 DHCP/IP config."
  exit 1
fi