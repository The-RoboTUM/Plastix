#!/usr/bin/env bash
# Startet den MicroXRCEAgent (Bruecke Pixhawk <-> ROS 2) im Hintergrund, ohne
# offenes Terminal. Antwort geht an POST /api/eve/px4_bridge/start, das auf
# px4_bridge_started schaut.
#
# Kein sudo, und das ist kein Versehen: /dev/ttyAMA0 gehoert root:dialout, der
# Benutzer eve ist in dialout, der Agent braucht sonst nichts. Mit sudo waere
# der Start aus dem Dashboard unmoeglich -- der Aufruf ist nicht interaktiv und
# bliebe an der Passwortabfrage haengen.
set -e

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source "$SCRIPT_DIR/octopus_px4_bridge_env.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

if [ ! -e "$PX4_SERIAL_DEV" ]; then
    echo "px4_bridge_failed: $PX4_SERIAL_DEV existiert nicht"
    echo "UART aktiviert? enable_uart=1 und dtoverlay=disable-bt in /boot/firmware/config.txt"
    exit 1
fi

if [ ! -w "$PX4_SERIAL_DEV" ]; then
    echo "px4_bridge_failed: keine Schreibrechte auf $PX4_SERIAL_DEV"
    ls -l "$(readlink -f "$PX4_SERIAL_DEV")"
    echo "Benutzer $(whoami) ist in: $(id -nG)"
    echo "Fehlt dialout? sudo usermod -aG dialout $(whoami), danach neu anmelden."
    exit 1
fi

# Ein alter Agent haelt die serielle Schnittstelle belegt, der neue bekaeme sie
# nicht. Deshalb immer erst raeumen.
echo "Stopping old MicroXRCEAgent if needed..."
"$SCRIPT_DIR/octopus_stop_px4_bridge.sh" || true

echo "Using serial: $PX4_SERIAL_DEV at $PX4_SERIAL_BAUD baud, verbosity $PX4_AGENT_VERBOSITY"

nohup MicroXRCEAgent serial \
  --dev "$PX4_SERIAL_DEV" \
  -b "$PX4_SERIAL_BAUD" \
  -v "$PX4_AGENT_VERBOSITY" \
  > "$LOG_FILE" 2>&1 < /dev/null &

# Der Agent oeffnet die Schnittstelle sofort, aber eine Session entsteht erst,
# wenn der Pixhawk sich meldet. Zwei Sekunden reichen fuer "laeuft ueberhaupt",
# nicht unbedingt fuer "Pixhawk ist dran" -- deshalb wird unten beides getrennt
# gemeldet.
sleep 2

PIDS=$(pgrep -f "$MATCH" || true)

if [ -z "$PIDS" ]; then
    echo "px4_bridge_failed"
    echo "--- $LOG_FILE ---"
    tail -40 "$LOG_FILE" || true
    exit 1
fi

echo "$PIDS" > "$PID_FILE"

if grep -q "session established" "$LOG_FILE" 2>/dev/null; then
    LINK="pixhawk=connected"
else
    LINK="pixhawk=waiting"
fi

echo "px4_bridge_started pids=$(echo "$PIDS" | tr '\n' ' ') $LINK"
