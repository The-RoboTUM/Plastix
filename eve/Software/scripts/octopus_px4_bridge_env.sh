#!/usr/bin/env bash
# Gemeinsame Werte der drei octopus_*px4_bridge*-Skripte. Wird von allen dreien
# gesourct und ist selbst nicht zum Ausfuehren gedacht.

# pgrep-Muster. Die Klammer verhindert, dass das Muster sich selbst trifft,
# wenn es in derselben Kommandozeile steht.
MATCH='[M]icroXRCEAgent'

# Serielle Verbindung zum Pixhawk. /dev/serial0 zeigt auf ttyAMA0; die Baudrate
# muss zu PX4s XRCE_DDS_BAUD passen.
PX4_SERIAL_DEV="${PX4_SERIAL_DEV:-/dev/serial0}"
PX4_SERIAL_BAUD="${PX4_SERIAL_BAUD:-921600}"

# 4 statt der 6 aus der Doku: 6 loggt jedes Topic einzeln und laesst das Log im
# Dauerbetrieb schnell wachsen. 4 zeigt Sessions und Teilnehmer, also genau das,
# woran man sieht, ob der Pixhawk da ist.
PX4_AGENT_VERBOSITY="${PX4_AGENT_VERBOSITY:-4}"

LOG_FILE=/tmp/octopus_px4_bridge.log
PID_FILE=/tmp/octopus_px4_bridge.pid
