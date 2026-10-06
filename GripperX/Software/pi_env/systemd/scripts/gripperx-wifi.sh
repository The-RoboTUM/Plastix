#!/bin/bash
# Fix 8 (#12): previously, "wpa_cli reconnect" was triggered immediately for
# EVERY state != COMPLETED - this also aborts a connection attempt that is
# still in progress but not yet complete
# (ASSOCIATING/4WAY_HANDSHAKE/GROUP_HANDSHAKE/SCANNING). The result was
# flapping: timer every 15s -> reconnect -> new, again interrupted attempt ->
# next tick reconnect -> ... Handshake+DHCP can take several seconds depending
# on the AP/hotspot, possibly longer than the old 15s interval.
# Reconnect now only happens on an actual disconnect, not during an
# in-progress connection setup.
STATE=$(wpa_cli -i wlan0 status 2>/dev/null | awk -F= '/^wpa_state/{print $2}')

case "$STATE" in
    COMPLETED)
        exit 0
        ;;
    ASSOCIATING|ASSOCIATED|4WAY_HANDSHAKE|GROUP_HANDSHAKE|SCANNING)
        logger -t gripperx-wifi "state=$STATE – connection setup in progress, no reconnect"
        ;;
    *)
        logger -t gripperx-wifi "state=$STATE – reconnecting"
        wpa_cli -i wlan0 reconnect 2>/dev/null || true
        ;;
esac
