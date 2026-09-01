#!/usr/bin/env bash

echo "Stopping Octopus debug stack..."
pkill -f "flight_camera_transform_node" || true
pkill -f "world_posearray_to_json_bridge_node" || true
pkill -f "grid_map_builder_node" || true
pkill -f "map_patch_backend_bridge_node" || true
pkill -f "camera_debug_backend_bridge_node" || true
pkill -f "camera_transform_status_backend_bridge_node" || true
pkill -f "trash_gps_goal_node" || true
pkill -f "eve_fake_gps_bridge_node" || true
pkill -f "device_status_backend_bridge_node" || true
pkill -f "rosbridge_websocket" || true
pkill -f "uvicorn api:app" || true
pkill -f "[d]etector_node.py" || true
echo "Stopped."

pkill -f "local_camera_grid_node" || true

pkill -f "local_camera_grid_backend_bridge_node" || true

# --- Eve-Seite auf der Pi ---
# Standardmaessig AUS: die Pi laeuft oft weiter, waehrend auf dem Laptop nur der
# Stack neu gestartet wird, und ein ungefragtes Abschalten der Kamera mitten in
# einer Demo waere die unangenehmere Ueberraschung. Mit OCTOPUS_STOP_EVE=true
# wird sie mit heruntergefahren.
if [ "${OCTOPUS_STOP_EVE:-false}" = "true" ]; then
  EVE_SSH_TARGET="${OCTOPUS_EVE_SSH_TARGET:-eve-pi}"
  EVE_SCRIPT_DIR="${OCTOPUS_EVE_SCRIPT_DIR:-~/PlastiX/eve/Software/scripts}"

  echo "Stopping Eve on $EVE_SSH_TARGET..."
  ssh -o BatchMode=yes -o ConnectTimeout=4 "$EVE_SSH_TARGET" \
    "$EVE_SCRIPT_DIR/octopus_stop_camera.sh; $EVE_SCRIPT_DIR/octopus_stop_px4_bridge.sh" 2>&1 \
    | tail -2 || echo "WARNING: Eve not reachable - nothing stopped on the Pi."
fi
