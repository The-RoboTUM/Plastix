// Load the dashboard's live_data.js in Node with just enough browser stubs to
// exercise getFleetSnapshot(), then check how a GripperX status is folded in.
// getFleetSnapshot() gegen einen GripperX-Status prüfen, ohne Browser.
//
// Lädt live_data.js in einen vm-Context mit gerade genug DOM-Attrappe und
// schiebt Gerätestatus hinein. Geprüft wird das, was auf der Mission Map
// falsch aussehen würde, wenn es kippt:
//
//   - kein Gerätestatus     -> konfigurierter Fallback bleibt (demo)
//   - live mit Fix          -> Position, nav-Zustand, Batterie n/a statt 0 %
//   - live ohne Datum       -> KEINE Position (sonst Marker auf Eves Startpunkt)
//   - Link >6 s still       -> "link lost", stale, und online == false,
//                              damit "Land collection available" nicht lügt
//   - Treffer über Alias    -> robot_2 landet auf dem GripperX-Profil
//
// Braucht node (nicht auf dem Demo-Laptop installiert):
//   node tests/fleet_device_status.mjs
//
// Kein Test-Runner, keine Abhängigkeiten - Ausgabe von Hand lesen. Der letzte
// Fehler beim Beenden kommt aus der DOM-Attrappe, nicht aus dem Code.

import fs from "node:fs";
import vm from "node:vm";

const source = fs.readFileSync(
  new URL("../live_data.js", import.meta.url), "utf8");

const noop = () => {};
// A canvas 2D context that swallows everything: the fleet logic under test does
// no drawing, but module-level render calls must not explode on the way in.
const ctx2d = new Proxy({}, {
  get: (t, k) => {
    if (k === "canvas") return { width: 800, height: 600 };
    if (k === "measureText") return () => ({ width: 10 });
    if (k === "createLinearGradient") return () => ({ addColorStop: noop });
    if (k === "getImageData") return () => ({ data: new Uint8ClampedArray(4) });
    return () => undefined;
  },
  set: () => true,
});
const el = new Proxy({}, {
  get: (t, k) => {
    if (k === "dataset") return {};
    if (k === "classList") return { toggle: noop, add: noop, remove: noop, contains: () => false };
    if (k === "style") return new Proxy({}, {
      get: (t2, k2) => (k2 === "removeProperty" || k2 === "setProperty" ? noop : undefined),
      set: () => true,
    });
    if (k === "querySelectorAll") return () => [];
    if (k === "addEventListener") return noop;
    if (k === "appendChild") return noop;
    if (k === "getContext") return () => ctx2d;
    if (k === "width" || k === "height") return 600;
    if (k === "getBoundingClientRect") return () => ({ width: 800, height: 600, top: 0, left: 0 });
    if (k === "value") return "";
    if (k === "textContent" || k === "innerHTML") return "";
    return undefined;
  },
  set: () => true,
});

const store = new Map();
const sandbox = {
  console,
  document: {
    getElementById: () => el,
    querySelector: () => el,
    querySelectorAll: () => [],
    addEventListener: noop,
    createElement: () => el,
    body: el,
  },
  localStorage: {
    getItem: (k) => (store.has(k) ? store.get(k) : null),
    setItem: (k, v) => store.set(k, String(v)),
    removeItem: (k) => store.delete(k),
  },
  window: { devicePixelRatio: 1, addEventListener: noop, location: { href: "" },
            setInterval: () => 0, setTimeout: () => 0, clearInterval: noop },
  navigator: { userAgent: "node" },
  fetch: async () => ({ ok: true, json: async () => ({}) }),
  setTimeout: () => 0,
  setInterval: () => 0,
  clearInterval: noop,
  requestAnimationFrame: () => 0,
  L: undefined,
};
sandbox.globalThis = sandbox;

const context = vm.createContext(sandbox);
try {
  vm.runInContext(source + "\n;globalThis.__x = { OCTOPUS, getFleetSnapshot, deviceLinkSummary };",
                  context, { filename: "live_data.js" });
} catch (e) {
  console.log("LOAD ERROR:", e.message);
}

const { OCTOPUS, getFleetSnapshot, deviceLinkSummary } = context.__x || {};
if (typeof getFleetSnapshot !== "function") {
  console.log("FAIL: getFleetSnapshot not reachable");
  process.exit(1);
}

function gripperx() {
  return getFleetSnapshot().find((r) => r.key === "gripperx");
}
function show(label, r) {
  console.log(`\n[${label}]`);
  console.log("  state        :", r.state);
  console.log("  status       :", r.status);
  console.log("  live         :", r.live, "| linkStale:", r.linkStale);
  console.log("  hasPosition  :", r.hasPosition);
  console.log("  lat/lon      :", r.location.lat, r.location.lon);
  console.log("  local x/y    :", r.local.x?.toFixed?.(3), r.local.y?.toFixed?.(3));
  console.log("  battery      :", r.battery.percent, "| unavailable:", r.battery.unavailable, "|", r.battery.reason);
  console.log("  demo         :", r.demo);
  if (r.device) console.log("  summary      :", deviceLinkSummary(r));
}

// --- 1. no device status at all: must stay on the configured fallback
show("no device status", gripperx());

const now = 1_800_000_000;

// --- 2. live, navigating, with a fix
OCTOPUS.latest.devicesServerTime = now;
OCTOPUS.latest.devices = {
  gripperx: {
    robot_id: "gripperx", source_id: "sim", source_topic: "/octopus/devices/gripperx/status",
    pose: { status: "ok", frame_id: "map", lat: 48.2517, lon: 11.6345, yaw_deg: 12.5, x: 1.2, y: 0.4 },
    nav: { status: "navigating", active_goal_id: 3, distance_remaining_m: 2.34 },
    armed: true,
    battery: { status: "unavailable", reason: "NO_SENSOR_INSTALLED", percent: null, voltage_v: null },
    link: { connected: true, last_rx_age_sec: 0.2 },
    backend_received_at: now - 0.4,
  },
};
show("live + fix + navigating", gripperx());

// --- 3. live but no datum: must NOT inherit the fallback position
OCTOPUS.latest.devices.gripperx.pose = { status: "no_datum", frame_id: "map", lat: null, lon: null };
OCTOPUS.latest.devices.gripperx.nav = { status: "idle", active_goal_id: null, distance_remaining_m: null };
show("live, no datum", gripperx());

// --- 4. stale link
OCTOPUS.latest.devices.gripperx.backend_received_at = now - 30;
OCTOPUS.latest.devices.gripperx.pose = { status: "ok", lat: 48.2517, lon: 11.6345 };
show("link stale (30s)", gripperx());

// --- 5. matched by alias instead of the exact key
delete OCTOPUS.latest.devices.gripperx;
OCTOPUS.latest.devices.robot_2 = {
  robot_id: "robot_2",
  pose: { status: "ok", lat: 48.2518, lon: 11.6346 },
  nav: { status: "collecting", active_goal_id: 7, distance_remaining_m: 0.1 },
  armed: false,
  battery: { status: "ok", percent: 81 },
  backend_received_at: now - 0.3,
};
show("matched via alias robot_2", gripperx());

// --- 6. a real battery percentage must survive
console.log("\n[other robots unaffected]");
for (const r of getFleetSnapshot()) {
  if (r.key === "gripperx") continue;
  console.log(`  ${r.name.padEnd(8)} demo=${r.demo} hasPosition=${r.hasPosition} state=${r.state}`);
}

// --- 7. readiness must not count a robot whose link went quiet
OCTOPUS.latest.devices = {
  gripperx: {
    robot_id: "gripperx",
    pose: { status: "ok", lat: 48.2517, lon: 11.6345 },
    nav: { status: "idle" }, armed: false, battery: { status: "ok", percent: 90 },
    backend_received_at: now - 40,
  },
};
const stale = gripperx();
console.log("\n[readiness with a stale GripperX]");
console.log("  linkStale:", stale.linkStale, "| online:", stale.online, "(must be false)");
OCTOPUS.latest.devices.gripperx.backend_received_at = now - 0.5;
const fresh2 = gripperx();
console.log("  fresh again -> online:", fresh2.online, "(must be true)");
