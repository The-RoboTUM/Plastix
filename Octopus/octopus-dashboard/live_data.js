const GRID_DISPLAY_DEFAULTS = {
  priors: true,
  detections: true,
  ground: true,
  water: true,
  obstacles: true,
  trash: true,
  fleet: true,
  home: false,
  coverage: true,
  confidence: true,
  unknown: true,
};

// The home station is a placeholder: homeStation() returns a hard-coded local
// (0.25, 0.25) m, i.e. 25 cm from the Eve datum, so it always sat on top of the
// drone. Nothing reads it - the readiness row for it still says "planning tool
// later" - so it defaults to off now. Its toggle was already there but the
// mission map ignored it, and a dashboard that ran before this change has
// home: true in localStorage and would keep drawing it; drop that one stored
// value once. Every other toggle stays as the operator left it, and ticking
// "home station" again sticks from then on.
function initialGridDisplay() {
  let stored = {};
  try {
    stored = JSON.parse(localStorage.getItem("octopusGridDisplay") || "{}") || {};
  } catch (error) {
    stored = {};
  }
  if (localStorage.getItem("octopusGridDisplayHomeDefaultOff") !== "1") {
    delete stored.home;
    localStorage.setItem("octopusGridDisplayHomeDefaultOff", "1");
    localStorage.setItem("octopusGridDisplay", JSON.stringify(stored));
  }
  return { ...GRID_DISPLAY_DEFAULTS, ...stored };
}

const GRID_COLORS = {
  detectedGround: { r: 46, g: 232, b: 111 },
  priorGround: { r: 22, g: 101, b: 52 },
  detectedWater: { r: 56, g: 189, b: 248 },
  priorWater: { r: 14, g: 116, b: 144 },
  detectedObstacle: { r: 248, g: 113, b: 113 },
  priorBuilding: { r: 127, g: 29, b: 29 },
  trash: { r: 251, g: 146, b: 60 },
  unknown: { r: 71, g: 85, b: 105 },
};

// Camera overlay grid modes. Exactly one grid is active at a time, and the
// trash-cell marking always follows the active one.
const CAMERA_GRID_MODES = ["off", "local", "gps"];

// Restores the stored mode, migrating the pre-mode settings: the separate GPS
// checkbox wins, otherwise "Grid: off" (cols=0) becomes the off mode.
function initialCameraGridMode() {
  const stored = localStorage.getItem("octopusCameraGridMode");
  if (CAMERA_GRID_MODES.includes(stored)) return stored;
  if (localStorage.getItem("octopusCameraGpsGrid") === "1") return "gps";
  if (localStorage.getItem("octopusCameraFeedGridCols") === "0") return "off";
  return "local";
}

// The local grid needs a real column count now that "off" is its own mode.
function initialLocalGridCols() {
  const stored = parseInt(localStorage.getItem("octopusCameraFeedGridCols") ?? "8", 10);
  return Number.isFinite(stored) && stored >= 2 ? Math.min(stored, 40) : 8;
}

// Camera crop: how much of each frame edge the operator cuts away, as a fraction
// of the full frame. Capped per side so the cropped region always still contains
// the principal point (cx/cy) — the footprint model is measured from it.
const CAMERA_CROP_SIDES = ["top", "right", "bottom", "left"];
const CAMERA_CROP_MAX_SIDE = 0.45;

function initialCameraCrop() {
  const stored = JSON.parse(localStorage.getItem("octopusCameraCrop") || "null") || {};
  const crop = {};
  CAMERA_CROP_SIDES.forEach((side) => {
    const value = parseFloat(stored[side]);
    crop[side] = Number.isFinite(value) ? Math.min(Math.max(value, 0), CAMERA_CROP_MAX_SIDE) : 0;
  });
  return crop;
}

const OCTOPUS = {
  missionPhase: localStorage.getItem("octopusMissionPhase") || "preflight",
  dashboardView: localStorage.getItem("octopusDashboardView") || "overview",
  lastRefresh: null,
  backendOk: false,
  lastError: null,
  latest: {
    tasks: [],
    battery: [],
    locations: [],
    stats: null,
    patch: null,
    globalMap: null,
    cameraTransformStatus: null,
    cameraDebug: null,
    localCameraGrid: null,
    devices: {},
    devicesServerTime: null,
    eveStatus: null,
    // Die drei Teilsysteme, die die Eve-Karte als Chips zeigt. Jeweils
    // { status, at } wie eveStatus -- "at" trennt "gerade als gestoppt
    // gemeldet" von "seit dem Laden nie erreicht".
    px4BridgeStatus: null,
    detectorStatus: null,
  },
  selected: null,
  selectedCellKey: null,
  timeline: [],
  seenPatchSignature: null,
  missionMap: {
    map: null,
    baseLayer: null,
    markerLayer: null,
    gridLayer: null,
    areaLayer: null,
    legendControl: null,
    legendEl: null,
    legendCellKeys: null,
    legendMarkerKeys: null,
    legendSignature: null,
    hasFit: false,
    polygonMode: false,
    polygonPoints: [],
    polygonPreviewLayer: null,
    placeEveMode: false,
  },
  legendCollapsed: localStorage.getItem("octopusLegendCollapsed") === "1",
  manualEve: JSON.parse(localStorage.getItem("octopusManualEve") || "null"),
  eveYawDeg: parseFloat(localStorage.getItem("octopusEveYawDeg") || "0") || 0,
  // "drone" = Kurs kommt aus der Odometrie, "manual" = Handeingabe.
  eveYawSource: localStorage.getItem("octopusEveYawSource") === "drone" ? "drone" : "manual",
  // Der Handwert wird beim Umschalten auf den Kompass NICHT ueberschrieben:
  // wer zurueckschaltet, will seinen eingestellten Winkel wiederhaben und ihn
  // nicht neu suchen muessen.
  eveYawManualDeg: parseFloat(localStorage.getItem("octopusEveYawDeg") || "0") || 0,
  projectDetections: (localStorage.getItem("octopusProjectDetections") ?? "1") === "1",
  missionArea: JSON.parse(localStorage.getItem("octopusMissionArea") || "null"),
  gridMode: localStorage.getItem("octopusGridMode") || "fixed_camera_footprint",
  gridSource: localStorage.getItem("octopusGridSource") || "global",
  mappingMode: localStorage.getItem("octopusMappingMode") || "local_camera_debug",
  cameraFootprint: JSON.parse(localStorage.getItem("octopusCameraFootprint") || '{"height_m":2.5,"resolution":0.10}'),
  osmPriors: JSON.parse(localStorage.getItem("octopusOsmPriors") || "null"),
  gridDisplay: initialGridDisplay(),
  gridView: { scale: 1, offsetX: 0, offsetY: 0, isPanning: false, lastX: 0, lastY: 0, moved: false },
  cameraFeed: {
    // "off" | "local" | "gps" — see CAMERA_GRID_MODES.
    overlayGrid: initialCameraGridMode(),
    gridCols: initialLocalGridCols(),
    highlightCells: (localStorage.getItem("octopusCameraFeedHighlight") ?? "1") === "1",
    cellNames: (localStorage.getItem("octopusCameraCellNames") ?? "1") === "1",
    gpsStepDeg: parseFloat(localStorage.getItem("octopusCameraGpsStepDeg") || "0") || 0,
    imageSignature: null,
  },
  // Fraction of each frame edge cut away — see the camera crop section below.
  cameraCrop: initialCameraCrop(),
};

const DEMO_MAP_ORIGIN = { lat: 48.2513611, lon: 11.6359722 };
const METERS_PER_DEGREE_LAT = 111320;

const PHASE_INFO = {
  idle: {
    title: "Idle",
    action: "System waiting",
    decision: "Define or load a mission area.",
    status: "muted",
  },
  setup: {
    title: "Setup",
    action: "Mission setup",
    decision: "Draw polygon, home position, no-go zones.",
    status: "warning",
  },
  preflight: {
    title: "Pre-flight Check",
    action: "Checking readiness",
    decision: "Start only when critical systems are fresh.",
    status: "warning",
  },
  ready: {
    title: "Ready",
    action: "Ready to start",
    decision: "Start mission or return to setup.",
    status: "fresh",
  },
  scanning: {
    title: "Scanning",
    action: "Drone scans mission area",
    decision: "Watch coverage, detections, and pose freshness.",
    status: "fresh",
  },
  mapping: {
    title: "Mapping",
    action: "Building local grid map",
    decision: "Check map patches and layer confidence.",
    status: "fresh",
  },
  verification: {
    title: "Trash Verification",
    action: "Reviewing candidate detections",
    decision: "Confirm, reject, or keep scanning.",
    status: "warning",
  },
  assignment: {
    title: "Task Assignment",
    action: "Assigning trash targets",
    decision: "Choose robot or automatic assignment.",
    status: "warning",
  },
  collection: {
    title: "Ground Collection",
    action: "Ground robots collect trash",
    decision: "Monitor robot progress and blocked targets.",
    status: "fresh",
  },
  return_home: {
    title: "Return Home",
    action: "Drone returning home",
    decision: "Watch battery and landing state.",
    status: "warning",
  },
  complete: {
    title: "Mission Complete",
    action: "Mission finished",
    decision: "Export report or replay mission.",
    status: "fresh",
  },
  paused: {
    title: "Paused",
    action: "Mission paused",
    decision: "Resume, return home, or abort.",
    status: "warning",
  },
  error: {
    title: "Error",
    action: "Operator attention required",
    decision: "Inspect failing subsystem before continuing.",
    status: "error",
  },
 };

const DASHBOARD_VIEWS = {
  // Audience-facing layout for demos and trade fairs: map + camera, big numbers,
  // no operator controls. Set the grid mode etc. in the overview first — the show
  // view deliberately has no knobs on screen.
  show: {
    label: "Show / Presentation",
    action: "Audience view",
    preset: "overview",
  },
  overview: {
    label: "Mission Overview",
    action: "Operator overview",
    preset: "overview",
  },
  debug: {
    label: "System / Debug",
    action: "Inspect ROS/backend/raw data",
    preset: "debug",
  },
};

const ROBOT_FLEET_PROFILES = {
  eve: {
    key: "eve",
    name: "Eve",
    icon: "🚁",
    mapLabel: "E",
    type: "drone",
    terrain: "air",
    role: "Drone / scan / detect",
    purpose: "Aerial detection and mission scanning",
    capability: "Detects trash and updates the map. Does not collect trash.",
    taskRule: "Detection/scanning only",
    tags: ["scan", "detect", "camera", "map update"],
    aliases: ["eve", "drone", "drone_1", "uav"],
    // Eve defines local (0, 0) - see eveDatum(). Even the demo fallback has to
    // sit on the origin, or the datum and the marker drift apart.
    fallback: { x: 0, y: 0, state: "scanning", battery: 87 },
  },
  robby: {
    key: "robby",
    name: "Robby",
    icon: "🤖",
    mapLabel: "R",
    type: "land",
    terrain: "ground",
    role: "Land robot / collect land trash",
    purpose: "Wheeled ground robot for reachable land trash",
    capability: "Collects trash on ground cells classified as reachable land.",
    taskRule: "Land trash only",
    tags: ["land", "wheeled", "collect"],
    aliases: ["robby", "robot_1", "ground_robot", "land_robot"],
    fallback: { x: 0.7, y: 0.5, state: "idle", battery: 72 },
  },
  gripperx: {
    key: "gripperx",
    name: "GripperX",
    icon: "🦾",
    mapLabel: "G",
    type: "land",
    terrain: "ground",
    role: "Land robot / gripper or suction collection",
    purpose: "Second wheeled collection robot with gripper/suction mechanism",
    capability: "Collects reachable land trash with arm, gripper or suction.",
    taskRule: "Land trash only",
    tags: ["land", "gripper", "suction", "collect"],
    aliases: ["gripperx", "gripper", "robot_2", "vacuum", "suction"],
    fallback: { x: 1.0, y: 0.45, state: "idle", battery: 66 },
  },
  sharx: {
    key: "sharx",
    name: "SharX",
    icon: "⛵",
    mapLabel: "S",
    type: "water",
    terrain: "water",
    role: "Boat / collect water trash",
    purpose: "Water robot for floating or water-based trash",
    capability: "Collects trash classified as water/floating. Should not take land tasks.",
    taskRule: "Water trash only",
    tags: ["water", "boat", "floating trash", "collect"],
    aliases: ["sharx", "boat", "boat_1", "water_robot", "surface"],
    fallback: { x: 3.3, y: 1.0, state: "idle", battery: 79 },
    // ABSOLUTE, and that is the whole point. A `fallback` is expressed in local
    // metres, which localToLatLng() resolves against getMissionOrigin() - and
    // that origin IS Eve. So a fallback-positioned robot slides across the map
    // every time the operator places Eve somewhere else, which is what made
    // SharX look like it was wandering. These are the coordinates SharX had
    // before any Eve placement, i.e. fallback (3.3, 1.0) against
    // DEMO_MAP_ORIGIN. Being a boat, it belongs in the water and must stay
    // there no matter where the drone goes.
    fixedLatLon: { lat: 48.2513701, lon: 11.6360167 },
  },
};

function $(id) {
  return document.getElementById(id);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function safeNumber(value, fallback = 0) {
  // Number(null) and Number("") are 0, so a missing confidence would render as
  // "0.00" instead of being left out. Treat empty values as the fallback.
  if (value === null || value === undefined || value === "") return fallback;
  const number = Number(value);
  return Number.isFinite(number) ? number : fallback;
}

function rgbToken(name) {
  return GRID_COLORS[name] || GRID_COLORS.unknown;
}

function rgbaToken(name, alpha = 1) {
  const c = rgbToken(name);
  return `rgba(${c.r}, ${c.g}, ${c.b}, ${clamp(alpha, 0, 1)})`;
}

function hexToken(name) {
  const c = rgbToken(name);
  const part = (v) => v.toString(16).padStart(2, "0");
  return `#${part(c.r)}${part(c.g)}${part(c.b)}`;
}

function formatPercent(value) {
  if (!Number.isFinite(value)) return "--";
  return `${Math.round(value * 100)}%`;
}

function formatTime(date = new Date()) {
  return date.toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" });
}

function parseTimestamp(value) {
  if (value === null || value === undefined || value === "" || value === "N/A") return null;

  if (typeof value === "number") {
    const ms = value > 10_000_000_000 ? value : value * 1000;
    const date = new Date(ms);
    return Number.isNaN(date.getTime()) ? null : date;
  }

  if (typeof value === "string") {
    const numeric = Number(value);
    if (Number.isFinite(numeric) && value.trim() !== "") {
      return parseTimestamp(numeric);
    }

    const date = new Date(value);
    return Number.isNaN(date.getTime()) ? null : date;
  }

  return null;
}

function ageSeconds(value) {
  const date = parseTimestamp(value);
  if (!date) return null;
  return Math.max(0, (Date.now() - date.getTime()) / 1000);
}

function formatAge(value) {
  const age = typeof value === "number" ? value : ageSeconds(value);
  if (age === null || !Number.isFinite(age)) return "unknown";
  if (age < 1) return "now";
  if (age < 60) return `${age.toFixed(1)} s ago`;
  if (age < 3600) return `${Math.round(age / 60)} min ago`;
  return `${Math.round(age / 3600)} h ago`;
}

function freshnessFromAge(age, freshLimit = 2.0, staleLimit = 10.0) {
  if (age === null || !Number.isFinite(age)) {
    return { state: "unknown", label: "not configured" };
  }
  if (age <= freshLimit) {
    return { state: "fresh", label: `fresh · ${formatAge(age)}` };
  }
  if (age <= staleLimit) {
    return { state: "stale", label: `stale · ${formatAge(age)}` };
  }
  return { state: "offline", label: `missing · ${formatAge(age)}` };
}

function statusPill(label, state = "unknown") {
  return `<span class="pill ${state}"><span class="dot"></span><span>${label}</span></span>`;
}

function miniStatus(label, state = "unknown") {
  return `<span class="mini-chip ${state}">${label}</span>`;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function addTimeline(message, level = "info") {
  const entry = {
    time: new Date(),
    message,
    level,
  };
  OCTOPUS.timeline.unshift(entry);
  OCTOPUS.timeline = OCTOPUS.timeline.slice(0, 30);
  renderTimeline();
}

async function apiGet(url) {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`${url} returned ${response.status}`);
  }
  return response.json();
}


function getResolutionFromInput(fallback = 0.10) {
  const input = $("grid-resolution-input");
  const value = safeNumber(input?.value, fallback);
  return clamp(value, 0.02, 2.0);
}

// Eve is the origin of the local frame, not the corner of the drawn search area.
// The collector robot is started on Eve's spot, so trash_gps_goal_node expresses
// every target relative to her and calls that map (0, 0) - see
// docs/octopus_to_robot_interface.md. Anchoring the dashboard anywhere else means
// the same "x = 3.6 m" points at two different places on the ground.
//
// Deliberately resolved without getFleetSnapshot(): the snapshot converts lat/lon
// into local meters and would call straight back into here.
function eveDatum() {
  const profile = ROBOT_FLEET_PROFILES.eve;
  // Same precedence getFleetSnapshot() gives Eve's position, so the datum and
  // the marker on the map can never disagree: live link, then the operator's
  // manual placement, then the database row.
  const candidates = [
    matchDeviceStatusForProfile(profile)?.pose,
    OCTOPUS.manualEve,
    matchLocationForProfile(profile),
  ];
  for (const candidate of candidates) {
    const lat = safeNumber(candidate?.lat, NaN);
    const lon = safeNumber(candidate?.lon, NaN);
    if (Number.isFinite(lat) && Number.isFinite(lon)) return { lat, lon };
  }
  // Nothing knows where Eve is yet. Fall back to the drawn area's corner and
  // finally to the demo origin - the same coordinate eve_fake_gps_bridge_node
  // publishes as its own fallback datum, so ROS and dashboard still agree.
  return OCTOPUS.missionArea?.origin || DEMO_MAP_ORIGIN;
}

function getMissionOrigin() {
  return eveDatum();
}

function metersPerDegreeLonForLat(lat) {
  return METERS_PER_DEGREE_LAT * Math.cos(safeNumber(lat, DEMO_MAP_ORIGIN.lat) * Math.PI / 180);
}

function localToLatLng(xMeters, yMeters) {
  const origin = getMissionOrigin();
  const lat = origin.lat + yMeters / METERS_PER_DEGREE_LAT;
  const lon = origin.lon + xMeters / metersPerDegreeLonForLat(origin.lat);
  return [lat, lon];
}

function latLngToLocal(lat, lon) {
  const origin = getMissionOrigin();
  return {
    x: (safeNumber(lon, origin.lon) - origin.lon) * metersPerDegreeLonForLat(origin.lat),
    y: (safeNumber(lat, origin.lat) - origin.lat) * METERS_PER_DEGREE_LAT,
  };
}

function boundsToMissionArea(bounds, source = "map", polygon = null) {
  const south = bounds.getSouth();
  const west = bounds.getWest();
  const north = bounds.getNorth();
  const east = bounds.getEast();
  // The drawn area contributes its size, not the local frame's anchor - that is
  // Eve (see eveDatum). This corner is only kept as the datum of last resort for
  // a session where Eve's position is not known at all.
  const origin = { lat: south, lon: west };
  const widthM = Math.max(0.1, (east - west) * metersPerDegreeLonForLat(south));
  const heightM = Math.max(0.1, (north - south) * METERS_PER_DEGREE_LAT);
  const resolution = getResolutionFromInput(0.10);
  const cols = Math.max(1, Math.ceil(widthM / resolution));
  const rows = Math.max(1, Math.ceil(heightM / resolution));
  const total = rows * cols;
  const warning = total > 25000 ? " Large grid. Increase resolution for better performance." : "";
  OCTOPUS.missionArea = {
    source,
    origin,
    bounds: { south, west, north, east },
    polygon,
    width_m: widthM,
    height_m: heightM,
    resolution,
    rows,
    cols,
    updated_at: new Date().toISOString(),
  };
  clearOsmPriors(false);
  localStorage.setItem("octopusMissionArea", JSON.stringify(OCTOPUS.missionArea));
  OCTOPUS.missionMap.hasFit = false;
  OCTOPUS.gridView = { scale: 1, offsetX: 0, offsetY: 0, isPanning: false, lastX: 0, lastY: 0, moved: false };
  updateResolutionLabel();
  clearPolygonPreview(false);
  addTimeline(`Search area set: ${widthM.toFixed(1)} m × ${heightM.toFixed(1)} m at ${resolution.toFixed(2)} m/cell (${cols}×${rows}).${warning}`, total > 25000 ? "warning" : "success");
  renderAll();
}

function updateResolutionLabel() {
  const input = $("grid-resolution-input");
  const label = $("grid-resolution-label");
  if (input) input.value = getResolutionFromInput(0.10).toFixed(2);
  if (label) label.textContent = "m/cell";
}

function defaultMissionAreaFromMapData(mapData = null) {
  const resolution = safeNumber(mapData?.resolution, 0.10);
  const cols = safeNumber(mapData?.cols, 50);
  const rows = safeNumber(mapData?.rows, 30);
  return {
    source: "default",
    origin: DEMO_MAP_ORIGIN,
    width_m: cols * resolution,
    height_m: rows * resolution,
    resolution,
    rows,
    cols,
  };
}

function getActiveGridMeta(mapData = null) {
  const area = OCTOPUS.missionArea || defaultMissionAreaFromMapData(mapData);
  const resolution = safeNumber(area.resolution, safeNumber(mapData?.resolution, 0.10));
  const cols = Math.max(1, safeNumber(area.cols, safeNumber(mapData?.cols, 50)));
  const rows = Math.max(1, safeNumber(area.rows, safeNumber(mapData?.rows, 30)));
  return {
    frame_id: mapData?.frame_id || "map",
    resolution,
    rows,
    cols,
    width_m: cols * resolution,
    height_m: rows * resolution,
    // The grid's local (0, 0), i.e. Eve - not the corner the operator happened
    // to drag the search area to. Only the extent comes from the drawn area.
    origin: getMissionOrigin(),
    source: area.source || "default",
  };
}

function fleetType(item) {
  const profile = profileForDeviceRecord(item);
  if (profile) return profile.type === "water" ? "boat" : profile.type === "land" ? "robot" : profile.type;
  const explicit = String(item.type || item.device_type || "").toLowerCase();
  if (explicit) return explicit;
  const id = String(item.id || item.device_id || "").toLowerCase();
  if (id.includes("drone") || id.includes("eve")) return "drone";
  if (id.includes("laptop") || id.includes("home")) return "base";
  return "robot";
}

function batteryForDevice(deviceId) {
  return (OCTOPUS.latest.battery || []).find((b) => String(b.id || b.device_id) === String(deviceId));
}

function profileForIdentifier(value, fallbackType = "") {
  const needle = String(value || "").toLowerCase();
  for (const profile of Object.values(ROBOT_FLEET_PROFILES)) {
    if (profile.aliases.some((alias) => needle.includes(alias))) return profile;
  }
  if (String(fallbackType).toLowerCase().includes("drone")) return ROBOT_FLEET_PROFILES.eve;
  if (String(fallbackType).toLowerCase().includes("boat") || String(fallbackType).toLowerCase().includes("water")) return ROBOT_FLEET_PROFILES.sharx;
  return null;
}

function profileForDeviceRecord(record = {}) {
  return profileForIdentifier(record.id || record.device_id || record.origin_id || record.name, record.type || record.device_type);
}

function matchBatteryForProfile(profile) {
  return (OCTOPUS.latest.battery || []).find((b) => {
    const id = String(b.id || b.device_id || "").toLowerCase();
    return profile.aliases.some((alias) => id.includes(alias));
  });
}

function matchLocationForProfile(profile) {
  return (OCTOPUS.latest.locations || []).find((loc) => {
    const id = String(loc.id || loc.origin_id || loc.device_id || loc.name || "").toLowerCase();
    return profile.aliases.some((alias) => id.includes(alias));
  });
}

// How long a device status may go unrefreshed before the link counts as stale.
// device_status_backend_bridge_node posts at 2 Hz, so anything past a few
// seconds means the robot or the link stopped, not a slow cycle.
const DEVICE_STATUS_STALE_SEC = 6;

function matchDeviceStatusForProfile(profile) {
  const devices = OCTOPUS.latest.devices || {};
  const direct = devices[profile.key];
  if (direct) return direct;
  const entry = Object.entries(devices).find(([id, record]) => {
    const candidate = String(record?.robot_id || id).toLowerCase();
    return profile.aliases.some((alias) => candidate.includes(alias));
  });
  return entry ? entry[1] : null;
}

// Seconds since the backend last accepted a status from this device, measured
// against the backend clock so a skewed browser clock cannot invent staleness.
function deviceStatusAge(device) {
  const receivedAt = safeNumber(device?.backend_received_at, NaN);
  const serverTime = safeNumber(OCTOPUS.latest.devicesServerTime, NaN);
  if (!Number.isFinite(receivedAt) || !Number.isFinite(serverTime)) return null;
  return Math.max(0, serverTime - receivedAt);
}

// What to call a robot that is publishing: its own nav state, unless it is
// telling us it cannot place itself yet.
function deviceStateLabel(device, age) {
  if (age !== null && age > DEVICE_STATUS_STALE_SEC) return "link lost";
  const pose = device?.pose || {};
  if (pose.status && pose.status !== "ok") {
    return pose.status === "no_datum" ? "waiting for datum" : `pose ${pose.status}`;
  }
  const nav = device?.nav || {};
  const navStatus = String(nav.status || "").trim();
  return navStatus || "online";
}

function batteryFromDeviceStatus(device) {
  const battery = device?.battery || {};
  const percent = battery.percent;
  const usable = typeof percent === "number" && Number.isFinite(percent);
  return {
    percent: usable ? percent : null,
    state: battery.status || "unknown",
    reason: battery.reason || null,
    // "no sensor installed" is not "empty battery" - keep them distinguishable
    // so the fleet card can say so instead of drawing a 0% bar.
    unavailable: !usable,
    ts: null,
    is_demo: false,
  };
}

function fallbackLocationForProfile(profile) {
  const p = profile.fallback || { x: 0, y: 0 };
  // A profile may pin itself to absolute coordinates instead of local metres.
  // is_demo stays true either way - this is still a configured position, not a
  // measurement - but is_fixed says it was placed deliberately and is allowed
  // on the map, where a drifting fallback is not.
  const fixed = profile.fixedLatLon;
  const [lat, lon] = fixed
    ? [fixed.lat, fixed.lon]
    : localToLatLng(p.x, p.y);
  return {
    id: profile.name,
    lat,
    lon,
    ts: new Date().toISOString(),
    state: p.state,
    is_demo: true,
    is_fixed: Boolean(fixed),
  };
}

function homeStation() {
  const [lat, lon] = localToLatLng(0.25, 0.25);
  return { id: "Home station", lat, lon, x: 0.25, y: 0.25, type: "home" };
}

// Substring matching, because the states come from several sources that do not
// share a vocabulary. GripperX reports exactly five (goal_gateway_node:
// idle / navigating / picking / cancelling / unavailable) and all five are
// accounted for below - "picking" and "cancelling" were falling through to
// "unknown", so a robot that was actively grasping rendered grey as if it had
// stopped answering.
//
// "unavailable" deliberately keeps falling through to "unknown". It does NOT
// mean the robot is unwell: it means the robot has no Nav2 at all (that is the
// twin without sim_navigation, and the real robot before autonomy is up).
// GripperX keeps "unavailable" and "idle" apart on purpose - idle is a nav stack
// that is not busy - and collapsing them here would report a navigation
// readiness that does not exist. The label the operator reads stays the robot's
// own word, from deviceStateLabel().
function robotStatusFromState(state, age) {
  const value = String(state || "").toLowerCase();
  if (value.includes("error") || value.includes("fail")) return "error";
  if (value.includes("offline")) return "offline";
  if (age !== null && age > 60 && !value.includes("idle")) return "stale";
  if (value.includes("return") || value.includes("assigned") || value.includes("driving") || value.includes("navigating") || value.includes("picking") || value.includes("cancelling")) return "warning";
  if (value.includes("scan") || value.includes("collect") || value.includes("online") || value.includes("idle")) return "fresh";
  return "unknown";
}

function getFleetSnapshot() {
  return Object.values(ROBOT_FLEET_PROFILES).map((profile) => {
    let battery = matchBatteryForProfile(profile) || { percent: profile.fallback?.battery, state: profile.fallback?.state, ts: null, is_demo: true };
    let location = matchLocationForProfile(profile) || fallbackLocationForProfile(profile);
    // Manual placement of the Eve drone takes precedence over live/fallback location.
    if (profile.key === "eve" && OCTOPUS.manualEve &&
        Number.isFinite(OCTOPUS.manualEve.lat) && Number.isFinite(OCTOPUS.manualEve.lon)) {
      location = {
        id: profile.name,
        lat: OCTOPUS.manualEve.lat,
        lon: OCTOPUS.manualEve.lon,
        ts: OCTOPUS.manualEve.ts || new Date().toISOString(),
        state: "manual placement",
        manual: true,
      };
    }
    let age = ageSeconds(location.ts || battery.ts);
    let state = battery.state || location.state || profile.fallback?.state || "unknown";
    let demo = Boolean(location.is_demo || battery.is_demo);
    // A manually placed Eve counts as real, positioned data (not demo/offline).
    if (location.manual) {
      state = "manual placement";
      demo = false;
    }

    // A robot reporting over the GripperX link outranks the database row and the
    // configured fallback: this is the only source that is actually live.
    const device = matchDeviceStatusForProfile(profile);
    let deviceAge = null;
    let linkStale = false;
    if (device) {
      deviceAge = deviceStatusAge(device);
      linkStale = deviceAge !== null && deviceAge > DEVICE_STATUS_STALE_SEC;
      const pose = device.pose || {};
      const lat = safeNumber(pose.lat, NaN);
      const lon = safeNumber(pose.lon, NaN);
      if (Number.isFinite(lat) && Number.isFinite(lon)) {
        location = { id: profile.name, lat, lon, ts: null, state: pose.status || "ok", live: true };
      } else {
        // The robot is talking but cannot place itself (typically pose.status
        // "no_datum"). Falling back to the configured demo position here would
        // draw a live-looking marker at a made-up spot, so drop the position
        // entirely and let the panels say why there is no marker.
        location = { id: profile.name, lat: null, lon: null, ts: null, state: pose.status || "no_position", live: true, missing: true };
      }
      battery = batteryFromDeviceStatus(device);
      state = deviceStateLabel(device, deviceAge);
      age = deviceAge;
      demo = false;
    }

    let status = demo ? "unknown" : robotStatusFromState(state, age);
    // A robot that is talking but cannot place itself needs attention: it will
    // not appear on the map, and "unknown" reads like missing data instead.
    if (device && location.missing && !linkStale) status = "warning";
    if (linkStale) status = "stale";
    // Eve is the local frame's origin, so her own local coordinate is 0, 0 by
    // definition - stated outright rather than left to a subtraction that only
    // happens to cancel out.
    const local = profile.key === "eve"
      ? { x: 0, y: 0 }
      : latLngToLocal(safeNumber(location.lat, NaN), safeNumber(location.lon, NaN));
    const currentTask = findCurrentTaskForRobot(profile);
    return {
      ...profile,
      id: profile.name,
      battery,
      location,
      local,
      state: demo ? "configured / no live data" : state,
      status,
      age,
      // Readiness ("Land collection available") reads this, so a robot whose
      // link went quiet must not still count as available.
      online: !demo && !linkStale && status !== "offline" && status !== "error" && (age === null || age < 120),
      currentTask,
      demo,
      device,
      deviceAge,
      linkStale,
      live: Boolean(device) && !linkStale,
      hasPosition: Number.isFinite(safeNumber(location.lat, NaN)) && Number.isFinite(safeNumber(location.lon, NaN)),
    };
  });
}

// Eve has no GripperX-style device link, so "is the drone connected" comes from
// the SSH reachability check the Eve camera panel already polls every 8 s: the Pi
// answering octopus_camera_status.sh at all means the drone is up. Deliberately
// NOT the camera_debug feed — test_camera_feed.py posts to that from the laptop,
// so a feed would report a drone that is not even powered on.
const EVE_ONLINE_STATUSES = new Set([
  "camera_running",
  "camera_started",
  "online_camera_stopped",
  "online_unknown",
]);
const EVE_STATUS_STALE_SEC = 30;

function eveLinkOnline() {
  const record = OCTOPUS.latest.eveStatus;
  if (!record || !EVE_ONLINE_STATUSES.has(record.status)) return false;
  const age = ageSeconds(record.at);
  return age === null || age <= EVE_STATUS_STALE_SEC;
}

// Who is actually on the air right now. Only a live link counts: robot.live is
// "the robot itself is publishing over the GripperX bridge", and Eve goes by her
// SSH check. A database row in locations/battery is mission bookkeeping that
// outlives the connection, so it must not make an unplugged robot look online.
function fleetLinkSummary() {
  const fleet = getFleetSnapshot();
  const online = fleet
    .filter((robot) => (robot.key === "eve" ? eveLinkOnline() : robot.live))
    .map((robot) => robot.name);
  return {
    online,
    total: fleet.length,
    note: OCTOPUS.latest.eveStatus ? "No live links" : "Waiting for link check",
  };
}

function findCurrentTaskForRobot(profile) {
  const robotName = profile.name.toLowerCase();
  return (OCTOPUS.latest.tasks || []).find((task) => {
    const assigned = String(task.assigned || task.assigned_to || "").toLowerCase();
    return assigned.includes(robotName) || profile.aliases.some((alias) => assigned.includes(alias));
  }) || null;
}

function taskTerrain(task = {}) {
  const raw = String(task.terrain || task.surface || task.semantic_class || task.land_class || task.class_name || "").toLowerCase();
  if (raw.includes("water") || raw.includes("floating") || raw.includes("river") || raw.includes("lake")) return "water";
  if (raw.includes("ground") || raw.includes("land") || raw.includes("grass") || raw.includes("road") || raw.includes("park")) return "ground";
  const assigned = String(task.assigned || task.assigned_to || "").toLowerCase();
  if (assigned.includes("sharx") || assigned.includes("boat")) return "water";
  if (assigned.includes("robby") || assigned.includes("gripper")) return "ground";
  return "unknown";
}

function suitableRobotNamesForTask(task = {}) {
  const terrain = taskTerrain(task);
  if (terrain === "water") return ["SharX"];
  if (terrain === "ground") return ["Robby", "GripperX"];
  return ["Confirm terrain first"];
}

function terrainStatusForTask(task = {}) {
  const terrain = taskTerrain(task);
  if (terrain === "water") return { label: "water trash → SharX", state: "fresh" };
  if (terrain === "ground") return { label: "land trash → Robby/GripperX", state: "fresh" };
  return { label: "unknown terrain → confirm before assignment", state: "warning" };
}


function deviceStyle(type, state = "") {
  const stateLower = String(state).toLowerCase();
  if (stateLower.includes("error") || stateLower.includes("offline")) return { color: "#fecaca", fillColor: "#ef4444" };
  if (type === "drone") return { color: "#ffffff", fillColor: "#38bdf8" };
  if (type === "boat") return { color: "#ffffff", fillColor: "#0ea5e9" };
  if (type === "base") return { color: "#ffffff", fillColor: "#a78bfa" };
  if (stateLower.includes("collect")) return { color: "#ffffff", fillColor: "#fb923c" };
  return { color: "#ffffff", fillColor: "#34d399" };
}

function missionRobotIcon(robot) {
  if (typeof L === "undefined") return null;
  const terrainClass = robot.type === "drone" ? "drone" : robot.type === "water" ? "water" : "land";
  const offline = robot.status === "offline" || robot.status === "error" ? " offline" : "";
  return L.divIcon({
    className: `robot-div-icon ${terrainClass}${offline}`,
    html: `<span aria-hidden="true">${robot.icon}</span>`,
    iconSize: [34, 34],
    iconAnchor: [17, 17],
  });
}

function initMissionMap() {
  const el = $("mission-map");
  if (!el || typeof L === "undefined" || OCTOPUS.missionMap.map) return;

  const map = L.map(el, {
    zoomControl: true,
    attributionControl: true,
    preferCanvas: true,
  }).setView([DEMO_MAP_ORIGIN.lat, DEMO_MAP_ORIGIN.lon], 19);

  // Leaflet takes the map's zoom ceiling from its layers, so with tiles on this
  // maxZoom IS the mission map's limit - "Grid only" zooms deeper purely because
  // removing the layer leaves no ceiling at all. A 0.10 m grid cell is about
  // 2 px at zoom 21, which is not enough to look at a single cell, so allow the
  // tiles to keep upscaling past their native level: beyond 19 OSM has nothing
  // sharper anyway, and the grid, markers and footprint stay crisp because they
  // are vectors.
  const baseLayer = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 25,
    maxNativeZoom: 19,
    attribution: "&copy; OpenStreetMap contributors",
  }).addTo(map);

  OCTOPUS.missionMap.map = map;
  OCTOPUS.missionMap.baseLayer = baseLayer;
  OCTOPUS.missionMap.gridLayer = L.layerGroup().addTo(map);
  OCTOPUS.missionMap.areaLayer = L.layerGroup().addTo(map);
  OCTOPUS.missionMap.polygonPreviewLayer = L.layerGroup().addTo(map);
  OCTOPUS.missionMap.markerLayer = L.layerGroup().addTo(map);

  if (L.control && L.control.scale) {
    L.control.scale({ metric: true, imperial: false, maxWidth: 140, position: "bottomleft" }).addTo(map);
  }

  const legend = L.control({ position: "bottomright" });
  legend.onAdd = function () {
    const div = L.DomUtil.create("div", "octopus-map-legend");
    L.DomEvent.disableClickPropagation(div);
    // Delegated, so it survives renderMissionLegend() replacing the innerHTML.
    div.addEventListener("click", (event) => {
      if (event.target.closest(".legend-toggle")) toggleMissionLegend();
    });
    OCTOPUS.missionMap.legendEl = div;
    renderMissionLegend();
    return div;
  };
  legend.addTo(map);
  OCTOPUS.missionMap.legendControl = legend;

  map.on("click", (event) => handleMissionMapClick(event));
  map.on("mousemove", (event) => handleMissionMapMouseMove(event));
  // Both camera grids thin out their labels by zoom level, so redraw after zooming.
  map.on("zoomend", () => {
    syncBasemapMuting();
    if (cameraGridMode() !== "off") renderMissionMap();
  });
  setTimeout(() => map.invalidateSize(), 150);
}

function setPolygonMode(enabled) {
  const state = OCTOPUS.missionMap;
  state.polygonMode = enabled;
  const drawButton = $("draw-polygon-button");
  const finishButton = $("finish-polygon-button");
  const mapEl = $("mission-map");
  if (drawButton) drawButton.classList.toggle("area-active", enabled);
  if (finishButton) finishButton.disabled = !enabled || state.polygonPoints.length < 3;
  if (mapEl) mapEl.classList.toggle("polygon-drawing", enabled);
  renderPolygonPreview();
}

function polygonStatusText() {
  const count = OCTOPUS.missionMap.polygonPoints.length;
  if (!OCTOPUS.missionMap.polygonMode) return null;
  if (count === 0) return "Polygon mode: click first search-area point";
  if (count < 3) return `Polygon mode: ${count} point${count === 1 ? "" : "s"} set · add at least ${3 - count} more`;
  return `Polygon mode: ${count} points set · click Finish to submit`;
}

function renderDrawHelpBadge() {
  const mapEl = $("mission-map");
  if (!mapEl) return;
  let badge = $("draw-help-badge");
  const text = polygonStatusText();
  if (!text) {
    if (badge) badge.remove();
    return;
  }
  if (!badge) {
    badge = document.createElement("div");
    badge.id = "draw-help-badge";
    badge.className = "draw-help-badge";
    mapEl.appendChild(badge);
  }
  badge.textContent = text;
}

function clearPolygonPreview(resetPoints = true) {
  const state = OCTOPUS.missionMap;
  state.polygonMode = false;
  if (resetPoints) state.polygonPoints = [];
  if (state.polygonPreviewLayer) state.polygonPreviewLayer.clearLayers();
  const drawButton = $("draw-polygon-button");
  const finishButton = $("finish-polygon-button");
  const mapEl = $("mission-map");
  if (drawButton) drawButton.classList.remove("area-active");
  if (finishButton) finishButton.disabled = true;
  if (mapEl) mapEl.classList.remove("polygon-drawing");
  renderDrawHelpBadge();
}

function renderPolygonPreview(mouseLatLng = null) {
  const state = OCTOPUS.missionMap;
  if (!state.polygonPreviewLayer || typeof L === "undefined") return;
  state.polygonPreviewLayer.clearLayers();

  const points = state.polygonPoints || [];
  const previewPoints = mouseLatLng && state.polygonMode ? [...points, mouseLatLng] : points;

  if (previewPoints.length >= 2) {
    L.polyline(previewPoints, {
      color: "#f59e0b",
      weight: 3,
      opacity: 0.95,
      dashArray: state.polygonMode ? "6 6" : null,
      interactive: false,
    }).addTo(state.polygonPreviewLayer);
  }

  if (previewPoints.length >= 3) {
    L.polygon(previewPoints, {
      color: "#f59e0b",
      weight: 2,
      opacity: 0.85,
      fillColor: "#f59e0b",
      fillOpacity: 0.10,
      interactive: false,
    }).addTo(state.polygonPreviewLayer);
  }

  points.forEach((pt, idx) => {
    L.circleMarker(pt, {
      radius: 5,
      color: "#ffffff",
      weight: 2,
      fillColor: "#f59e0b",
      fillOpacity: 0.95,
      interactive: false,
    }).bindTooltip(`P${idx + 1}`, { permanent: false }).addTo(state.polygonPreviewLayer);
  });

  const finishButton = $("finish-polygon-button");
  if (finishButton) finishButton.disabled = !state.polygonMode || points.length < 3;
  renderDrawHelpBadge();
}

function handleMissionMapMouseMove(event) {
  if (!OCTOPUS.missionMap.polygonMode) return;
  renderPolygonPreview(event.latlng);
}

function handleMissionMapClick(event) {
  if (OCTOPUS.missionMap.placeEveMode) {
    placeEveAt(event.latlng.lat, event.latlng.lng);
    return;
  }
  if (!OCTOPUS.missionMap.polygonMode) return;
  OCTOPUS.missionMap.polygonPoints.push(event.latlng);
  const count = OCTOPUS.missionMap.polygonPoints.length;
  addTimeline(`Search polygon point ${count} set.`, "info");
  renderPolygonPreview();
}

function setEvePlacementMode(enabled) {
  const state = OCTOPUS.missionMap;
  state.placeEveMode = enabled;
  if (enabled && state.polygonMode) setPolygonMode(false);
  const btn = $("set-eve-button");
  const mapEl = $("mission-map");
  if (btn) btn.classList.toggle("area-active", enabled);
  if (mapEl) mapEl.classList.toggle("polygon-drawing", enabled);
  if (enabled) addTimeline("Set Eve enabled. Click on the mission map to place the Eve drone.", "info");
}

function placeEveAt(lat, lon) {
  OCTOPUS.manualEve = { lat, lon, ts: new Date().toISOString() };
  localStorage.setItem("octopusManualEve", JSON.stringify(OCTOPUS.manualEve));
  setEvePlacementMode(false);
  // No local x/y to report here: placing Eve moves the origin, it does not move
  // her within the frame. She is local (0, 0) before and after.
  addTimeline(`Eve placed manually at ${lat.toFixed(7)}, ${lon.toFixed(7)} — this is now local (0, 0).`, "success");
  syncEveFakeGps();
  renderAll();
}

function clearManualEve() {
  setEvePlacementMode(false);
  if (!OCTOPUS.manualEve) {
    addTimeline("No manual Eve placement to clear.", "info");
    return;
  }
  OCTOPUS.manualEve = null;
  localStorage.removeItem("octopusManualEve");
  addTimeline("Manual Eve placement cleared. Using live/fallback position.", "warning");
  syncEveFakeGps();
  renderAll();
}

// --- Eve fake GPS start coordinate -> backend -> ROS ---
// Eve's placement only exists in this browser, but the collector robot needs it:
// it starts at the same spot, so this coordinate is the datum every trash GPS goal
// is relative to. Pushing it to the backend is what lets
// /octopus/fake_eve_gps_start follow the marker when it is dragged.
const EVE_FAKE_GPS_HEARTBEAT_SEC = 10;

async function syncEveFakeGps(force = false) {
  const pose = eveGeoPose();
  if (!pose) return;

  const last = OCTOPUS.lastEveFakeGps;
  const moved =
    !last ||
    Math.abs(last.lat - pose.lat) > 1e-9 ||
    Math.abs(last.lon - pose.lon) > 1e-9 ||
    Math.abs(last.yawDeg - pose.yawDeg) > 1e-6;
  // Resend periodically even when nothing moved, so a backend that was restarted
  // picks the coordinate up again without the operator touching the marker.
  const stale = !last || (Date.now() - last.postedAt) / 1000 > EVE_FAKE_GPS_HEARTBEAT_SEC;
  if (!force && !moved && !stale) return;

  const payload = {
    lat: pose.lat,
    lon: pose.lon,
    // Eve's own coordinate in the local map frame. Constant by definition - this
    // coordinate is what the frame is anchored on - and sent along so a consumer
    // reading the datum does not have to infer it.
    x: 0.0,
    y: 0.0,
    frame_id: "map",
    yaw_deg: pose.yawDeg,
    manual: Boolean(pose.manual),
    demo: Boolean(pose.demo),
    source: "dashboard",
    ts: new Date().toISOString(),
  };

  try {
    const response = await fetch("/api/eve/fake_gps", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    OCTOPUS.lastEveFakeGps = { lat: pose.lat, lon: pose.lon, yawDeg: pose.yawDeg, postedAt: Date.now() };
    if (moved && last) {
      addTimeline(`Eve start coordinate sent to ROS: ${pose.lat.toFixed(7)}, ${pose.lon.toFixed(7)}`, "info");
    }
  } catch (error) {
    console.warn("Eve fake GPS sync failed", error);
  }
}

function initEveFakeGpsSync() {
  syncEveFakeGps(true);
  setInterval(() => syncEveFakeGps(), 2000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initEveFakeGpsSync);
} else {
  initEveFakeGpsSync();
}

function finishPolygonArea() {
  const points = OCTOPUS.missionMap.polygonPoints || [];
  if (points.length < 3 || typeof L === "undefined") {
    addTimeline("Polygon needs at least 3 points before it can define a search area.", "warning");
    renderPolygonPreview();
    return;
  }

  const bounds = L.latLngBounds(points);
  const polygon = points.map((p) => ({ lat: p.lat, lon: p.lng }));
  OCTOPUS.missionMap.polygonPoints = [];
  setPolygonMode(false);
  boundsToMissionArea(bounds, "drawn polygon", polygon);
}

function startPolygonArea() {
  if (OCTOPUS.missionMap.placeEveMode) setEvePlacementMode(false);
  OCTOPUS.missionMap.polygonPoints = [];
  setPolygonMode(true);
  addTimeline("Draw polygon enabled. Click search-area points on the mission map, then press Finish.", "info");
}

function clearMissionArea() {
  OCTOPUS.missionArea = null;
  localStorage.removeItem("octopusMissionArea");
  clearPolygonPreview(true);
  OCTOPUS.missionMap.hasFit = false;
  OCTOPUS.gridView = { scale: 1, offsetX: 0, offsetY: 0, isPanning: false, lastX: 0, lastY: 0, moved: false };
  addTimeline("Custom search area cleared. Using default/demo grid area.", "warning");
  renderAll();
}

// The legend states what is on the map right now, so every row is keyed and only
// appears once a renderer reports having drawn that thing this pass. Two sets, one
// per layer, because each layer is also redrawn on its own: the grid overlay from
// the layer/display controls, the markers from the camera poll. Each renderer
// resets its own set exactly where it clears its own layer.
const MISSION_LEGEND_ROWS = [
  { key: "ground", mark: "swatch ground", label: "ground" },
  { key: "water", mark: "swatch water", label: "water" },
  { key: "trash", mark: "swatch trash", label: "trash cell" },
  { key: "obstacle", mark: "swatch obstacle", label: "obstacle" },
  { key: "building", mark: "swatch building", label: "building" },
  { key: "coverage", mark: "swatch coverage", label: "OSM prior" },
  { key: "footprint", mark: "line footprint", label: "camera view" },
  { key: "detection", mark: "dot detection", label: "trash seen" },
  { key: "drone", mark: "dot drone", label: "Eve" },
  { key: "robot", mark: "dot robot", label: "robot" },
  { key: "home", mark: "dot home", label: "home" },
  { key: "task", mark: "dot task", label: "task" },
];

// overviewClass() -> legend row. Water and buildings share a row whether they were
// scanned or come from an OSM prior; the remaining priors are the soft overlay.
const MISSION_LEGEND_CELL_KEYS = {
  detected_ground: "ground",
  detected_water: "water",
  prior_water: "water",
  detected_trash: "trash",
  detected_obstacle: "obstacle",
  prior_building: "building",
  prior_ground: "coverage",
  prior_unknown: "coverage",
};

function noteMissionLegend(kind, key) {
  if (!key) return;
  const set = kind === "cell"
    ? OCTOPUS.missionMap.legendCellKeys
    : OCTOPUS.missionMap.legendMarkerKeys;
  if (set) set.add(key);
}

function renderMissionLegend() {
  const state = OCTOPUS.missionMap;
  const div = state.legendEl;
  if (!div) return;

  const present = new Set([
    ...(state.legendCellKeys || []),
    ...(state.legendMarkerKeys || []),
  ]);
  const rows = MISSION_LEGEND_ROWS.filter((row) => present.has(row.key));
  const collapsed = Boolean(OCTOPUS.legendCollapsed);

  // The markers are rebuilt on every camera poll, so skip the DOM work unless the
  // set of rows or the collapsed state actually changed.
  const signature = `${collapsed ? "c" : "o"}:${rows.map((r) => r.key).join(",")}`;
  if (state.legendSignature === signature) return;
  state.legendSignature = signature;

  div.classList.toggle("is-collapsed", collapsed);
  const body = rows.length
    ? rows.map((row) => {
        const [type, cls] = row.mark.split(" ");
        return `<div class="legend-row"><span class="legend-${type} ${cls}"></span>${escapeHtml(row.label)}</div>`;
      }).join("")
    : `<div class="legend-row legend-empty">nothing drawn yet</div>`;

  div.innerHTML =
    `<button type="button" class="legend-toggle" aria-expanded="${collapsed ? "false" : "true"}">` +
    `<span>Map</span><span class="legend-chevron" aria-hidden="true">${collapsed ? "\u25B8" : "\u25BE"}</span>` +
    `</button><div class="legend-body">${body}</div>`;
}

function toggleMissionLegend() {
  OCTOPUS.legendCollapsed = !OCTOPUS.legendCollapsed;
  localStorage.setItem("octopusLegendCollapsed", OCTOPUS.legendCollapsed ? "1" : "0");
  OCTOPUS.missionMap.legendSignature = null;
  renderMissionLegend();
}

function renderMissionAreaOverlay() {
  const state = OCTOPUS.missionMap;
  if (!state.map || !state.areaLayer) return;
  state.areaLayer.clearLayers();

  // The grid extent used to be outlined as a white dashed rectangle from the
  // datum to (width_m, height_m). It still bounds which cells get drawn, but as
  // an outline it was a big empty box hanging off Eve, so only the operator's
  // own mission-area polygon is drawn now.
  const polygon = OCTOPUS.missionArea?.polygon;
  if (Array.isArray(polygon) && polygon.length >= 3) {
    const latLngs = polygon.map((p) => [p.lat, p.lon]);
    L.polygon(latLngs, {
      color: "#f59e0b",
      weight: 3,
      opacity: 0.95,
      fillColor: "#f59e0b",
      fillOpacity: 0.12,
      interactive: false,
    }).addTo(state.areaLayer);
  }
}


function getMergedGridData(mapData = OCTOPUS.latest.globalMap || {}) {
  const baseCells = mapData?.cells || {};
  const priorCells = OCTOPUS.osmPriors?.cells || {};
  const mergedCells = { ...priorCells };
  Object.entries(baseCells).forEach(([key, cell]) => {
    mergedCells[key] = { ...(mergedCells[key] || {}), ...cell };
  });
  return { ...(mapData || {}), cells: mergedCells };
}

function realObstacleProbability(cell) {
  return safeNumber(cell?.obstacle_probability, 0);
}

function priorObstacleProbability(cell) {
  return safeNumber(cell?.semantic_obstacle_probability, 0);
}

function combinedObstacleProbability(cell) {
  return Math.max(realObstacleProbability(cell), priorObstacleProbability(cell));
}

function semanticClass(cell) {
  return String(cell?.semantic_class || cell?.land_class || "unknown").toLowerCase();
}

function isScannedCell(cell) {
  return safeNumber(cell?.coverage, 0) > 0.001 || safeNumber(cell?.confidence, 0) > 0.001 || Boolean(cell?.source_id);
}

function overviewClass(cell) {
  const scanned = isScannedCell(cell);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);
  const sem = semanticClass(cell);

  if (scanned) {
    if (trash >= 0.30) return "detected_trash";
    if (realObstacle >= 0.35) return "detected_obstacle";
    if (sem === "water") return "detected_water";
    return "detected_ground";
  }

  if (sem === "water") return "prior_water";
  if (sem === "building") return "prior_building";
  if (sem === "ground" || sem === "land" || sem === "vegetation" || sem === "park") return "prior_ground";
  return "prior_unknown";
}

function cellDisplayCategory(cell) {
  const cls = overviewClass(cell);
  if (cls.includes("trash")) return "trash";
  if (cls.includes("obstacle") || cls.includes("building")) return "obstacles";
  if (cls.includes("water")) return "water";
  if (cls.includes("ground")) return "ground";
  return "unknown";
}

function cellPassesDisplayFilters(cell) {
  const scanned = isScannedCell(cell);
  const display = OCTOPUS.gridDisplay || GRID_DISPLAY_DEFAULTS;
  if (scanned && !display.detections) return false;
  if (!scanned && !display.priors) return false;
  const category = cellDisplayCategory(cell);
  if (category === "trash") return display.trash;
  if (category === "obstacles") return display.obstacles;
  if (category === "water") return display.water;
  if (category === "ground") return display.ground;
  if (category === "unknown") return display.unknown;
  return true;
}

function saveGridDisplaySettings() {
  localStorage.setItem("octopusGridDisplay", JSON.stringify(OCTOPUS.gridDisplay));
}

function syncGridDisplayControls() {
  document.querySelectorAll("[data-grid-display]").forEach((input) => {
    const key = input.dataset.gridDisplay;
    input.checked = Boolean(OCTOPUS.gridDisplay?.[key]);
  });
}

function overviewBaseColor(cell) {
  const cls = overviewClass(cell);
  const semConf = clamp(safeNumber(cell?.semantic_confidence, 0.25), 0.10, 0.85);
  const coverage = clamp(safeNumber(cell?.coverage, 0), 0, 1);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);

  // Bright colors = observed/scanned grid evidence.
  if (cls === "detected_trash") return rgbaToken("trash", 0.62 + 0.35 * trash);
  if (cls === "detected_obstacle") return rgbaToken("detectedObstacle", 0.58 + 0.35 * realObstacle);
  if (cls === "detected_water") return rgbaToken("detectedWater", 0.60 + 0.28 * Math.max(coverage, semConf));
  if (cls === "detected_ground") return rgbaToken("detectedGround", 0.42 + 0.35 * Math.max(coverage, 0.35));

  // Soft colors = semantic priors from OSM. They should never visually overpower real evidence.
  if (cls === "prior_water") return rgbaToken("priorWater", 0.18 + semConf * 0.24);
  if (cls === "prior_building") return rgbaToken("priorBuilding", 0.18 + semConf * 0.25);
  if (cls === "prior_ground") return rgbaToken("priorGround", 0.16 + semConf * 0.22);
  return rgbaToken("unknown", 0.18);
}

function overviewLeafletColor(cell) {
  const cls = overviewClass(cell);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);

  if (cls === "detected_trash") return { color: hexToken("trash"), fillColor: hexToken("trash"), fillOpacity: 0.72 + 0.20 * trash };
  if (cls === "detected_obstacle") return { color: hexToken("detectedObstacle"), fillColor: hexToken("detectedObstacle"), fillOpacity: 0.64 + 0.22 * realObstacle };
  if (cls === "detected_water") return { color: hexToken("detectedWater"), fillColor: hexToken("detectedWater"), fillOpacity: 0.58 };
  if (cls === "detected_ground") return { color: hexToken("detectedGround"), fillColor: hexToken("detectedGround"), fillOpacity: 0.48 };

  if (cls === "prior_water") return { color: hexToken("priorWater"), fillColor: hexToken("priorWater"), fillOpacity: 0.28 };
  if (cls === "prior_building") return { color: hexToken("priorBuilding"), fillColor: hexToken("priorBuilding"), fillOpacity: 0.28 };
  if (cls === "prior_ground") return { color: hexToken("priorGround"), fillColor: hexToken("priorGround"), fillOpacity: 0.20 };
  return { color: hexToken("unknown"), fillColor: hexToken("unknown"), fillOpacity: 0.12 };
}

function drawOverviewCell(ctx, cell, x, y, w, h) {
  const dpr = window.devicePixelRatio || 1;
  const scanned = isScannedCell(cell);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);
  const priorObstacle = clamp(priorObstacleProbability(cell), 0, 1);
  const sem = semanticClass(cell);

  ctx.fillStyle = overviewBaseColor(cell);
  ctx.fillRect(x, y, w, h);

  // Building priors can be shown as faint obstacle hatching. Water is not treated as an obstacle here.
  const shouldHatchPrior = !scanned && sem === "building" && priorObstacle >= 0.35;
  const shouldHatchDetected = scanned && realObstacle >= 0.35;
  if (shouldHatchPrior || shouldHatchDetected) {
    ctx.save();
    const strength = shouldHatchDetected ? realObstacle : priorObstacle;
    ctx.strokeStyle = shouldHatchDetected
      ? rgbaToken("detectedObstacle", 0.50 + 0.45 * strength)
      : rgbaToken("detectedObstacle", 0.18 + 0.22 * strength);
    ctx.lineWidth = Math.max(1, dpr);
    ctx.beginPath();
    const step = Math.max(4 * dpr, Math.min(w, h) * 0.55);
    for (let i = -h; i < w + h; i += step) {
      ctx.moveTo(x + i, y + h);
      ctx.lineTo(x + i + h, y);
    }
    ctx.stroke();
    ctx.restore();
  }

  if (trash >= 0.30 && Math.max(w, h) >= 3) {
    ctx.beginPath();
    ctx.arc(x + w / 2, y + h / 2, Math.max(1.5 * dpr, Math.min(w, h) * (0.18 + 0.20 * trash)), 0, Math.PI * 2);
    ctx.fillStyle = rgbaToken("trash", 0.58 + 0.40 * trash);
    ctx.fill();
  }
}

function clearOsmPriors(announce = true) {
  OCTOPUS.osmPriors = null;
  localStorage.removeItem("octopusOsmPriors");
  updateOsmButton("Load OSM priors");
  if (announce) addTimeline("OSM semantic priors cleared.", "warning");
}

function updateOsmButton(text, disabled = false) {
  const button = $("load-osm-priors-button");
  if (!button) return;
  button.textContent = text;
  button.disabled = disabled;
}

function classifyOsmElement(element) {
  const tags = element.tags || {};
  if (tags.building) {
    return { semantic_class: "building", semantic_obstacle_probability: 0.70, semantic_confidence: 0.60 };
  }
  if (tags.natural === "water" || tags.water || tags.waterway || tags.landuse === "reservoir" || tags.landuse === "basin") {
    return { semantic_class: "water", semantic_obstacle_probability: 0.00, semantic_confidence: 0.65 };
  }
  if (["forest", "grass", "meadow", "recreation_ground", "village_green"].includes(tags.landuse) || ["wood", "scrub", "grassland", "beach", "heath"].includes(tags.natural) || ["park", "garden"].includes(tags.leisure)) {
    return { semantic_class: "ground", semantic_obstacle_probability: 0.10, semantic_confidence: 0.45 };
  }
  return null;
}

function pointInPolygon(point, polygon) {
  const x = point.x;
  const y = point.y;
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const xi = polygon[i].x, yi = polygon[i].y;
    const xj = polygon[j].x, yj = polygon[j].y;
    const intersect = ((yi > y) !== (yj > y)) && (x < ((xj - xi) * (y - yi)) / ((yj - yi) || 1e-9) + xi);
    if (intersect) inside = !inside;
  }
  return inside;
}

function polygonLocalBounds(poly) {
  const xs = poly.map((p) => p.x);
  const ys = poly.map((p) => p.y);
  return {
    minX: Math.min(...xs), maxX: Math.max(...xs),
    minY: Math.min(...ys), maxY: Math.max(...ys),
  };
}

function osmGeometryToLocalPolygon(element) {
  if (!Array.isArray(element.geometry) || element.geometry.length < 3) return null;
  return element.geometry.map((p) => latLngToLocal(p.lat, p.lon));
}

function overpassQueryForMissionArea(meta) {
  const sw = localToLatLng(0, 0);
  const ne = localToLatLng(meta.width_m, meta.height_m);
  const south = Math.min(sw[0], ne[0]);
  const west = Math.min(sw[1], ne[1]);
  const north = Math.max(sw[0], ne[0]);
  const east = Math.max(sw[1], ne[1]);
  return `
[out:json][timeout:25];
(
  way["building"](${south},${west},${north},${east});
  way["natural"="water"](${south},${west},${north},${east});
  way["water"](${south},${west},${north},${east});
  way["waterway"](${south},${west},${north},${east});
  way["landuse"](${south},${west},${north},${east});
  way["natural"](${south},${west},${north},${east});
  way["leisure"](${south},${west},${north},${east});
);
out tags geom;`;
}

function buildOsmPriorCells(elements, meta) {
  const cells = {};
  const resolution = meta.resolution;

  // Default: inside the planned search grid is assumed ground with low confidence.
  for (let row = 0; row < meta.rows; row++) {
    for (let col = 0; col < meta.cols; col++) {
      const key = `${row},${col}`;
      cells[key] = {
        row, col,
        x: (col + 0.5) * resolution,
        y: (row + 0.5) * resolution,
        cell_size_m: resolution,
        coverage: 0,
        trash_probability: 0,
        obstacle_probability: 0,
        confidence: 0,
        semantic_class: "ground",
        semantic_obstacle_probability: 0.05,
        semantic_confidence: 0.25,
        semantic_source: "default_ground_prior",
      };
    }
  }

  let featureCount = 0;
  for (const element of elements || []) {
    const classification = classifyOsmElement(element);
    if (!classification) continue;
    const polygon = osmGeometryToLocalPolygon(element);
    if (!polygon) continue;
    featureCount += 1;
    const b = polygonLocalBounds(polygon);
    const minCol = clamp(Math.floor(b.minX / resolution) - 1, 0, meta.cols - 1);
    const maxCol = clamp(Math.ceil(b.maxX / resolution) + 1, 0, meta.cols - 1);
    const minRow = clamp(Math.floor(b.minY / resolution) - 1, 0, meta.rows - 1);
    const maxRow = clamp(Math.ceil(b.maxY / resolution) + 1, 0, meta.rows - 1);

    for (let row = minRow; row <= maxRow; row++) {
      for (let col = minCol; col <= maxCol; col++) {
        const x = (col + 0.5) * resolution;
        const y = (row + 0.5) * resolution;
        if (!pointInPolygon({ x, y }, polygon)) continue;
        const key = `${row},${col}`;
        const previous = cells[key] || {};
        cells[key] = {
          ...previous,
          row, col, x, y,
          cell_size_m: resolution,
          semantic_class: classification.semantic_class,
          semantic_obstacle_probability: Math.max(safeNumber(previous.semantic_obstacle_probability, 0), classification.semantic_obstacle_probability),
          semantic_confidence: Math.max(safeNumber(previous.semantic_confidence, 0), classification.semantic_confidence),
          semantic_source: "osm_overpass",
          osm_id: element.id,
        };
      }
    }
  }

  return { cells, featureCount };
}

async function loadOsmPriors() {
  const mapData = OCTOPUS.latest.globalMap || {};
  const meta = getActiveGridMeta(mapData);
  const total = meta.rows * meta.cols;
  if (total > 25000) {
    addTimeline("OSM prior loading skipped: grid too large. Increase grid cell size or define a smaller polygon.", "warning");
    return;
  }

  updateOsmButton("Loading OSM...", true);
  try {
    const query = overpassQueryForMissionArea(meta);
    const response = await fetch("https://overpass-api.de/api/interpreter", {
      method: "POST",
      headers: { "Content-Type": "application/x-www-form-urlencoded; charset=UTF-8" },
      body: `data=${encodeURIComponent(query)}`,
    });
    if (!response.ok) throw new Error(`Overpass returned HTTP ${response.status}`);
    const data = await response.json();
    const built = buildOsmPriorCells(data.elements || [], meta);
    OCTOPUS.osmPriors = {
      status: "ok",
      source: "overpass",
      loaded_at: new Date().toISOString(),
      rows: meta.rows,
      cols: meta.cols,
      resolution: meta.resolution,
      features: built.featureCount,
      cells: built.cells,
    };
    localStorage.setItem("octopusOsmPriors", JSON.stringify(OCTOPUS.osmPriors));
    updateOsmButton(`OSM priors: ${built.featureCount}`, false);
    addTimeline(`OSM priors loaded: ${built.featureCount} features, ${Object.keys(built.cells).length} semantic grid cells.`, "success");
    renderAll();
  } catch (error) {
    console.error("OSM prior loading failed", error);
    updateOsmButton("Load OSM priors", false);
    addTimeline(`OSM prior loading failed: ${error.message}`, "error");
  }
}

function leafletColorForCell(cell, layer) {
  if (layer === "overview") return overviewLeafletColor(cell);
  const css = colorForCell(cell, layer);
  const match = css.match(/rgba\((\d+),\s*(\d+),\s*(\d+),\s*([0-9.]+)\)/);
  if (!match) return { color: "#64a0c8", fillOpacity: 0.25 };
  return {
    color: `rgb(${match[1]}, ${match[2]}, ${match[3]})`,
    fillColor: `rgb(${match[1]}, ${match[2]}, ${match[3]})`,
    fillOpacity: clamp(Number(match[4]) * 0.58, 0.08, 0.62),
  };
}

function getSourceCellSize(mapData, cell) {
  return clamp(
    safeNumber(cell?.resolution ?? cell?.cell_size ?? cell?.cell_size_m ?? mapData?.resolution, 0.10),
    0.01,
    5.0,
  );
}

function cellMetricBounds(cell, mapData = OCTOPUS.latest.globalMap || {}) {
  const sourceSize = getSourceCellSize(mapData, cell);
  const row = safeNumber(cell?.row, NaN);
  const col = safeNumber(cell?.col, NaN);
  const centerX = Number.isFinite(safeNumber(cell?.x, NaN)) ? safeNumber(cell.x) : (Number.isFinite(col) ? (col + 0.5) * sourceSize : 0);
  const centerY = Number.isFinite(safeNumber(cell?.y, NaN)) ? safeNumber(cell.y) : (Number.isFinite(row) ? (row + 0.5) * sourceSize : 0);
  return {
    x0: centerX - sourceSize / 2,
    y0: centerY - sourceSize / 2,
    x1: centerX + sourceSize / 2,
    y1: centerY + sourceSize / 2,
    centerX,
    centerY,
    sourceSize,
  };
}

function metricBoundsIntersectsArea(bounds, meta) {
  return bounds.x1 >= 0 && bounds.y1 >= 0 && bounds.x0 <= meta.width_m && bounds.y0 <= meta.height_m;
}

function renderMissionGridOverlay() {
  const state = OCTOPUS.missionMap;
  if (!state.map || !state.gridLayer) return;

  const mode = $("mission-map-mode")?.value || "grid_overlay";
  const layerName = $("grid-layer-select")?.value || "overview";
  const mapData = getMergedGridData(OCTOPUS.latest.globalMap || {});

  state.gridLayer.clearLayers();
  state.legendCellKeys = new Set();
  renderMissionAreaOverlay();

  if (!mapData) {
    renderMissionLegend();
    return;
  }

  const meta = getActiveGridMeta(mapData);
  const cells = mapData.cells || {};

  Object.values(cells).forEach((cell) => {
    if (!cellPassesDisplayFilters(cell)) return;
    if (layerName === "osm_priors" && isScannedCell(cell)) return;
    const row = safeNumber(cell.row, NaN);
    const col = safeNumber(cell.col, NaN);
    if (!Number.isFinite(row) || !Number.isFinite(col)) return;

    const mb = cellMetricBounds(cell, mapData);
    if (!metricBoundsIntersectsArea(mb, meta)) return;

    const sw = localToLatLng(Math.max(0, mb.x0), Math.max(0, mb.y0));
    const ne = localToLatLng(Math.min(meta.width_m, mb.x1), Math.min(meta.height_m, mb.y1));
    const style = leafletColorForCell(cell, layerName);
    const rect = L.rectangle([sw, ne], {
      color: style.color,
      weight: 0.35,
      opacity: 0.65,
      fillColor: style.fillColor,
      fillOpacity: mode === "grid_only" ? Math.min(0.88, style.fillOpacity + 0.20) : clamp(style.fillOpacity * 1.25, 0.16, 0.82),
      interactive: true,
    });
    rect.on("click", () => {
      OCTOPUS.selectedCellKey = `${row},${col}`;
      OCTOPUS.selected = { type: "grid_cell", cell };
      renderInspector();
      drawGridMap(OCTOPUS.latest.globalMap);
    });
    noteMissionLegend("cell", MISSION_LEGEND_CELL_KEYS[overviewClass(cell)]);
    rect.addTo(state.gridLayer);
  });

  renderMissionLegend();
}

// From this zoom on, the basemap is muted. 22 is deep in: the ~4.45 m camera
// footprint is 22 px at zoom 19 and ~179 px at 22, so this is the point where the
// screen is about the mission itself rather than the neighbourhood. Everything
// looser than that keeps the map in full colour, including the range you place Eve
// against and the range OSM still has real detail for (maxNativeZoom is 19).
const BASEMAP_MUTE_FROM_ZOOM = 22;

function syncBasemapMuting() {
  const state = OCTOPUS.missionMap;
  if (!state.map) return;
  const muted = state.map.getZoom() >= BASEMAP_MUTE_FROM_ZOOM;
  state.map.getContainer().classList.toggle("basemap-muted", muted);
}

function renderMissionMap() {
  initMissionMap();

  const state = OCTOPUS.missionMap;
  if (!state.map) return;

  const mode = $("mission-map-mode")?.value || "grid_overlay";
  if (mode === "grid_only") {
    if (state.map.hasLayer(state.baseLayer)) state.map.removeLayer(state.baseLayer);
    state.map.getContainer().classList.add("grid-only-map");
  } else {
    if (!state.map.hasLayer(state.baseLayer)) state.baseLayer.addTo(state.map);
    state.map.getContainer().classList.remove("grid-only-map");
  }

  syncBasemapMuting();

  state.markerLayer.clearLayers();
  state.legendMarkerKeys = new Set();
  renderMissionGridOverlay();

  const bounds = [];
  const tasks = OCTOPUS.latest.tasks || [];

  // Fleet markers use configured robot roles first, then live backend data when available.
  getFleetSnapshot().forEach((robot) => {
    const lat = safeNumber(robot.location.lat, NaN);
    const lon = safeNumber(robot.location.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon) || robot.hasPosition === false) return;
    // A robot that has never reported gets NO marker. `is_demo` means the
    // position came from ROBOT_FLEET_PROFILES[key].fallback - a number typed
    // into this file, not a measurement - and drawing it puts a robot-shaped
    // icon on the map at a place no robot has ever been. That is the same
    // mistake the no_datum rule already refuses further down: a live-looking
    // marker on a made-up spot. SharX and Robby were standing on the mission
    // map of every demo this way.
    // Eve is exempt on purpose: her marker is the operator's own handle for
    // placing the datum, and it has to exist before it can be dragged.
    if (robot.location?.is_demo && !robot.location?.is_fixed && robot.key !== "eve") return;
    const icon = missionRobotIcon(robot);
    const style = deviceStyle(robot.type === "water" ? "boat" : robot.type, robot.state);
    const isEve = robot.key === "eve";
    const marker = icon
      ? L.marker([lat, lon], { icon, draggable: isEve })
      : L.circleMarker([lat, lon], { radius: 8, color: style.color, weight: 2, fillColor: style.fillColor, fillOpacity: 0.92 });

    const dragHint = isEve ? "<br><em>drag to move · manually placed</em>" : "";
    marker.bindTooltip(`${robot.icon} ${robot.name} · ${robot.role}<br>${robot.state || "unknown"} · ${robot.capability}${dragHint}`, {
      permanent: false,
      direction: "top",
    });

    marker.on("click", () => {
      OCTOPUS.selected = { type: "fleet", id: robot.name, device_type: robot.type, robot };
      renderInspector();
    });

    if (isEve) {
      marker.on("dragend", (dragEvent) => {
        const ll = dragEvent.target.getLatLng();
        placeEveAt(ll.lat, ll.lng);
      });
    }

    noteMissionLegend("marker", isEve ? "drone" : "robot");
    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

  // Gated on the same toggle the grid renderers use. The mission map used to
  // draw this unconditionally, which is why unticking "home station" still left
  // a marker sitting on the drone.
  if (OCTOPUS.gridDisplay?.home) {
    const home = homeStation();
    const homeMarker = L.circleMarker([home.lat, home.lon], {
      radius: 8,
      color: "#ffffff",
      weight: 2,
      fillColor: "#a78bfa",
      fillOpacity: 0.92,
    }).bindTooltip("⌂ Home station", { direction: "top" });
    homeMarker.on("click", () => {
      OCTOPUS.selected = { type: "home", home };
      renderInspector();
    });
    noteMissionLegend("marker", "home");
    homeMarker.addTo(state.markerLayer);
    bounds.push([home.lat, home.lon]);
  }

  tasks.forEach((task) => {
    const lat = safeNumber(task.lat, NaN);
    const lon = safeNumber(task.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const marker = L.circleMarker([lat, lon], {
      radius: 6,
      color: "#ffffff",
      weight: 1.5,
      fillColor: "#e37222",
      fillOpacity: 0.86,
    }).bindTooltip(`Task #${task.id} · ${task.status || "unknown"}`, { direction: "top" });
    marker.on("click", () => {
      const local = latLngToLocal(lat, lon);
      OCTOPUS.selected = { type: "task", task, local };
      renderInspector();
    });
    noteMissionLegend("marker", "task");
    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

  // Live camera detections projected onto the map from Eve's position + footprint,
  // plus the active camera grid (local or GPS) shared with the camera overlay.
  const footprintDrawn = renderProjectedDetections(state, bounds);
  renderCameraGridOnMap(state, footprintDrawn);

  const meta = getActiveGridMeta(OCTOPUS.latest.globalMap);
  bounds.push(localToLatLng(0, 0));
  bounds.push(localToLatLng(meta.width_m, meta.height_m));

  renderMissionLegend();

  if (!state.hasFit && bounds.length > 0) {
    state.map.fitBounds(bounds, { padding: [28, 28], maxZoom: 20 });
    state.hasFit = true;
  }
}

function fitMissionMap() {
  const state = OCTOPUS.missionMap;
  if (!state.map) return;
  state.hasFit = false;
  renderMissionMap();
}

// Project the live camera detections onto world/local coordinates using Eve's
// position, heading and the camera ground footprint. Assumes a downward-looking
// camera; image-top = drone forward. Heading (yaw) defaults to 0 = facing north.
function projectDetectionsToMap() {
  const eve = getFleetSnapshot().find((r) => r.key === "eve");
  if (!eve || eve.demo || !Number.isFinite(safeNumber(eve.local?.x, NaN))) {
    return { eve: null, corners: null, points: [] };
  }
  const fp = octopusComputeCameraFootprintMeta().footprint;
  const widthM = fp.left_m + fp.right_m;
  const heightM = fp.top_m + fp.bottom_m;
  const yawDeg = safeNumber(OCTOPUS.eveYawDeg, 0);
  const th = (yawDeg * Math.PI) / 180;
  const cos = Math.cos(th);
  const sin = Math.sin(th);
  const ex = eve.local.x;
  const ey = eve.local.y;

  // camera-frame offset (right, forward) -> world local (east=x, north=y), rotated by heading.
  const toWorld = (offRight, offFwd) => {
    const east = offRight * cos + offFwd * sin;
    const north = -offRight * sin + offFwd * cos;
    return localToLatLng(ex + east, ey + north);
  };

  const corners = [
    toWorld(-fp.left_m, -fp.bottom_m),
    toWorld(fp.right_m, -fp.bottom_m),
    toWorld(fp.right_m, fp.top_m),
    toWorld(-fp.left_m, fp.top_m),
  ];
  const evePos = toWorld(0, 0);
  const headingTip = toWorld(0, fp.top_m); // middle of the forward footprint edge

  const points = cameraFeedDetections()
    .map((det) => {
      const u = clamp(safeNumber(det.u, NaN), 0, 1);
      const v = clamp(safeNumber(det.v, NaN), 0, 1);
      if (!Number.isFinite(u) || !Number.isFinite(v)) return null;
      const offRight = u * widthM - fp.left_m;
      const offFwd = v * heightM - fp.bottom_m;
      const [lat, lon] = toWorld(offRight, offFwd);
      return { det, lat, lon };
    })
    .filter(Boolean);

  return { eve, corners, points, evePos, headingTip, yawDeg };
}

// Returns true when the camera footprint outline was drawn, so the GPS grid does
// not draw a second one on top.
function renderProjectedDetections(state, bounds) {
  if (OCTOPUS.projectDetections === false) return false;
  const proj = projectDetectionsToMap();
  if (!proj.eve) return false;
  const gpsGrid = isGpsGridActive() ? octopusGpsGridModel() : null;
  const localLayout = isLocalGridActive() ? localGridLayout() : null;

  // Camera footprint outline on the ground (what Eve currently sees).
  if (proj.corners) {
    const poly = L.polygon(proj.corners, {
      color: "#d4ff00",
      opacity: 0.9,
      weight: 1.5,
      dashArray: "5,5",
      fill: true,
      fillColor: "#d4ff00",
      fillOpacity: 0.05,
      interactive: false,
    });
    noteMissionLegend("marker", "footprint");
    poly.addTo(state.markerLayer);
    proj.corners.forEach((c) => bounds.push(c));
  }

  // Heading arrow from Eve toward the forward (image-top) direction.
  if (proj.evePos && proj.headingTip) {
    L.polyline([proj.evePos, proj.headingTip], {
      color: "#38bdf8",
      weight: 3,
      opacity: 0.95,
      interactive: false,
    }).addTo(state.markerLayer);
    L.circleMarker(proj.headingTip, {
      radius: 4,
      color: "#0b1220",
      weight: 1.5,
      fillColor: "#38bdf8",
      fillOpacity: 1,
      interactive: false,
    })
      .bindTooltip(`Eve heading ${Math.round(proj.yawDeg)}°`, { direction: "top" })
      .addTo(state.markerLayer);
  }

  // Projected trash detections as markers on the map.
  proj.points.forEach(({ det, lat, lon }) => {
    const conf = safeNumber(det.confidence, NaN);
    const confText = Number.isFinite(conf) ? ` ${conf.toFixed(2)}` : "";
    // With the GPS grid on, the tooltip reads off the coordinate and grid cell;
    // with the local grid on, the local cell name. Cell names can be switched off,
    // the GPS coordinate stays either way.
    const showNames = cellNamesEnabled();
    const cell = gpsGrid && showNames ? gpsGrid.cellFor(lat, lon) : null;
    const localName = localLayout && showNames ? activeCellNameForDetection(det, null, localLayout) : null;
    const geoText = gpsGrid
      ? `<br>${escapeHtml(gpsGrid.format(lat))}°N ${escapeHtml(gpsGrid.format(lon))}°E` +
        (cell ? ` · cell <strong>${escapeHtml(cell.name)}</strong>` : "")
      : localName
        ? `<br>cell <strong>${escapeHtml(localName)}</strong>`
        : "";
    const marker = L.circleMarker([lat, lon], {
      radius: 7,
      color: "#0b1220",
      weight: 2,
      fillColor: "#fb923c",
      fillOpacity: 0.95,
    }).bindTooltip(
      `🗑 ${escapeHtml(det.class_name || "rubbish")}${escapeHtml(confText)}${geoText}<br><em>projected from camera</em>`,
      { direction: "top" }
    );
    marker.on("click", () => {
      OCTOPUS.selected = { type: "projected_detection", detection: det, lat, lon, local: latLngToLocal(lat, lon) };
      renderInspector();
    });
    noteMissionLegend("marker", "detection");
    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

  return Boolean(proj.corners);
}

// The active camera grid on the mission map, drawn with the same cells and cell
// names as the camera overlay so both views can be read against each other.
// Both modes need the GPS model: it carries Eve's pose and the footprint
// projection that turns image coordinates into ground coordinates.
function renderCameraGridOnMap(state, footprintDrawn) {
  const mode = cameraGridMode();
  if (mode === "off" || typeof L === "undefined") return null;
  const grid = octopusGpsGridModel();
  if (!grid) return null;

  const layer = state.markerLayer;

  // Footprint outline, unless renderProjectedDetections already drew it.
  if (!footprintDrawn) {
    L.polygon(grid.corners.map((c) => [c.lat, c.lon]), {
      color: "#d4ff00",
      opacity: 0.85,
      weight: 1.5,
      dashArray: "5,5",
      fillColor: "#d4ff00",
      fillOpacity: 0.05,
      interactive: false,
    }).addTo(layer);
  }

  const addLabel = (lat, lon, text, extraClass, size, anchor) => {
    L.marker([lat, lon], {
      interactive: false,
      keyboard: false,
      icon: L.divIcon({
        className: `gps-grid-label ${extraClass}`,
        html: escapeHtml(text),
        iconSize: size,
        iconAnchor: anchor,
      }),
    }).addTo(layer);
  };

  if (mode === "gps") drawGpsGraticuleOnMap(state, layer, grid, addLabel);
  else drawLocalGridOnMap(state, layer, grid, addLabel);

  if (markCellsEnabled()) markCameraCellsOnMap(layer, grid, mode);

  return grid;
}

// Pixel size of one cell at the current zoom, used to thin out the labels.
function mapCellPixelSize(state, cornerA, cornerB) {
  if (!state.map) return { w: 999, h: 999 };
  const p0 = state.map.latLngToLayerPoint(cornerA);
  const p1 = state.map.latLngToLayerPoint(cornerB);
  return { w: Math.abs(p1.x - p0.x), h: Math.abs(p1.y - p0.y) };
}

function drawGpsGraticuleOnMap(state, layer, grid, addLabel) {
  const b = grid.bounds;
  const pix = mapCellPixelSize(state, [b.latMin, b.lonMin], [b.latMin + grid.step, b.lonMin + grid.step]);
  const lonEvery = Math.max(1, Math.ceil(64 / Math.max(pix.w, 1)));
  const latEvery = Math.max(1, Math.ceil(26 / Math.max(pix.h, 1)));

  grid.lonLines.forEach((lon, i) => {
    const major = i % lonEvery === 0;
    L.polyline([[b.latMin, lon], [b.latMax, lon]], {
      color: "#38bdf8",
      weight: major ? 1.2 : 0.6,
      opacity: major ? 0.8 : 0.35,
      interactive: false,
    }).addTo(layer);
    // Longitude values along the southern edge, centered under their line.
    if (major) addLabel(b.latMin, lon, `${grid.format(lon)}°E`, "axis-lon", [86, 12], [43, -2]);
  });

  grid.latLines.forEach((lat, j) => {
    const major = j % latEvery === 0;
    L.polyline([[lat, b.lonMin], [lat, b.lonMax]], {
      color: "#38bdf8",
      weight: major ? 1.2 : 0.6,
      opacity: major ? 0.8 : 0.35,
      interactive: false,
    }).addTo(layer);
    // Latitude values along the western edge, just inside the region.
    if (major) addLabel(lat, b.lonMin, `${grid.format(lat)}°N`, "axis-lat", [86, 12], [-3, 6]);
  });

  // Cell names, only for the cells over the camera footprint and only when
  // there is room for the text.
  if (cellNamesEnabled() && Math.min(pix.w, pix.h) >= 26) {
    for (let r = 0; r < grid.region.rows; r++) {
      for (let c = 0; c < grid.region.cols; c++) {
        const center = grid.cellCenter(c, r);
        if (center.lat < grid.bbox.latMin || center.lat > grid.bbox.latMax) continue;
        if (center.lon < grid.bbox.lonMin || center.lon > grid.bbox.lonMax) continue;
        addLabel(center.lat, center.lon, `${gpsColumnLetters(c)}${r + 1}`, "cell", [40, 12], [20, 6]);
      }
    }
  }
}

// The local grid on the map: the same cols/rows as over the camera image, but
// projected onto the ground, so it sits rotated inside the footprint outline.
// Lines of constant u/v stay straight under the projection, so two points each.
function drawLocalGridOnMap(state, layer, grid, addLabel) {
  const layout = localGridLayout();
  if (!layout) return;

  const at = (u, v) => {
    const geo = grid.uvToGeo(u, v);
    return [geo.lat, geo.lon];
  };
  const pix = mapCellPixelSize(state, at(0, 1), at(1 / layout.cols, 1 - 1 / layout.rows));

  for (let c = 0; c <= layout.cols; c++) {
    const u = c / layout.cols;
    L.polyline([at(u, 0), at(u, 1)], {
      color: "#d4ff00",
      weight: c === 0 || c === layout.cols ? 1.4 : 0.7,
      opacity: 0.6,
      interactive: false,
    }).addTo(layer);
  }
  for (let r = 0; r <= layout.rows; r++) {
    const v = 1 - r / layout.rows;
    L.polyline([at(0, v), at(1, v)], {
      color: "#d4ff00",
      weight: r === 0 || r === layout.rows ? 1.4 : 0.7,
      opacity: 0.6,
      interactive: false,
    }).addTo(layer);
  }

  if (cellNamesEnabled() && Math.min(pix.w, pix.h) >= 26) {
    for (let r = 0; r < layout.rows; r++) {
      for (let c = 0; c < layout.cols; c++) {
        const b = layout.cellUvBounds(c, r);
        const center = at((b.uMin + b.uMax) / 2, (b.vMin + b.vMax) / 2);
        addLabel(center[0], center[1], localCellName(c, r), "cell", [40, 12], [20, 6]);
      }
    }
  }
}

// Mark the active grid's cells that contain a detection, as filled polygons in
// the same orange the camera overlay uses.
function markCameraCellsOnMap(layer, grid, mode) {
  const style = {
    color: "#fb923c",
    weight: 2.5,
    opacity: 1,
    fillColor: "#fb923c",
    fillOpacity: 0.3,
    interactive: false,
  };
  const addLabel = (lat, lon, text) => {
    if (!cellNamesEnabled()) return;
    L.marker([lat, lon], {
      interactive: false,
      keyboard: false,
      icon: L.divIcon({
        className: "gps-grid-label cell marked",
        html: escapeHtml(text),
        iconSize: [40, 12],
        iconAnchor: [20, 6],
      }),
    }).addTo(layer);
  };

  if (mode === "gps") {
    gpsCellsForDetections(grid).forEach(({ col, row, name }) => {
      const b = grid.cellBounds(col, row);
      L.polygon([
        [b.latMin, b.lonMin],
        [b.latMin, b.lonMax],
        [b.latMax, b.lonMax],
        [b.latMax, b.lonMin],
      ], style).addTo(layer);
      addLabel((b.latMin + b.latMax) / 2, (b.lonMin + b.lonMax) / 2, name);
    });
    return;
  }

  const layout = localGridLayout();
  if (!layout) return;
  localCellsForDetections(layout).forEach(({ col, row, name }) => {
    const b = layout.cellUvBounds(col, row);
    const corners = [
      grid.uvToGeo(b.uMin, b.vMin),
      grid.uvToGeo(b.uMax, b.vMin),
      grid.uvToGeo(b.uMax, b.vMax),
      grid.uvToGeo(b.uMin, b.vMax),
    ];
    L.polygon(corners.map((c) => [c.lat, c.lon]), style).addTo(layer);
    const center = grid.uvToGeo((b.uMin + b.uMax) / 2, (b.vMin + b.vMax) / 2);
    addLabel(center.lat, center.lon, name);
  });
}

function valueForLayer(cell, layer) {
  if (!cell) return null;
  if (layer === "coverage") return safeNumber(cell.coverage, 0);
  if (layer === "trash_probability" || layer === "trash_focus") return safeNumber(cell.trash_probability ?? cell.semantic_trash_probability, 0);
  if (layer === "obstacle_probability" || layer === "obstacle_focus") return Math.max(realObstacleProbability(cell), priorObstacleProbability(cell));
  if (layer === "confidence" || layer === "debug") return safeNumber(cell.confidence || cell.semantic_confidence, 0);
  if (layer === "osm_priors") return safeNumber(cell.semantic_confidence, 0);
  if (layer === "overview") return Math.max(safeNumber(cell.coverage, 0), safeNumber(cell.trash_probability, 0), realObstacleProbability(cell), safeNumber(cell.semantic_confidence, 0));
  return 0;
}

function colorForCell(cell, layer) {
  if (!cell) return "#070b13";
  if (layer === "overview") return overviewBaseColor(cell);

  const value = clamp(valueForLayer(cell, layer), 0, 1);
  const scanned = isScannedCell(cell);
  const confidence = scanned
    ? clamp(safeNumber(cell.confidence, 0.75), 0.20, 1)
    : clamp(safeNumber(cell.semantic_confidence, 0.25), 0.10, 0.75);
  const alpha = scanned
    ? clamp(0.20 + 0.75 * value * confidence, 0.10, 0.95)
    : clamp(0.08 + 0.36 * value * confidence, 0.06, 0.45);

  if (value <= 0) return scanned ? "rgba(15, 23, 42, 0.82)" : overviewBaseColor(cell);

  if (layer === "coverage") {
    return rgbaToken("detectedWater", alpha);
  }

  if (layer === "trash_probability" || layer === "trash_focus") {
    return rgbaToken("trash", alpha);
  }

  if (layer === "obstacle_probability" || layer === "obstacle_focus") {
    return rgbaToken("detectedObstacle", alpha);
  }

  if (layer === "osm_priors") {
    return overviewBaseColor(cell);
  }

  if (layer === "confidence" || layer === "debug") {
    return rgbaToken("detectedGround", alpha);
  }

  return rgbaToken("detectedWater", alpha);
}

function resizeGridCanvas() {
  const canvas = $("grid-map-canvas");
  if (!canvas) return;

  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(320, Math.floor(rect.width * dpr));
  const height = Math.max(220, Math.floor(rect.height * dpr));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
}

function getGridGeometry(mapData) {
  const canvas = $("grid-map-canvas");
  const meta = getActiveGridMeta(mapData);
  const view = OCTOPUS.gridView;
  const baseCell = Math.min(canvas.width / meta.cols, canvas.height / meta.rows);
  const cellSize = Math.max(0.25, baseCell * view.scale);
  const gridW = meta.cols * cellSize;
  const gridH = meta.rows * cellSize;
  const originX = (canvas.width - gridW) / 2 + view.offsetX;
  const originY = (canvas.height - gridH) / 2 + view.offsetY;
  return { canvas, ...meta, cellSize, gridW, gridH, originX, originY };
}

function gridCellToCanvas(geom, row, col) {
  return {
    x: geom.originX + col * geom.cellSize,
    y: geom.originY + geom.gridH - (row + 1) * geom.cellSize,
  };
}

function localMetersToCanvas(geom, xMeters, yMeters) {
  return {
    x: geom.originX + (xMeters / geom.resolution) * geom.cellSize,
    y: geom.originY + geom.gridH - (yMeters / geom.resolution) * geom.cellSize,
  };
}

function canvasToGridCell(geom, x, y) {
  const col = Math.floor((x - geom.originX) / geom.cellSize);
  const row = Math.floor((geom.gridH - (y - geom.originY)) / geom.cellSize);
  return { row, col };
}

function drawGridLines(ctx, geom) {
  ctx.save();
  ctx.strokeStyle = "rgba(100,160,200,0.18)";
  ctx.lineWidth = Math.max(1, window.devicePixelRatio || 1);
  const maxLines = 220;
  const colStep = Math.max(1, Math.ceil(geom.cols / maxLines));
  const rowStep = Math.max(1, Math.ceil(geom.rows / maxLines));
  for (let c = 0; c <= geom.cols; c += colStep) {
    const x = geom.originX + c * geom.cellSize;
    if (x < -20 || x > geom.canvas.width + 20) continue;
    ctx.beginPath(); ctx.moveTo(x, geom.originY); ctx.lineTo(x, geom.originY + geom.gridH); ctx.stroke();
  }
  for (let r = 0; r <= geom.rows; r += rowStep) {
    const y = geom.originY + geom.gridH - r * geom.cellSize;
    if (y < -20 || y > geom.canvas.height + 20) continue;
    ctx.beginPath(); ctx.moveTo(geom.originX, y); ctx.lineTo(geom.originX + geom.gridW, y); ctx.stroke();
  }
  ctx.strokeStyle = "rgba(248,250,252,0.8)";
  ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
  ctx.strokeRect(geom.originX, geom.originY, geom.gridW, geom.gridH);
  ctx.restore();
}

function drawGridMarker(ctx, xPx, yPx, label, style) {
  ctx.save();
  ctx.beginPath();
  ctx.arc(xPx, yPx, 7 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
  ctx.fillStyle = style.fillColor;
  ctx.fill();
  ctx.lineWidth = 2 * (window.devicePixelRatio || 1);
  ctx.strokeStyle = style.color;
  ctx.stroke();
  ctx.font = `${10 * (window.devicePixelRatio || 1)}px ui-sans-serif, system-ui`;
  ctx.fillStyle = "#f8fafc";
  ctx.textAlign = "center";
  ctx.fillText(label, xPx, yPx - 12 * (window.devicePixelRatio || 1));
  ctx.restore();
}

function drawFleetAndTasksOnGrid(ctx, mapData, geom) {
  const display = OCTOPUS.gridDisplay || GRID_DISPLAY_DEFAULTS;
  if (display.trash) (OCTOPUS.latest.tasks || []).forEach((task) => {
    const lat = safeNumber(task.lat, NaN);
    const lon = safeNumber(task.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const local = latLngToLocal(lat, lon);
    if (local.x < 0 || local.x > geom.width_m || local.y < 0 || local.y > geom.height_m) return;
    const p = localMetersToCanvas(geom, local.x, local.y);
    if (p.x < -20 || p.x > geom.canvas.width + 20 || p.y < -20 || p.y > geom.canvas.height + 20) return;
    drawGridMarker(ctx, p.x, p.y, `T${task.id}`, { color: "#ffffff", fillColor: "#e37222" });
  });

  if (!display.fleet && !display.home) return;

  if (display.home) {
    const home = homeStation();
    const hp = localMetersToCanvas(geom, home.x, home.y);
    if (hp.x >= -20 && hp.x <= geom.canvas.width + 20 && hp.y >= -20 && hp.y <= geom.canvas.height + 20) {
      drawGridMarker(ctx, hp.x, hp.y, "H", { color: "#ffffff", fillColor: "#a78bfa" });
    }
  }

  if (!display.fleet) return;
  getFleetSnapshot().forEach((robot) => {
    const local = robot.local || {};
    // A robot with no reported position converts to local (0, 0) - the datum -
    // which would draw it on top of Eve's start point as if that were a fix.
    if (robot.hasPosition === false) return;
    if (!Number.isFinite(local.x) || !Number.isFinite(local.y)) return;
    if (local.x < 0 || local.x > geom.width_m || local.y < 0 || local.y > geom.height_m) return;
    const p = localMetersToCanvas(geom, local.x, local.y);
    if (p.x < -20 || p.x > geom.canvas.width + 20 || p.y < -20 || p.y > geom.canvas.height + 20) return;
    drawGridMarker(ctx, p.x, p.y, robot.mapLabel, deviceStyle(robot.type === "water" ? "boat" : robot.type, robot.state));
  });
}

function drawGridScale(ctx, geom) {
  const dpr = window.devicePixelRatio || 1;
  const options = [0.5, 1, 2, 5, 10, 20, 50];
  const targetPx = Math.min(140 * dpr, geom.canvas.width * 0.25);
  let scaleMeters = options[0];
  for (const candidate of options) {
    const px = (candidate / geom.resolution) * geom.cellSize;
    if (px <= targetPx) scaleMeters = candidate;
  }
  const scalePx = (scaleMeters / geom.resolution) * geom.cellSize;
  const x = geom.canvas.width - scalePx - 24 * dpr;
  const y = geom.canvas.height - 28 * dpr;
  ctx.save();
  ctx.strokeStyle = "#f8fafc";
  ctx.fillStyle = "#f8fafc";
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  ctx.moveTo(x, y); ctx.lineTo(x + scalePx, y);
  ctx.moveTo(x, y - 5 * dpr); ctx.lineTo(x, y + 5 * dpr);
  ctx.moveTo(x + scalePx, y - 5 * dpr); ctx.lineTo(x + scalePx, y + 5 * dpr);
  ctx.stroke();
  ctx.font = `${11 * dpr}px ui-sans-serif, system-ui`;
  ctx.textAlign = "center";
  ctx.fillText(`${scaleMeters} m`, x + scalePx / 2, y - 8 * dpr);
  ctx.textAlign = "right";
  ctx.fillStyle = "rgba(248,250,252,0.84)";
  ctx.fillText(`${geom.width_m.toFixed(1)} m × ${geom.height_m.toFixed(1)} m`, geom.canvas.width - 12 * dpr, 18 * dpr);
  ctx.restore();
}

function drawGridMap(mapData) {
  if (getSelectedGridSource() === "local_camera") {
    drawLocalCameraGrid(OCTOPUS.latest.localCameraGrid);
    return;
  }
  const canvas = $("grid-map-canvas");
  const info = $("grid-map-info");
  const selector = $("grid-layer-select");
  if (!canvas || !info || !selector) return;

  resizeGridCanvas();

  const ctx = canvas.getContext("2d");
  const layer = selector.value || "overview";
  const mergedMapData = getMergedGridData(mapData || {});
  const cells = mergedMapData?.cells || {};
  const backendCells = mapData?.cells || {};
  const priorCells = OCTOPUS.osmPriors?.cells || {};
  const geom = getGridGeometry(mapData || {});

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#060a12";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = "rgba(15, 23, 42, 0.78)";
  ctx.fillRect(geom.originX, geom.originY, geom.gridW, geom.gridH);
  drawGridLines(ctx, geom);

  Object.entries(cells).forEach(([key, cell]) => {
    if (!cellPassesDisplayFilters(cell)) return;
    if (layer === "osm_priors" && isScannedCell(cell)) return;
    const row = safeNumber(cell.row, NaN);
    const col = safeNumber(cell.col, NaN);
    if (!Number.isFinite(row) || !Number.isFinite(col)) return;

    const mb = cellMetricBounds(cell, mapData);
    if (!metricBoundsIntersectsArea(mb, geom)) return;

    const p0 = localMetersToCanvas(geom, Math.max(0, mb.x0), Math.max(0, mb.y0));
    const p1 = localMetersToCanvas(geom, Math.min(geom.width_m, mb.x1), Math.min(geom.height_m, mb.y1));
    const x = Math.min(p0.x, p1.x);
    const y = Math.min(p0.y, p1.y);
    const w = Math.abs(p1.x - p0.x);
    const h = Math.abs(p1.y - p0.y);
    if (x + w < 0 || x > canvas.width || y + h < 0 || y > canvas.height) return;

    const pad = Math.min(0.5, Math.max(w, h) * 0.10);
    const fx = x + pad;
    const fy = y + pad;
    const fw = Math.max(1, w - 2 * pad);
    const fh = Math.max(1, h - 2 * pad);

    if (layer === "overview") {
      drawOverviewCell(ctx, cell, fx, fy, fw, fh);
    } else {
      ctx.fillStyle = colorForCell(cell, layer);
      ctx.fillRect(fx, fy, fw, fh);
    }

    const trash = safeNumber(cell.trash_probability ?? cell.semantic_trash_probability, 0);
    if (OCTOPUS.gridDisplay.trash && trash >= 0.65 && layer !== "trash_probability" && layer !== "trash_focus" && Math.max(w, h) >= 3) {
      ctx.beginPath();
      ctx.arc(x + w / 2, y + h / 2, Math.max(2, Math.min(w, h) * 0.28), 0, Math.PI * 2);
      ctx.fillStyle = rgbaToken("trash", 0.92);
      ctx.fill();
    }

    if (OCTOPUS.selectedCellKey === key) {
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = Math.max(2, Math.floor(window.devicePixelRatio || 1));
      ctx.strokeRect(x + 1, y + 1, Math.max(1, w - 2), Math.max(1, h - 2));
    }
  });

  drawFleetAndTasksOnGrid(ctx, mapData, geom);
  drawGridScale(ctx, geom);

  const updatedCells = Object.keys(backendCells).length;
  const osmPriorCells = Object.keys(priorCells).length;
  const coverageStats = computeCoverageStats(mapData);
  const patchAge = ageSeconds(OCTOPUS.latest.patch?.received_at);
  const patchFreshness = freshnessFromAge(patchAge);

  info.innerHTML = `
    <span class="mini-chip">Frame: <strong>${escapeHtml(mapData?.frame_id || "map")}</strong></span>
    <span class="mini-chip">${geom.width_m.toFixed(1)} m × ${geom.height_m.toFixed(1)} m</span>
    <span class="mini-chip">${geom.cols} × ${geom.rows} planning cells</span>
    <span class="mini-chip">Grid cell: ${geom.resolution.toFixed(2)} m</span>
    <span class="mini-chip">Layer: <strong>${escapeHtml(layer)}</strong></span>
    <span class="mini-chip">OSM priors: <strong>${osmPriorCells}</strong></span>
    <span class="mini-chip">Visible layers: <strong>${Object.entries(OCTOPUS.gridDisplay).filter(([,v]) => v).length}/${Object.keys(GRID_DISPLAY_DEFAULTS).length}</strong></span>
    <span class="mini-chip">Coverage: <strong>${formatPercent(coverageStats.coverageRatio)}</strong></span>
    <span class="mini-chip">Updated: <strong>${updatedCells}</strong></span>
    <span class="mini-chip ${patchFreshness.state}">Patch: ${patchFreshness.label}</span>
  `;
}

function computeCoverageStats(mapData) {
  const meta = getActiveGridMeta(mapData);
  const totalCells = Math.max(1, meta.rows * meta.cols);
  const totalArea = Math.max(0.0001, meta.width_m * meta.height_m);
  const cells = Object.values(mapData?.cells || {});
  let coveredArea = 0;
  let coveredCells = 0;
  cells.forEach((cell) => {
    if (safeNumber(cell.coverage, 0) <= 0) return;
    const mb = cellMetricBounds(cell, mapData);
    if (!metricBoundsIntersectsArea(mb, meta)) return;
    const x0 = clamp(mb.x0, 0, meta.width_m);
    const x1 = clamp(mb.x1, 0, meta.width_m);
    const y0 = clamp(mb.y0, 0, meta.height_m);
    const y1 = clamp(mb.y1, 0, meta.height_m);
    coveredArea += Math.max(0, x1 - x0) * Math.max(0, y1 - y0);
    coveredCells += 1;
  });
  const coverageRatio = clamp(coveredArea / totalArea, 0, 1);
  return { total: totalCells, totalArea, coveredArea, coveredCells, coverageRatio };
}

function computeHealthItems() {
  const patchAge = ageSeconds(OCTOPUS.latest.patch?.received_at);
  const mapAge = ageSeconds(OCTOPUS.latest.globalMap?.last_update);
  const statsAge = ageSeconds(OCTOPUS.latest.stats?.last_update);

  const hasBattery = Array.isArray(OCTOPUS.latest.battery) && OCTOPUS.latest.battery.length > 0;
  const freshestBatteryAge = hasBattery
    ? Math.min(...OCTOPUS.latest.battery.map((b) => ageSeconds(b.ts)).filter((x) => x !== null))
    : null;

  const transformView = cameraTransformView(
    OCTOPUS.latest.cameraTransformStatus,
    OCTOPUS.latest.cameraTransformError || ""
  );
  const transformMarkers = cameraTransformMarkerRow(transformView);

  return [
    {
      name: "Backend API",
      state: OCTOPUS.backendOk ? "fresh" : "offline",
      detail: OCTOPUS.backendOk ? "requests successful" : OCTOPUS.lastError || "no response",
    },
    {
      name: "ROS map patch bridge",
      ...freshnessFromAge(patchAge, 2, 10),
      detail: patchAge === null ? "no /api/map_patch received" : formatAge(patchAge),
    },
    {
      name: "Global map state",
      ...freshnessFromAge(mapAge, 3, 15),
      detail: OCTOPUS.latest.globalMap ? "accumulated grid available" : "waiting for map",
    },
    {
      name: "Camera transform",
      state: transformView.uiState,
      detail: transformView.detail,
    },
    {
      name: "AprilTags",
      state: transformMarkers.state,
      detail: transformMarkers.detail,
    },
    {
      name: "Fleet telemetry",
      ...freshnessFromAge(freshestBatteryAge, 5, 30),
      detail: hasBattery ? `${OCTOPUS.latest.battery.length} device entries` : "no battery/robot data",
    },
    {
      name: "Detector node",
      state: "unknown",
      label: "not configured",
      detail: "later: /detector_node/detections rate",
    },
    {
      name: "Camera stream",
      state: "unknown",
      label: "not configured",
      detail: "later: /camera/image_raw/compressed",
    },
    {
      name: "Pixhawk pose",
      state: "unknown",
      label: "not configured",
      detail: "later: /fmu/out/vehicle_*",
    },
  ];
}

function healthSummary(items) {
  const critical = items.filter((item) => item.state === "offline" || item.state === "error").length;
  const stale = items.filter((item) => item.state === "stale" || item.state === "warning").length;
  const fresh = items.filter((item) => item.state === "fresh" || item.state === "ok").length;

  if (critical > 0) return { state: "error", label: `${critical} critical` };
  if (stale > 0) return { state: "warning", label: `${stale} stale` };
  if (fresh > 0) return { state: "fresh", label: "OK" };
  return { state: "unknown", label: "Waiting" };
}

function applyDashboardView(view = OCTOPUS.dashboardView, announce = false) {
  const safeView = DASHBOARD_VIEWS[view] ? view : "overview";
  OCTOPUS.dashboardView = safeView;
  localStorage.setItem("octopusDashboardView", safeView);
  document.body.dataset.view = safeView;
  const select = $("dashboard-view-select");
  if (select) select.value = safeView;

  // Set sensible grid presets per mode, but do not destroy layer checkbox preferences.
  const gridSelect = $("grid-layer-select");
  if (gridSelect && announce) {
    const preset = DASHBOARD_VIEWS[safeView].preset;
    if (preset && gridSelect.value !== preset) gridSelect.value = preset;
  }

  if (announce) addTimeline(`Dashboard view changed to ${DASHBOARD_VIEWS[safeView].label}`, "info");
  setTimeout(() => {
    if (OCTOPUS.missionMap.map) OCTOPUS.missionMap.map.invalidateSize();
    if (OCTOPUS.latest.globalMap || OCTOPUS.osmPriors) drawGridMap(OCTOPUS.latest.globalMap || {});
  }, 80);
  renderFleet();
  renderTasks();
}


function getSelectedGridSource() {
  return OCTOPUS.gridSource || localStorage.getItem("octopusGridSource") || "global";
}

function drawLocalCameraGrid(localData = OCTOPUS.latest.localCameraGrid) {
  const canvas = $("grid-map-canvas");
  const info = $("grid-map-info");
  if (!canvas) return;

  resizeGridCanvas();

  const ctx = canvas.getContext("2d");
  const widthPx = canvas.width;
  const heightPx = canvas.height;

  ctx.clearRect(0, 0, widthPx, heightPx);
  ctx.fillStyle = "rgba(3, 7, 18, 0.96)";
  ctx.fillRect(0, 0, widthPx, heightPx);

  const patch = localData?.patch || null;
  if (!patch) {
    ctx.fillStyle = "rgba(203, 213, 225, 0.82)";
    ctx.font = "700 15px Inter, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText("No local camera grid data yet", widthPx / 2, heightPx / 2);
    if (info) {
      info.innerHTML = `<span class="mini-chip">Local Camera Grid · waiting for /api/local_camera_grid/latest</span>`;
    }
    return;
  }

  const footprintWidth = safeNumber(patch.footprint_width_m, 4.46);
  const footprintHeight = safeNumber(patch.footprint_height_m, 3.34);
  const resolution = safeNumber(patch.resolution_m, 0.10);
  const cells = Array.isArray(patch.updated_cells) ? patch.updated_cells : [];

  const pad = 34;
  const usableW = Math.max(1, widthPx - pad * 2);
  const usableH = Math.max(1, heightPx - pad * 2);
  const scale = Math.min(usableW / footprintWidth, usableH / footprintHeight);
  const gridW = footprintWidth * scale;
  const gridH = footprintHeight * scale;
  const x0 = (widthPx - gridW) / 2;
  const y0 = (heightPx - gridH) / 2;

  ctx.fillStyle = "rgba(15, 23, 42, 0.98)";
  ctx.fillRect(x0, y0, gridW, gridH);
  ctx.strokeStyle = "rgba(100, 160, 200, 0.92)";
  ctx.lineWidth = 2;
  ctx.strokeRect(x0, y0, gridW, gridH);

  const cols = Math.ceil(footprintWidth / resolution);
  const rows = Math.ceil(footprintHeight / resolution);

  ctx.lineWidth = 1;
  ctx.strokeStyle = "rgba(148, 163, 184, 0.13)";

  for (let c = 0; c <= cols; c += 1) {
    const x = x0 + c * resolution * scale;
    ctx.beginPath();
    ctx.moveTo(x, y0);
    ctx.lineTo(x, y0 + gridH);
    ctx.stroke();
  }

  for (let r = 0; r <= rows; r += 1) {
    const y = y0 + r * resolution * scale;
    ctx.beginPath();
    ctx.moveTo(x0, y);
    ctx.lineTo(x0 + gridW, y);
    ctx.stroke();
  }

  ctx.strokeStyle = "rgba(56, 189, 248, 0.55)";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(x0 + gridW / 2, y0);
  ctx.lineTo(x0 + gridW / 2, y0 + gridH);
  ctx.moveTo(x0, y0 + gridH / 2);
  ctx.lineTo(x0 + gridW, y0 + gridH / 2);
  ctx.stroke();

  for (const cell of cells) {
    const col = Math.floor(safeNumber(cell.col, safeNumber(cell.x, 0) / resolution));
    const row = Math.floor(safeNumber(cell.row, safeNumber(cell.y, 0) / resolution));

    const x = x0 + col * resolution * scale;
    const y = y0 + gridH - (row + 1) * resolution * scale;
    const w = Math.max(3, resolution * scale);
    const h = Math.max(3, resolution * scale);

    ctx.fillStyle = "rgba(251, 146, 60, 0.86)";
    ctx.fillRect(x, y, w, h);

    ctx.strokeStyle = "rgba(255, 255, 255, 0.80)";
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, w, h);

    ctx.fillStyle = "rgba(255, 255, 255, 0.95)";
    ctx.beginPath();
    ctx.arc(x + w / 2, y + h / 2, 3.5, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = "rgba(203, 213, 225, 0.95)";
  ctx.font = "700 12px Inter, sans-serif";
  ctx.textAlign = "left";
  ctx.fillText("camera footprint", x0, Math.max(14, y0 - 10));
  ctx.textAlign = "right";
  ctx.fillText(`${footprintWidth.toFixed(2)} m × ${footprintHeight.toFixed(2)} m`, x0 + gridW, Math.max(14, y0 - 10));

  const receivedAt = localData?.received_at || patch.timestamp;
  const age = ageSeconds(receivedAt);
  const fresh = freshnessFromAge(age, 2, 10);

  if (info) {
    const first = cells[0] || null;
    const firstText = first
      ? `first: u=${safeNumber(first.u).toFixed(3)}, v=${safeNumber(first.v).toFixed(3)} · x=${safeNumber(first.x).toFixed(2)} m, y=${safeNumber(first.y).toFixed(2)} m`
      : "no updated cells";

    info.innerHTML = `
      <span class="mini-chip">Local Camera Grid · ${fresh.label}</span>
      <span class="mini-chip">${footprintWidth.toFixed(2)} m × ${footprintHeight.toFixed(2)} m</span>
      <span class="mini-chip">${resolution.toFixed(2)} m/cell</span>
      <span class="mini-chip">${cells.length} detection cell(s)</span>
      <span class="mini-chip">${firstText}</span>
      <span class="mini-chip">debug only · robots use global map</span>
    `;
  }
}

function renderMissionPhase() {
  const select = $("mission-phase-select");
  const actionEl = $("mission-next-action");
  if (!select || !actionEl) return;

  select.value = OCTOPUS.missionPhase;
  const info = PHASE_INFO[OCTOPUS.missionPhase] || PHASE_INFO.idle;
  actionEl.className = `pill ${info.status}`;
  actionEl.innerHTML = `<span class="dot"></span><span>${escapeHtml(info.action)} · ${escapeHtml(info.decision)}</span>`;
}

function renderTopStatus() {
  const backend = $("backend-status-pill");
  const ros = $("ros-status-pill");
  const refresh = $("last-refresh-pill");

  if (backend) {
    backend.className = `pill ${OCTOPUS.backendOk ? "fresh" : "offline"}`;
    backend.innerHTML = `<span class="dot"></span><span>Backend ${OCTOPUS.backendOk ? "OK" : "OFFLINE"}</span>`;
  }

  const patchFreshness = freshnessFromAge(ageSeconds(OCTOPUS.latest.patch?.received_at), 2, 10);
  if (ros) {
    ros.className = `pill ${patchFreshness.state}`;
    ros.innerHTML = `<span class="dot"></span><span>ROS bridge ${patchFreshness.label}</span>`;
  }

  if (refresh) {
    refresh.className = "pill muted";
    refresh.innerHTML = `<span class="dot"></span><span>${OCTOPUS.lastRefresh ? formatTime(OCTOPUS.lastRefresh) : "--:--:--"}</span>`;
  }
}

function renderKpis() {
  const stats = OCTOPUS.latest.stats || {};
  const mapData = OCTOPUS.latest.globalMap || {};
  const battery = OCTOPUS.latest.battery || [];
  const patch = OCTOPUS.latest.patch;
  const cells = mapData.cells || {};
  const coverageStats = computeCoverageStats(mapData);
  const healthItems = computeHealthItems();
  const health = healthSummary(healthItems);

  $("kpi-coverage").textContent = formatPercent(coverageStats.coverageRatio);
  $("kpi-coverage-sub").textContent = `${coverageStats.coveredArea.toFixed(1)} / ${coverageStats.totalArea.toFixed(1)} m² scanned`;

  const trash = cameraTrashSummary();
  $("kpi-detections").textContent = trash.available ? trash.count : "--";
  $("kpi-detections-sub").textContent = trash.note;

  $("kpi-confirmed").textContent = safeNumber(stats.trash_collected, 0);

  const links = fleetLinkSummary();
  $("kpi-fleet").textContent = `${links.online.length}/${links.total}`;
  $("kpi-fleet-sub").textContent = links.online.length ? links.online.join(" · ") : links.note;

  const patchCells = patch?.updated_cells?.length ?? 0;
  $("kpi-map-patch").textContent = patchCells || "--";
  $("kpi-map-patch-sub").textContent = patch ? `${Object.keys(cells).length} accumulated cells` : "No patch yet";

  $("kpi-health").textContent = health.label;
  $("kpi-health").className = `value ${health.state === "error" ? "error-text" : health.state === "warning" ? "warning-text" : ""}`;
  $("kpi-health-sub").textContent = `${healthItems.filter((i) => i.state === "fresh" || i.state === "ok").length}/${healthItems.length} fresh/configured`;
}

// One line of what a robot on the GripperX link is actually reporting: what it
// is doing, whether the arm is live, and whether it can place itself at all.
// Beantwortet die eine Frage, die im Betrieb zuerst kommt: redet der Roboter
// mit uns? Bewusst grob -- vier Zustaende, kein Zahlenwerk. Die Details stehen
// weiterhin darunter in der Zeile mit Batterie und Alter.
//
// Eve ist der Sonderfall: sie meldet sich nicht ueber den Geraetestatus wie die
// Sammelroboter, sondern ueber ihre Teilsysteme. "Verbunden" heisst bei ihr
// deshalb: die Pi antwortet ueberhaupt.
function robotLinkBadge(robot) {
  if (robot.key === "eve") {
    const camera = OCTOPUS.latest.eveStatus?.status;
    const px4 = OCTOPUS.latest.px4BridgeStatus?.status;
    if (!camera && !px4) return { cls: "unknown", label: "checking", title: "Eve wurde noch nicht abgefragt" };
    // "offline" von beiden Endpunkten heisst: die Pi ist per SSH nicht
    // erreichbar. Ein einzelnes gestopptes Teilsystem ist dagegen kein
    // Verbindungsproblem.
    const reachable = (camera && camera !== "offline") || (px4 && px4 !== "offline");
    return reachable
      ? { cls: "online", label: "connected", title: "Die Pi antwortet" }
      : { cls: "offline", label: "no link", title: "Die Pi antwortet nicht (SSH)" };
  }

  if (robot.demo) {
    return { cls: "demo", label: "demo data", title: "Kein echter Link - angezeigt werden Platzhalterwerte" };
  }
  if (!robot.device) {
    return { cls: "offline", label: "no link", title: "Dieser Roboter hat sich noch nie gemeldet" };
  }
  if (robot.linkStale) {
    return { cls: "stale", label: "stale", title: `Letzte Meldung vor mehr als ${DEVICE_STATUS_STALE_SEC} s` };
  }
  return { cls: "online", label: "connected", title: "Meldet sich laufend ueber den Roboter-Link" };
}

function deviceLinkSummary(robot) {
  const device = robot.device || {};
  const nav = device.nav || {};
  const pose = device.pose || {};
  const parts = [];

  const goal = nav.active_goal_id;
  parts.push(goal === null || goal === undefined ? "no active goal" : `goal #${goal}`);

  const remaining = safeNumber(nav.distance_remaining_m, NaN);
  if (Number.isFinite(remaining)) parts.push(`${remaining.toFixed(1)} m to go`);

  parts.push(device.armed ? "armed" : "disarmed");

  if (!robot.hasPosition) {
    parts.push(pose.status === "no_datum" ? "no datum yet, not on the map" : "no position");
  }

  return parts.join(" · ");
}

// --- Eve-Teilsysteme als Chips in der Flottenkarte ---
// Die drei Prozesse, ohne die Eve nichts liefert. Sie stehen bei Eve und nicht
// in einem eigenen Panel, weil die Frage im Betrieb "laeuft Eve?" lautet und
// nicht "laeuft Prozess X irgendwo".
//
// on      = laeuft
// pending = laeuft an oder faehrt herunter, kein Fehler
// off     = erreichbar, aber gestoppt -- ein Klick startet es
// unknown = nicht erreichbar oder noch nie geantwortet
const EVE_SUBSYSTEMS = [
  {
    key: "camera",
    label: "Camera",
    title: "camera_node auf der Pi",
    read: () => OCTOPUS.latest.eveStatus,
    map: {
      camera_running: "on",
      camera_started: "on",
      online_camera_stopped: "off",
      camera_stopped: "off",
      offline: "unknown",
    },
    start: () => startEveCamera(),
    stop: () => stopEveCamera(),
  },
  {
    key: "px4",
    label: "PX4",
    title: "MicroXRCEAgent auf der Pi - liefert die Fluglage",
    read: () => OCTOPUS.latest.px4BridgeStatus,
    map: {
      px4_bridge_running: "on",
      px4_bridge_started: "on",
      online_px4_bridge_stopped: "off",
      px4_bridge_stopped: "off",
      offline: "unknown",
    },
    start: () => startPx4Bridge(),
    stop: () => stopPx4Bridge(),
  },
  {
    key: "detector",
    label: "Detector",
    title: "YOLO auf diesem Rechner",
    read: () => OCTOPUS.latest.detectorStatus,
    map: {
      detector_running: "on",
      detector_started: "on",
      detector_loading: "pending",
      detector_stopped: "off",
      detector_failed: "unknown",
    },
    start: () => startDetector(),
    stop: () => stopDetector(),
  },
];

function eveSubsystemSummary() {
  const states = EVE_SUBSYSTEMS.map(eveSubsystemState);
  if (states.every((st) => st === "unknown")) return "no contact";
  const up = states.filter((st) => st === "on").length;
  const loading = states.filter((st) => st === "pending").length;
  return `${up}/${states.length} systems up${loading ? ` · ${loading} loading` : ""}`;
}

function eveSubsystemState(sub) {
  const record = sub.read();
  if (!record || !record.status) return "unknown";
  return sub.map[record.status] || "unknown";
}

const EVE_SUBSYSTEM_HINTS = {
  on: "läuft — Klick stoppt",
  off: "gestoppt — Klick startet",
  pending: "lädt gerade",
  unknown: "kein Kontakt",
};

function renderEveSubsystemChips() {
  return EVE_SUBSYSTEMS.map((sub) => {
    const state = eveSubsystemState(sub);
    const hint = EVE_SUBSYSTEM_HINTS[state] || "";
    // disabled im pending-Zustand: waehrend YOLO laedt, waere ein zweiter
    // Klick ein Stop mitten im Start.
    const disabled = state === "pending" ? " disabled" : "";
    return `<button type="button" class="subsystem-chip is-${state}" data-subsystem="${sub.key}"${disabled}
              title="${escapeHtml(sub.title)} — ${escapeHtml(hint)}"
              aria-label="${escapeHtml(sub.label)}: ${escapeHtml(hint)}"><span class="dot" aria-hidden="true"></span>${escapeHtml(sub.label)}</button>`;
  }).join("");
}

function bindEveSubsystemChips(root) {
  root.querySelectorAll("[data-subsystem]").forEach((chip) => {
    chip.addEventListener("click", (event) => {
      // Der Chip sitzt in der anklickbaren Roboterkarte. Ohne das hier waehlt
      // ein Chip-Klick nebenbei Eve im Inspector aus.
      event.stopPropagation();
      const sub = EVE_SUBSYSTEMS.find((s) => s.key === chip.dataset.subsystem);
      if (!sub) return;
      const state = eveSubsystemState(sub);
      if (state === "pending") return;
      if (state === "on") sub.stop(); else sub.start();
    });
  });
}

function renderFleet() {
  const el = $("fleet-content");
  if (!el) return;

  const detailed = OCTOPUS.dashboardView === "fleet";
  const robots = getFleetSnapshot();
  el.innerHTML = `<div class="compact-list">${robots.map((robot) => {
    const percent = clamp(safeNumber(robot.battery.percent, 0), 0, 100);
    // A robot that reports "no battery sensor" must not render as 0% - that
    // reads as an empty battery about to strand it.
    const batteryLabel = robot.battery.unavailable
      ? `n/a${robot.battery.reason ? ` (${escapeHtml(String(robot.battery.reason).toLowerCase().replace(/_/g, " "))})` : ""}`
      : `${percent.toFixed(0)}%`;
    const fresh = freshnessFromAge(robot.age, 5, 45);
    const status = robot.status === "unknown" ? fresh.state : robot.status;
    // Eve meldet sich nicht ueber den Geraetestatus, sondern ueber ihre
    // Teilsysteme. Der Zeitstempel aus der Datenbank ist bei ihr Demo-Altbestand
    // und stand als "missing · 6078 h ago" direkt unter einem "connected" --
    // zwei Aussagen, von denen nur eine stimmen kann.
    const updateLabel = robot.key === "eve"
      ? eveSubsystemSummary()
      : robot.demo
        ? "demo/fallback"
        : robot.device
          ? `${robot.linkStale ? "link stale" : "live"}${robot.deviceAge === null ? "" : ` · ${robot.deviceAge.toFixed(1)}s ago`}`
          : escapeHtml(fresh.label);
    const currentTask = robot.currentTask ? `Task #${escapeHtml(robot.currentTask.id)}` : "none";
    const tags = robot.tags.map((tag) => {
      const cls = tag.includes("water") || tag.includes("boat") || tag.includes("floating") ? "water" : tag.includes("land") ? "land" : tag.includes("scan") || tag.includes("detect") || tag.includes("camera") ? "scan" : "";
      return `<span class="capability-tag ${cls}">${escapeHtml(tag)}</span>`;
    }).join("");
    // Die Frage, die im Betrieb zuerst kommt: redet der Roboter mit uns?
    // Sie stand bisher nur zwischen den Zeilen ("demo/fallback" vs. eine
    // Altersangabe) -- jetzt steht sie als eigenes Abzeichen oben rechts.
    const link = robotLinkBadge(robot);
    // Im Overview teilen sich vier Roboter ~190 px Hoehe. Eve bleibt
    // ausfuehrlich, die anderen schrumpfen auf eine Zeile -- alles andere
    // stuende ohnehin schon im Abzeichen.
    const compactRow = OCTOPUS.dashboardView === "overview" && robot.key !== "eve";
    // Auch bei Eve: das Abzeichen oben rechts und die drei Chips sagen den
    // Zustand bereits, ein zusaetzlicher Statuspill kostet nur eine Zeile.
    const overviewRow = OCTOPUS.dashboardView === "overview";
    // Die Karte ist ein div, kein button: in ihr sitzen die Subsystem-Chips,
    // und ein button in einem button ist ungueltiges HTML.
    return `
      <div class="item-card robot-card ${robot.key === "eve" ? "is-primary" : "robot-card-compact"} link-${link.cls}" data-device-id="${escapeHtml(robot.name)}" role="button" tabindex="0" aria-label="Select ${escapeHtml(robot.name)}">
        <div class="item-top">
          <div class="robot-topline">
            <span class="robot-icon" aria-hidden="true">${robot.icon}</span>
            <div>
              <div class="robot-name">${escapeHtml(robot.name)}</div>
              <div class="robot-role">${escapeHtml(robot.role)}</div>
            </div>
          </div>
          <span class="link-badge link-${link.cls}" title="${escapeHtml(link.title)}"><span class="dot" aria-hidden="true"></span>${escapeHtml(link.label)}</span>
        </div>
        ${robot.key === "eve" ? `<div class="subsystem-chips">${renderEveSubsystemChips()}</div>` : ""}
        <div class="item-meta">
          ${compactRow || overviewRow ? "" : statusPill(escapeHtml(robot.state || "unknown"), status)}
          Battery: <span class="accent">${batteryLabel}</span>${compactRow ? "" : ` · ${updateLabel}`}
          ${detailed ? `<br />${escapeHtml(robot.capability)}` : ""}
          ${robot.device && detailed ? `<br />${escapeHtml(deviceLinkSummary(robot))}` : ""}
          ${detailed ? `<br />Position: ${robot.hasPosition ? `${safeNumber(robot.location.lat, 0).toFixed(6)}, ${safeNumber(robot.location.lon, 0).toFixed(6)}` : "no position reported"}<br />Current task: ${currentTask}<br />Assignment rule: ${escapeHtml(robot.taskRule)}` : ""}
        </div>
        <div class="progress"><span style="width:${robot.battery.unavailable ? 0 : percent}%"></span></div>
        ${detailed ? `<div class="capability-tags">${tags}</div>` : ""}
      </div>
    `;
  }).join("")}</div>`;

  el.querySelectorAll("[data-device-id]").forEach((card) => {
    const select = () => {
      const robot = robots.find((r) => r.name === card.dataset.deviceId);
      if (!robot) return;
      OCTOPUS.selected = { type: "fleet", id: robot.name, device_type: robot.type, robot };
      renderInspector();
    };
    card.addEventListener("click", select);
    // Die Karte war ein <button> und liess sich mit der Tastatur bedienen. Als
    // div mit role="button" muss das von Hand nachgeruestet werden.
    card.addEventListener("keydown", (event) => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        select();
      }
    });
  });

  bindEveSubsystemChips(el);
}

function renderTasks() {
  const el = $("tasks-content");
  if (!el) return;

  const tasks = OCTOPUS.latest.tasks || [];
  if (!tasks.length) {
    el.innerHTML = `<div class="item-card"><div class="item-title">No tasks</div><div class="item-meta">Detection queue is empty. Eve should create trash candidates from camera detections.</div></div>`;
    return;
  }

  el.innerHTML = `<div class="compact-list">${tasks.slice(0, OCTOPUS.dashboardView === "mapping" ? 12 : 8).map((t) => {
    const age = ageSeconds(t.ts);
    const fresh = freshnessFromAge(age, 30, 180);
    const assigned = t.assigned || t.assigned_to || "unassigned";
    const terrain = terrainStatusForTask(t);
    const suggestions = suitableRobotNamesForTask(t).join(" / ");
    return `
      <button class="item-card" type="button" data-task-id="${escapeHtml(t.id)}" style="text-align:left; width:100%;">
        <div class="item-top">
          <div class="item-title">Trash task #${escapeHtml(t.id)}</div>
          ${statusPill(escapeHtml(t.status || "unknown"), taskStateToStatus(t.status))}
        </div>
        <div class="item-meta">
          ${statusPill(terrain.label, terrain.state)}<br />
          Suggested robot: <span class="accent">${escapeHtml(suggestions)}</span><br />
          Assigned: <span class="accent">${escapeHtml(assigned)}</span><br />
          Position: ${safeNumber(t.lat, 0).toFixed(5)}, ${safeNumber(t.lon, 0).toFixed(5)} · Data: ${fresh.label}
        </div>
      </button>
    `;
  }).join("")}</div>`;

  el.querySelectorAll("[data-task-id]").forEach((card) => {
    card.addEventListener("click", () => {
      const task = tasks.find((t) => String(t.id) === String(card.dataset.taskId));
      if (!task) return;
      const local = latLngToLocal(safeNumber(task.lat, 0), safeNumber(task.lon, 0));
      OCTOPUS.selected = { type: "task", task, local };
      renderInspector();
    });
  });
}

function taskStateToStatus(status) {
  const value = String(status || "").toLowerCase();
  if (["completed", "collected", "done"].includes(value)) return "fresh";
  if (["in_progress", "assigned", "running"].includes(value)) return "warning";
  if (["failed", "blocked", "rejected"].includes(value)) return "error";
  return "unknown";
}

function renderSystemHealth() {
  const el = $("system-health-content");
  if (!el) return;

  const items = computeHealthItems();
  el.innerHTML = `
    <table class="status-table">
      <tbody>
        ${items.map((item) => `
          <tr>
            <td>${statusPill(escapeHtml(item.name), item.state)}</td>
            <td>${escapeHtml(item.detail || item.label || "")}</td>
          </tr>
        `).join("")}
      </tbody>
    </table>
  `;
}

function renderReadiness() {
  const el = $("readiness-content");
  if (!el) return;

  const fleet = getFleetSnapshot();
  const eve = fleet.find((r) => r.key === "eve");
  const landReady = fleet.some((r) => r.terrain === "ground" && r.online);
  const waterReady = fleet.some((r) => r.terrain === "water" && r.online);
  const patchFresh = freshnessFromAge(ageSeconds(OCTOPUS.latest.patch?.received_at), 2, 10);
  const mapCells = Object.keys(OCTOPUS.latest.globalMap?.cells || {}).length;

  const transformView = cameraTransformView(
    OCTOPUS.latest.cameraTransformStatus,
    OCTOPUS.latest.cameraTransformError || ""
  );
  const transformMarkers = cameraTransformMarkerRow(transformView);

  const rows = [
    ["Mission polygon defined", "unknown", "planning tool later"],
    ["Home position set", "unknown", "planning tool later"],
    ["Backend API", OCTOPUS.backendOk ? "fresh" : "offline", OCTOPUS.backendOk ? "OK" : "offline"],
    ["ROS map patch bridge", patchFresh.state, patchFresh.label],
    ["Camera transform", transformView.uiState, transformView.detail],
    ["AprilTags visible", transformMarkers.state, transformMarkers.detail],
    ["Local grid map", mapCells > 0 ? "fresh" : "warning", mapCells > 0 ? `${mapCells} cells` : "empty"],
    ["Eve drone available", eve?.online ? "fresh" : "warning", eve?.online ? "scan/detect role configured" : "waiting"],
    ["Land collection available", landReady ? "fresh" : "warning", landReady ? "Robby/GripperX ready" : "waiting"],
    ["Water collection available", waterReady ? "fresh" : "warning", waterReady ? "SharX ready" : "waiting"],
    ["Emergency action configured", "unknown", "placeholder only"],
  ];

  el.innerHTML = `
    <table class="status-table">
      <tbody>
        ${rows.map(([name, state, detail]) => `
          <tr>
            <td>${statusPill(escapeHtml(name), state)}</td>
            <td>${escapeHtml(detail)}</td>
          </tr>
        `).join("")}
      </tbody>
    </table>
  `;
}

function renderInspector() {
  const el = $("inspector-content");
  if (!el) return;

  const selected = OCTOPUS.selected;
  if (!selected) {
    el.innerHTML = `
      <div class="item-card">
        <div class="item-title">No object selected</div>
        <div class="item-meta">Click a grid cell to inspect layer values. Later: click drone, robot, polygon, or trash marker.</div>
      </div>
    `;
    return;
  }

  if (selected.type === "grid_cell") {
    const c = selected.cell || {};
    el.innerHTML = `
      <div class="item-card">
        <div class="item-top">
          <div class="item-title">Grid cell ${escapeHtml(c.row)}, ${escapeHtml(c.col)}</div>
          ${statusPill("selected", "fresh")}
        </div>
        <table class="status-table">
          <tbody>
            <tr><td>world x/y</td><td>${safeNumber(c.x, 0).toFixed(2)}, ${safeNumber(c.y, 0).toFixed(2)}</td></tr>
            <tr><td>coverage</td><td>${safeNumber(c.coverage, 0).toFixed(2)}</td></tr>
            <tr><td>trash probability</td><td>${safeNumber(c.trash_probability ?? c.semantic_trash_probability, 0).toFixed(2)}</td></tr>
            <tr><td>true obstacle probability</td><td>${realObstacleProbability(c).toFixed(2)}</td></tr>
            <tr><td>prior obstacle probability</td><td>${priorObstacleProbability(c).toFixed(2)}</td></tr>
            <tr><td>confidence</td><td>${safeNumber(c.confidence || c.semantic_confidence, 0).toFixed(2)}</td></tr>
            <tr><td>semantic class</td><td>${escapeHtml(c.semantic_class || "unknown")}</td></tr>
            <tr><td>semantic confidence</td><td>${safeNumber(c.semantic_confidence, 0).toFixed(2)}</td></tr>
            <tr><td>source</td><td>${escapeHtml(c.source_id || c.semantic_source || c.source || "unknown")}</td></tr>
          </tbody>
        </table>
      </div>
    `;
    return;
  }

  if (selected.type === "fleet") {
    const robot = selected.robot || getFleetSnapshot().find((r) => r.name === selected.id) || {};
    const percent = clamp(safeNumber(robot.battery?.percent, 0), 0, 100);
    const fresh = freshnessFromAge(robot.age, 5, 45);
    el.innerHTML = `
      <div class="item-card robot-card">
        <div class="item-top">
          <div class="robot-topline">
            <span class="robot-icon" aria-hidden="true">${robot.icon || "●"}</span>
            <div>
              <div class="robot-name">${escapeHtml(robot.name || selected.id)}</div>
              <div class="robot-role">${escapeHtml(robot.role || selected.device_type || "robot")}</div>
            </div>
          </div>
          ${statusPill(escapeHtml(robot.state || "unknown"), robot.status || fresh.state)}
        </div>
        <table class="status-table">
          <tbody>
            <tr><td>purpose</td><td>${escapeHtml(robot.purpose || "unknown")}</td></tr>
            <tr><td>capability</td><td>${escapeHtml(robot.capability || "unknown")}</td></tr>
            <tr><td>assignment rule</td><td>${escapeHtml(robot.taskRule || "unknown")}</td></tr>
            <tr><td>battery</td><td>${robot.battery?.unavailable
              ? `n/a${robot.battery.reason ? ` (${escapeHtml(String(robot.battery.reason).toLowerCase().replace(/_/g, " "))})` : ""}`
              : `${percent.toFixed(0)}%`}</td></tr>
            <tr><td>lat/lon</td><td>${robot.hasPosition === false
              ? "not reported"
              : `${safeNumber(robot.location?.lat, 0).toFixed(6)}, ${safeNumber(robot.location?.lon, 0).toFixed(6)}`}</td></tr>
            <tr><td>local x/y</td><td>${robot.hasPosition === false
              ? "not reported"
              : `${safeNumber(robot.local?.x, 0).toFixed(2)}, ${safeNumber(robot.local?.y, 0).toFixed(2)} m`}</td></tr>
            <tr><td>current task</td><td>${robot.currentTask ? `Task #${escapeHtml(robot.currentTask.id)}` : "none"}</td></tr>
            <tr><td>last update</td><td>${robot.demo ? "demo/fallback position" : escapeHtml(fresh.label)}</td></tr>
            ${robot.device ? `
            <tr><td>link</td><td>${robot.linkStale ? "stale" : "live"}${robot.deviceAge === null ? "" : ` · status ${robot.deviceAge.toFixed(1)}s old`}</td></tr>
            <tr><td>reports</td><td>${escapeHtml(deviceLinkSummary(robot))}</td></tr>
            <tr><td>source topic</td><td>${escapeHtml(robot.device.source_topic || "unknown")}</td></tr>
            <tr><td>reported by</td><td>${escapeHtml(robot.device.source_id || robot.device.robot_id || "unknown")}</td></tr>` : ""}
          </tbody>
        </table>
      </div>
    `;
    return;
  }

  if (selected.type === "home") {
    const home = selected.home || homeStation();
    el.innerHTML = `
      <div class="item-card">
        <div class="item-top"><div class="item-title">Home station</div>${statusPill("base", "fresh")}</div>
        <table class="status-table"><tbody>
          <tr><td>role</td><td>Start/return location for robots</td></tr>
          <tr><td>lat/lon</td><td>${safeNumber(home.lat, 0).toFixed(6)}, ${safeNumber(home.lon, 0).toFixed(6)}</td></tr>
          <tr><td>local x/y</td><td>${safeNumber(home.x, 0).toFixed(2)}, ${safeNumber(home.y, 0).toFixed(2)} m</td></tr>
        </tbody></table>
      </div>`;
    return;
  }

  if (selected.type === "task") {
    const t = selected.task || {};
    const local = selected.local || {};
    el.innerHTML = `
      <div class="item-card">
        <div class="item-top">
          <div class="item-title">Task #${escapeHtml(t.id)}</div>
          ${statusPill(escapeHtml(t.status || "unknown"), taskStateToStatus(t.status))}
        </div>
        <table class="status-table">
          <tbody>
            <tr><td>terrain</td><td>${escapeHtml(taskTerrain(t))}</td></tr>
            <tr><td>suitable robot</td><td>${escapeHtml(suitableRobotNamesForTask(t).join(" / "))}</td></tr>
            <tr><td>assigned</td><td>${escapeHtml(t.assigned || t.assigned_to || "unassigned")}</td></tr>
            <tr><td>lat/lon</td><td>${safeNumber(t.lat, 0).toFixed(6)}, ${safeNumber(t.lon, 0).toFixed(6)}</td></tr>
            <tr><td>local x/y</td><td>${safeNumber(local.x, 0).toFixed(2)}, ${safeNumber(local.y, 0).toFixed(2)} m</td></tr>
            <tr><td>timestamp</td><td>${escapeHtml(t.ts || "unknown")}</td></tr>
          </tbody>
        </table>
      </div>
    `;
    return;
  }

  el.innerHTML = `<pre class="json-box">${escapeHtml(JSON.stringify(selected, null, 2))}</pre>`;
}


function bboxText(bbox) {
  if (!bbox) return "--";
  return `${safeNumber(bbox.x1, 0)},${safeNumber(bbox.y1, 0)} → ${safeNumber(bbox.x2, 0)},${safeNumber(bbox.y2, 0)}`;
}

function formatUv(value) {
  const number = safeNumber(value, NaN);
  return Number.isFinite(number) ? number.toFixed(4) : "--";
}

function formatConfidence(value) {
  const number = safeNumber(value, NaN);
  return Number.isFinite(number) ? number.toFixed(2) : "--";
}

function renderCameraDebug() {
  const el = $("camera-debug-content");
  if (!el) return;

  const data = OCTOPUS.latest.cameraDebug || null;
  const image = data?.image || null;
  const detectionPayload = data?.detections || null;
  const detections = Array.isArray(detectionPayload?.detections) ? detectionPayload.detections : [];

  const frameAge = ageSeconds(image?.received_at);
  const detectionAge = ageSeconds(detectionPayload?.received_at || detectionPayload?.timestamp);
  const frameFresh = freshnessFromAge(frameAge, 2.0, 8.0);
  const detectionFresh = freshnessFromAge(detectionAge, 2.0, 8.0);

  const cameraMessage = image
    ? detections.length > 0
      ? `${detections.length} detection${detections.length === 1 ? "" : "s"}`
      : "Camera frame live, no confirmed detections"
    : "No camera debug frame yet";

  const rows = detections.map((det) => `
    <tr>
      <td>#${escapeHtml(det.id ?? "--")}</td>
      <td>${escapeHtml(det.class_name || "rubbish")}</td>
      <td>${formatConfidence(det.confidence)}</td>
      <td>${formatUv(det.u)} / ${formatUv(det.v)}</td>
      <td>${escapeHtml(bboxText(det.bbox))}</td>
      <td>${statusPill(escapeHtml(det.status || "detected"), det.status === "confirmed" ? "fresh" : "warning")}</td>
    </tr>
  `).join("");

  el.innerHTML = `
    <div class="camera-debug-layout">
      <div>
        <div class="camera-debug-frame">
          ${image?.data_url
            ? `<img id="camera-debug-image" src="${image.data_url}" alt="Latest detector debug camera frame with bounding boxes" />`
            : `<div class="camera-debug-placeholder">Waiting for /detector_node/debug_image/compressed</div>`
          }
        </div>
      </div>
      <div>
        <div class="camera-debug-meta">
          ${statusPill(escapeHtml(frameFresh.label), frameFresh.state)}
          ${statusPill(escapeHtml(detectionFresh.label), detectionFresh.state)}
          <span class="mini-chip">${escapeHtml(cameraMessage)}</span>
          <span class="mini-chip">frame: ${escapeHtml(image?.frame_id || detectionPayload?.frame_id || "camera")}</span>
        </div>
        ${detections.length === 0
          ? `<div class="item-card"><div class="item-title">No detections</div><div class="item-meta">${escapeHtml(cameraMessage)}</div></div>`
          : `<table class="camera-debug-table">
              <thead><tr><th>ID</th><th>class</th><th>conf</th><th>u / v</th><th>bbox</th><th>status</th></tr></thead>
              <tbody>${rows}</tbody>
            </table>`
        }
      </div>
    </div>
  `;
}

function mergeDetectionCluster(items) {
  if (items.length === 1) return items[0];
  // Prefer a member that has a bbox (so we can draw a real frame), then carry the
  // highest confidence and a confirmed status/id if any member is confirmed.
  const hasBbox = (d) => d.bbox && Number.isFinite(safeNumber(d.bbox.x1, NaN));
  const base = items.find(hasBbox) || items[0];
  let conf = null;
  items.forEach((d) => {
    const c = safeNumber(d.confidence, NaN);
    if (Number.isFinite(c) && (conf === null || c > conf)) conf = c;
  });
  const confirmedItem = items.find((d) => d.status === "confirmed");
  return {
    ...base,
    id: (confirmedItem || base).id,
    confidence: conf !== null ? conf : base.confidence,
    status: confirmedItem ? "confirmed" : base.status,
  };
}

// Every detection the feed could draw, in FULL-frame u/v — before the camera
// crop is applied. Used for the crop's own bookkeeping (how many detections it
// currently hides); everything else wants cameraFeedDetections().
function cameraFeedDetectionsUncropped() {
  const payload = OCTOPUS.latest.cameraDebug?.detections || null;
  const list = Array.isArray(payload?.detections) ? payload.detections : [];
  // Only detections with usable normalized image coordinates can be placed on the feed.
  const usable = list.filter((det) => {
    const u = safeNumber(det.u, NaN);
    const v = safeNumber(det.v, NaN);
    return Number.isFinite(u) && Number.isFinite(v);
  });

  // The detector emits the same physical object twice — once as a raw YOLO
  // detection (with bbox) and once as a confirmed track (u/v only, no bbox) — with
  // different ids, so its id-based de-dup misses them and we would draw two
  // overlapping boxes. Merge detections that sit at the same image location.
  const MERGE_DIST = 0.05; // normalized u/v distance (~32 px on a 640-wide frame)
  const clusters = [];
  usable.forEach((det) => {
    const u = safeNumber(det.u, 0);
    const v = safeNumber(det.v, 0);
    const hit = clusters.find((c) => Math.hypot(c.u - u, c.v - v) <= MERGE_DIST);
    if (hit) hit.items.push(det);
    else clusters.push({ u, v, items: [det] });
  });
  return clusters.map((c) => mergeDetectionCluster(c.items));
}

// The detections the dashboard works with: re-normalized into the cropped frame,
// with everything that sits in a cut-away edge dropped.
function cameraFeedDetections() {
  const crop = cameraCropSettings();
  const list = cameraFeedDetectionsUncropped();
  if (!crop.active) return list;
  return list.map((det) => cropDetection(det, crop)).filter(Boolean);
}

// What the detector is seeing RIGHT NOW: how many pieces of trash sit in the
// current (cropped) frame and how sure it is about them. This deliberately does
// NOT fall back to task/stat counts from the database — those are mission
// bookkeeping that survives long after the object left the frame, so mixing them
// in would make the KPI look live while showing history.
function cameraTrashSummary() {
  const payload = OCTOPUS.latest.cameraDebug?.detections || null;
  if (!payload) return { available: false, count: 0, note: "No camera feed" };

  // The test feed / detector_node post at >= 1 Hz, so anything older than a few
  // seconds means the source died and the last count is not "current" any more.
  const age = ageSeconds(payload.received_at);
  if (age !== null && age > 5) {
    return { available: false, count: 0, note: `Detector stale · ${formatAge(age)}` };
  }

  const list = cameraFeedDetections();
  const confirmed = list.filter((det) => det.status === "confirmed").length;
  const confidences = list
    .map((det) => safeNumber(det.confidence, NaN))
    .filter((value) => Number.isFinite(value));
  const average = confidences.length
    ? confidences.reduce((sum, value) => sum + value, 0) / confidences.length
    : null;

  let note;
  if (!list.length) note = "Nothing in frame";
  else if (confirmed) note = `${confirmed} confirmed · avg ${Math.round(average * 100)}%`;
  else note = `avg ${Math.round(average * 100)}% confidence`;

  return { available: true, count: list.length, note };
}

// Compute the rectangle occupied by an object-fit:contain image inside its container.
function containedImageRect(container, natW, natH) {
  const cw = container.clientWidth;
  const ch = container.clientHeight;
  if (!natW || !natH || !cw || !ch) return { x: 0, y: 0, w: cw, h: ch, cw, ch };
  const scale = Math.min(cw / natW, ch / natH);
  const w = natW * scale;
  const h = natH * scale;
  return { x: (cw - w) / 2, y: (ch - h) / 2, w, h, cw, ch };
}

// -----------------------------------------------------------------------------
// Camera crop — the operator cuts the unusable frame edges away (drone legs in
// view, lens vignette, ground outside the area of interest). This is a region of
// interest, not just a zoom: the camera footprint, both grids, the cell names and
// the map projection are all computed from the cropped region, and detections
// that land in a cut-away edge are dropped instead of mapped.
//
// u runs from the left frame edge, v from the BOTTOM one (the detector's
// convention), so the bottom crop is v's lower bound and the top crop is v's
// upper bound.
// -----------------------------------------------------------------------------

// Validated crop, plus the kept fraction of each axis. kx/ky are never 0 because
// each side is capped at CAMERA_CROP_MAX_SIDE.
function cameraCropSettings() {
  const stored = OCTOPUS.cameraCrop || {};
  const crop = {};
  CAMERA_CROP_SIDES.forEach((side) => {
    crop[side] = clamp(safeNumber(stored[side], 0), 0, CAMERA_CROP_MAX_SIDE);
  });
  crop.kx = 1 - crop.left - crop.right;
  crop.ky = 1 - crop.top - crop.bottom;
  crop.active = crop.kx < 0.9999 || crop.ky < 0.9999;
  return crop;
}

function setCameraCropSide(side, fraction) {
  if (!CAMERA_CROP_SIDES.includes(side)) return;
  const next = {};
  const current = cameraCropSettings();
  CAMERA_CROP_SIDES.forEach((key) => { next[key] = current[key]; });
  next[side] = clamp(safeNumber(fraction, 0), 0, CAMERA_CROP_MAX_SIDE);
  OCTOPUS.cameraCrop = next;
  localStorage.setItem("octopusCameraCrop", JSON.stringify(next));
}

function resetCameraCrop() {
  const next = {};
  CAMERA_CROP_SIDES.forEach((side) => { next[side] = 0; });
  OCTOPUS.cameraCrop = next;
  localStorage.setItem("octopusCameraCrop", JSON.stringify(next));
}

// Rects for the camera overlay under the active crop:
//  - `rect` is the visible (cropped) region, letterboxed inside the container —
//    everything in normalized u/v maps into it;
//  - `full` is where the whole frame would sit at the same scale, for the values
//    that stay in full-frame pixel space (the detector bounding boxes).
function croppedImageRects(container, natW, natH) {
  const crop = cameraCropSettings();
  // When the ROS bridge already cut this frame (Eve sends only the crop), the
  // image IS the visible region — cropping it again in the browser would cut
  // twice. Otherwise the frame is still full and the browser does the cut.
  const preCropped = cameraFrameIsPreCropped(crop);
  const rect = preCropped
    ? containedImageRect(container, natW, natH)
    : containedImageRect(container, natW * crop.kx, natH * crop.ky);
  const full = { w: rect.w / crop.kx, h: rect.h / crop.ky };
  full.x = rect.x - crop.left * full.w;
  full.y = rect.y - crop.top * full.h;
  return { rect, full, crop, preCropped };
}

// Whether the frame that arrived was already cropped at the source with exactly
// the crop the operator asked for.
function cameraFrameIsPreCropped(crop = cameraCropSettings()) {
  if (!crop.active) return false;
  const applied = OCTOPUS.latest.cameraDebug?.image?.crop;
  if (!applied) return false;
  return CAMERA_CROP_SIDES.every(
    (side) => Math.abs(safeNumber(applied[side], -1) - crop[side]) <= 0.005
  );
}

// Re-normalize one detection into the cropped frame, or null when it sits in a
// cut-away edge.
function cropDetection(det, crop = cameraCropSettings()) {
  if (!crop.active) return det;
  const uFull = safeNumber(det.u, NaN);
  const vFull = safeNumber(det.v, NaN);
  const u = (uFull - crop.left) / crop.kx;
  const v = (vFull - crop.bottom) / crop.ky;
  if (!(u >= 0 && u <= 1) || !(v >= 0 && v <= 1)) return null;
  return { ...det, u, v, u_full: uFull, v_full: vFull };
}

// How many detections the crop currently hides, for the feed's status chips.
function cameraCropHiddenCount() {
  const crop = cameraCropSettings();
  if (!crop.active) return 0;
  const total = cameraFeedDetectionsUncropped().length;
  return Math.max(0, total - cameraFeedDetections().length);
}

// The cropped frame in sensor pixels, used both for the footprint model and for
// the "effective resolution" readout in the Camera & Pipeline panel.
function croppedSensorRegion(cam = OCTOPUS_HBVCAM_640X480, crop = cameraCropSettings()) {
  const x0 = crop.left * cam.image_width;
  const y0 = crop.top * cam.image_height;
  return {
    x0,
    y0,
    x1: cam.image_width - crop.right * cam.image_width,
    y1: cam.image_height - crop.bottom * cam.image_height,
    width_px: cam.image_width * crop.kx,
    height_px: cam.image_height * crop.ky,
    crop,
  };
}

// Position and clip the camera <img> so the cropped region alone fills the frame
// (object-fit:contain on the cropped region). The overlay uses the same rects, so
// image and overlay stay pixel-aligned.
function applyCameraCropToImage(img, container) {
  if (!img || !container) return;
  const crop = cameraCropSettings();
  const natW = img.naturalWidth;
  const natH = img.naturalHeight;

  if (!crop.active || !natW || !natH) {
    // Back to the stylesheet's plain contained image.
    ["left", "top", "right", "bottom", "width", "height", "clip-path", "object-fit"]
      .forEach((prop) => img.style.removeProperty(prop));
    return;
  }

  const rects = croppedImageRects(container, natW, natH);
  if (rects.preCropped) {
    // Already cut at the source: plain contained image, nothing to clip.
    ["left", "top", "right", "bottom", "width", "height", "clip-path", "object-fit"]
      .forEach((prop) => img.style.removeProperty(prop));
    return;
  }

  const full = rects.full;
  img.style.left = `${full.x}px`;
  img.style.top = `${full.y}px`;
  img.style.right = "auto";
  img.style.bottom = "auto";
  img.style.width = `${full.w}px`;
  img.style.height = `${full.h}px`;
  img.style.objectFit = "fill";
  img.style.clipPath =
    `inset(${(crop.top * 100).toFixed(4)}% ${(crop.right * 100).toFixed(4)}% ` +
    `${(crop.bottom * 100).toFixed(4)}% ${(crop.left * 100).toFixed(4)}%)`;
}

// -----------------------------------------------------------------------------
// Camera grid mode — the local (image-aligned) grid and the GPS graticule are
// mutually exclusive. Whichever is active is drawn on the camera image and on the
// mission map, and the trash-cell marking always follows it.
// -----------------------------------------------------------------------------

function cameraGridMode() {
  const mode = OCTOPUS.cameraFeed?.overlayGrid;
  return CAMERA_GRID_MODES.includes(mode) ? mode : "off";
}

function isLocalGridActive() {
  return cameraGridMode() === "local";
}

function isGpsGridActive() {
  return cameraGridMode() === "gps";
}

function markCellsEnabled() {
  return Boolean(OCTOPUS.cameraFeed.highlightCells);
}

// Whether the A1-style cell names are drawn on the camera image and the map. The
// readout below the feed lists them regardless — it is text, not overlay clutter.
function cellNamesEnabled() {
  return Boolean(OCTOPUS.cameraFeed.cellNames);
}

// Local grid layout: the column count drives roughly square cells, rows follow
// from the aspect ratio. That ratio comes from the camera FOOTPRINT (not from the
// image rect), because the camera overlay and the mission map must end up with
// the exact same cols/rows — otherwise the same cell name would mean different
// cells in the two views.
function localGridLayout() {
  const cols = clamp(parseInt(OCTOPUS.cameraFeed.gridCols, 10) || 0, 0, 40);
  if (cols < 1) return null;
  const meta = octopusComputeCameraFootprintMeta();
  const aspect = meta.width_m > 0 && meta.height_m > 0 ? meta.width_m / meta.height_m : 4 / 3;
  const rows = Math.max(1, Math.round(cols / aspect));
  return {
    cols,
    rows,
    // u/v bounds of one cell, in the detector's convention (v=0 at the image
    // bottom), so the cell can be projected to the map via the GPS model.
    cellUvBounds: (col, row) => ({
      uMin: col / cols,
      uMax: (col + 1) / cols,
      vMin: 1 - (row + 1) / rows,
      vMax: 1 - row / rows,
    }),
  };
}

// The same layout plus pixel sizes for one image rect.
function localGridGeometry(rect) {
  const layout = localGridLayout();
  if (!layout || !rect || !(rect.w > 0) || !(rect.h > 0)) return null;
  return { ...layout, cellW: rect.w / layout.cols, cellH: rect.h / layout.rows };
}

// Same A1-style naming as the GPS grid: columns A.. left to right, rows 1.. from
// the top of the image (= north when Eve's yaw is 0).
function localCellName(col, row) {
  return `${gpsColumnLetters(col)}${row + 1}`;
}

// The local grid cells that contain at least one detection, de-duplicated.
// Indices are image-space: col from the left edge, row from the TOP edge, so the
// detector's bottom-left v is flipped exactly once, here.
function localCellsForDetections(geom, detections = cameraFeedDetections()) {
  if (!geom) return [];
  const cells = new Map();
  detections.forEach((det) => {
    const u = safeNumber(det.u, NaN);
    const v = safeNumber(det.v, NaN);
    if (!Number.isFinite(u) || !Number.isFinite(v)) return;
    const col = Math.floor(clamp(u, 0, 0.999999) * geom.cols);
    const row = Math.floor((1 - clamp(v, 0, 0.999999)) * geom.rows);
    const key = `${col},${row}`;
    if (!cells.has(key)) {
      cells.set(key, { col, row, name: localCellName(col, row), detections: [] });
    }
    cells.get(key).detections.push(det);
  });
  return [...cells.values()];
}

// The GPS grid cells that contain at least one detection, de-duplicated.
function gpsCellsForDetections(grid, readout = null) {
  if (!grid) return [];
  const cells = new Map();
  (readout || gpsDetectionReadout(grid)).forEach((entry) => {
    if (!entry.cell) return;
    const key = `${entry.cell.col},${entry.cell.row}`;
    if (!cells.has(key)) cells.set(key, { ...entry.cell, detections: [] });
    cells.get(key).detections.push(entry.det);
  });
  return [...cells.values()];
}

// The active grid's cell name for one detection, for the labels shared by the
// camera overlay, the map tooltips and the readout. Null when no grid is active.
function activeCellNameForDetection(det, grid = null, geom = null) {
  if (isGpsGridActive()) {
    const model = grid || octopusGpsGridModel();
    if (!model) return null;
    const geo = model.uvToGeo(clamp(safeNumber(det.u, 0), 0, 1), clamp(safeNumber(det.v, 0), 0, 1));
    return model.cellFor(geo.lat, geo.lon)?.name || null;
  }
  if (isLocalGridActive() && geom) {
    const col = Math.floor(clamp(safeNumber(det.u, 0), 0, 0.999999) * geom.cols);
    const row = Math.floor((1 - clamp(safeNumber(det.v, 0), 0, 0.999999)) * geom.rows);
    return localCellName(col, row);
  }
  return null;
}

// -----------------------------------------------------------------------------
// GPS grid — a geographic (lat/lon) graticule drawn both over the camera image
// and on the mission map, so an operator can read off which coordinate / grid
// cell a piece of trash sits on.
//
// The whole thing is derived from Eve's position on the mission map (drag the
// marker or use "Set Eve"), her yaw, and the set drone height: height + the
// HBVCAM intrinsics give the ground footprint, which anchors the camera image
// in world coordinates. These are demo coordinates, not surveyed GPS.
// -----------------------------------------------------------------------------

const OCTOPUS_GPS_NICE_STEPS = [
  0.000001, 0.000002, 0.000005,
  0.00001, 0.00002, 0.00005,
  0.0001, 0.0002, 0.0005,
  0.001, 0.002, 0.005, 0.01,
];
const OCTOPUS_GPS_MAX_LINES = 48;

function gpsNiceStep(target) {
  const found = OCTOPUS_GPS_NICE_STEPS.find((s) => s >= target);
  return found || OCTOPUS_GPS_NICE_STEPS[OCTOPUS_GPS_NICE_STEPS.length - 1];
}

function gpsNextNiceStep(step) {
  const found = OCTOPUS_GPS_NICE_STEPS.find((s) => s > step * 1.0001);
  return found || step * 2;
}

// Column letters for the grid cells: 0 -> A, 25 -> Z, 26 -> AA.
function gpsColumnLetters(index) {
  let i = index;
  let out = "";
  do {
    out = String.fromCharCode(65 + (i % 26)) + out;
    i = Math.floor(i / 26) - 1;
  } while (i >= 0);
  return out;
}

function gpsDecimals(step) {
  return clamp(Math.ceil(-Math.log10(step)), 4, 8);
}

function formatGpsValue(value, decimals) {
  return value.toFixed(decimals);
}

// Liang-Barsky clip of a segment against an axis-aligned rectangle.
function clipSegmentToRect(p0, p1, rect) {
  const dx = p1.x - p0.x;
  const dy = p1.y - p0.y;
  const edges = [
    { p: -dx, q: p0.x - rect.x },
    { p: dx, q: rect.x + rect.w - p0.x },
    { p: -dy, q: p0.y - rect.y },
    { p: dy, q: rect.y + rect.h - p0.y },
  ];
  let t0 = 0;
  let t1 = 1;
  for (const { p, q } of edges) {
    if (p === 0) {
      if (q < 0) return null;
      continue;
    }
    const r = q / p;
    if (p < 0) {
      if (r > t1) return null;
      if (r > t0) t0 = r;
    } else {
      if (r < t0) return null;
      if (r < t1) t1 = r;
    }
  }
  return {
    a: { x: p0.x + t0 * dx, y: p0.y + t0 * dy },
    b: { x: p0.x + t1 * dx, y: p0.y + t1 * dy },
  };
}

// Eve's pose for the GPS grid. Unlike projectDetectionsToMap() this also accepts
// the configured fallback position, so the grid is demoable before a live fix.
function eveGeoPose() {
  const eve = getFleetSnapshot().find((r) => r.key === "eve");
  if (!eve) return null;
  const lat = safeNumber(eve.location?.lat, NaN);
  const lon = safeNumber(eve.location?.lon, NaN);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
  return {
    lat,
    lon,
    yawDeg: safeNumber(OCTOPUS.eveYawDeg, 0),
    manual: Boolean(eve.location?.manual),
    demo: Boolean(eve.demo),
  };
}

function octopusGpsGridModel() {
  const pose = eveGeoPose();
  if (!pose) return null;

  const meta = octopusComputeCameraFootprintMeta();
  const fp = meta.footprint;
  const widthM = fp.left_m + fp.right_m;
  const heightM = fp.top_m + fp.bottom_m;
  const th = (pose.yawDeg * Math.PI) / 180;
  const cos = Math.cos(th);
  const sin = Math.sin(th);
  const mPerDegLon = metersPerDegreeLonForLat(pose.lat);

  // Camera frame (right / forward, in meters on the ground) <-> geographic.
  const camToGeo = (offRight, offFwd) => ({
    lat: pose.lat + (-offRight * sin + offFwd * cos) / METERS_PER_DEGREE_LAT,
    lon: pose.lon + (offRight * cos + offFwd * sin) / mPerDegLon,
  });
  const geoToCam = (lat, lon) => {
    const east = (lon - pose.lon) * mPerDegLon;
    const north = (lat - pose.lat) * METERS_PER_DEGREE_LAT;
    return { right: east * cos - north * sin, fwd: east * sin + north * cos };
  };
  // Normalized image coordinates: u from the left edge, v from the bottom edge
  // (matching the detector's u/v convention).
  const geoToUv = (lat, lon) => {
    const c = geoToCam(lat, lon);
    return { u: (c.right + fp.left_m) / widthM, v: (c.fwd + fp.bottom_m) / heightM };
  };
  const uvToGeo = (u, v) => camToGeo(u * widthM - fp.left_m, v * heightM - fp.bottom_m);

  const corners = [
    camToGeo(-fp.left_m, -fp.bottom_m),
    camToGeo(fp.right_m, -fp.bottom_m),
    camToGeo(fp.right_m, fp.top_m),
    camToGeo(-fp.left_m, fp.top_m),
  ];
  const bbox = {
    latMin: Math.min(...corners.map((c) => c.lat)),
    latMax: Math.max(...corners.map((c) => c.lat)),
    lonMin: Math.min(...corners.map((c) => c.lon)),
    lonMax: Math.max(...corners.map((c) => c.lon)),
  };

  // One degree step for both axes, so the graticule reads like a real one.
  const requested = safeNumber(OCTOPUS.cameraFeed.gpsStepDeg, 0);
  const autoStep = gpsNiceStep(Math.max(bbox.latMax - bbox.latMin, bbox.lonMax - bbox.lonMin) / 6);
  let step = requested > 0 ? requested : autoStep;

  // Region: footprint bbox snapped outward to step multiples, plus one cell of
  // margin. Cell indices are counted from its north-west corner, so the camera
  // and the map always agree on cell names.
  const regionFor = (s) => {
    const colFrom = Math.floor(bbox.lonMin / s) - 1;
    const colTo = Math.ceil(bbox.lonMax / s) + 1;
    const rowFrom = Math.floor(bbox.latMin / s) - 1;
    const rowTo = Math.ceil(bbox.latMax / s) + 1;
    return { colFrom, colTo, rowFrom, rowTo, cols: colTo - colFrom, rows: rowTo - rowFrom };
  };
  let region = regionFor(step);
  let clamped = false;
  while ((region.cols > OCTOPUS_GPS_MAX_LINES || region.rows > OCTOPUS_GPS_MAX_LINES) && step < 0.01) {
    step = gpsNextNiceStep(step);
    region = regionFor(step);
    clamped = true;
  }

  const decimals = gpsDecimals(step);
  const lonLines = [];
  for (let i = 0; i <= region.cols; i++) lonLines.push((region.colFrom + i) * step);
  const latLines = [];
  for (let j = 0; j <= region.rows; j++) latLines.push((region.rowTo - j) * step); // north -> south

  // Cell name for a coordinate, e.g. "C3". Null outside the region.
  const cellFor = (lat, lon) => {
    const c = Math.floor(lon / step) - region.colFrom;
    const r = region.rowTo - 1 - Math.floor(lat / step);
    if (c < 0 || r < 0 || c >= region.cols || r >= region.rows) return null;
    return { col: c, row: r, name: `${gpsColumnLetters(c)}${r + 1}` };
  };
  const cellCenter = (c, r) => ({
    lat: (region.rowTo - r - 0.5) * step,
    lon: (region.colFrom + c + 0.5) * step,
  });
  // Geographic extent of one cell, so it can be filled as an area on either view.
  const cellBounds = (c, r) => ({
    latMin: (region.rowTo - r - 1) * step,
    latMax: (region.rowTo - r) * step,
    lonMin: (region.colFrom + c) * step,
    lonMax: (region.colFrom + c + 1) * step,
  });

  return {
    pose,
    footprint: fp,
    widthM,
    heightM,
    heightAboveGround: meta.camera_height_m,
    corners,
    bbox,
    step,
    stepAuto: requested <= 0,
    clamped,
    decimals,
    stepLatMeters: step * METERS_PER_DEGREE_LAT,
    stepLonMeters: step * mPerDegLon,
    region,
    lonLines,
    latLines,
    bounds: {
      latMin: region.rowFrom * step,
      latMax: region.rowTo * step,
      lonMin: region.colFrom * step,
      lonMax: region.colTo * step,
    },
    camToGeo,
    geoToCam,
    geoToUv,
    uvToGeo,
    cellFor,
    cellCenter,
    cellBounds,
    format: (value) => formatGpsValue(value, decimals),
  };
}

// The geo coordinate + grid cell of every current camera detection.
function gpsDetectionReadout(model, detections = null) {
  const grid = model || octopusGpsGridModel();
  if (!grid) return [];
  return (detections || cameraFeedDetections())
    .map((det) => {
      const u = safeNumber(det.u, NaN);
      const v = safeNumber(det.v, NaN);
      if (!Number.isFinite(u) || !Number.isFinite(v)) return null;
      const geo = grid.uvToGeo(clamp(u, 0, 1), clamp(v, 0, 1));
      return { det, ...geo, cell: grid.cellFor(geo.lat, geo.lon) };
    })
    .filter(Boolean);
}

// Draw the graticule over the camera image rect. Lines of constant lat/lon are
// straight in image space (the mapping is a rotation + scale), so two projected
// points per line, clipped to the image rect, are enough.
function drawCameraGpsGrid(ctx, rect, grid) {
  const toPixel = (lat, lon) => {
    const { u, v } = grid.geoToUv(lat, lon);
    return { x: rect.x + u * rect.w, y: rect.y + (1 - v) * rect.h };
  };

  // Pixel size of one cell, used to decide how much labelling fits. The camera
  // image is rotated against the graticule, so measure along both axes.
  const origin = toPixel(grid.bbox.latMin, grid.bbox.lonMin);
  const eastStep = toPixel(grid.bbox.latMin, grid.bbox.lonMin + grid.step);
  const northStep = toPixel(grid.bbox.latMin + grid.step, grid.bbox.lonMin);
  const cellPixW = Math.hypot(eastStep.x - origin.x, eastStep.y - origin.y);
  const cellPixH = Math.hypot(northStep.x - origin.x, northStep.y - origin.y);
  const lonEvery = Math.max(1, Math.ceil(52 / Math.max(cellPixW, 1)));
  const latEvery = Math.max(1, Math.ceil(30 / Math.max(cellPixH, 1)));

  ctx.save();
  ctx.beginPath();
  ctx.rect(rect.x, rect.y, rect.w, rect.h);
  ctx.clip();

  const drawLine = (p0, p1, major) => {
    const clipped = clipSegmentToRect(p0, p1, rect);
    if (!clipped) return null;
    ctx.beginPath();
    ctx.moveTo(clipped.a.x, clipped.a.y);
    ctx.lineTo(clipped.b.x, clipped.b.y);
    ctx.strokeStyle = major ? "rgba(56,189,248,0.85)" : "rgba(56,189,248,0.34)";
    ctx.lineWidth = major ? 1.4 : 0.8;
    ctx.stroke();
    return clipped;
  };

  const labels = [];
  ctx.font = "700 10px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.textBaseline = "middle";

  grid.lonLines.forEach((lon, i) => {
    const major = i % lonEvery === 0;
    const clipped = drawLine(
      toPixel(grid.bounds.latMin, lon),
      toPixel(grid.bounds.latMax, lon),
      major
    );
    if (!clipped || !major) return;
    // Label at whichever end of the visible segment sits lower in the frame.
    const at = clipped.a.y > clipped.b.y ? clipped.a : clipped.b;
    labels.push({ text: `${grid.format(lon)}°E`, x: at.x, y: at.y, align: "center", dy: -8 });
  });

  grid.latLines.forEach((lat, j) => {
    const major = j % latEvery === 0;
    const clipped = drawLine(
      toPixel(lat, grid.bounds.lonMin),
      toPixel(lat, grid.bounds.lonMax),
      major
    );
    if (!clipped || !major) return;
    const at = clipped.a.x < clipped.b.x ? clipped.a : clipped.b;
    labels.push({ text: `${grid.format(lat)}°N`, x: at.x, y: at.y, align: "left", dy: 0, dx: 4 });
  });

  // Cell names at the cell centers, once they are big enough to read.
  if (cellNamesEnabled() && Math.min(cellPixW, cellPixH) >= 24) {
    ctx.fillStyle = "rgba(212,255,0,0.72)";
    ctx.font = "800 10px ui-sans-serif, system-ui, sans-serif";
    ctx.textAlign = "center";
    for (let r = 0; r < grid.region.rows; r++) {
      for (let c = 0; c < grid.region.cols; c++) {
        const center = grid.cellCenter(c, r);
        const p = toPixel(center.lat, center.lon);
        if (p.x < rect.x + 6 || p.x > rect.x + rect.w - 6) continue;
        if (p.y < rect.y + 6 || p.y > rect.y + rect.h - 6) continue;
        ctx.fillText(`${gpsColumnLetters(c)}${r + 1}`, p.x, p.y);
      }
    }
  }

  // Axis value labels on top, each on its own dark plate for legibility.
  ctx.font = "700 10px ui-monospace, SFMono-Regular, Menlo, monospace";
  labels.forEach((label) => {
    ctx.textAlign = label.align;
    const w = ctx.measureText(label.text).width;
    const x = clamp(label.x + (label.dx || 0), rect.x + 2, rect.x + rect.w - 2);
    const y = clamp(label.y + (label.dy || 0), rect.y + 8, rect.y + rect.h - 8);
    const plateX = label.align === "center" ? x - w / 2 : x;
    ctx.fillStyle = "rgba(2,6,23,0.72)";
    ctx.fillRect(plateX - 3, y - 7, w + 6, 14);
    ctx.fillStyle = "#7dd3fc";
    ctx.fillText(label.text, x, y);
  });

  ctx.restore();
  ctx.textAlign = "left";
}

const CAMERA_MARK_COLOR = "251,146,60";

// Neon-yellow local grid over the camera image footprint.
function drawLocalGridOnCamera(ctx, rect, geom) {
  ctx.strokeStyle = "rgba(212,255,0,0.8)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let c = 0; c <= geom.cols; c++) {
    const x = Math.round(rect.x + geom.cellW * c) + 0.5;
    ctx.moveTo(x, rect.y);
    ctx.lineTo(x, rect.y + rect.h);
  }
  for (let r = 0; r <= geom.rows; r++) {
    const y = Math.round(rect.y + geom.cellH * r) + 0.5;
    ctx.moveTo(rect.x, y);
    ctx.lineTo(rect.x + rect.w, y);
  }
  ctx.stroke();

  // Frame border around the camera footprint.
  ctx.strokeStyle = "rgba(212,255,0,1)";
  ctx.lineWidth = 2;
  ctx.strokeRect(rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2);

  // Cell names, once the cells are big enough to read them.
  if (cellNamesEnabled() && Math.min(geom.cellW, geom.cellH) >= 26) {
    ctx.save();
    ctx.fillStyle = "rgba(212,255,0,0.6)";
    ctx.font = "800 10px ui-sans-serif, system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let r = 0; r < geom.rows; r++) {
      for (let c = 0; c < geom.cols; c++) {
        ctx.fillText(localCellName(c, r), rect.x + (c + 0.5) * geom.cellW, rect.y + r * geom.cellH + 3);
      }
    }
    ctx.restore();
  }
}

// Mark the local grid cells that contain a detection. Drawn on top of the grid so
// the trash cells stand out against the neon lines.
function markLocalCellsOnCamera(ctx, rect, geom, detections) {
  const cells = localCellsForDetections(geom, detections);
  if (!cells.length) return;

  ctx.save();
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "800 11px ui-sans-serif, system-ui, sans-serif";
  cells.forEach(({ col, row, name }) => {
    const x = rect.x + col * geom.cellW;
    const y = rect.y + row * geom.cellH;
    ctx.fillStyle = `rgba(${CAMERA_MARK_COLOR},0.30)`;
    ctx.fillRect(x, y, geom.cellW, geom.cellH);
    ctx.strokeStyle = `rgba(${CAMERA_MARK_COLOR},1)`;
    ctx.lineWidth = 2.5;
    ctx.strokeRect(x + 1.5, y + 1.5, geom.cellW - 3, geom.cellH - 3);
    if (cellNamesEnabled() && Math.min(geom.cellW, geom.cellH) >= 26) {
      drawCellNamePlate(ctx, name, x + geom.cellW / 2, y + geom.cellH / 2);
    }
  });
  ctx.restore();
}

// Mark the GPS grid cells that contain a detection. A GPS cell is a rotated
// rectangle in image space, so it is filled as a quad through geoToUv.
function markGpsCellsOnCamera(ctx, rect, grid, detections) {
  const cells = gpsCellsForDetections(grid, gpsDetectionReadout(grid, detections));
  if (!cells.length) return;

  const toPixel = (lat, lon) => {
    const { u, v } = grid.geoToUv(lat, lon);
    return { x: rect.x + u * rect.w, y: rect.y + (1 - v) * rect.h };
  };

  ctx.save();
  ctx.beginPath();
  ctx.rect(rect.x, rect.y, rect.w, rect.h);
  ctx.clip();
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.font = "800 11px ui-sans-serif, system-ui, sans-serif";

  cells.forEach(({ col, row, name }) => {
    const b = grid.cellBounds(col, row);
    const quad = [
      toPixel(b.latMin, b.lonMin),
      toPixel(b.latMin, b.lonMax),
      toPixel(b.latMax, b.lonMax),
      toPixel(b.latMax, b.lonMin),
    ];
    ctx.beginPath();
    ctx.moveTo(quad[0].x, quad[0].y);
    quad.slice(1).forEach((p) => ctx.lineTo(p.x, p.y));
    ctx.closePath();
    ctx.fillStyle = `rgba(${CAMERA_MARK_COLOR},0.30)`;
    ctx.fill();
    ctx.strokeStyle = `rgba(${CAMERA_MARK_COLOR},1)`;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    const center = toPixel((b.latMin + b.latMax) / 2, (b.lonMin + b.lonMax) / 2);
    const cellPix = Math.hypot(quad[1].x - quad[0].x, quad[1].y - quad[0].y);
    if (cellNamesEnabled() && cellPix >= 26) drawCellNamePlate(ctx, name, center.x, center.y);
  });

  ctx.restore();
  ctx.textAlign = "left";
}

// Cell name on a dark plate, so it stays readable over the marked cell's fill.
function drawCellNamePlate(ctx, name, x, y) {
  const w = ctx.measureText(name).width;
  ctx.fillStyle = "rgba(2,6,23,0.72)";
  ctx.fillRect(x - w / 2 - 4, y - 8, w + 8, 16);
  ctx.fillStyle = "#fdba74";
  ctx.fillText(name, x, y);
}

function drawCameraFeedOverlay() {
  const frame = $("camera-feed-frame");
  const img = $("camera-feed-image");
  const canvas = $("camera-feed-overlay");
  if (!frame || !canvas) return;

  const dpr = window.devicePixelRatio || 1;
  const cw = frame.clientWidth;
  const ch = frame.clientHeight;
  if (cw <= 0 || ch <= 0) return;

  canvas.width = Math.round(cw * dpr);
  canvas.height = Math.round(ch * dpr);
  canvas.style.width = `${cw}px`;
  canvas.style.height = `${ch}px`;

  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cw, ch);

  const hasImage = !!(img && img.src && img.complete && img.naturalWidth > 0);
  const gpsEnabled = isGpsGridActive();

  // Keep the <img> itself in sync with the crop before the early exits, so a
  // reset still un-crops a frame that no longer draws an overlay.
  applyCameraCropToImage(img, frame);

  if (!hasImage && !gpsEnabled) return;

  // Without a frame the GPS grid still draws, over the area the camera image
  // would occupy (sensor aspect ratio), so the feature is usable without the
  // pipeline running.
  // `rect` is the visible cropped region — everything normalized maps into it.
  // `fullRect` is the whole frame at the same scale, for detector bboxes, which
  // stay in full-frame pixel space.
  const rects = hasImage
    ? croppedImageRects(frame, img.naturalWidth, img.naturalHeight)
    : croppedImageRects(frame, OCTOPUS_HBVCAM_640X480.image_width, OCTOPUS_HBVCAM_640X480.image_height);
  const rect = rects.rect;
  const fullRect = rects.full;

  // Nothing may spill into the cut-away edges.
  ctx.save();
  ctx.beginPath();
  ctx.rect(rect.x, rect.y, rect.w, rect.h);
  ctx.clip();

  const detections = cameraFeedDetections();

  // GPS mode: the graticule, plus the cells that carry a detection.
  const gpsGrid = gpsEnabled ? octopusGpsGridModel() : null;
  if (gpsGrid) {
    drawCameraGpsGrid(ctx, rect, gpsGrid);
    if (markCellsEnabled()) markGpsCellsOnCamera(ctx, rect, gpsGrid, detections);
  }

  if (!hasImage) {
    ctx.restore();
    return;
  }

  // Local mode: the image-aligned grid, plus the cells that carry a detection.
  const localGeom = isLocalGridActive() ? localGridGeometry(rect) : null;
  if (localGeom) {
    drawLocalGridOnCamera(ctx, rect, localGeom);
    if (markCellsEnabled()) markLocalCellsOnCamera(ctx, rect, localGeom, detections);
  }

  // Green bounding box + center dot + label per detection. The detector no longer
  // burns anything into the frame, so the box, dot and label all come from the
  // dashboard here, drawn from the detections payload.
  if (detections.length) {
    const green = "46,232,111";
    ctx.font = "700 12px ui-sans-serif, system-ui, sans-serif";
    ctx.textBaseline = "middle";
    detections.forEach((det) => {
      const bbox = det.bbox || null;
      let bx;
      let by;
      let bw;
      let bh;
      const x1 = safeNumber(bbox?.x1, NaN);
      const y1 = safeNumber(bbox?.y1, NaN);
      const x2 = safeNumber(bbox?.x2, NaN);
      const y2 = safeNumber(bbox?.y2, NaN);
      if ([x1, y1, x2, y2].every(Number.isFinite)) {
        // bbox is in the detector debug frame's own pixel space (top-left origin),
        // exactly like the baked-in green box — scale into the contained rect, no v flip.
        const normalized = Math.max(x2, y2) <= 1.5;
        const sx = normalized ? fullRect.w : fullRect.w / (img.naturalWidth || 1);
        const sy = normalized ? fullRect.h : fullRect.h / (img.naturalHeight || 1);
        bx = fullRect.x + x1 * sx;
        by = fullRect.y + y1 * sy;
        bw = (x2 - x1) * sx;
        bh = (y2 - y1) * sy;
      } else {
        // No bbox (e.g. a confirmed marker) — box the v-flipped u/v point instead.
        const u = clamp(safeNumber(det.u, 0), 0, 1);
        const v = clamp(safeNumber(det.v, 0), 0, 1);
        const px = rect.x + u * rect.w;
        const py = rect.y + (1 - v) * rect.h;
        const half = Math.max(14, rect.w * 0.03);
        bx = px - half;
        by = py - half;
        bw = half * 2;
        bh = half * 2;
      }
      // Slight padding so the frame sits just outside the object.
      bx -= 3; by -= 3; bw += 6; bh += 6;

      // Dark halo underneath for contrast against bright ground, then the green box.
      ctx.strokeStyle = "rgba(0,0,0,0.55)";
      ctx.lineWidth = 4;
      ctx.strokeRect(bx, by, bw, bh);
      ctx.strokeStyle = `rgba(${green},1)`;
      ctx.lineWidth = 2;
      ctx.strokeRect(bx, by, bw, bh);

      // Center dot at the middle of the trash object.
      const dotX = bx + bw / 2;
      const dotY = by + bh / 2;
      ctx.beginPath();
      ctx.arc(dotX, dotY, 4, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(${green},1)`;
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = "rgba(0,0,0,0.6)";
      ctx.stroke();

      // Label chip: "class 0.80" (no id, decimal confidence), above the box.
      // With a grid active, the cell the object sits in is appended — the GPS cell
      // in GPS mode, the local cell in local mode.
      const parts = [det.class_name || "rubbish"];
      const conf = safeNumber(det.confidence, NaN);
      if (Number.isFinite(conf)) parts.push(conf.toFixed(2));
      const cellName = cellNamesEnabled() ? activeCellNameForDetection(det, gpsGrid, localGeom) : null;
      if (cellName) parts.push(`· ${cellName}`);
      const label = parts.join(" ");
      const padX = 5;
      const chipH = 18;
      const tw = ctx.measureText(label).width;
      let lx = bx;
      let ly = by - chipH - 2;
      if (ly < rect.y) ly = by + bh + 2;
      lx = clamp(lx, rect.x, rect.x + rect.w - tw - padX * 2);
      ly = clamp(ly, rect.y, rect.y + rect.h - chipH);
      ctx.fillStyle = "rgba(0,0,0,0.5)";
      ctx.fillRect(lx - 1, ly - 1, tw + padX * 2 + 2, chipH + 2);
      ctx.fillStyle = `rgba(${green},0.95)`;
      ctx.fillRect(lx, ly, tw + padX * 2, chipH);
      ctx.fillStyle = "#06140b";
      ctx.fillText(label, lx + padX, ly + chipH / 2);
    });
  }

  ctx.restore(); // end of the crop clip
}

function renderCameraFeed() {
  const img = $("camera-feed-image");
  const placeholder = $("camera-feed-placeholder");
  const meta = $("camera-feed-meta");
  if (!img || !meta) return;

  const data = OCTOPUS.latest.cameraDebug || null;
  const image = data?.image || null;
  const detectionPayload = data?.detections || null;
  const dataUrl = image?.data_url || null;
  const detections = cameraFeedDetections();

  const frameAge = ageSeconds(image?.received_at);
  const detectionAge = ageSeconds(detectionPayload?.received_at || detectionPayload?.timestamp);
  const frameFresh = freshnessFromAge(frameAge, 2.0, 8.0);
  const detectionFresh = freshnessFromAge(detectionAge, 2.0, 8.0);

  if (dataUrl) {
    if (placeholder) placeholder.style.display = "none";
    img.style.display = "block";
    const signature = `${image?.received_at || ""}:${dataUrl.length}`;
    if (OCTOPUS.cameraFeed.imageSignature !== signature) {
      OCTOPUS.cameraFeed.imageSignature = signature;
      img.onload = () => drawCameraFeedOverlay();
      img.src = dataUrl;
    } else {
      drawCameraFeedOverlay();
    }
  } else {
    if (placeholder) placeholder.style.display = "";
    img.style.display = "none";
    if (img.src) img.removeAttribute("src");
    OCTOPUS.cameraFeed.imageSignature = null;
    drawCameraFeedOverlay();
  }

  const countClass = detections.length ? "" : "empty";
  const countLabel = detections.length
    ? `${detections.length} trash detection${detections.length === 1 ? "" : "s"}`
    : "No trash detected";

  // Crop chip, so the operator can never mistake a cropped feed for the full one.
  const crop = cameraCropSettings();
  const hidden = cameraCropHiddenCount();
  const cropChip = crop.active
    ? `<span class="mini-chip camera-crop-chip" title="${escapeHtml(cameraCropTooltip())}">Crop ${escapeHtml(cameraCropShortLabel())}` +
      `${hidden ? ` · ${hidden} hidden` : ""}</span>`
    : "";

  meta.innerHTML = `
    ${statusPill(escapeHtml(`Frame ${frameFresh.label}`), frameFresh.state)}
    ${statusPill(escapeHtml(`Detections ${detectionFresh.label}`), detectionFresh.state)}
    <span class="camera-feed-count ${countClass}"><span class="swatch"></span>${escapeHtml(countLabel)}</span>
    ${cropChip}
    <span class="spacer"></span>
    <span class="mini-chip">frame: ${escapeHtml(image?.frame_id || detectionPayload?.frame_id || "camera")}</span>
  `;

  renderCameraGridReadout();
}

// Readout below the camera feed, for whichever grid is active: cell size, Eve's
// own position, and the cell (plus coordinate, in GPS mode) of every detection
// currently in frame.
function renderCameraGridReadout() {
  const box = $("camera-gps-readout");
  if (!box) return;

  const mode = cameraGridMode();
  if (mode === "off") {
    box.hidden = true;
    box.innerHTML = "";
    return;
  }

  box.hidden = false;
  box.innerHTML = (mode === "gps" ? gpsReadoutChips() : localReadoutChips()).join("");
}

function gpsReadoutChips() {
  const grid = octopusGpsGridModel();
  if (!grid) {
    return [`<span class="gps-chip hint">No Eve position yet — place Eve on the mission map to anchor the GPS grid.</span>`];
  }

  const chips = [];
  chips.push(
    `<span class="gps-chip">GPS grid <b>${escapeHtml(grid.format(grid.step))}°</b> ≈ ` +
    `${grid.stepLonMeters.toFixed(2)} m × ${grid.stepLatMeters.toFixed(2)} m / cell` +
    `${grid.clamped ? " (coarsened to fit)" : grid.stepAuto ? " (auto)" : ""}</span>`
  );
  chips.push(
    `<span class="gps-chip">Eve <b>${escapeHtml(grid.format(grid.pose.lat))}°N ` +
    `${escapeHtml(grid.format(grid.pose.lon))}°E</b> · h=${grid.heightAboveGround.toFixed(2)} m · ` +
    `yaw ${Math.round(grid.pose.yawDeg)}°${grid.pose.manual ? " · dragged" : grid.pose.demo ? " · fallback" : ""}</span>`
  );

  const readout = gpsDetectionReadout(grid);
  if (readout.length) {
    readout.forEach(({ det, lat, lon, cell }) => {
      const conf = safeNumber(det.confidence, NaN);
      chips.push(
        `<span class="gps-chip trash">🗑 <b>${escapeHtml(cell ? cell.name : "--")}</b> ` +
        `${escapeHtml(grid.format(lat))}°N ${escapeHtml(grid.format(lon))}°E` +
        `${Number.isFinite(conf) ? ` · ${conf.toFixed(2)}` : ""}</span>`
      );
    });
  } else {
    chips.push(
      `<span class="gps-chip hint">No trash in frame — drag Eve on the map to move the footprint.</span>`
    );
  }
  return chips;
}

// The local grid is defined in image space, so it needs no Eve position — only
// the footprint size, to say how big one cell is on the ground.
function localReadoutChips() {
  const layout = localGridLayout();
  if (!layout) {
    return [`<span class="gps-chip hint">Local grid has no columns configured.</span>`];
  }

  const meta = octopusComputeCameraFootprintMeta();
  const chips = [
    `<span class="gps-chip">Local grid <b>${layout.cols}×${layout.rows}</b> cells ≈ ` +
    `${(meta.width_m / layout.cols).toFixed(2)} m × ${(meta.height_m / layout.rows).toFixed(2)} m / cell ` +
    `· footprint ${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m at h=${meta.camera_height_m.toFixed(2)} m</span>`,
  ];

  const cells = localCellsForDetections(layout);
  if (cells.length) {
    cells.forEach(({ name, detections }) => {
      const best = detections.reduce(
        (acc, det) => Math.max(acc, safeNumber(det.confidence, 0)),
        0
      );
      const count = detections.length > 1 ? ` ×${detections.length}` : "";
      chips.push(
        `<span class="gps-chip trash">🗑 <b>${escapeHtml(name)}</b>${escapeHtml(count)}` +
        `${best > 0 ? ` · ${best.toFixed(2)}` : ""}</span>`
      );
    });
  } else {
    chips.push(`<span class="gps-chip hint">No trash in frame.</span>`);
  }
  return chips;
}

async function refreshCameraDebug() {
  try {
    const data = await apiGet("/api/camera_debug/latest");
    OCTOPUS.latest.cameraDebug = data.status === "ok" ? data : null;
    resyncCameraCropIfNeeded(data?.crop);
  } catch (error) {
    OCTOPUS.latest.cameraDebug = null;
    console.warn("Camera debug refresh failed", error);
  }
  renderCameraDebug();
  renderCameraFeed();
  // The trash KPI and the mission map both read the camera feed - the KPI for its
  // count, the map for the detections projected onto the ground - so both belong
  // on the 1 Hz camera poll. Left on the 5 s mission refresh, the map lagged the
  // feed by up to five seconds and kept drawing trash that had already left the
  // frame, which is the opposite of showing what the drone sees right now.
  renderKpis();
  renderMissionMap();
}

function renderMapPatch() {
  const el = $("map-patch-content");
  if (!el) return;

  const patch = OCTOPUS.latest.patch;
  if (!patch) {
    el.innerHTML = `<div class="item-card"><div class="item-title">No map patch received yet</div><div class="item-meta">POST to /api/map_patch to test the ROS2-to-dashboard path.</div></div>`;
    return;
  }

  const cells = patch.updated_cells || [];
  const age = ageSeconds(patch.received_at);
  const fresh = freshnessFromAge(age, 2, 10);

  el.innerHTML = `
    <div class="compact-list">
      <div class="item-card">
        <div class="item-top">
          <div class="item-title">${cells.length} updated cells</div>
          ${statusPill(fresh.label, fresh.state)}
        </div>
        <div class="item-meta">Frame: <span class="accent">${escapeHtml(patch.frame_id || "unknown")}</span></div>
      </div>
      <pre class="json-box">${escapeHtml(JSON.stringify({
        frame_id: patch.frame_id,
        received_at: patch.received_at,
        updated_cells_preview: cells.slice(0, 3),
      }, null, 2))}</pre>
    </div>
  `;
}

function renderTimeline() {
  const el = $("timeline-content");
  if (!el) return;

  if (!OCTOPUS.timeline.length) {
    el.innerHTML = `
      <div class="timeline-list">
        <div class="timeline-event"><span class="time">${formatTime()}</span><span>Dashboard loaded. Waiting for mission data.</span></div>
      </div>
    `;
    return;
  }

  el.innerHTML = `<div class="timeline-list">${OCTOPUS.timeline.slice(0, 8).map((event) => {
    const className = event.level === "error" ? "error-text" : event.level === "warning" ? "warning-text" : event.level === "success" ? "success-text" : "";
    return `<div class="timeline-event"><span class="time">${formatTime(event.time)}</span><span class="${className}">${escapeHtml(event.message)}</span></div>`;
  }).join("")}</div>`;
}

function renderAll() {
  renderMissionPhase();
  renderTopStatus();
  renderKpis();
  renderFleet();
  renderTasks();
  renderSystemHealth();
  renderReadiness();
  renderInspector();
  renderMapPatch();
  renderTimeline();
  renderMissionMap();
  renderCameraFeed();

  if (getSelectedGridSource() === "local_camera" || OCTOPUS.latest.globalMap || OCTOPUS.osmPriors) {
    drawGridMap(OCTOPUS.latest.globalMap || {});
  }
  updateOsmButton(OCTOPUS.osmPriors ? `OSM priors: ${OCTOPUS.osmPriors.features || 0}` : "Load OSM priors");
}

async function loadTasks() {
  OCTOPUS.latest.tasks = await apiGet("/api/tasks");
}

async function loadBattery() {
  OCTOPUS.latest.battery = await apiGet("/api/battery");
}

async function loadLocations() {
  try {
    OCTOPUS.latest.locations = await apiGet("/api/locations?limit=100");
  } catch (error) {
    // Locations are optional during early prototypes. Keep the rest of the dashboard alive.
    OCTOPUS.latest.locations = [];
  }
}

async function loadStats() {
  OCTOPUS.latest.stats = await apiGet("/api/stats");
}

async function loadMapPatch() {
  const data = await apiGet("/api/map_patch/latest");
  OCTOPUS.latest.patch = data.status === "ok" ? data.patch : null;

  if (OCTOPUS.latest.patch) {
    const signature = `${OCTOPUS.latest.patch.received_at || ""}-${OCTOPUS.latest.patch.updated_cells?.length || 0}`;
    if (signature !== OCTOPUS.seenPatchSignature) {
      OCTOPUS.seenPatchSignature = signature;
      addTimeline(`Map patch received: ${OCTOPUS.latest.patch.updated_cells?.length || 0} updated cells`, "success");
    }
  }
}

async function loadGlobalMap() {
  const data = await apiGet("/api/global_map/latest");
  OCTOPUS.latest.globalMap = data.status === "ok" ? data.map : null;
}


async function loadLocalCameraGrid() {
  try {
    const data = await apiGet("/api/local_camera_grid/latest");
    OCTOPUS.latest.localCameraGrid = data.status === "ok" ? data : null;
  } catch (error) {
    console.warn("Local camera grid refresh failed", error);
    OCTOPUS.latest.localCameraGrid = null;
  }
}

// Live state of the ground robots on the GripperX link. They publish
// /octopus/devices/<id>/status, device_status_backend_bridge_node forwards it to
// the backend, and this is where the dashboard picks it up.
async function loadDeviceStatus() {
  try {
    const data = await apiGet("/api/devices/status");
    OCTOPUS.latest.devices = data.devices || {};
    // The backend's clock, not the browser's: the robot's timestamps and
    // backend_received_at are both server-side, so ages must be measured
    // against the server or a skewed laptop clock invents staleness.
    OCTOPUS.latest.devicesServerTime = data.server_time ?? null;
  } catch (error) {
    console.warn("Device status refresh failed", error);
    OCTOPUS.latest.devices = {};
    OCTOPUS.latest.devicesServerTime = null;
  }
}

async function refreshAll() {
  await loadLocalCameraGrid();
  try {
    await Promise.all([
      loadTasks(),
      loadBattery(),
      loadLocations(),
      loadStats(),
      loadMapPatch(),
      loadGlobalMap(),
      loadDeviceStatus(),
    ]);
    OCTOPUS.backendOk = true;
    OCTOPUS.lastError = null;
    OCTOPUS.lastRefresh = new Date();
  } catch (error) {
    OCTOPUS.backendOk = false;
    OCTOPUS.lastError = error.message;
    addTimeline(`Backend/API error: ${error.message}`, "error");
    console.error("Octopus dashboard refresh failed", error);
  }

  renderAll();
}

function canvasToLocalMeters(geom, x, y) {
  return {
    x: ((x - geom.originX) / geom.cellSize) * geom.resolution,
    y: ((geom.gridH - (y - geom.originY)) / geom.cellSize) * geom.resolution,
  };
}

function findCellAtLocal(mapData, xMeters, yMeters) {
  const cells = getMergedGridData(mapData || {}).cells || {};
  for (const [key, cell] of Object.entries(cells)) {
    const mb = cellMetricBounds(cell, mapData);
    if (xMeters >= mb.x0 && xMeters <= mb.x1 && yMeters >= mb.y0 && yMeters <= mb.y1) {
      return { key, cell };
    }
  }
  return null;
}

function onGridClick(event) {
  if (OCTOPUS.gridView.moved) {
    OCTOPUS.gridView.moved = false;
    return;
  }
  const mapData = OCTOPUS.latest.globalMap || {};
  if (!mapData && !OCTOPUS.osmPriors) return;

  const canvas = $("grid-map-canvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const x = (event.clientX - rect.left) * dpr;
  const y = (event.clientY - rect.top) * dpr;

  const geom = getGridGeometry(mapData);
  const local = canvasToLocalMeters(geom, x, y);
  if (local.x < 0 || local.x > geom.width_m || local.y < 0 || local.y > geom.height_m) return;

  const hit = findCellAtLocal(mapData, local.x, local.y);
  let key;
  let cell;
  if (hit) {
    key = hit.key;
    cell = hit.cell;
  } else {
    const { row, col } = canvasToGridCell(geom, x, y);
    if (row < 0 || row >= geom.rows || col < 0 || col >= geom.cols) return;
    key = `${row},${col}`;
    cell = { row, col, x: local.x, y: local.y, coverage: 0, trash_probability: 0, obstacle_probability: 0, confidence: 0 };
  }

  OCTOPUS.selectedCellKey = key;
  OCTOPUS.selected = { type: "grid_cell", cell };
  renderInspector();
  drawGridMap(mapData);
}

function onGridWheel(event) {
  const mapData = OCTOPUS.latest.globalMap || {};
  if (!mapData && !OCTOPUS.osmPriors) return;
  event.preventDefault();
  const canvas = $("grid-map-canvas");
  const rect = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const mouseX = (event.clientX - rect.left) * dpr;
  const mouseY = (event.clientY - rect.top) * dpr;
  const oldScale = OCTOPUS.gridView.scale;
  const factor = event.deltaY < 0 ? 1.15 : 1 / 1.15;
  const newScale = clamp(oldScale * factor, 0.35, 24);
  OCTOPUS.gridView.offsetX = mouseX - (mouseX - OCTOPUS.gridView.offsetX) * (newScale / oldScale);
  OCTOPUS.gridView.offsetY = mouseY - (mouseY - OCTOPUS.gridView.offsetY) * (newScale / oldScale);
  OCTOPUS.gridView.scale = newScale;
  drawGridMap(mapData);
}

function onGridPointerDown(event) {
  const canvas = $("grid-map-canvas");
  const dpr = window.devicePixelRatio || 1;
  OCTOPUS.gridView.isPanning = true;
  OCTOPUS.gridView.moved = false;
  OCTOPUS.gridView.lastX = event.clientX * dpr;
  OCTOPUS.gridView.lastY = event.clientY * dpr;
  canvas.setPointerCapture?.(event.pointerId);
}

function onGridPointerMove(event) {
  if (!OCTOPUS.gridView.isPanning) return;
  const mapData = OCTOPUS.latest.globalMap || {};
  if (!mapData && !OCTOPUS.osmPriors) return;
  const dpr = window.devicePixelRatio || 1;
  const x = event.clientX * dpr;
  const y = event.clientY * dpr;
  const dx = x - OCTOPUS.gridView.lastX;
  const dy = y - OCTOPUS.gridView.lastY;
  if (Math.abs(dx) + Math.abs(dy) > 2) OCTOPUS.gridView.moved = true;
  OCTOPUS.gridView.offsetX += dx;
  OCTOPUS.gridView.offsetY += dy;
  OCTOPUS.gridView.lastX = x;
  OCTOPUS.gridView.lastY = y;
  drawGridMap(mapData);
}

function onGridPointerUp(event) {
  const canvas = $("grid-map-canvas");
  OCTOPUS.gridView.isPanning = false;
  canvas.releasePointerCapture?.(event.pointerId);
}

function zoomGridBy(factor) {
  const mapData = OCTOPUS.latest.globalMap || {};
  if (!mapData && !OCTOPUS.osmPriors) return;
  OCTOPUS.gridView.scale = clamp(OCTOPUS.gridView.scale * factor, 0.35, 24);
  drawGridMap(mapData);
}

function fitLocalGrid() {
  OCTOPUS.gridView = { scale: 1, offsetX: 0, offsetY: 0, isPanning: false, lastX: 0, lastY: 0, moved: false };
  if (OCTOPUS.latest.globalMap) drawGridMap(OCTOPUS.latest.globalMap);
}


function onGridKeyDown(event) {
  const key = event.key.toLowerCase();
  const step = 32 * (window.devicePixelRatio || 1);
  if (["arrowleft", "arrowright", "arrowup", "arrowdown", "+", "=", "-", "f"].includes(key)) {
    event.preventDefault();
  } else {
    return;
  }

  if (key === "arrowleft") OCTOPUS.gridView.offsetX += step;
  if (key === "arrowright") OCTOPUS.gridView.offsetX -= step;
  if (key === "arrowup") OCTOPUS.gridView.offsetY += step;
  if (key === "arrowdown") OCTOPUS.gridView.offsetY -= step;
  if (key === "+" || key === "=") zoomGridBy(1.2);
  if (key === "-") zoomGridBy(1 / 1.2);
  if (key === "f") fitLocalGrid();

  drawGridMap(OCTOPUS.latest.globalMap || {});
}

function setupCameraFeedControls() {
  const modeSelect = $("camera-grid-mode");
  const colsControl = $("camera-feed-cols-control");
  const colsSelect = $("camera-feed-grid-cols");
  const stepControl = $("camera-gps-step-control");
  const gpsStep = $("camera-gps-step");
  const namesControl = $("camera-cell-names-control");

  // Only the active mode's own controls are shown. Cell names exist in both grid
  // modes, so that one is hidden only when no grid is drawn at all.
  const syncModeControls = () => {
    const mode = cameraGridMode();
    if (colsControl) colsControl.hidden = mode !== "local";
    if (stepControl) stepControl.hidden = mode !== "gps";
    if (namesControl) namesControl.hidden = mode === "off";
  };

  if (modeSelect) {
    modeSelect.value = cameraGridMode();
    modeSelect.addEventListener("change", () => {
      OCTOPUS.cameraFeed.overlayGrid = CAMERA_GRID_MODES.includes(modeSelect.value)
        ? modeSelect.value
        : "off";
      localStorage.setItem("octopusCameraGridMode", OCTOPUS.cameraFeed.overlayGrid);
      syncModeControls();
      redrawCameraGrid();
      addTimeline(cameraGridModeMessage(), "info");
    });
  }

  if (colsSelect) {
    colsSelect.value = String(OCTOPUS.cameraFeed.gridCols);
    colsSelect.addEventListener("change", () => {
      OCTOPUS.cameraFeed.gridCols = parseInt(colsSelect.value, 10) || 8;
      localStorage.setItem("octopusCameraFeedGridCols", String(OCTOPUS.cameraFeed.gridCols));
      redrawCameraGrid();
    });
  }

  const highlight = $("camera-feed-highlight");
  if (highlight) {
    highlight.checked = OCTOPUS.cameraFeed.highlightCells;
    highlight.addEventListener("change", () => {
      OCTOPUS.cameraFeed.highlightCells = highlight.checked;
      localStorage.setItem("octopusCameraFeedHighlight", highlight.checked ? "1" : "0");
      redrawCameraGrid();
    });
  }

  const cellNames = $("camera-cell-names");
  if (cellNames) {
    cellNames.checked = OCTOPUS.cameraFeed.cellNames;
    cellNames.addEventListener("change", () => {
      OCTOPUS.cameraFeed.cellNames = cellNames.checked;
      localStorage.setItem("octopusCameraCellNames", cellNames.checked ? "1" : "0");
      redrawCameraGrid();
    });
  }

  if (gpsStep) {
    gpsStep.value = String(OCTOPUS.cameraFeed.gpsStepDeg || 0);
    gpsStep.addEventListener("change", () => {
      OCTOPUS.cameraFeed.gpsStepDeg = parseFloat(gpsStep.value) || 0;
      localStorage.setItem("octopusCameraGpsStepDeg", String(OCTOPUS.cameraFeed.gpsStepDeg));
      redrawCameraGrid();
    });
  }
  syncModeControls();

  const frame = $("camera-feed-frame");
  if (frame && typeof ResizeObserver !== "undefined") {
    const observer = new ResizeObserver(() => drawCameraFeedOverlay());
    observer.observe(frame);
  }
  window.addEventListener("resize", drawCameraFeedOverlay);
}

// Timeline line for a mode switch, naming the cell size the operator now reads.
function cameraGridModeMessage() {
  const mode = cameraGridMode();
  if (mode === "off") return "Camera grid off.";

  if (mode === "local") {
    const layout = localGridLayout();
    const meta = octopusComputeCameraFootprintMeta();
    return layout
      ? `Local grid on: ${layout.cols}×${layout.rows} cells over the camera footprint ` +
        `(~${(meta.width_m / layout.cols).toFixed(2)} m per cell).`
      : "Local grid on, but no columns are configured.";
  }

  const grid = octopusGpsGridModel();
  return grid
    ? `GPS grid on: ${grid.format(grid.step)}° per cell (~${grid.stepLonMeters.toFixed(2)} m) around Eve.`
    : "GPS grid on, but Eve has no position yet. Place Eve on the mission map.";
}

// The active grid lives on the camera overlay and on the mission map, so anything
// that changes it (mode, cells, step, drone height, Eve position, yaw) redraws both.
function redrawCameraGrid() {
  drawCameraFeedOverlay();
  renderCameraGridReadout();
  if (OCTOPUS.missionMap?.map) renderMissionMap();
}

function setupEventListeners() {
  const phaseSelect = $("mission-phase-select");
  if (phaseSelect) {
    phaseSelect.addEventListener("change", () => {
      OCTOPUS.missionPhase = phaseSelect.value;
      localStorage.setItem("octopusMissionPhase", OCTOPUS.missionPhase);
      addTimeline(`Mission phase changed to ${PHASE_INFO[OCTOPUS.missionPhase]?.title || OCTOPUS.missionPhase}`, "info");
      renderMissionPhase();
    });
  }

  const viewSelect = $("dashboard-view-select");
  if (viewSelect) {
    viewSelect.addEventListener("change", () => {
      applyDashboardView(viewSelect.value, true);
      renderAll();
      // Canvas needs a re-measure once the overview layout is applied.
      setTimeout(drawCameraFeedOverlay, 90);
    });
  }

  setupCameraFeedControls();

  const mapModeSelector = $("mission-map-mode");
  if (mapModeSelector) {
    mapModeSelector.addEventListener("change", () => {
      renderMissionMap();
      addTimeline(`Mission map mode changed to ${mapModeSelector.value}`, "info");
    });
  }

  const missionFitButton = $("mission-fit-button");
  if (missionFitButton) {
    missionFitButton.addEventListener("click", fitMissionMap);
  }

  const setEveButton = $("set-eve-button");
  if (setEveButton) {
    setEveButton.addEventListener("click", () => {
      setEvePlacementMode(!OCTOPUS.missionMap.placeEveMode);
    });
  }

  const clearEveButton = $("clear-eve-button");
  if (clearEveButton) {
    clearEveButton.addEventListener("click", clearManualEve);
  }

  initEveYawSourceToggle();

  const yawInput = $("eve-yaw-input");
  const yawLabel = $("eve-yaw-label");
  if (yawInput) {
    const initial = Math.round((((OCTOPUS.eveYawDeg || 0) % 360) + 360) % 360);
    yawInput.value = String(initial);
    if (yawLabel) yawLabel.textContent = `${initial}°`;
    let yawRafPending = false;
    yawInput.addEventListener("input", () => {
      OCTOPUS.eveYawDeg = parseFloat(yawInput.value) || 0;
      // Getrennt gemerkt: wer vom Kompass zurueckschaltet, bekommt seinen
      // eigenen Winkel wieder und nicht den letzten Kompasswert.
      OCTOPUS.eveYawManualDeg = OCTOPUS.eveYawDeg;
      localStorage.setItem("octopusEveYawDeg", String(OCTOPUS.eveYawDeg));
      if (yawLabel) yawLabel.textContent = `${Math.round(OCTOPUS.eveYawDeg)}°`;
      if (!yawRafPending) {
        yawRafPending = true;
        requestAnimationFrame(() => {
          yawRafPending = false;
          // Yaw rotates the camera footprint, so the active grid follows it.
          if (cameraGridMode() !== "off") redrawCameraGrid();
          else renderMissionMap();
        });
      }
    });
  }

  const projectButton = $("project-detections-button");
  if (projectButton) {
    projectButton.classList.toggle("area-active", OCTOPUS.projectDetections !== false);
    projectButton.addEventListener("click", () => {
      OCTOPUS.projectDetections = !(OCTOPUS.projectDetections !== false);
      localStorage.setItem("octopusProjectDetections", OCTOPUS.projectDetections ? "1" : "0");
      projectButton.classList.toggle("area-active", OCTOPUS.projectDetections);
      addTimeline(`Detection projection ${OCTOPUS.projectDetections ? "enabled" : "disabled"}.`, "info");
      renderMissionMap();
    });
  }

  const drawPolygonButton = $("draw-polygon-button");
  if (drawPolygonButton) {
    drawPolygonButton.addEventListener("click", () => {
      if (OCTOPUS.missionMap.polygonMode) {
        clearPolygonPreview(true);
        addTimeline("Polygon drawing cancelled.", "warning");
      } else {
        startPolygonArea();
      }
    });
  }

  const finishPolygonButton = $("finish-polygon-button");
  if (finishPolygonButton) {
    finishPolygonButton.addEventListener("click", finishPolygonArea);
  }

  const clearAreaButton = $("clear-area-button");
  if (clearAreaButton) {
    clearAreaButton.addEventListener("click", clearMissionArea);
  }

  const useViewButton = $("use-view-area-button");
  if (useViewButton) {
    useViewButton.addEventListener("click", () => {
      if (!OCTOPUS.missionMap.map) return;
      boundsToMissionArea(OCTOPUS.missionMap.map.getBounds(), "visible map view");
    });
  }

  const resolutionInput = $("grid-resolution-input");
  if (resolutionInput) {
    resolutionInput.addEventListener("change", () => {
      updateResolutionLabel();
      if (OCTOPUS.missionArea?.bounds && OCTOPUS.missionMap.map && typeof L !== "undefined") {
        const b = OCTOPUS.missionArea.bounds;
        boundsToMissionArea(
          L.latLngBounds([b.south, b.west], [b.north, b.east]),
          OCTOPUS.missionArea.source || "resolution update",
          OCTOPUS.missionArea.polygon || null,
        );
      } else {
        fitLocalGrid();
        renderMissionMap();
      }
      addTimeline("Grid cell size changed. Existing scanned patches keep their real-world size.", "info");
    });
  }


  const osmButton = $("load-osm-priors-button");
  if (osmButton) {
    osmButton.addEventListener("click", () => loadOsmPriors());
  }

  const layerSelector = $("grid-layer-select");
  if (layerSelector) {
    layerSelector.addEventListener("change", () => {
      if (OCTOPUS.latest.globalMap || OCTOPUS.osmPriors) drawGridMap(OCTOPUS.latest.globalMap || {});
      renderMissionGridOverlay();
    });
  }

  syncGridDisplayControls();
  document.querySelectorAll("[data-grid-display]").forEach((input) => {
    input.addEventListener("change", () => {
      const key = input.dataset.gridDisplay;
      OCTOPUS.gridDisplay[key] = input.checked;
      saveGridDisplaySettings();
      drawGridMap(OCTOPUS.latest.globalMap || {});
      renderMissionGridOverlay();
      addTimeline(`Display layer ${key} ${input.checked ? "enabled" : "hidden"}.`, "info");
    });
  });

  document.addEventListener("click", (event) => {
    const menu = $("grid-display-menu");
    if (menu && menu.open && !menu.contains(event.target)) menu.open = false;
  });

  const canvas = $("grid-map-canvas");
  if (canvas) {
    canvas.addEventListener("keydown", onGridKeyDown);
    canvas.addEventListener("click", onGridClick);
    canvas.addEventListener("wheel", onGridWheel, { passive: false });
    canvas.addEventListener("pointerdown", onGridPointerDown);
    canvas.addEventListener("pointermove", onGridPointerMove);
    canvas.addEventListener("pointerup", onGridPointerUp);
    canvas.addEventListener("pointerleave", onGridPointerUp);
  }

  const fitButton = $("fit-grid-button");
  if (fitButton) {
    fitButton.addEventListener("click", fitLocalGrid);
  }

  const zoomIn = $("grid-zoom-in-button");
  if (zoomIn) zoomIn.addEventListener("click", () => zoomGridBy(1.2));

  const zoomOut = $("grid-zoom-out-button");
  if (zoomOut) zoomOut.addEventListener("click", () => zoomGridBy(1 / 1.2));

  window.addEventListener("resize", () => {
    if (OCTOPUS.latest.globalMap) drawGridMap(OCTOPUS.latest.globalMap);
    if (OCTOPUS.missionMap.map) OCTOPUS.missionMap.map.invalidateSize();
  });
}

initMissionMap();
updateResolutionLabel();
applyDashboardView(OCTOPUS.dashboardView, false);
setupEventListeners();
renderMissionPhase();
renderTimeline();
refreshAll();
setInterval(refreshAll, 5000);
// Camera poll rate. test_camera_feed.py posts at 5 fps (200 ms) and detector_node
// is in the same range, so 400 ms picks up every second frame - fast enough that
// the feed and the detections on the map read as live rather than stepping. This
// drives renderCameraFeed, renderKpis and renderMissionMap, so raising it further
// costs a full Leaflet layer rebuild per tick.
const CAMERA_POLL_MS = 400;
refreshCameraDebug();
setInterval(refreshCameraDebug, CAMERA_POLL_MS);


// --- OCTOPUS EVE CAMERA FRONTEND ---
async function eveFetchJson(url, options = {}) {
  const response = await fetch(url, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.detail || `HTTP ${response.status}`);
  }
  return data;
}

function setEveCameraUi(status, detail = "") {
  const pill = document.getElementById("eve-camera-status-pill");
  const summary = document.getElementById("eve-camera-summary");

  let label = "Eve camera";
  let cls = "unknown";

  if (status === "camera_running") {
    label = "Eve camera running";
    cls = "ok";
  } else if (status === "camera_started") {
    label = "Eve camera started";
    cls = "ok";
  } else if (status === "online_camera_stopped") {
    label = "Eve online, camera stopped";
    cls = "warning";
  } else if (status === "camera_stopped") {
    label = "Eve camera stopped";
    cls = "warning";
  } else if (status === "offline") {
    label = "Eve offline";
    cls = "offline";
  } else if (status === "camera_failed" || status === "camera_stop_failed") {
    label = "Eve camera error";
    cls = "error";
  }

  if (pill) {
    pill.className = `pill ${cls}`;
    const span = pill.querySelector("span:last-child");
    if (span) span.textContent = label;
  }

  if (summary) {
    summary.textContent = detail ? `${label}: ${detail}` : label;
  }
}

async function refreshEveCameraStatus() {
  try {
    const data = await eveFetchJson("/api/eve/status");
    OCTOPUS.latest.eveStatus = { status: data.status, at: Date.now() / 1000 };
    setEveCameraUi(data.status, data.ssh?.stdout || "");
    if (typeof renderKpis === "function") renderKpis();
    if (typeof renderFleet === "function") renderFleet();
    return data;
  } catch (error) {
    OCTOPUS.latest.eveStatus = { status: "offline", at: Date.now() / 1000 };
    setEveCameraUi("offline", error.message);
    if (typeof renderKpis === "function") renderKpis();
    if (typeof renderFleet === "function") renderFleet();
    return null;
  }
}

async function startEveCamera() {
  setEveCameraUi("unknown", "starting camera...");
  try {
    const data = await eveFetchJson("/api/eve/start_camera", { method: "POST" });
    setEveCameraUi(data.status, data.ssh?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline("Eve camera start command executed.", data.status === "camera_started" ? "success" : "warning");
    }
  } catch (error) {
    setEveCameraUi("camera_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`Eve camera start failed: ${error.message}`, "error");
  }
}

async function stopEveCamera() {
  setEveCameraUi("unknown", "stopping camera...");
  try {
    const data = await eveFetchJson("/api/eve/stop_camera", { method: "POST" });
    setEveCameraUi(data.status, data.ssh?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline("Eve camera stop command executed.", data.status === "camera_stopped" ? "success" : "warning");
    }
  } catch (error) {
    setEveCameraUi("camera_stop_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`Eve camera stop failed: ${error.message}`, "error");
  }
}

async function showEveCameraLog() {
  const logEl = document.getElementById("eve-camera-log");
  if (!logEl) return;

  logEl.style.display = "block";
  logEl.textContent = "Loading Eve camera log...";

  try {
    const data = await eveFetchJson("/api/eve/camera_log");
    logEl.textContent = data.log || "(empty log)";
  } catch (error) {
    logEl.textContent = `Failed to load log: ${error.message}`;
  }
}

function initEveCameraControls() {
  const startBtn = document.getElementById("eve-start-camera-btn");
  const stopBtn = document.getElementById("eve-stop-camera-btn");
  const refreshBtn = document.getElementById("eve-refresh-status-btn");
  const logBtn = document.getElementById("eve-show-log-btn");

  if (startBtn) startBtn.addEventListener("click", startEveCamera);
  if (stopBtn) stopBtn.addEventListener("click", stopEveCamera);
  if (refreshBtn) refreshBtn.addEventListener("click", refreshEveCameraStatus);
  if (logBtn) logBtn.addEventListener("click", showEveCameraLog);

  refreshEveCameraStatus();
  setInterval(refreshEveCameraStatus, 8000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initEveCameraControls);
} else {
  initEveCameraControls();
}
// --- END OCTOPUS EVE CAMERA FRONTEND ---


// --- OCTOPUS EVE PX4 BRIDGE + DETECTOR FRONTEND ---
// Beide Bloecke folgen dem Muster der Eve-Kamera darueber: ein setXUi(), ein
// refresh, start/stop, ein Log-Button und ein Poll-Intervall.

// Ein Block, weil die drei Zustandsanzeigen sich nur in ihren Element-Ids und
// ihrer Statuszuordnung unterscheiden. Ein zweites Mal dasselbe hinzuschreiben
// hiesse, jede spaetere Aenderung an zwei Stellen zu machen.
function setControlBlockUi(summaryId, label, cls, detail) {
  const summary = document.getElementById(summaryId);
  if (!summary) return;
  summary.textContent = detail ? `${label}: ${detail}` : label;
  summary.dataset.state = cls;
}

const PX4_BRIDGE_LABELS = {
  px4_bridge_running: ["PX4 bridge running", "ok"],
  px4_bridge_started: ["PX4 bridge started", "ok"],
  online_px4_bridge_stopped: ["Eve online, PX4 bridge stopped", "warning"],
  px4_bridge_stopped: ["PX4 bridge stopped", "warning"],
  offline: ["Eve offline", "offline"],
  px4_bridge_failed: ["PX4 bridge error", "error"],
  px4_bridge_stop_failed: ["PX4 bridge error", "error"],
};

function setPx4BridgeUi(status, detail = "") {
  const [label, cls] = PX4_BRIDGE_LABELS[status] || ["PX4 bridge", "unknown"];
  setControlBlockUi("px4-bridge-summary", label, cls, detail);
}

async function refreshPx4BridgeStatus() {
  try {
    const data = await eveFetchJson("/api/eve/px4_bridge/status");
    OCTOPUS.latest.px4BridgeStatus = { status: data.status, at: Date.now() / 1000 };
    setPx4BridgeUi(data.status, data.ssh?.stdout || "");
    if (typeof renderFleet === "function") renderFleet();
    return data;
  } catch (error) {
    OCTOPUS.latest.px4BridgeStatus = { status: "offline", at: Date.now() / 1000 };
    setPx4BridgeUi("offline", error.message);
    if (typeof renderFleet === "function") renderFleet();
    return null;
  }
}

async function startPx4Bridge() {
  setPx4BridgeUi("unknown", "starting PX4 bridge...");
  try {
    const data = await eveFetchJson("/api/eve/px4_bridge/start", { method: "POST" });
    // Der Agent laeuft auch ohne Pixhawk am anderen Ende. Das sichtbar zu
    // machen spart die Suche im Log, wenn spaeter keine Odometrie ankommt.
    let detail = data.ssh?.stdout || "";
    if (data.pixhawk === "waiting") {
      detail = "agent up, but no Pixhawk session yet — is the Pixhawk powered?";
    } else if (data.pixhawk === "connected") {
      detail = "Pixhawk connected";
    }
    setPx4BridgeUi(data.status, detail);
    if (typeof addTimeline === "function") {
      addTimeline("Eve PX4 bridge start command executed.",
        data.status === "px4_bridge_started" ? "success" : "warning");
    }
  } catch (error) {
    setPx4BridgeUi("px4_bridge_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`PX4 bridge start failed: ${error.message}`, "error");
  }
}

async function stopPx4Bridge() {
  setPx4BridgeUi("unknown", "stopping PX4 bridge...");
  try {
    const data = await eveFetchJson("/api/eve/px4_bridge/stop", { method: "POST" });
    setPx4BridgeUi(data.status, data.ssh?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline("Eve PX4 bridge stop command executed.",
        data.status === "px4_bridge_stopped" ? "success" : "warning");
    }
  } catch (error) {
    setPx4BridgeUi("px4_bridge_stop_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`PX4 bridge stop failed: ${error.message}`, "error");
  }
}

async function showPx4BridgeLog() {
  const logEl = document.getElementById("px4-bridge-log");
  if (!logEl) return;
  logEl.style.display = "block";
  logEl.textContent = "Loading PX4 bridge log...";
  try {
    const data = await eveFetchJson("/api/eve/px4_bridge/log");
    logEl.textContent = data.log || "(empty log)";
  } catch (error) {
    logEl.textContent = `Failed to load log: ${error.message}`;
  }
}

const DETECTOR_LABELS = {
  detector_running: ["Detector running", "ok"],
  detector_started: ["Detector started", "ok"],
  detector_loading: ["Detector loading model", "warning"],
  detector_stopped: ["Detector stopped", "warning"],
  detector_failed: ["Detector error", "error"],
  detector_stop_failed: ["Detector error", "error"],
};

function setDetectorUi(status, detail = "") {
  const [label, cls] = DETECTOR_LABELS[status] || ["Detector", "unknown"];
  setControlBlockUi("detector-summary", label, cls, detail);
}

async function refreshDetectorStatus() {
  try {
    const data = await eveFetchJson("/api/detector/status");
    OCTOPUS.latest.detectorStatus = { status: data.status, at: Date.now() / 1000 };
    setDetectorUi(data.status, data.local?.stdout || "");
    if (typeof renderFleet === "function") renderFleet();
    return data;
  } catch (error) {
    OCTOPUS.latest.detectorStatus = { status: "detector_failed", at: Date.now() / 1000 };
    setDetectorUi("detector_failed", error.message);
    if (typeof renderFleet === "function") renderFleet();
    return null;
  }
}

async function startDetector() {
  setDetectorUi("unknown", "starting detector...");
  try {
    const data = await eveFetchJson("/api/detector/start", { method: "POST" });
    setDetectorUi(data.status, data.local?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline("Detector start command executed.",
        data.status === "detector_started" ? "success" : "warning");
    }
    // Der Start-Endpunkt kehrt zurueck, sobald der Prozess steht -- YOLO laedt
    // dann noch. Ein Nachfassen macht aus "started" das ehrlichere
    // "loading"/"running", ohne dass jemand auf Refresh druecken muss.
    setTimeout(refreshDetectorStatus, 4000);
  } catch (error) {
    setDetectorUi("detector_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`Detector start failed: ${error.message}`, "error");
  }
}

async function stopDetector() {
  setDetectorUi("unknown", "stopping detector...");
  try {
    const data = await eveFetchJson("/api/detector/stop", { method: "POST" });
    setDetectorUi(data.status, data.local?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline("Detector stop command executed.",
        data.status === "detector_stopped" ? "success" : "warning");
    }
  } catch (error) {
    setDetectorUi("detector_stop_failed", error.message);
    if (typeof addTimeline === "function") addTimeline(`Detector stop failed: ${error.message}`, "error");
  }
}

async function showDetectorLog() {
  const logEl = document.getElementById("detector-log");
  if (!logEl) return;
  logEl.style.display = "block";
  logEl.textContent = "Loading detector log...";
  try {
    const data = await eveFetchJson("/api/detector/log");
    logEl.textContent = data.log || "(empty log)";
  } catch (error) {
    logEl.textContent = `Failed to load log: ${error.message}`;
  }
}

// Aufgeklappt beginnt der Block am unteren Rand des Panels -- ohne das hier
// sieht man einen Streifen Buttons und muesste selbst scrollen.
function initSystemsDetails() {
  const details = document.querySelector(".systems-details");
  if (!details) return;
  details.addEventListener("toggle", () => {
    if (!details.open) return;
    const scroller = details.closest(".fleet-scroll");
    if (!scroller) return;
    // Nach dem Reflow, sonst ist die neue Hoehe noch nicht bekannt.
    requestAnimationFrame(() => {
      scroller.scrollTop = details.offsetTop - scroller.offsetTop;
    });
  });
}

function initPx4BridgeAndDetectorControls() {
  const bind = (id, handler) => {
    const el = document.getElementById(id);
    if (el) el.addEventListener("click", handler);
  };

  bind("px4-bridge-start-btn", startPx4Bridge);
  bind("px4-bridge-stop-btn", stopPx4Bridge);
  bind("px4-bridge-refresh-btn", refreshPx4BridgeStatus);
  bind("px4-bridge-log-btn", showPx4BridgeLog);

  bind("detector-start-btn", startDetector);
  bind("detector-stop-btn", stopDetector);
  bind("detector-refresh-btn", refreshDetectorStatus);
  bind("detector-log-btn", showDetectorLog);

  initSystemsDetails();
  refreshPx4BridgeStatus();
  refreshDetectorStatus();
  // Dieselben 8 s wie die Kamera. Jeder Tick der PX4-Zeile ist ein SSH-Aufruf,
  // haeufiger lohnt sich nicht -- der Zustand aendert sich nur, wenn jemand
  // einen Knopf drueckt.
  setInterval(refreshPx4BridgeStatus, 8000);
  setInterval(refreshDetectorStatus, 8000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initPx4BridgeAndDetectorControls);
} else {
  initPx4BridgeAndDetectorControls();
}
// --- END OCTOPUS EVE PX4 BRIDGE + DETECTOR FRONTEND ---


// --- OCTOPUS CAMERA-TO-GRID PIPELINE FRONTEND ---
async function pipelineFetchJson(url, options = {}) {
  const response = await fetch(url, options);
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(data.detail || `HTTP ${response.status}`);
  }
  return data;
}

function setPipelineUi(status, detail = "") {
  const pill = document.getElementById("pipeline-status-pill");
  const summary = document.getElementById("pipeline-summary");

  let label = "Pipeline";
  let cls = "unknown";

  if (status === "pipeline_running" || status === "pipeline_started") {
    label = "Pipeline running";
    cls = "ok";
  } else if (status === "pipeline_partial") {
    label = "Pipeline partial";
    cls = "warning";
  } else if (status === "pipeline_stopped") {
    label = "Pipeline stopped";
    cls = "muted";
  } else if (status === "pipeline_failed" || status === "pipeline_stop_failed") {
    label = "Pipeline error";
    cls = "error";
  } else if (status === "offline") {
    label = "Pipeline offline";
    cls = "offline";
  }

  if (pill) {
    pill.className = `pill ${cls}`;
    const labelEl = pill.querySelector("span:last-child");
    if (labelEl) labelEl.textContent = label;
  }

  if (summary) {
    summary.textContent = detail ? `${label}: ${detail}` : label;
  }
}

async function refreshPipelineStatus() {
  try {
    const data = await pipelineFetchJson("/api/pipeline/status");
    setPipelineUi(data.status, data.local?.stdout || "");
    return data;
  } catch (error) {
    setPipelineUi("offline", error.message);
    return null;
  }
}

async function startCameraGridPipeline() {
  setPipelineUi("unknown", "starting local ROS2 pipeline...");
  try {
    const data = await pipelineFetchJson("/api/pipeline/start", { method: "POST" });
    setPipelineUi(data.status, data.local?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline(
        "Camera-to-grid pipeline start command executed.",
        data.status === "pipeline_started" ? "success" : "warning"
      );
    }
  } catch (error) {
    setPipelineUi("pipeline_failed", error.message);
    if (typeof addTimeline === "function") {
      addTimeline(`Camera-to-grid pipeline start failed: ${error.message}`, "error");
    }
  }
}

async function stopCameraGridPipeline() {
  setPipelineUi("unknown", "stopping local ROS2 pipeline...");
  try {
    const data = await pipelineFetchJson("/api/pipeline/stop", { method: "POST" });
    setPipelineUi(data.status, data.local?.stdout || "");
    if (typeof addTimeline === "function") {
      addTimeline(
        "Camera-to-grid pipeline stop command executed.",
        data.status === "pipeline_stopped" ? "success" : "warning"
      );
    }
  } catch (error) {
    setPipelineUi("pipeline_stop_failed", error.message);
    if (typeof addTimeline === "function") {
      addTimeline(`Camera-to-grid pipeline stop failed: ${error.message}`, "error");
    }
  }
}

async function showCameraGridPipelineLogs() {
  const logEl = document.getElementById("pipeline-log");
  if (!logEl) return;

  logEl.style.display = "block";
  logEl.textContent = "Loading pipeline logs...";

  try {
    const data = await pipelineFetchJson("/api/pipeline/logs");
    logEl.textContent = data.logs || "(empty logs)";
  } catch (error) {
    logEl.textContent = `Failed to load pipeline logs: ${error.message}`;
  }
}

function initCameraGridPipelineControls() {
  const startBtn = document.getElementById("pipeline-start-btn");
  const stopBtn = document.getElementById("pipeline-stop-btn");
  const refreshBtn = document.getElementById("pipeline-refresh-btn");
  const logBtn = document.getElementById("pipeline-log-btn");

  if (startBtn) startBtn.addEventListener("click", startCameraGridPipeline);
  if (stopBtn) stopBtn.addEventListener("click", stopCameraGridPipeline);
  if (refreshBtn) refreshBtn.addEventListener("click", refreshPipelineStatus);
  if (logBtn) logBtn.addEventListener("click", showCameraGridPipelineLogs);

  refreshPipelineStatus();
  setInterval(refreshPipelineStatus, 8000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initCameraGridPipelineControls);
} else {
  initCameraGridPipelineControls();
}
// --- END OCTOPUS CAMERA-TO-GRID PIPELINE FRONTEND ---


// --- OCTOPUS FORCE PIPELINE CONTROLS IN SYSTEM DEBUG ---
function octopusEnsurePipelineControlsVisible() {
  if (document.getElementById("pipeline-start-btn")) {
    return;
  }

  const eveSummary = document.getElementById("eve-camera-summary");

  let target = eveSummary;

  if (!target) {
    const panels = Array.from(document.querySelectorAll(".panel, section, div"));
    const systemPanel = panels.find((el) => {
      const text = el.textContent || "";
      return text.includes("System Health") && text.includes("Connect Eve Camera");
    });

    if (systemPanel) {
      target = systemPanel;
    }
  }

  if (!target) {
    return;
  }

  const wrapper = document.createElement("div");
  wrapper.className = "pipeline-control-block";
  wrapper.innerHTML = `
    <div class="pipeline-control-title">
      <strong>Camera-to-Grid Pipeline</strong>
      <span>starts local ROS2 mapping, backend bridge, and camera-marker transform nodes</span>
    </div>
    <div class="pipeline-controls">
      <button id="pipeline-start-btn" type="button">Start Camera-to-Grid Pipeline</button>
      <button id="pipeline-stop-btn" type="button">Stop Pipeline</button>
      <button id="pipeline-refresh-btn" type="button">Refresh Pipeline Status</button>
      <button id="pipeline-log-btn" type="button">Show Pipeline Logs</button>
    </div>
    <div id="pipeline-summary" class="muted">Pipeline status: unknown</div>
    <pre id="pipeline-log" class="pipeline-log-box"></pre>
  `;

  if (eveSummary && eveSummary.parentNode) {
    eveSummary.insertAdjacentElement("afterend", wrapper);
  } else {
    target.appendChild(wrapper);
  }
}

function octopusBindPipelineControlsAgain() {
  octopusEnsurePipelineControlsVisible();

  const startBtn = document.getElementById("pipeline-start-btn");
  const stopBtn = document.getElementById("pipeline-stop-btn");
  const refreshBtn = document.getElementById("pipeline-refresh-btn");
  const logBtn = document.getElementById("pipeline-log-btn");

  if (startBtn) startBtn.onclick = startCameraGridPipeline;
  if (stopBtn) stopBtn.onclick = stopCameraGridPipeline;
  if (refreshBtn) refreshBtn.onclick = refreshPipelineStatus;
  if (logBtn) logBtn.onclick = showCameraGridPipelineLogs;
}

setInterval(octopusBindPipelineControlsAgain, 1000);

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", octopusBindPipelineControlsAgain);
} else {
  octopusBindPipelineControlsAgain();
}
// --- END OCTOPUS FORCE PIPELINE CONTROLS IN SYSTEM DEBUG ---


// --- Camera transform status UI ---
// The status can come from either transform pipeline and the two payloads differ:
// camera_marker_transform_node reports AprilTag visibility ("ok" / "stale_*",
// is_transform_allowed, *_marker_ids), flight_camera_transform_node reports drone
// pose readiness ("ready" / "pose_only" / …, transform_ready, reason). Everything
// below normalizes both into one shape so no panel claims "0/4 markers" while the
// flight path is the one running.
const CAMERA_TRANSFORM_STALE_SEC = 6;

function cameraTransformUiState(state, transformAllowed) {
  const value = String(state || "unknown").toLowerCase();

  if (value === "ok" && transformAllowed) return "fresh";
  if (value === "ready" && transformAllowed) return "fresh";
  if (value === "stale_warning") return "warning";
  if (value === "stale_drop") return "error";
  if (value === "not_ready") return "warning";
  if (value === "pose_only") return "warning";
  if (value === "pose_ready_projection_disabled") return "warning";
  if (value === "unknown") return "unknown";

  return transformAllowed ? "fresh" : "warning";
}

function cameraTransformLabel(state) {
  const value = String(state || "unknown").toLowerCase();

  if (value === "ok") return "Transform OK";
  if (value === "ready") return "Transform OK";
  if (value === "not_ready") return "Transform not ready";
  if (value === "pose_only") return "Transform pose only";
  if (value === "pose_ready_projection_disabled") return "Projection disabled";
  if (value === "stale_warning") return "Transform stale";
  if (value === "stale_drop") return "Transform blocked";
  return "Transform unknown";
}

// Single source of truth for every panel that shows the transform state.
function cameraTransformView(status, errorMessage = "") {
  const payload = status || {};
  // Only the AprilTag pipeline knows about markers. Without that field the
  // marker rows are not "0 of 4 visible", they are simply not applicable.
  const markerBased = Array.isArray(payload.required_marker_ids);
  const detected = Array.isArray(payload.detected_marker_ids) ? payload.detected_marker_ids : [];
  const missing = Array.isArray(payload.missing_marker_ids) ? payload.missing_marker_ids : [];
  const required = markerBased ? payload.required_marker_ids : [];
  const allowed = markerBased
    ? Boolean(payload.is_transform_allowed)
    : Boolean(payload.transform_ready);

  const state = String(payload.state || "unknown").toLowerCase();
  // Both nodes publish at 1 Hz. If nothing new arrived the backend still serves the
  // last payload forever, so age is what tells us the bridge or node died.
  const age = ageSeconds(payload.backend_received_at);
  const stale = age !== null && age > CAMERA_TRANSFORM_STALE_SEC;
  const hasStatus = Boolean(payload.state);

  let uiState;
  let label;
  if (errorMessage) {
    uiState = "offline";
    label = "Transform offline";
  } else if (!hasStatus) {
    uiState = "unknown";
    label = "Transform unknown";
  } else if (stale) {
    uiState = "error";
    label = "Transform stale";
  } else {
    uiState = cameraTransformUiState(state, allowed);
    label = cameraTransformLabel(state);
  }

  let detail;
  if (errorMessage) {
    detail = errorMessage;
  } else if (!hasStatus) {
    detail = "no status received yet";
  } else if (stale) {
    detail = `${state} · last update ${formatAge(age)}`;
  } else if (markerBased) {
    detail = `${state} · ${detected.length}/${required.length} markers · ${allowed ? "allowed" : "blocked"}`;
  } else {
    detail = `${state} · ${payload.reason || (allowed ? "transform allowed" : "transform blocked")}`;
  }

  return {
    status: payload,
    state,
    hasStatus,
    stale,
    age,
    uiState,
    label,
    detail,
    allowed,
    markerBased,
    detected,
    missing,
    required,
    mode: payload.mode || null,
  };
}

// Marker readiness as its own row: not applicable on the flight path.
function cameraTransformMarkerRow(view) {
  if (!view.markerBased) {
    return {
      state: "unknown",
      detail: view.hasStatus
        ? `not applicable · ${view.mode || "flight pose path"}`
        : "no status received yet",
    };
  }
  return {
    state: view.detected.length === view.required.length && view.required.length > 0 ? "fresh" : "warning",
    detail: view.missing.length
      ? `${view.detected.length}/${view.required.length} visible · missing ${view.missing.join(", ")}`
      : `${view.detected.length}/${view.required.length} visible`,
  };
}

function cameraTransformAgeText(age) {
  if (age === null || age === undefined || !Number.isFinite(Number(age))) return "none";
  const seconds = Number(age);
  if (seconds < 1) return `${seconds.toFixed(2)} s`;
  if (seconds < 60) return `${seconds.toFixed(1)} s`;
  return `${Math.round(seconds / 60)} min`;
}

function setCameraTransformText(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

// --- Eve-Yaw: Drohnenkompass oder Handeingabe ---
// Der Kurs aus /octopus/flight_camera_transform/status ist nur dann als
// Kartenausrichtung brauchbar, wenn der Node ihn selbst dafuer haelt:
// live_yaw_is_compass ist true bei erdfestem Frame (NED), frischer Odometrie
// und vorhandener Zahl. Alles andere wird hier nicht nachgerechnet -- der Node
// sitzt naeher an den Daten.
function droneYawReading() {
  const st = OCTOPUS.latest.cameraTransformStatus || {};
  const deg = Number(st.live_yaw_deg);
  if (!st.live_yaw_is_compass || !Number.isFinite(deg)) return null;
  return {
    deg: ((deg % 360) + 360) % 360,
    ageSec: Number.isFinite(Number(st.live_yaw_age_sec)) ? Number(st.live_yaw_age_sec) : null,
    source: st.live_yaw_source || "drone",
  };
}

function eveYawUsesDrone() {
  return OCTOPUS.eveYawSource === "drone" && droneYawReading() !== null;
}

function setEveYawSource(source) {
  const next = source === "drone" ? "drone" : "manual";
  OCTOPUS.eveYawSource = next;
  localStorage.setItem("octopusEveYawSource", next);

  if (next === "manual") {
    // Zurueck auf den Wert, den jemand von Hand gesetzt hat -- nicht auf den
    // zuletzt vom Kompass gelieferten.
    OCTOPUS.eveYawDeg = OCTOPUS.eveYawManualDeg;
    localStorage.setItem("octopusEveYawDeg", String(OCTOPUS.eveYawDeg));
  }

  applyEveYawFromDrone(true);

  if (typeof addTimeline === "function") {
    const reading = droneYawReading();
    addTimeline(
      next === "drone"
        ? (reading
            ? `Eve yaw follows the drone compass (${reading.deg.toFixed(0)}°).`
            : "Eve yaw set to follow the drone compass, but no usable heading is arriving yet.")
        : `Eve yaw back to manual (${Math.round(OCTOPUS.eveYawDeg)}°).`,
      next === "drone" && !reading ? "warning" : "info",
    );
  }
}

// Faehrt Regler, Beschriftung und Ankerknopf auf den aktuellen Stand nach.
// force=true zeichnet die Karte auch dann neu, wenn sich der Winkel nicht
// geaendert hat -- beim Umschalten der Quelle aendert sich sonst nichts
// Sichtbares, obwohl die Bedeutung eine andere ist.
function applyEveYawFromDrone(force = false) {
  const btn = document.getElementById("eve-yaw-source-btn");
  const input = document.getElementById("eve-yaw-input");
  const label = document.getElementById("eve-yaw-label");
  const reading = droneYawReading();
  const live = eveYawUsesDrone();

  if (btn) {
    btn.setAttribute("aria-pressed", live ? "true" : "false");
    // Ohne brauchbaren Kurs ist der Anker nicht anklickbar: sonst schaltet man
    // auf eine Quelle um, die nichts liefert, und der Regler friert ein.
    btn.disabled = reading === null && OCTOPUS.eveYawSource !== "drone";
    btn.title = reading === null
      ? "No usable heading from the drone (needs fresh NED odometry). Yaw stays manual."
      : live
        ? `Yaw follows the drone compass — ${reading.deg.toFixed(1)}° from ${reading.source}. Click for manual.`
        : `Manual yaw. Click to follow the drone compass (${reading.deg.toFixed(1)}°).`;
  }

  const previous = OCTOPUS.eveYawDeg;

  if (live) {
    OCTOPUS.eveYawDeg = reading.deg;
  } else if (OCTOPUS.eveYawSource === "drone") {
    // Auf Kompass gestellt, aber es kommt nichts: der zuletzt bekannte
    // Handwert ist die ehrlichere Anzeige als ein eingefrorener Kompasswert.
    OCTOPUS.eveYawDeg = OCTOPUS.eveYawManualDeg;
  }

  const rounded = Math.round(((OCTOPUS.eveYawDeg % 360) + 360) % 360);
  if (input) {
    input.disabled = live;
    if (Math.round(parseFloat(input.value)) !== rounded) input.value = String(rounded);
  }
  if (label) {
    label.textContent = `${rounded}°`;
    label.classList.toggle("is-live", live);
  }

  const changed = Math.abs((OCTOPUS.eveYawDeg || 0) - (previous || 0)) > 1e-6;
  if (changed || force) {
    // Bewusst erst im naechsten Frame: diese Funktion laeuft auch waehrend der
    // Skriptauswertung (init), und das Neuzeichnen greift auf
    // OCTOPUS_HBVCAM_640X480 zu, das weiter unten per const deklariert ist.
    // Direkt aufgerufen stirbt es dort an der Temporal Dead Zone -- und nimmt
    // den Rest der Skriptauswertung mit, inklusive des Status-Pollings.
    requestAnimationFrame(() => {
      if (typeof cameraGridMode === "function" && cameraGridMode() !== "off") redrawCameraGrid();
      else if (typeof renderMissionMap === "function") renderMissionMap();
    });
  }
}

function initEveYawSourceToggle() {
  const btn = document.getElementById("eve-yaw-source-btn");
  if (!btn) return;
  btn.addEventListener("click", () => {
    setEveYawSource(eveYawUsesDrone() ? "manual" : "drone");
  });
  // Ohne force: beim Init gibt es noch nichts neu zu zeichnen, und der erste
  // Statusabruf faehrt den Regler eine Sekunde spaeter ohnehin nach.
  applyEveYawFromDrone(false);
}

function setCameraTransformUi(status, errorMessage = "") {
  const currentStatus = errorMessage
    ? { state: "offline", error: errorMessage }
    : (status || {});

  OCTOPUS.latest.cameraTransformStatus = currentStatus;
  OCTOPUS.latest.cameraTransformError = errorMessage || "";

  // Der Kurs steckt in derselben Nachricht, die ohnehin jede Sekunde kommt --
  // ein eigener Endpunkt dafuer waere ein zweiter Poll fuer dieselbe Quelle.
  applyEveYawFromDrone();

  const view = cameraTransformView(currentStatus, errorMessage);
  const markerRow = cameraTransformMarkerRow(view);

  const pill = document.getElementById("camera-transform-status-pill");
  if (pill) {
    pill.className = `pill ${view.uiState}`;
    pill.innerHTML = `<span class="dot"></span><span>${view.label}</span>`;
  }

  const summary = document.getElementById("camera-transform-summary");
  if (summary) {
    summary.textContent = `Camera transform status: ${view.detail}`;
  }

  setCameraTransformText("camera-transform-state", view.state);
  setCameraTransformText("camera-transform-markers", markerRow.detail);
  setCameraTransformText("camera-transform-missing", view.missing.length ? view.missing.join(", ") : "none");
  setCameraTransformText(
    "camera-transform-age",
    // Homography age is AprilTag-only; on the flight path the age that matters is
    // how long ago the status itself arrived.
    view.markerBased
      ? cameraTransformAgeText(currentStatus?.homography_age_sec)
      : cameraTransformAgeText(view.age)
  );
  setCameraTransformText("camera-transform-allowed", view.allowed ? "yes" : "no");
  setCameraTransformText(
    "camera-transform-detections",
    `${currentStatus?.last_input_detection_count ?? 0} in / ${currentStatus?.last_transformed_detection_count ?? 0} out`
  );

  if (typeof renderReadiness === "function") renderReadiness();
  if (typeof renderSystemHealth === "function") renderSystemHealth();
  if (typeof renderKpis === "function") renderKpis();
}

async function refreshCameraTransformStatus() {
  try {
    const response = await fetch("/api/camera_transform/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);

    const data = await response.json();
    setCameraTransformUi(data.camera_transform_status || {});
  } catch (error) {
    setCameraTransformUi({}, error.message);
  }
}

function initCameraTransformStatusUi() {
  refreshCameraTransformStatus();
  setInterval(refreshCameraTransformStatus, 2000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initCameraTransformStatusUi);
} else {
  initCameraTransformStatusUi();
}


// --- View-specific layout mode ---
function syncDashboardViewClass() {
  const viewSelect = document.getElementById("view-select");
  const view = viewSelect?.value || OCTOPUS?.dashboardView || localStorage.getItem("octopusDashboardView") || "overview";
  document.body.classList.toggle("octopus-view-debug", view === "debug");
}

function initDashboardViewClassSync() {
  syncDashboardViewClass();

  const viewSelect = document.getElementById("view-select");
  if (viewSelect) {
    viewSelect.addEventListener("change", () => {
      window.setTimeout(syncDashboardViewClass, 0);
    });
  }

  window.setInterval(syncDashboardViewClass, 1000);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", initDashboardViewClassSync);
} else {
  initDashboardViewClassSync();
}


// -----------------------------------------------------------------------------
// Grid mode extension: fixed camera footprint / manual / expanding / rolling
// -----------------------------------------------------------------------------

const OCTOPUS_HBVCAM_640X480 = {
  image_width: 640.0,
  image_height: 480.0,
  fx: 359.3292231592479,
  fy: 359.2290038414162,
  cx: 312.8204647201454,
  cy: 237.947360594595,
};

const OCTOPUS_GRID_MODE_LABELS = {
  fixed_camera_footprint: "Fixed camera footprint",
  manual_mission_area: "Manual mission area",
  expanding_global_map: "Expanding global map",
  rolling_local_map: "Rolling local map",
};

function octopusGridMode() {
  // Only the fixed camera-footprint grid is used now; the other modes were removed.
  return "fixed_camera_footprint";
}

function octopusCameraFootprintSettings() {
  const stored = OCTOPUS.cameraFootprint || {};
  const heightInput = $("camera-footprint-height-input");
  const resolutionInput = $("grid-resolution-input");

  const heightM = clamp(
    safeNumber(heightInput?.value, safeNumber(stored.height_m, 2.5)),
    0.20,
    50.0
  );

  const resolution = clamp(
    safeNumber(resolutionInput?.value, safeNumber(stored.resolution, 0.10)),
    0.02,
    2.0
  );

  return { height_m: heightM, resolution };
}

function octopusComputeCameraFootprintMeta() {
  const settings = octopusCameraFootprintSettings();
  const cam = OCTOPUS_HBVCAM_640X480;
  const h = settings.height_m;
  const resolution = settings.resolution;

  // Ground-plane footprint for a downward-looking pinhole camera, measured over
  // the CROPPED sensor region — cutting a frame edge away really does narrow the
  // field of view, so the footprint (and with it the grids and the map
  // projection) has to shrink with it.
  // Left/right/top/bottom stay asymmetric because cx/cy are not exactly centered.
  const region = croppedSensorRegion(cam);
  const leftM = h * (cam.cx - region.x0) / cam.fx;
  const rightM = h * (region.x1 - cam.cx) / cam.fx;
  const topM = h * (cam.cy - region.y0) / cam.fy;
  const bottomM = h * (region.y1 - cam.cy) / cam.fy;

  const widthM = leftM + rightM;
  const heightM = topM + bottomM;

  const cols = Math.max(1, Math.ceil(widthM / resolution));
  const rows = Math.max(1, Math.ceil(heightM / resolution));

  return {
    cols,
    rows,
    width_m: widthM,
    height_m: heightM,
    resolution,
    source: region.crop.active ? "cropped camera footprint" : "fixed camera footprint",
    mode: "fixed_camera_footprint",
    camera_height_m: h,
    camera_model: cam,
    crop: region.crop,
    sensor_region: region,
    footprint: {
      left_m: leftM,
      right_m: rightM,
      top_m: topM,
      bottom_m: bottomM,
    },
  };
}

function octopusRollingLocalMapMeta() {
  const resolution = octopusCameraFootprintSettings().resolution;
  const widthM = 10.0;
  const heightM = 10.0;

  return {
    cols: Math.max(1, Math.ceil(widthM / resolution)),
    rows: Math.max(1, Math.ceil(heightM / resolution)),
    width_m: widthM,
    height_m: heightM,
    resolution,
    source: "rolling local map placeholder",
    mode: "rolling_local_map",
  };
}

const OCTOPUS_ORIGINAL_GET_ACTIVE_GRID_META = typeof getActiveGridMeta === "function" ? getActiveGridMeta : null;

getActiveGridMeta = function patchedGetActiveGridMeta(mapData = {}) {
  const mode = octopusGridMode();

  if (mode === "fixed_camera_footprint") {
    return octopusComputeCameraFootprintMeta();
  }

  if (mode === "rolling_local_map") {
    return octopusRollingLocalMapMeta();
  }

  if (mode === "expanding_global_map") {
    const backendMeta = typeof mapMetaFromBackend === "function"
      ? mapMetaFromBackend(mapData || {})
      : { cols: 50, rows: 30, width_m: 5.0, height_m: 3.0, resolution: 0.10, source: "backend fallback" };

    return {
      ...backendMeta,
      mode: "expanding_global_map",
      source: "backend / expanding global map",
    };
  }

  // Manual mission area keeps the old dashboard behavior.
  if (OCTOPUS_ORIGINAL_GET_ACTIVE_GRID_META) {
    const oldMeta = OCTOPUS_ORIGINAL_GET_ACTIVE_GRID_META(mapData);
    return {
      ...oldMeta,
      mode: "manual_mission_area",
      source: oldMeta.source || "manual mission area",
    };
  }

  return octopusComputeCameraFootprintMeta();
};

function octopusUpdateCameraFootprintSummary() {
  const summary = $("camera-footprint-summary");
  const heightControl = $("camera-footprint-height-control");
  const modeSelect = $("grid-mode-select");
  const mode = octopusGridMode();

  if (modeSelect && modeSelect.value !== mode) modeSelect.value = mode;

  if (heightControl) {
    heightControl.style.display = mode === "fixed_camera_footprint" ? "" : "none";
  }

  if (!summary) return;

  let meta;
  if (mode === "fixed_camera_footprint") {
    meta = octopusComputeCameraFootprintMeta();
    const crop = meta.crop || cameraCropSettings();
    summary.textContent =
      `Camera footprint: ${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m · ` +
      `${meta.cols}×${meta.rows} cells · h=${meta.camera_height_m.toFixed(2)} m` +
      (crop.active ? ` · crop ${cameraCropShortLabel(crop)}` : "");
    summary.title =
      `HBVCAM 640x480, fx=${meta.camera_model.fx.toFixed(1)}, fy=${meta.camera_model.fy.toFixed(1)}, ` +
      `cx=${meta.camera_model.cx.toFixed(1)}, cy=${meta.camera_model.cy.toFixed(1)}` +
      (crop.active ? `\n${cameraCropTooltip(crop)}` : "");
    return;
  }

  if (mode === "rolling_local_map") {
    meta = octopusRollingLocalMapMeta();
    summary.textContent = `Rolling local map: ${meta.width_m.toFixed(1)} m × ${meta.height_m.toFixed(1)} m · ${meta.cols}×${meta.rows} cells`;
    summary.title = "Placeholder rolling map size. Later this should follow the drone pose.";
    return;
  }

  if (mode === "expanding_global_map") {
    summary.textContent = "Expanding global map: using backend map bounds";
    summary.title = "Later this should expand automatically when detections/coverage leave current bounds.";
    return;
  }

  summary.textContent = "Manual mission area: using drawn/search polygon bounds";
  summary.title = "Uses the current mission/search area dimensions.";
}

function octopusPersistCameraFootprintSettings() {
  const settings = octopusCameraFootprintSettings();
  OCTOPUS.cameraFootprint = settings;
  localStorage.setItem("octopusCameraFootprint", JSON.stringify(settings));
}

function octopusRefreshGridModeView(announce = false) {
  octopusPersistCameraFootprintSettings();
  octopusUpdateCameraFootprintSummary();

  if (OCTOPUS.latest.globalMap || OCTOPUS.osmPriors) {
    drawGridMap(OCTOPUS.latest.globalMap || {});
  }

  if (typeof renderKpis === "function") renderKpis();

  // Drone height changes the footprint, and with it the active grid on both views.
  if (cameraGridMode() !== "off" && typeof redrawCameraGrid === "function") redrawCameraGrid();

  if (announce && typeof addTimeline === "function") {
    const mode = octopusGridMode();
    const meta = getActiveGridMeta(OCTOPUS.latest.globalMap || {});
    addTimeline(
      `Grid mode: ${OCTOPUS_GRID_MODE_LABELS[mode] || mode} · ` +
      `${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m · ` +
      `${meta.cols}×${meta.rows} cells at ${meta.resolution.toFixed(2)} m/cell`,
      "info"
    );
  }
}

function octopusSetupGridModeControls() {
  const heightInput = $("camera-footprint-height-input");
  const resolutionInput = $("grid-resolution-input");

  if (heightInput) {
    heightInput.value = safeNumber(OCTOPUS.cameraFootprint?.height_m, 2.5).toFixed(2);
    heightInput.addEventListener("input", () => {
      octopusRefreshGridModeView(false);
    });
    heightInput.addEventListener("change", () => {
      octopusRefreshGridModeView(true);
    });
  }

  if (resolutionInput) {
    resolutionInput.value = safeNumber(OCTOPUS.cameraFootprint?.resolution, safeNumber(resolutionInput.value, 0.10)).toFixed(2);
    resolutionInput.addEventListener("input", () => {
      if (octopusGridMode() === "fixed_camera_footprint") {
        octopusRefreshGridModeView(false);
      }
    });
  }

  octopusRefreshGridModeView(false);
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", octopusSetupGridModeControls);
} else {
  octopusSetupGridModeControls();
}


// -----------------------------------------------------------------------------
// Camera crop controls — the sliders in the Camera & Pipeline panel. Changing a
// side changes the camera footprint, so the whole footprint-derived view is
// refreshed with it (grid map, camera overlay, mission map, KPIs).
// -----------------------------------------------------------------------------

function cameraCropShortLabel(crop = cameraCropSettings()) {
  const pct = (value) => `${Math.round(value * 100)}%`;
  return `T${pct(crop.top)} B${pct(crop.bottom)} L${pct(crop.left)} R${pct(crop.right)}`;
}

function cameraCropTooltip(crop = cameraCropSettings()) {
  const cam = OCTOPUS_HBVCAM_640X480;
  if (!crop.active) return `No camera crop — the full ${cam.image_width} × ${cam.image_height} frame is used.`;
  const region = croppedSensorRegion(cam, crop);
  return `Camera crop ${cameraCropShortLabel(crop)} — effective frame ` +
    `${Math.round(region.width_px)} × ${Math.round(region.height_px)} px of ` +
    `${cam.image_width} × ${cam.image_height}. The camera footprint, both grids, the cell ` +
    `names and the map projection all follow the cropped region; detections in the ` +
    `cut-away edges are ignored.`;
}

function renderCameraCropSummary() {
  const crop = cameraCropSettings();
  const cam = OCTOPUS_HBVCAM_640X480;
  const region = croppedSensorRegion(cam, crop);
  const meta = octopusComputeCameraFootprintMeta();

  const chip = $("camera-crop-chip");
  if (chip) {
    chip.textContent = crop.active ? `Crop ${cameraCropShortLabel(crop)}` : "Crop off";
    chip.classList.toggle("active", crop.active);
    chip.title = cameraCropTooltip(crop);
  }

  // Derselbe Wert nochmal in der zugeklappten Ueberschrift: sonst muesste man
  // den Block aufklappen, nur um zu sehen, ob ueberhaupt geschnitten wird.
  const summaryChip = $("camera-crop-chip-summary");
  if (summaryChip) {
    summaryChip.textContent = crop.active ? cameraCropShortLabel(crop) : "off";
    summaryChip.classList.toggle("active", crop.active);
  }

  const summary = $("camera-crop-summary");
  if (!summary) return;

  const lines = [
    `Effective frame <b>${Math.round(region.width_px)} × ${Math.round(region.height_px)} px</b>` +
    ` · ${Math.round(crop.kx * crop.ky * 100)}% of the sensor`,
    `Camera footprint <b>${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m</b>` +
    ` at h=${meta.camera_height_m.toFixed(2)} m`,
  ];

  if (crop.active) {
    const hidden = cameraCropHiddenCount();
    if (hidden) lines.push(`${hidden} detection${hidden === 1 ? "" : "s"} in the cut-away edges — ignored`);
  } else {
    lines.push("No crop — the full frame is used.");
  }

  summary.innerHTML = lines.map((line) => `<div>${line}</div>`).join("");
  summary.title = cameraCropTooltip(crop);
}

function cameraCropMessage() {
  const crop = cameraCropSettings();
  if (!crop.active) return "Camera crop reset — the full frame is used again.";
  const region = croppedSensorRegion(OCTOPUS_HBVCAM_640X480, crop);
  const meta = octopusComputeCameraFootprintMeta();
  return `Camera crop ${cameraCropShortLabel(crop)}: effective frame ` +
    `${Math.round(region.width_px)} × ${Math.round(region.height_px)} px, footprint ` +
    `${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m.`;
}

// Send the crop to the backend, where the ROS camera-debug bridge picks it up and
// cuts the frame edges away BEFORE the frame is sent — so Eve only ships the part
// the operator wants. Called on the committed value, not on every slider pixel.
async function pushCameraCropToBackend() {
  const crop = cameraCropSettings();
  const payload = {};
  CAMERA_CROP_SIDES.forEach((side) => { payload[side] = crop[side]; });
  try {
    const response = await fetch("/api/camera_debug/crop", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
  } catch (error) {
    console.warn("Camera crop sync failed", error);
  }
}

// The backend keeps the crop in memory only, so after a backend restart it can
// disagree with the panel. The panel is the authority — push ours again.
function resyncCameraCropIfNeeded(backendCrop) {
  if (!backendCrop) return;
  const crop = cameraCropSettings();
  const differs = CAMERA_CROP_SIDES.some(
    (side) => Math.abs(safeNumber(backendCrop[side], 0) - crop[side]) > 1e-6
  );
  if (differs) pushCameraCropToBackend();
}

function octopusRefreshCameraCropView(announce = false) {
  // Footprint summary, grid map, KPIs, and (when a grid is active) the camera
  // overlay plus the mission map.
  octopusRefreshGridModeView(false);
  renderCameraCropSummary();

  // The feed's own chips and the overlay, also with the grid switched off.
  if (typeof renderCameraFeed === "function") renderCameraFeed();
  else drawCameraFeedOverlay();

  // The footprint outline on the mission map follows the crop even with no grid.
  if (cameraGridMode() === "off" && OCTOPUS.missionMap?.map && typeof renderMissionMap === "function") {
    renderMissionMap();
  }

  if (announce && typeof addTimeline === "function") addTimeline(cameraCropMessage(), "info");
}

function syncCameraCropInputs() {
  const crop = cameraCropSettings();
  CAMERA_CROP_SIDES.forEach((side) => {
    const percent = Math.round(crop[side] * 100);
    const range = $(`camera-crop-${side}`);
    const number = $(`camera-crop-${side}-num`);
    if (range && document.activeElement !== range) range.value = String(percent);
    if (number && document.activeElement !== number) number.value = String(percent);
  });
}

function setupCameraCropControls() {
  const maxPercent = Math.round(CAMERA_CROP_MAX_SIDE * 100);

  CAMERA_CROP_SIDES.forEach((side) => {
    [$(`camera-crop-${side}`), $(`camera-crop-${side}-num`)].forEach((input) => {
      if (!input) return;
      input.max = String(maxPercent);
      // "input" tracks the drag live, "change" is the committed value that also
      // gets a timeline entry — one line per adjustment, not per pixel.
      const apply = (announce) => {
        setCameraCropSide(side, (parseFloat(input.value) || 0) / 100);
        syncCameraCropInputs();
        octopusRefreshCameraCropView(announce);
      };
      input.addEventListener("input", () => apply(false));
      input.addEventListener("change", () => {
        apply(true);
        pushCameraCropToBackend();
      });
    });
  });

  const reset = $("camera-crop-reset-btn");
  if (reset) {
    reset.addEventListener("click", () => {
      resetCameraCrop();
      syncCameraCropInputs();
      octopusRefreshCameraCropView(true);
      pushCameraCropToBackend();
    });
  }

  syncCameraCropInputs();
  renderCameraCropSummary();
  drawCameraFeedOverlay();
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", setupCameraCropControls);
} else {
  setupCameraCropControls();
}


// -----------------------------------------------------------------------------
// Fixed camera footprint marker override
// In fixed camera footprint mode:
// - Eve/drone is shown at the center of the camera footprint.
// - Home station is shown at x=center, y=0.
// -----------------------------------------------------------------------------

const OCTOPUS_ORIGINAL_DRAW_FLEET_AND_TASKS_ON_GRID =
  typeof drawFleetAndTasksOnGrid === "function" ? drawFleetAndTasksOnGrid : null;

function octopusMetricToCanvasPoint(geom, xMeters, yMeters) {
  if (typeof metricToCanvas === "function") {
    return metricToCanvas(geom, xMeters, yMeters);
  }

  return {
    x: geom.originX + (xMeters / geom.resolution) * geom.cellSize,
    y: geom.originY + geom.gridH - (yMeters / geom.resolution) * geom.cellSize,
  };
}

function octopusDrawFixedCameraFootprintMarkers(ctx, mapData, geom) {
  const display = OCTOPUS.gridDisplay || GRID_DISPLAY_DEFAULTS || {};

  const centerX = geom.width_m / 2.0;
  const centerY = geom.height_m / 2.0;

  if (display.home) {
    const home = octopusMetricToCanvasPoint(geom, centerX, 0.0);
    drawGridMarker(ctx, home.x, home.y, "H", {
      color: "#ffffff",
      fillColor: "#a78bfa",
    });
  }

  if (display.robots !== false) {
    const drone = octopusMetricToCanvasPoint(geom, centerX, centerY);
    drawGridMarker(ctx, drone.x, drone.y, "E", {
      color: "#ffffff",
      fillColor: "#38bdf8",
    });
  }
}

drawFleetAndTasksOnGrid = function patchedDrawFleetAndTasksOnGrid(ctx, mapData, geom) {
  if (typeof octopusGridMode === "function" && octopusGridMode() === "fixed_camera_footprint") {
    octopusDrawFixedCameraFootprintMarkers(ctx, mapData, geom);
    return;
  }

  if (OCTOPUS_ORIGINAL_DRAW_FLEET_AND_TASKS_ON_GRID) {
    OCTOPUS_ORIGINAL_DRAW_FLEET_AND_TASKS_ON_GRID(ctx, mapData, geom);
  }
};



function setupGridSourceControls() {
  const select = $("grid-source-select");
  if (!select) return;

  select.value = getSelectedGridSource();
  select.addEventListener("change", () => {
    OCTOPUS.gridSource = select.value;
    localStorage.setItem("octopusGridSource", OCTOPUS.gridSource);
    if (typeof drawGridMap === "function") {
      drawGridMap(OCTOPUS.latest.globalMap || {});
    }
    if (typeof renderSystemHealth === "function") renderSystemHealth();
  });
}

setTimeout(setupGridSourceControls, 0);



const OCTOPUS_MAPPING_MODES = {
  local_camera_debug: {
    label: "Local Camera Debug",
    gridSource: "local_camera",
    description: `<strong>Local Camera Debug:</strong> shows what Eve currently sees in the camera footprint. This is dashboard/debug only and is not sent to ground robots.`,
    attitude: "Not required",
    position: "Ignored",
    note: "Best for checking detector output and u/v → grid cell matching.",
    command: "OCTOPUS_MAPPING_MODE=flight_global_mission ./Octopus/scripts/start_octopus_debug_stack.sh",
  },
  indoor_static_mission: {
    label: "Indoor Static Mission Map",
    gridSource: "global",
    description: `<strong>Indoor Static Mission Map:</strong> for ceiling/static indoor tests without GPS. Uses manual height and PX4 attitude, but should use a fixed/frozen map origin instead of drifting PX4 x/y.`,
    attitude: "Used",
    position: "Ignored / frozen",
    note: "Best for a hanging drone looking at a fixed ground area.",
    command: "OCTOPUS_MAPPING_MODE=indoor_static_mission ./Octopus/scripts/start_octopus_debug_stack.sh",
  },
  flight_global_mission: {
    label: "Flight Global Mission Map",
    gridSource: "global",
    description: `<strong>Flight Global Mission Map:</strong> real mission mode. Uses PX4 pose, attitude and height/ground-plane projection to create persistent world/map coordinates for robots.`,
    attitude: "Used",
    position: "Used",
    note: "Best for outdoor/real flight and robot task assignment.",
    command: "OCTOPUS_MAPPING_MODE=flight_global_mission ./Octopus/scripts/start_octopus_debug_stack.sh",
  },
};

function selectedMappingMode() {
  const value = OCTOPUS.mappingMode || localStorage.getItem("octopusMappingMode") || "local_camera_debug";
  return OCTOPUS_MAPPING_MODES[value] ? value : "local_camera_debug";
}

function setGridSourceFromMappingMode(modeKey) {
  const mode = OCTOPUS_MAPPING_MODES[modeKey] || OCTOPUS_MAPPING_MODES.local_camera_debug;
  OCTOPUS.gridSource = mode.gridSource;
  localStorage.setItem("octopusGridSource", OCTOPUS.gridSource);

  const gridSourceSelect = $("grid-source-select");
  if (gridSourceSelect) gridSourceSelect.value = OCTOPUS.gridSource;
}

function updateMappingSettingsPanel() {
  const modeKey = selectedMappingMode();
  const mode = OCTOPUS_MAPPING_MODES[modeKey] || OCTOPUS_MAPPING_MODES.local_camera_debug;

  const modeSelect = $("mapping-mode-select");
  if (modeSelect) modeSelect.value = modeKey;

  const description = $("mapping-mode-description");
  if (description) {
    const commandHint = mode.command
      ? `<br><span class="muted">Startup command: <code>${mode.command}</code></span>`
      : "";
    description.innerHTML = `${mode.description}<br><span class="muted">${mode.note}</span>${commandHint}`;
  }

  const gridSourceLabel = $("mapping-grid-source-value");
  if (gridSourceLabel) {
    gridSourceLabel.textContent = getSelectedGridSource() === "local_camera"
      ? "Local Camera Grid"
      : "Global Mission Grid";
  }

  const resolution = safeNumber(
    OCTOPUS.latest.localCameraGrid?.patch?.resolution_m ??
    OCTOPUS.cameraFootprint?.resolution ??
    $("grid-resolution-input")?.value,
    0.10
  );

  const resolutionValue = $("mapping-resolution-value");
  if (resolutionValue) resolutionValue.textContent = `${resolution.toFixed(2)} m / cell`;

  const footprint = OCTOPUS.latest.localCameraGrid?.patch || {};
  const footprintWidth = safeNumber(footprint.footprint_width_m, 4.46);
  const footprintHeight = safeNumber(footprint.footprint_height_m, 3.34);

  const footprintValue = $("mapping-footprint-value");
  if (footprintValue) footprintValue.textContent = `${footprintWidth.toFixed(2)} m × ${footprintHeight.toFixed(2)} m`;

  const height = safeNumber(
    OCTOPUS.cameraFootprint?.height_m ?? $("camera-footprint-height-input")?.value,
    2.50
  );

  const heightValue = $("mapping-height-value");
  if (heightValue) heightValue.textContent = `${height.toFixed(2)} m`;

  const attitudeValue = $("mapping-attitude-value");
  if (attitudeValue) attitudeValue.textContent = mode.attitude;

  const positionValue = $("mapping-position-value");
  if (positionValue) positionValue.textContent = mode.position;

  const robotWarning = document.querySelector(".mapping-warning-chip");
  if (robotWarning) robotWarning.textContent = "Robot output source: Global Mission Map only";
}

function setupMappingSettingsControls() {
  const modeSelect = $("mapping-mode-select");
  if (modeSelect && !modeSelect.dataset.bound) {
    modeSelect.dataset.bound = "true";
    modeSelect.value = selectedMappingMode();

    modeSelect.addEventListener("change", () => {
      OCTOPUS.mappingMode = modeSelect.value;
      localStorage.setItem("octopusMappingMode", OCTOPUS.mappingMode);

      setGridSourceFromMappingMode(OCTOPUS.mappingMode);
      updateMappingSettingsPanel();

      if (typeof drawGridMap === "function") drawGridMap(OCTOPUS.latest.globalMap || {});
      if (typeof renderSystemHealth === "function") renderSystemHealth();
    });
  }

  const gridSourceSelect = $("grid-source-select");
  if (gridSourceSelect && !gridSourceSelect.dataset.mappingSyncBound) {
    gridSourceSelect.dataset.mappingSyncBound = "true";
    gridSourceSelect.addEventListener("change", () => {
      OCTOPUS.gridSource = gridSourceSelect.value;
      localStorage.setItem("octopusGridSource", OCTOPUS.gridSource);
      updateMappingSettingsPanel();
    });
  }

  updateMappingSettingsPanel();
}

setTimeout(setupMappingSettingsControls, 0);
setInterval(updateMappingSettingsPanel, 2000);

