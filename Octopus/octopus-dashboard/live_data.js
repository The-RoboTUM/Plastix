const GRID_DISPLAY_DEFAULTS = {
  priors: true,
  detections: true,
  ground: true,
  water: true,
  obstacles: true,
  trash: true,
  fleet: true,
  home: true,
  coverage: true,
  confidence: true,
  unknown: true,
};

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
    hasFit: false,
    polygonMode: false,
    polygonPoints: [],
    polygonPreviewLayer: null,
  },
  missionArea: JSON.parse(localStorage.getItem("octopusMissionArea") || "null"),
  gridMode: localStorage.getItem("octopusGridMode") || "fixed_camera_footprint",
  gridSource: localStorage.getItem("octopusGridSource") || "global",
  mappingMode: localStorage.getItem("octopusMappingMode") || "local_camera_debug",
  cameraFootprint: JSON.parse(localStorage.getItem("octopusCameraFootprint") || '{"height_m":2.5,"resolution":0.10}'),
  osmPriors: JSON.parse(localStorage.getItem("octopusOsmPriors") || "null"),
  gridDisplay: { ...GRID_DISPLAY_DEFAULTS, ...(JSON.parse(localStorage.getItem("octopusGridDisplay") || "{}")) },
  gridView: { scale: 1, offsetX: 0, offsetY: 0, isPanning: false, lastX: 0, lastY: 0, moved: false },
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
  overview: {
    label: "Mission Overview",
    action: "Operator overview",
    preset: "overview",
  },
  planning: {
    label: "Mission Planning",
    action: "Define area, priors, grid and home station",
    preset: "osm_priors",
  },
  mapping: {
    label: "Mapping / Detections",
    action: "Inspect coverage, trash, obstacles and priors",
    preset: "overview",
  },
  fleet: {
    label: "Robots / Fleet",
    action: "Monitor Eve, Robby, GripperX and SharX",
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
    fallback: { x: 2.8, y: 2.4, state: "scanning", battery: 87 },
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
  },
};

function $(id) {
  return document.getElementById(id);
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function safeNumber(value, fallback = 0) {
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

function getMissionOrigin() {
  return OCTOPUS.missionArea?.origin || DEMO_MAP_ORIGIN;
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
    origin: area.origin || DEMO_MAP_ORIGIN,
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

function fallbackLocationForProfile(profile) {
  const p = profile.fallback || { x: 0, y: 0 };
  const [lat, lon] = localToLatLng(p.x, p.y);
  return {
    id: profile.name,
    lat,
    lon,
    ts: new Date().toISOString(),
    state: p.state,
    is_demo: true,
  };
}

function homeStation() {
  const [lat, lon] = localToLatLng(0.25, 0.25);
  return { id: "Home station", lat, lon, x: 0.25, y: 0.25, type: "home" };
}

function robotStatusFromState(state, age) {
  const value = String(state || "").toLowerCase();
  if (value.includes("error") || value.includes("fail")) return "error";
  if (value.includes("offline")) return "offline";
  if (age !== null && age > 60 && !value.includes("idle")) return "stale";
  if (value.includes("return") || value.includes("assigned") || value.includes("driving") || value.includes("navigating")) return "warning";
  if (value.includes("scan") || value.includes("collect") || value.includes("online") || value.includes("idle")) return "fresh";
  return "unknown";
}

function getFleetSnapshot() {
  return Object.values(ROBOT_FLEET_PROFILES).map((profile) => {
    const battery = matchBatteryForProfile(profile) || { percent: profile.fallback?.battery, state: profile.fallback?.state, ts: null, is_demo: true };
    const location = matchLocationForProfile(profile) || fallbackLocationForProfile(profile);
    const age = ageSeconds(location.ts || battery.ts);
    const state = battery.state || location.state || profile.fallback?.state || "unknown";
    const demo = Boolean(location.is_demo || battery.is_demo);
    const status = demo ? "unknown" : robotStatusFromState(state, age);
    const local = latLngToLocal(safeNumber(location.lat, NaN), safeNumber(location.lon, NaN));
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
      online: !demo && status !== "offline" && status !== "error" && (age === null || age < 120),
      currentTask,
      demo,
    };
  });
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

  const baseLayer = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 21,
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
    div.innerHTML = `
      <div class="legend-title">Octopus map</div>
      <div><span class="legend-swatch ground"></span> scanned ground</div>
      <div><span class="legend-swatch water"></span> water / shoreline</div>
      <div><span class="legend-swatch trash"></span> trash or task target</div>
      <div><span class="legend-swatch obstacle"></span> real obstacle</div>
      <div><span class="legend-swatch building"></span> building / obstacle prior</div>
      <div><span class="legend-swatch coverage"></span> soft overlay = OSM prior</div>
      <div><span class="legend-dot drone"></span> drone</div>
      <div><span class="legend-dot robot"></span> robot</div>
      <div><span class="legend-dot home"></span> home/base</div>
      <div><span class="legend-dot task"></span> task</div>
      <div class="legend-note" id="mission-area-legend-note">Search area: default 5 m × 3 m</div>
    `;
    L.DomEvent.disableClickPropagation(div);
    return div;
  };
  legend.addTo(map);
  OCTOPUS.missionMap.legendControl = legend;

  map.on("click", (event) => handleMissionMapClick(event));
  map.on("mousemove", (event) => handleMissionMapMouseMove(event));
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
  if (!OCTOPUS.missionMap.polygonMode) return;
  OCTOPUS.missionMap.polygonPoints.push(event.latlng);
  const count = OCTOPUS.missionMap.polygonPoints.length;
  addTimeline(`Search polygon point ${count} set.`, "info");
  renderPolygonPreview();
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

function renderMissionAreaOverlay() {
  const state = OCTOPUS.missionMap;
  if (!state.map || !state.areaLayer) return;
  state.areaLayer.clearLayers();
  const meta = getActiveGridMeta(OCTOPUS.latest.globalMap);
  const sw = localToLatLng(0, 0);
  const ne = localToLatLng(meta.width_m, meta.height_m);
  L.rectangle([sw, ne], {
    color: "#ffffff",
    weight: 2,
    opacity: 0.92,
    fillOpacity: 0.02,
    dashArray: "5 5",
    interactive: false,
  }).addTo(state.areaLayer);

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

  const note = document.getElementById("mission-area-legend-note");
  if (note) {
    note.textContent = `Search area: ${meta.width_m.toFixed(1)} m × ${meta.height_m.toFixed(1)} m · ${meta.resolution.toFixed(2)} m/cell · ${meta.cols}×${meta.rows}`;
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

  const mode = $("mission-map-mode")?.value || "standard";
  const layerName = $("grid-layer-select")?.value || "overview";
  const mapData = getMergedGridData(OCTOPUS.latest.globalMap || {});

  state.gridLayer.clearLayers();
  renderMissionAreaOverlay();

  if (mode === "standard" || !mapData) return;

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
    rect.addTo(state.gridLayer);
  });
}

function renderMissionMap() {
  initMissionMap();

  const state = OCTOPUS.missionMap;
  if (!state.map) return;

  const mode = $("mission-map-mode")?.value || "standard";
  if (mode === "grid_only") {
    if (state.map.hasLayer(state.baseLayer)) state.map.removeLayer(state.baseLayer);
    state.map.getContainer().classList.add("grid-only-map");
  } else {
    if (!state.map.hasLayer(state.baseLayer)) state.baseLayer.addTo(state.map);
    state.map.getContainer().classList.remove("grid-only-map");
  }

  state.markerLayer.clearLayers();
  renderMissionGridOverlay();

  const bounds = [];
  const tasks = OCTOPUS.latest.tasks || [];

  // Fleet markers use configured robot roles first, then live backend data when available.
  getFleetSnapshot().forEach((robot) => {
    const lat = safeNumber(robot.location.lat, NaN);
    const lon = safeNumber(robot.location.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const icon = missionRobotIcon(robot);
    const style = deviceStyle(robot.type === "water" ? "boat" : robot.type, robot.state);
    const marker = icon
      ? L.marker([lat, lon], { icon })
      : L.circleMarker([lat, lon], { radius: 8, color: style.color, weight: 2, fillColor: style.fillColor, fillOpacity: 0.92 });

    marker.bindTooltip(`${robot.icon} ${robot.name} · ${robot.role}<br>${robot.state || "unknown"} · ${robot.capability}`, {
      permanent: false,
      direction: "top",
    });

    marker.on("click", () => {
      OCTOPUS.selected = { type: "fleet", id: robot.name, device_type: robot.type, robot };
      renderInspector();
    });

    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

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
  homeMarker.addTo(state.markerLayer);
  bounds.push([home.lat, home.lon]);

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
    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

  const meta = getActiveGridMeta(OCTOPUS.latest.globalMap);
  bounds.push(localToLatLng(0, 0));
  bounds.push(localToLatLng(meta.width_m, meta.height_m));

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

  const transform = OCTOPUS.latest.cameraTransformStatus || {};
  const transformStateRaw = String(transform.state || "unknown").toLowerCase();
  const transformAllowed = Boolean(transform.is_transform_allowed);
  const transformDetected = Array.isArray(transform.detected_marker_ids) ? transform.detected_marker_ids : [];
  const transformMissing = Array.isArray(transform.missing_marker_ids) ? transform.missing_marker_ids : [];
  const transformRequired = Array.isArray(transform.required_marker_ids) ? transform.required_marker_ids : [61, 65, 57, 11];
  const transformUiState = cameraTransformUiState(transformStateRaw, transformAllowed);
  const transformDetail = transform.error
    ? transform.error
    : `${transformStateRaw} · ${transformDetected.length}/${transformRequired.length} markers · ${transformAllowed ? "allowed" : "blocked"}`;

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
      state: transformUiState,
      detail: transformDetail,
    },
    {
      name: "AprilTags",
      state: transformDetected.length === transformRequired.length && transformRequired.length > 0 ? "fresh" : "warning",
      detail: transformMissing.length ? `missing ${transformMissing.join(", ")}` : "all required markers visible",
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
  const tasks = OCTOPUS.latest.tasks || [];
  const battery = OCTOPUS.latest.battery || [];
  const patch = OCTOPUS.latest.patch;
  const cells = mapData.cells || {};
  const coverageStats = computeCoverageStats(mapData);
  const healthItems = computeHealthItems();
  const health = healthSummary(healthItems);

  $("kpi-coverage").textContent = formatPercent(coverageStats.coverageRatio);
  $("kpi-coverage-sub").textContent = `${coverageStats.coveredArea.toFixed(1)} / ${coverageStats.totalArea.toFixed(1)} m² scanned`;

  $("kpi-detections").textContent = tasks.length || safeNumber(stats.open_tasks, 0);
  $("kpi-detections-sub").textContent = `${safeNumber(stats.open_tasks, 0)} open tasks`;

  $("kpi-confirmed").textContent = safeNumber(stats.trash_collected, 0);

  const fleetSnapshot = getFleetSnapshot();
  const onlineFleet = fleetSnapshot.filter((r) => r.online).length;
  $("kpi-fleet").textContent = `${onlineFleet}/${fleetSnapshot.length}`;
  $("kpi-fleet-sub").textContent = "Eve · Robby · GripperX · SharX";

  const patchCells = patch?.updated_cells?.length ?? 0;
  $("kpi-map-patch").textContent = patchCells || "--";
  $("kpi-map-patch-sub").textContent = patch ? `${Object.keys(cells).length} accumulated cells` : "No patch yet";

  $("kpi-health").textContent = health.label;
  $("kpi-health").className = `value ${health.state === "error" ? "error-text" : health.state === "warning" ? "warning-text" : ""}`;
  $("kpi-health-sub").textContent = `${healthItems.filter((i) => i.state === "fresh" || i.state === "ok").length}/${healthItems.length} fresh/configured`;
}

function renderFleet() {
  const el = $("fleet-content");
  if (!el) return;

  const detailed = OCTOPUS.dashboardView === "fleet";
  const robots = getFleetSnapshot();
  el.innerHTML = `<div class="compact-list">${robots.map((robot) => {
    const percent = clamp(safeNumber(robot.battery.percent, 0), 0, 100);
    const fresh = freshnessFromAge(robot.age, 5, 45);
    const status = robot.status === "unknown" ? fresh.state : robot.status;
    const currentTask = robot.currentTask ? `Task #${escapeHtml(robot.currentTask.id)}` : "none";
    const tags = robot.tags.map((tag) => {
      const cls = tag.includes("water") || tag.includes("boat") || tag.includes("floating") ? "water" : tag.includes("land") ? "land" : tag.includes("scan") || tag.includes("detect") || tag.includes("camera") ? "scan" : "";
      return `<span class="capability-tag ${cls}">${escapeHtml(tag)}</span>`;
    }).join("");
    return `
      <button class="item-card robot-card ${robot.key === "eve" ? "is-primary" : ""}" data-device-id="${escapeHtml(robot.name)}" type="button" style="text-align:left; width:100%;" aria-label="Select ${escapeHtml(robot.name)}">
        <div class="item-top">
          <div class="robot-topline">
            <span class="robot-icon" aria-hidden="true">${robot.icon}</span>
            <div>
              <div class="robot-name">${escapeHtml(robot.name)}</div>
              <div class="robot-role">${escapeHtml(robot.role)}</div>
            </div>
          </div>
          ${statusPill(escapeHtml(robot.state || "unknown"), status)}
        </div>
        <div class="item-meta">
          ${escapeHtml(robot.capability)}<br />
          Battery: <span class="accent">${percent.toFixed(0)}%</span> · Last update: ${robot.demo ? "demo/fallback" : escapeHtml(fresh.label)}
          ${detailed ? `<br />Position: ${safeNumber(robot.location.lat, 0).toFixed(6)}, ${safeNumber(robot.location.lon, 0).toFixed(6)}<br />Current task: ${currentTask}<br />Assignment rule: ${escapeHtml(robot.taskRule)}` : ""}
        </div>
        <div class="progress"><span style="width:${percent}%"></span></div>
        ${detailed ? `<div class="capability-tags">${tags}</div>` : ""}
      </button>
    `;
  }).join("")}</div>`;

  el.querySelectorAll("[data-device-id]").forEach((card) => {
    card.addEventListener("click", () => {
      const robot = robots.find((r) => r.name === card.dataset.deviceId);
      if (!robot) return;
      OCTOPUS.selected = { type: "fleet", id: robot.name, device_type: robot.type, robot };
      renderInspector();
    });
  });
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

  const transform = OCTOPUS.latest.cameraTransformStatus || {};
  const transformStateRaw = String(transform.state || "unknown").toLowerCase();
  const transformAllowed = Boolean(transform.is_transform_allowed);
  const transformDetected = Array.isArray(transform.detected_marker_ids) ? transform.detected_marker_ids : [];
  const transformMissing = Array.isArray(transform.missing_marker_ids) ? transform.missing_marker_ids : [];
  const transformRequired = Array.isArray(transform.required_marker_ids) ? transform.required_marker_ids : [61, 65, 57, 11];
  const transformState = cameraTransformUiState(transformStateRaw, transformAllowed);
  const markerState = transformDetected.length === transformRequired.length && transformRequired.length > 0 ? "fresh" : "warning";

  const rows = [
    ["Mission polygon defined", "unknown", "planning tool later"],
    ["Home position set", "unknown", "planning tool later"],
    ["Backend API", OCTOPUS.backendOk ? "fresh" : "offline", OCTOPUS.backendOk ? "OK" : "offline"],
    ["ROS map patch bridge", patchFresh.state, patchFresh.label],
    ["Camera transform", transformState, `${transformStateRaw} · ${transformAllowed ? "transform allowed" : "transform blocked"}`],
    ["AprilTags visible", markerState, transformMissing.length ? `${transformDetected.length}/${transformRequired.length} visible · missing ${transformMissing.join(", ")}` : `${transformDetected.length}/${transformRequired.length} visible`],
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
            <tr><td>battery</td><td>${percent.toFixed(0)}%</td></tr>
            <tr><td>lat/lon</td><td>${safeNumber(robot.location?.lat, 0).toFixed(6)}, ${safeNumber(robot.location?.lon, 0).toFixed(6)}</td></tr>
            <tr><td>local x/y</td><td>${safeNumber(robot.local?.x, 0).toFixed(2)}, ${safeNumber(robot.local?.y, 0).toFixed(2)} m</td></tr>
            <tr><td>current task</td><td>${robot.currentTask ? `Task #${escapeHtml(robot.currentTask.id)}` : "none"}</td></tr>
            <tr><td>last update</td><td>${robot.demo ? "demo/fallback position" : escapeHtml(fresh.label)}</td></tr>
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

async function refreshCameraDebug() {
  try {
    const data = await apiGet("/api/camera_debug/latest");
    OCTOPUS.latest.cameraDebug = data.status === "ok" ? data : null;
  } catch (error) {
    OCTOPUS.latest.cameraDebug = null;
    console.warn("Camera debug refresh failed", error);
  }
  renderCameraDebug();
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
    });
  }

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
refreshCameraDebug();
setInterval(refreshCameraDebug, 1000);


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
    setEveCameraUi(data.status, data.ssh?.stdout || "");
    return data;
  } catch (error) {
    setEveCameraUi("offline", error.message);
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
function cameraTransformUiState(state, transformAllowed) {
  const value = String(state || "unknown").toLowerCase();

  if (value === "ok" && transformAllowed) return "fresh";
  if (value === "stale_warning") return "warning";
  if (value === "stale_drop") return "error";
  if (value === "not_ready") return "warning";
  if (value === "unknown") return "unknown";

  return transformAllowed ? "fresh" : "warning";
}

function cameraTransformLabel(state) {
  const value = String(state || "unknown").toLowerCase();

  if (value === "ok") return "Transform OK";
  if (value === "not_ready") return "Transform not ready";
  if (value === "stale_warning") return "Transform stale";
  if (value === "stale_drop") return "Transform blocked";
  return "Transform unknown";
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

function setCameraTransformUi(status, errorMessage = "") {
  const currentStatus = errorMessage
    ? {
        state: "offline",
        has_homography: false,
        is_transform_allowed: false,
        detected_marker_ids: [],
        missing_marker_ids: [],
        required_marker_ids: [61, 65, 57, 11],
        error: errorMessage,
      }
    : (status || {});

  OCTOPUS.latest.cameraTransformStatus = currentStatus;

  const state = currentStatus?.state || "unknown";
  const transformAllowed = Boolean(currentStatus?.is_transform_allowed);
  const detected = Array.isArray(currentStatus?.detected_marker_ids) ? currentStatus.detected_marker_ids : [];
  const missing = Array.isArray(currentStatus?.missing_marker_ids) ? currentStatus.missing_marker_ids : [];
  const required = Array.isArray(currentStatus?.required_marker_ids) ? currentStatus.required_marker_ids : [61, 65, 57, 11];

  const uiState = errorMessage ? "offline" : cameraTransformUiState(state, transformAllowed);
  const label = errorMessage ? "Transform offline" : cameraTransformLabel(state);

  const pill = document.getElementById("camera-transform-status-pill");
  if (pill) {
    pill.className = `pill ${uiState}`;
    pill.innerHTML = `<span class="dot"></span><span>${label}</span>`;
  }

  const summary = document.getElementById("camera-transform-summary");
  if (summary) {
    const markerText = `${detected.length}/${required.length} markers`;
    const missingText = missing.length ? `missing ${missing.join(", ")}` : "all required markers visible";
    const allowedText = transformAllowed ? "transform allowed" : "transform blocked";
    summary.textContent = errorMessage
      ? `Camera transform status: ${errorMessage}`
      : `Camera transform status: ${state} · ${markerText} · ${missingText} · ${allowedText}`;
  }

  setCameraTransformText("camera-transform-state", state);
  setCameraTransformText("camera-transform-markers", `${detected.length}/${required.length} visible`);
  setCameraTransformText("camera-transform-missing", missing.length ? missing.join(", ") : "none");
  setCameraTransformText("camera-transform-age", cameraTransformAgeText(status?.homography_age_sec));
  setCameraTransformText("camera-transform-allowed", transformAllowed ? "yes" : "no");
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
  return OCTOPUS.gridMode || localStorage.getItem("octopusGridMode") || "fixed_camera_footprint";
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

  // Ground-plane footprint for a downward-looking pinhole camera.
  // Left/right/top/bottom are asymmetric because cx/cy are not exactly centered.
  const leftM = h * cam.cx / cam.fx;
  const rightM = h * (cam.image_width - cam.cx) / cam.fx;
  const topM = h * cam.cy / cam.fy;
  const bottomM = h * (cam.image_height - cam.cy) / cam.fy;

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
    source: "fixed camera footprint",
    mode: "fixed_camera_footprint",
    camera_height_m: h,
    camera_model: cam,
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
    summary.textContent =
      `Camera footprint: ${meta.width_m.toFixed(2)} m × ${meta.height_m.toFixed(2)} m · ` +
      `${meta.cols}×${meta.rows} cells · h=${meta.camera_height_m.toFixed(2)} m`;
    summary.title =
      `HBVCAM 640x480, fx=${meta.camera_model.fx.toFixed(1)}, fy=${meta.camera_model.fy.toFixed(1)}, ` +
      `cx=${meta.camera_model.cx.toFixed(1)}, cy=${meta.camera_model.cy.toFixed(1)}`;
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
  const modeSelect = $("grid-mode-select");
  const heightInput = $("camera-footprint-height-input");
  const resolutionInput = $("grid-resolution-input");

  if (modeSelect) {
    modeSelect.value = octopusGridMode();
    modeSelect.addEventListener("change", () => {
      OCTOPUS.gridMode = modeSelect.value;
      localStorage.setItem("octopusGridMode", OCTOPUS.gridMode);
      OCTOPUS.gridView = { ...OCTOPUS.gridView, scale: 1, offsetX: 0, offsetY: 0 };
      octopusRefreshGridModeView(true);
    });
  }

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

  if (display.home !== false) {
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
  },
  indoor_static_mission: {
    label: "Indoor Static Mission Map",
    gridSource: "global",
    description: `<strong>Indoor Static Mission Map:</strong> for ceiling/static indoor tests without GPS. Uses manual height and PX4 attitude, but should use a fixed/frozen map origin instead of drifting PX4 x/y.`,
    attitude: "Used",
    position: "Ignored / frozen",
    note: "Best for a hanging drone looking at a fixed ground area.",
  },
  flight_global_mission: {
    label: "Flight Global Mission Map",
    gridSource: "global",
    description: `<strong>Flight Global Mission Map:</strong> real mission mode. Uses PX4 pose, attitude and height/ground-plane projection to create persistent world/map coordinates for robots.`,
    attitude: "Used",
    position: "Used",
    note: "Best for outdoor/real flight and robot task assignment.",
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
  if (description) description.innerHTML = `${mode.description}<br><span class="muted">${mode.note}</span>`;

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

