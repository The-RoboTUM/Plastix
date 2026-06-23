const OCTOPUS = {
  missionPhase: localStorage.getItem("octopusMissionPhase") || "idle",
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
  osmPriors: JSON.parse(localStorage.getItem("octopusOsmPriors") || "null"),
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
  const explicit = String(item.type || item.device_type || "").toLowerCase();
  if (explicit) return explicit;
  const id = String(item.id || item.device_id || "").toLowerCase();
  if (id.includes("drone") || id.includes("eve")) return "drone";
  if (id.includes("laptop")) return "base";
  return "robot";
}

function batteryForDevice(deviceId) {
  return (OCTOPUS.latest.battery || []).find((b) => String(b.id || b.device_id) === String(deviceId));
}

function deviceStyle(type, state = "") {
  const stateLower = String(state).toLowerCase();
  if (stateLower.includes("error") || stateLower.includes("offline")) return { color: "#ef4444", fillColor: "#ef4444" };
  if (type === "drone") return { color: "#ffffff", fillColor: "#0065bd" };
  if (type === "base") return { color: "#cbd5e1", fillColor: "#64748b" };
  if (stateLower.includes("collect")) return { color: "#ffffff", fillColor: "#e37222" };
  return { color: "#ffffff", fillColor: "#22c55e" };
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
      <div><span class="legend-swatch ground"></span> bright green = detected ground</div>
      <div><span class="legend-swatch water"></span> bright blue = detected/semantic water</div>
      <div><span class="legend-swatch trash"></span> bright orange = detected trash</div>
      <div><span class="legend-swatch obstacle"></span> bright red = detected obstacle</div>
      <div><span class="legend-swatch coverage"></span> faint colors = OSM priors</div>
      <div><span class="legend-swatch building"></span> faint red/purple = building prior</div>
      <div><span class="legend-dot drone"></span> drone</div>
      <div><span class="legend-dot robot"></span> robot/base</div>
      <div><span class="legend-dot task"></span> task / trash target</div>
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

function overviewBaseColor(cell) {
  const cls = overviewClass(cell);
  const semConf = clamp(safeNumber(cell?.semantic_confidence, 0.25), 0.10, 0.85);
  const coverage = clamp(safeNumber(cell?.coverage, 0), 0, 1);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);

  // Bright colors = observed/scanned grid evidence.
  if (cls === "detected_trash") return `rgba(227, 114, 34, ${0.55 + 0.38 * trash})`;
  if (cls === "detected_obstacle") return `rgba(239, 68, 68, ${0.55 + 0.38 * realObstacle})`;
  if (cls === "detected_water") return `rgba(59, 130, 246, ${0.58 + 0.28 * Math.max(coverage, semConf)})`;
  if (cls === "detected_ground") return `rgba(34, 197, 94, ${0.38 + 0.35 * Math.max(coverage, 0.35)})`;

  // Faint colors = priors from OSM/default map assumptions.
  if (cls === "prior_water") return `rgba(59, 130, 246, ${0.20 + semConf * 0.26})`;
  if (cls === "prior_building") return `rgba(239, 68, 68, ${0.16 + semConf * 0.28})`;
  if (cls === "prior_ground") return `rgba(34, 197, 94, ${0.13 + semConf * 0.20})`;
  return "rgba(100, 116, 139, 0.18)";
}

function overviewLeafletColor(cell) {
  const cls = overviewClass(cell);
  const trash = clamp(safeNumber(cell?.trash_probability, 0), 0, 1);
  const realObstacle = clamp(realObstacleProbability(cell), 0, 1);

  if (cls === "detected_trash") return { color: "#e37222", fillColor: "#e37222", fillOpacity: 0.70 + 0.20 * trash };
  if (cls === "detected_obstacle") return { color: "#ef4444", fillColor: "#ef4444", fillOpacity: 0.62 + 0.24 * realObstacle };
  if (cls === "detected_water") return { color: "#3b82f6", fillColor: "#3b82f6", fillOpacity: 0.58 };
  if (cls === "detected_ground") return { color: "#22c55e", fillColor: "#22c55e", fillOpacity: 0.48 };

  if (cls === "prior_water") return { color: "#3b82f6", fillColor: "#3b82f6", fillOpacity: 0.24 };
  if (cls === "prior_building") return { color: "#ef4444", fillColor: "#ef4444", fillOpacity: 0.24 };
  if (cls === "prior_ground") return { color: "#22c55e", fillColor: "#22c55e", fillOpacity: 0.16 };
  return { color: "#64748b", fillColor: "#64748b", fillOpacity: 0.10 };
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
      ? `rgba(248, 113, 113, ${0.50 + 0.45 * strength})`
      : `rgba(248, 113, 113, ${0.16 + 0.22 * strength})`;
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
    ctx.fillStyle = `rgba(255, 148, 65, ${0.55 + 0.42 * trash})`;
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
  const locations = OCTOPUS.latest.locations || [];
  const tasks = OCTOPUS.latest.tasks || [];

  locations.forEach((loc) => {
    const lat = safeNumber(loc.lat, NaN);
    const lon = safeNumber(loc.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

    const id = loc.id || loc.origin_id || loc.device_id || "unknown";
    const b = batteryForDevice(id) || {};
    const type = fleetType({ ...loc, ...b, id });
    const style = deviceStyle(type, b.state || loc.state);
    const radius = type === "drone" ? 9 : type === "base" ? 7 : 7;

    const marker = L.circleMarker([lat, lon], {
      radius,
      color: style.color,
      weight: 2,
      fillColor: style.fillColor,
      fillOpacity: 0.92,
    }).bindTooltip(`${id} (${type})`, { permanent: false, direction: "top" });

    marker.on("click", () => {
      const local = latLngToLocal(lat, lon);
      OCTOPUS.selected = { type: "fleet", id, device_type: type, location: loc, battery: b, local };
      renderInspector();
    });

    marker.addTo(state.markerLayer);
    bounds.push([lat, lon]);
  });

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
  if (layer === "trash_probability") return safeNumber(cell.trash_probability ?? cell.semantic_trash_probability, 0);
  if (layer === "obstacle_probability") return Math.max(realObstacleProbability(cell), priorObstacleProbability(cell));
  if (layer === "confidence") return safeNumber(cell.confidence || cell.semantic_confidence, 0);
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
    return `rgba(100, 160, 200, ${alpha})`;
  }

  if (layer === "trash_probability") {
    return `rgba(227, 114, 34, ${alpha})`;
  }

  if (layer === "obstacle_probability") {
    return `rgba(239, 68, 68, ${alpha})`;
  }

  if (layer === "confidence") {
    return `rgba(34, 197, 94, ${alpha})`;
  }

  return `rgba(100, 160, 200, ${alpha})`;
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
  (OCTOPUS.latest.tasks || []).forEach((task) => {
    const lat = safeNumber(task.lat, NaN);
    const lon = safeNumber(task.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const local = latLngToLocal(lat, lon);
    if (local.x < 0 || local.x > geom.width_m || local.y < 0 || local.y > geom.height_m) return;
    const p = localMetersToCanvas(geom, local.x, local.y);
    if (p.x < -20 || p.x > geom.canvas.width + 20 || p.y < -20 || p.y > geom.canvas.height + 20) return;
    drawGridMarker(ctx, p.x, p.y, `T${task.id}`, { color: "#ffffff", fillColor: "#e37222" });
  });

  (OCTOPUS.latest.locations || []).forEach((loc) => {
    const lat = safeNumber(loc.lat, NaN);
    const lon = safeNumber(loc.lon, NaN);
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const id = loc.id || loc.origin_id || loc.device_id || "?";
    const b = batteryForDevice(id) || {};
    const type = fleetType({ ...loc, ...b, id });
    const local = latLngToLocal(lat, lon);
    if (local.x < 0 || local.x > geom.width_m || local.y < 0 || local.y > geom.height_m) return;
    const p = localMetersToCanvas(geom, local.x, local.y);
    if (p.x < -20 || p.x > geom.canvas.width + 20 || p.y < -20 || p.y > geom.canvas.height + 20) return;
    const label = type === "drone" ? "D" : type === "base" ? "B" : String(id).replace("octopus_", "R").replace("robot_", "R").replace("home_", "H");
    drawGridMarker(ctx, p.x, p.y, label, deviceStyle(type, b.state || loc.state));
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
    if (trash >= 0.65 && layer !== "trash_probability" && Math.max(w, h) >= 3) {
      ctx.beginPath();
      ctx.arc(x + w / 2, y + h / 2, Math.max(2, Math.min(w, h) * 0.28), 0, Math.PI * 2);
      ctx.fillStyle = "rgba(227, 114, 34, 0.9)";
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

  const onlineFleet = battery.length;
  $("kpi-fleet").textContent = onlineFleet;
  $("kpi-fleet-sub").textContent = `${safeNumber(stats.drones, 0)} drones · ${safeNumber(stats.robots, 0)} robots in DB`;

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

  const battery = OCTOPUS.latest.battery || [];
  if (!battery.length) {
    el.innerHTML = `<div class="item-card"><div class="item-title">No fleet data</div><div class="item-meta">Waiting for /api/battery data.</div></div>`;
    return;
  }

  el.innerHTML = `<div class="compact-list">${battery.map((b) => {
    const age = ageSeconds(b.ts);
    const fresh = freshnessFromAge(age, 5, 30);
    const percent = clamp(safeNumber(b.percent, 0), 0, 100);
    const typeGuess = String(b.id || "").toLowerCase().includes("drone") ? "Drone" : "Robot";
    return `
      <div class="item-card" data-device-id="${escapeHtml(b.id)}">
        <div class="item-top">
          <div class="item-title">${escapeHtml(b.id || "unknown")}</div>
          ${statusPill(fresh.label, fresh.state)}
        </div>
        <div class="item-meta">${typeGuess} · state: ${escapeHtml(b.state || "unknown")} · battery: ${percent}%</div>
        <div class="progress"><span style="width:${percent}%"></span></div>
      </div>
    `;
  }).join("")}</div>`;
}

function renderTasks() {
  const el = $("tasks-content");
  if (!el) return;

  const tasks = OCTOPUS.latest.tasks || [];
  if (!tasks.length) {
    el.innerHTML = `<div class="item-card"><div class="item-title">No tasks</div><div class="item-meta">Later this becomes the detection and trash assignment queue.</div></div>`;
    return;
  }

  el.innerHTML = `<div class="compact-list">${tasks.slice(0, 8).map((t) => {
    const age = ageSeconds(t.ts);
    const fresh = freshnessFromAge(age, 30, 180);
    const assigned = t.assigned || t.assigned_to || "unassigned";
    return `
      <div class="item-card">
        <div class="item-top">
          <div class="item-title">Task #${escapeHtml(t.id)}</div>
          ${statusPill(escapeHtml(t.status || "unknown"), taskStateToStatus(t.status))}
        </div>
        <div class="item-meta">
          Assigned: <span class="accent">${escapeHtml(assigned)}</span><br />
          Position: ${safeNumber(t.lat, 0).toFixed(5)}, ${safeNumber(t.lon, 0).toFixed(5)}<br />
          Data: ${fresh.label}
        </div>
      </div>
    `;
  }).join("")}</div>`;
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

  const battery = OCTOPUS.latest.battery || [];
  const hasDrone = battery.some((b) => String(b.id || "").toLowerCase().includes("drone"));
  const hasRobot = battery.some((b) => String(b.id || "").toLowerCase().includes("robot"));
  const patchFresh = freshnessFromAge(ageSeconds(OCTOPUS.latest.patch?.received_at), 2, 10);
  const mapCells = Object.keys(OCTOPUS.latest.globalMap?.cells || {}).length;

  const rows = [
    ["Mission polygon defined", "unknown", "planning tool later"],
    ["Home position set", "unknown", "planning tool later"],
    ["Backend API", OCTOPUS.backendOk ? "fresh" : "offline", OCTOPUS.backendOk ? "OK" : "offline"],
    ["ROS map patch bridge", patchFresh.state, patchFresh.label],
    ["Local grid map", mapCells > 0 ? "fresh" : "warning", mapCells > 0 ? `${mapCells} cells` : "empty"],
    ["Drone available", hasDrone ? "fresh" : "warning", hasDrone ? "seen in fleet" : "waiting"],
    ["Ground robot available", hasRobot ? "fresh" : "warning", hasRobot ? "seen in fleet" : "waiting"],
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
    const loc = selected.location || {};
    const b = selected.battery || {};
    const local = selected.local || {};
    el.innerHTML = `
      <div class="item-card">
        <div class="item-top">
          <div class="item-title">${escapeHtml(selected.id)}</div>
          ${statusPill(escapeHtml(selected.device_type || "device"), selected.device_type === "drone" ? "fresh" : "ok")}
        </div>
        <table class="status-table">
          <tbody>
            <tr><td>state</td><td>${escapeHtml(b.state || loc.state || "unknown")}</td></tr>
            <tr><td>battery</td><td>${b.percent !== undefined ? `${safeNumber(b.percent, 0).toFixed(0)}%` : "unknown"}</td></tr>
            <tr><td>lat/lon</td><td>${safeNumber(loc.lat, 0).toFixed(6)}, ${safeNumber(loc.lon, 0).toFixed(6)}</td></tr>
            <tr><td>local x/y</td><td>${safeNumber(local.x, 0).toFixed(2)}, ${safeNumber(local.y, 0).toFixed(2)} m</td></tr>
            <tr><td>last seen</td><td>${formatAge(ageSeconds(loc.ts || b.ts))}</td></tr>
          </tbody>
        </table>
      </div>
    `;
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

  if (OCTOPUS.latest.globalMap || OCTOPUS.osmPriors) {
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

async function refreshAll() {
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

  const canvas = $("grid-map-canvas");
  if (canvas) {
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
setupEventListeners();
renderMissionPhase();
renderTimeline();
refreshAll();
setInterval(refreshAll, 5000);
