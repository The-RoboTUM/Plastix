async function loadTasks(){
  const res = await fetch('/api/tasks');
  const tasks = await res.json();
  const el = document.getElementById("tasks-content");
  if(!tasks.length){ el.innerHTML = "<i>Keine Tasks</i>"; return; }
  let html = "<ul class='simple'>";
  tasks.forEach(t => {
    html += `<li><strong>Task #${t.id}</strong> — ${t.status} — ${t.assigned || t.assigned_to || 'unassigned'}<br/><small>${(t.lat||'')}, ${(t.lon||'')} @ ${t.ts||''}</small></li>`;
  });
  html += "</ul>";
  el.innerHTML = html;
}

async function loadBattery(){
  const res = await fetch('/api/battery');
  const battery = await res.json();
  const el = document.getElementById("battery-content");
  let html = "<ul class='simple'>";
  battery.forEach(b => html += `<li><strong>${b.id}</strong> — ${b.percent}% — ${b.state} <small>(${b.ts})</small></li>`);
  html += "</ul>";
  el.innerHTML = html;
}

async function loadStats(){
  const res = await fetch('/api/stats');
  const stats = await res.json();
  const el = document.getElementById("stats-content");
  el.innerHTML = `
    <ul class="simple">
      <li>Runtime: <strong>${stats.runtime}</strong></li>
      <li>Robots: <strong>${stats.robots}</strong></li>
      <li>Drones: <strong>${stats.drones}</strong></li>
      <li>Trash collected: <strong>${stats.trash_collected}</strong></li>
      <li>Open tasks: <strong>${stats.open_tasks}</strong></li>
      <li>Last update: <small>${stats.last_update}</small></li>
    </ul>
  `;
}

async function loadMapPatch(){
  const el = document.getElementById("map-patch-content");
  if(!el){ return; }

  const res = await fetch('/api/map_patch/latest');
  const data = await res.json();

  if(data.status === "empty" || !data.patch){
    el.innerHTML = "<i>No map patch received yet.</i>";
    return;
  }

  const patch = data.patch;
  const cells = patch.updated_cells || [];

  if(!cells.length){
    el.innerHTML = "<i>Latest patch contains no updated cells.</i>";
    return;
  }

  let html = `
    <small>Frame: <strong>${patch.frame_id || "unknown"}</strong></small><br/>
    <small>Received: ${patch.received_at || "unknown"}</small>
    <table class="table table-sm mt-2">
      <thead>
        <tr>
          <th>Cell</th>
          <th>x,y</th>
          <th>Trash</th>
          <th>Conf.</th>
        </tr>
      </thead>
      <tbody>
  `;

  cells.slice(0, 5).forEach(c => {
    const trash = c.trash_probability ?? c.semantic_trash_probability ?? 0;
    const confidence = c.confidence ?? 0;
    html += `
      <tr>
        <td>${c.row}, ${c.col}</td>
        <td>${Number(c.x).toFixed(2)}, ${Number(c.y).toFixed(2)}</td>
        <td>${Number(trash).toFixed(2)}</td>
        <td>${Number(confidence).toFixed(2)}</td>
      </tr>
    `;
  });

  html += `
      </tbody>
    </table>
  `;

  el.innerHTML = html;
}

function valueForLayer(cell, layer){
  if(!cell){ return null; }

  if(layer === "coverage"){
    return cell.coverage ?? 0;
  }

  if(layer === "trash_probability"){
    return cell.trash_probability ?? 0;
  }

  if(layer === "obstacle_probability"){
    return cell.obstacle_probability ?? 0;
  }

  if(layer === "confidence"){
    return cell.confidence ?? 0;
  }

  return 0;
}

function colorForValue(value, layer){
  if(value === null){
    return "#f8f9fa";
  }

  const v = Math.max(0, Math.min(1, Number(value)));

  if(layer === "coverage"){
    if(v <= 0){ return "#f8f9fa"; }
    return `rgba(40, 120, 220, ${0.20 + 0.70 * v})`;
  }

  if(layer === "trash_probability"){
    if(v <= 0){ return "#f8f9fa"; }
    return `rgba(220, 50, 50, ${0.20 + 0.75 * v})`;
  }

  if(layer === "obstacle_probability"){
    if(v <= 0){ return "#f8f9fa"; }
    return `rgba(40, 40, 40, ${0.20 + 0.75 * v})`;
  }

  if(layer === "confidence"){
    if(v <= 0){ return "#f8f9fa"; }
    return `rgba(30, 160, 80, ${0.20 + 0.70 * v})`;
  }

  return "#f8f9fa";
}

function drawGridMap(mapData){
  const canvas = document.getElementById("grid-map-canvas");
  const info = document.getElementById("grid-map-info");
  const selector = document.getElementById("grid-layer-select");

  if(!canvas || !info || !selector){ return; }

  const ctx = canvas.getContext("2d");

  const rows = mapData.rows || 30;
  const cols = mapData.cols || 50;
  const cells = mapData.cells || {};
  const layer = selector.value || "coverage";

  const cellW = canvas.width / cols;
  const cellH = canvas.height / rows;

  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = "#f8f9fa";
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  Object.keys(cells).forEach(key => {
    const cell = cells[key];
    const row = Number(cell.row);
    const col = Number(cell.col);

    if(Number.isNaN(row) || Number.isNaN(col)){ return; }

    const value = valueForLayer(cell, layer);
    ctx.fillStyle = colorForValue(value, layer);

    const x = col * cellW;
    const y = canvas.height - (row + 1) * cellH;

    ctx.fillRect(x, y, cellW, cellH);
  });

  ctx.strokeStyle = "rgba(0,0,0,0.15)";
  ctx.lineWidth = 1;
  ctx.strokeRect(0, 0, canvas.width, canvas.height);

  const updatedCells = Object.keys(cells).length;
  info.innerHTML = `
    <small>
      Frame: <strong>${mapData.frame_id || "map"}</strong> |
      Size: ${cols} x ${rows} cells |
      Updated cells: <strong>${updatedCells}</strong> |
      Layer: <strong>${layer}</strong>
    </small>
  `;
}

async function loadGlobalMap(){
  const canvas = document.getElementById("grid-map-canvas");
  if(!canvas){ return; }

  const res = await fetch('/api/global_map/latest');
  const data = await res.json();

  if(data.status !== "ok" || !data.map){
    return;
  }

  drawGridMap(data.map);
}

const layerSelector = document.getElementById("grid-layer-select");
if(layerSelector){
  layerSelector.addEventListener("change", () => {
    loadGlobalMap();
  });
}

async function refreshAll(){
  try {
    await Promise.all([loadTasks(), loadBattery(), loadStats(), loadMapPatch(), loadGlobalMap()]);
  } catch(e){
    console.error("API error", e);
  }
}

// initial load + periodic refresh
refreshAll();
setInterval(refreshAll, 5000);
