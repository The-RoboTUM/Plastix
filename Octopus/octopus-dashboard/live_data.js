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

async function refreshAll(){
  try {
    await Promise.all([loadTasks(), loadBattery(), loadStats(), loadMapPatch()]);
  } catch(e){
    console.error("API error", e);
  }
}

// initial load + periodic refresh
refreshAll();
setInterval(refreshAll, 5000);
