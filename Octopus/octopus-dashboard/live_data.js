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

async function refreshAll(){
  try {
    await Promise.all([loadTasks(), loadBattery(), loadStats()]);
  } catch(e){
    console.error("API error", e);
  }
}

// initial load + periodic refresh
refreshAll();
setInterval(refreshAll, 5000);
