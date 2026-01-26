// Beispiel-Daten: ersetzt später durch echtes API-Fetch
const tasks = [
  { id: 1, lat: 48.137, lon: 11.576, assigned: "robot_1", status: "in_progress", ts: "2025-11-13T12:00:00Z" },
  { id: 2, lat: 48.138, lon: 11.580, assigned: "robot_2", status: "open", ts: "2025-11-13T12:02:00Z" }
];

const battery = [
  { id: "drone_1", percent: 87, state: "active", ts: "2025-11-13T12:05:00Z" },
  { id: "robot_1", percent: 65, state: "active", ts: "2025-11-13T12:04:00Z" },
  { id: "robot_2", percent: 92, state: "idle", ts: "2025-11-13T12:01:00Z" },
  { id: "robot_3", percent: 71, state: "charging", ts: "2025-11-13T11:50:00Z" }
];

const stats = {
  runtime: "2h 13min",
  robots: 3,
  drones: 1,
  trash_collected: 24,
  open_tasks: 1,
  last_update: "2025-11-13T12:06:00Z"
};

// Render-Funktionen
function renderTasks(){
  const el = document.getElementById("tasks-content");
  if(!tasks.length) { el.innerHTML = "<i>Keine Tasks</i>"; return; }
  let html = "<ul class='simple'>";
  tasks.forEach(t => {
    html += `<li><strong>Task #${t.id}</strong> — ${t.status} — ${t.assigned}<br/><small>${t.lat.toFixed(6)}, ${t.lon.toFixed(6)} @ ${t.ts}</small></li>`;
  });
  html += "</ul>";
  el.innerHTML = html;
}

function renderBattery(){
  const el = document.getElementById("battery-content");
  let html = "<ul class='simple'>";
  battery.forEach(b => {
    html += `<li><strong>${b.id}</strong> — ${b.percent}% — ${b.state} <small>(${b.ts})</small></li>`;
  });
  html += "</ul>";
  el.innerHTML = html;
}

function renderStats(){
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

// initial render
renderTasks();
renderBattery();
renderStats();


setInterval(() => {
  renderTasks(); renderBattery(); renderStats();
}, 5000);
