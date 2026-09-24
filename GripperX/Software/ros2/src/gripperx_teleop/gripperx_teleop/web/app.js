/* GripperX teleop console — browser side.
 *
 * The one rule worth stating up front: this page never sends "key released".
 * Every beat carries the COMPLETE set of keys currently held. A beat that goes
 * missing therefore cannot leave the robot driving — the node simply stops
 * hearing the set and lets it age out, which is the same dead-man condition
 * the terminal teleop has always had. Key-up only makes the stop faster.
 */
'use strict';

const SESSION = 'ui-' + Math.random().toString(36).slice(2, 10);

/* Keys that are HELD (dead-man). Everything else is a one-shot edge. */
const HOLD_KEYS = {
  w: 'w', s: 's', a: 'a', d: 'd',
  // REBOUND 2026-08-24. The arrows are no longer four manoeuvres: left/right
  // still pick the crab's side, but up/down now rotate the crab's DIRECTION OF
  // TRAVEL, and the two spins moved to 0 and 9. Arrow-up meaning "spin
  // clockwise" read as "forward" to anyone who had not been told.
  ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right',
  '0': '0', '9': '9',
};
/* The same set seen from the other side: the names the NODE uses. The drawn
 * keycaps carry these in data-key, because that is the vocabulary shared with
 * the server -- the browser's own "ArrowLeft" spelling only exists in the
 * keyboard event map above. */
const NODE_HOLD_KEYS = new Set(Object.values(HOLD_KEYS));

/* One-shot actions, by their lowercase key. */
const ACTION_KEYS = {
  k: 'mode_keyboard', g: 'mode_autonomous', p: 'pick',
  o: 'open_gripper', i: 'home', u: 'arm_gate', l: 'disarm_gate',
};

const BEAT_ACTIVE_MS = 50;    // while anything is held
const BEAT_IDLE_MS = 250;     // just keeping the control lease alive

const held = new Set();       // node key names, e.g. "w", "left"
const queue = [];             // one-shot events waiting for the next beat
let hasControl = false;
let holder = null;
let wantControl = true;       // false once we have decided to observe
let inFlight = false;
let beatTimer = null;
let lastFrameAt = 0;
let lastEventId = 0;
let renderFailures = 0;       // consecutive render() faults, surfaced as a banner
let pendingKick = false;      // a beat came in while one was already in flight
let claimForce = false;       // the operator pressed "Take control"

const $ = (id) => document.getElementById(id);

/* ── Transport ──────────────────────────────────────────────────────────── */

async function beat(force) {
  if (inFlight) { if (force) pendingKick = true; return; }
  inFlight = true;
  const events = queue.splice(0, queue.length);
  const body = {
    session: SESSION,
    keys: [...held],
    events,
    claim: wantControl && !hasControl,
  };
  if (claimForce) { body.claim = true; body.force = true; claimForce = false; }
  try {
    const res = await fetch('/api/input', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await res.json();
    hasControl = !!data.control;
    holder = data.holder;
    if (!hasControl && holder && holder !== SESSION) {
      wantControl = false;                 // someone else is driving
    } else if (!holder) {
      wantControl = true;                  // the seat is free again
    }
  } catch (err) {
    // The node is gone or the network dropped. Nothing to do from here — the
    // node stops the robot by itself when the beats stop arriving. Just say so.
    hasControl = false;
    if (events.length) queue.unshift(...events);   // retry one-shots
  } finally {
    inFlight = false;
    if (pendingKick) { pendingKick = false; beat(false); }
  }
}

function schedule() {
  clearTimeout(beatTimer);
  const period = held.size ? BEAT_ACTIVE_MS : BEAT_IDLE_MS;
  beatTimer = setTimeout(() => { beat(false); schedule(); }, period);
}

/* Send right now rather than waiting up to a full beat — this is what makes a
 * key press and, more importantly, a key RELEASE feel immediate. */
function kick() { beat(true); schedule(); }

function press(key) {
  if (held.has(key)) return;
  held.add(key);
  paint();
  kick();
}

function release(key) {
  if (!held.delete(key)) return;
  paint();
  kick();
}

function releaseAll() {
  if (!held.size) return;
  held.clear();
  paint();
  kick();
}

function fire(event) {
  queue.push(event);
  kick();
}

/* ── Keyboard ───────────────────────────────────────────────────────────── */

window.addEventListener('keydown', (ev) => {
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return;

  const holdKey = HOLD_KEYS[ev.key] || HOLD_KEYS[ev.key.toLowerCase()];
  if (holdKey) {
    ev.preventDefault();                 // arrows must not scroll the page
    press(holdKey);
    return;
  }
  if (ev.key === ' ' || ev.key === 'Spacebar') {
    ev.preventDefault();
    if (!ev.repeat) estop();
    return;
  }
  if (ev.repeat) return;                 // one-shots must not auto-repeat

  const lower = ev.key.toLowerCase();
  if (ACTION_KEYS[lower]) {
    ev.preventDefault();
    flash(lower);
    fire(ACTION_KEYS[lower]);
    return;
  }
  /* Shift+Q, not a bare Q: in a terminal a stray Q only ends your own session,
   * but here it would shut the teleop node down from across the room. */
  if (lower === 'q' && ev.shiftKey) {
    ev.preventDefault();
    requestQuit();
  }
});

window.addEventListener('keyup', (ev) => {
  const holdKey = HOLD_KEYS[ev.key] || HOLD_KEYS[ev.key.toLowerCase()];
  if (holdKey) { ev.preventDefault(); release(holdKey); }
});

/* Any way of losing the keyboard is a release. Without this, alt-tabbing away
 * mid-drive would leave a key "held" with no key-up ever coming — the browser
 * stops delivering key events but the page would keep re-asserting the set. */
for (const evt of ['blur', 'pagehide']) {
  window.addEventListener(evt, releaseAll);
}
document.addEventListener('visibilitychange', () => {
  if (document.hidden) releaseAll();
});

/* ── Pointer / touch on the drawn keys ──────────────────────────────────── */

function wireKeycaps() {
  for (const cap of document.querySelectorAll('.keycap')) {
    const nodeKey = NODE_HOLD_KEYS.has(cap.dataset.key) ? cap.dataset.key : null;
    const event = cap.dataset.event;

    if (nodeKey) {
      cap.addEventListener('pointerdown', (ev) => {
        ev.preventDefault();
        cap.setPointerCapture(ev.pointerId);
        press(nodeKey);
      });
      for (const evt of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        cap.addEventListener(evt, () => release(nodeKey));
      }
    } else if (event === 'quit') {
      cap.addEventListener('click', requestQuit);
    } else if (event) {
      cap.addEventListener('click', () => { flash(cap.dataset.key); fire(event); });
    }
  }

  const stop = $('btn-estop');
  stop.addEventListener('pointerdown', (ev) => { ev.preventDefault(); estop(); });
  $('btn-takeover').addEventListener('click', () => {
    claimForce = true; wantControl = true; kick();
  });
}

function estop() {
  releaseAll();
  fire('estop');
  const btn = $('btn-estop');
  btn.classList.add('on');
  setTimeout(() => btn.classList.remove('on'), 180);
}

function requestQuit() {
  releaseAll();
  if (window.confirm('Shut the teleop node down?\n\nThe robot is stopped and '
                     + 'the wheels straightened first, but you will need a '
                     + 'terminal to start teleop again.')) {
    fire('quit');
  }
}

function flash(key) {
  const cap = document.querySelector(`.keycap[data-key="${CSS.escape(key)}"]`);
  if (!cap) return;
  cap.classList.add('on');
  setTimeout(() => cap.classList.remove('on'), 150);
}

/* ── Painting: local key state ──────────────────────────────────────────── */

function paint() {
  for (const cap of document.querySelectorAll('.keycap')) {
    const nodeKey = cap.dataset.key;
    if (NODE_HOLD_KEYS.has(nodeKey)) cap.classList.toggle('on', held.has(nodeKey));
  }
}

/* ── Telemetry ──────────────────────────────────────────────────────────── */

function connect() {
  const source = new EventSource('/api/telemetry');
  source.onmessage = (ev) => {
    lastFrameAt = performance.now();
    let frame;
    try {
      frame = JSON.parse(ev.data);
    } catch (err) {
      return;                            // a truncated frame; the next one is 50 ms away
    }
    try {
      render(frame);
      renderFailures = 0;
    } catch (err) {
      /* A render fault is NOT harmless: it freezes half the page while the
       * other half keeps updating, which reads as "the robot stopped doing
       * that" rather than "the page stopped drawing it". Swallowing it
       * silently is how the wheel-view bug survived. */
      renderFailures += 1;
      if (renderFailures === 1 || renderFailures % 50 === 0) console.error(err);
    }
  };
  source.onerror = () => { /* EventSource reconnects on its own */ };
}

/* ── Rendering ──────────────────────────────────────────────────────────── */

const text = (el, value) => { if (el.textContent !== value) el.textContent = value; };

function render(f) {
  const stale = (performance.now() - lastFrameAt) > 1500;

  renderRole(f, stale);
  renderBanner(f);
  renderState(f);
  renderReadouts(f);
  renderRobot(f);
  renderLog(f.events || []);
}

function renderRole(f, stale) {
  const dot = $('link-dot');
  const chip = $('chip-role');
  const takeover = $('btn-takeover');

  dot.className = 'dot ' + (stale ? 'dead' : (f.link_fresh ? 'live' : 'stale'));
  chip.classList.remove('driving', 'observer', 'lost');

  /* Holding the control lease is NOT the same as driving the robot. In any
   * mode but keyboard, teleop_mux forwards Nav2 and drops what this page
   * publishes, so "you are driving" would be false however firmly this session
   * holds the lease. The chip reports what is actually reaching the wheels. */
  const teleopLive = (f.mode === 'keyboard');

  if (stale) {
    text(chip, 'node unreachable');
    chip.classList.add('lost');
    takeover.classList.add('idle');
  } else if (!teleopLive) {
    text(chip, hasControl ? 'lease held · keys inert' : 'observing · keys inert');
    chip.classList.add('observer');
    takeover.classList.toggle('idle', hasControl || !holder);
  } else if (hasControl) {
    text(chip, 'you are driving');
    chip.classList.add('driving');
    takeover.classList.add('idle');
  } else if (holder) {
    text(chip, 'observing — ' + holder + ' drives');
    chip.classList.add('observer');
    takeover.classList.remove('idle');
  } else {
    text(chip, 'claiming control…');
    takeover.classList.add('idle');
  }

  text($('chip-mode'), 'mode ' + (f.mode || '—'));
  /* And say it on the deck as well: grey keys are keys that do nothing. */
  document.querySelector('.deck').classList.toggle('inert', !stale && !teleopLive);
}

function renderBanner(f) {
  const el = $('banner');
  let level = null, message = '';

  if (f.rivals && f.rivals.length) {
    /* Above the emergency stop on purpose: this is the only state on this page
     * where hitting the stop does not necessarily stop the robot. */
    level = 'alarm';
    message = 'ANOTHER TELEOP IS RUNNING (' + f.rivals.join(', ') + '). It '
            + 'publishes the same cmd_vel as this page, so your emergency stop '
            + 'does NOT stop what it commands. Shut one of them down.';
  } else if (renderFailures >= 3) {
    level = 'warn';
    message = 'This page is failing to draw the telemetry it is receiving '
            + '(' + renderFailures + ' frames). What you see may be stale — '
            + 'check the browser console and do not drive on it.';
  } else if (f.estop_latched) {
    level = 'alarm';
    message = 'EMERGENCY STOP LATCHED — release every key; input stays ignored '
            + 'until nothing is held.';
  } else if (!hasControl && holder && holder !== SESSION) {
    level = 'info';
    message = 'Observer mode. You can watch and you can hit the emergency stop, '
            + 'but the drive keys belong to ' + holder + '.';
  } else if (f.psi_jump_ago_sec !== null && f.psi_jump_ago_sec !== undefined
             && f.psi_jump_ago_sec < 2.5) {
    /* A transient warning outranks the standing mode notice below: the mode is
     * still true in three seconds, the dead-band crossing is not. */
    level = 'warn';
    message = 'Crab heading jumped a dead band — no pose exists in that 45° '
            + 'gap, so all four modules are swinging across it. Traction is '
            + 'NOT withheld for this: the teleop guard only covers a change of '
            + 'manoeuvre.';
  } else if (f.mode && f.mode !== 'keyboard') {
    level = 'info';
    message = 'Mode is ' + f.mode + ': teleop_mux is forwarding Nav2, not this '
            + 'page. W/S, A/D and the manoeuvre keys do nothing until you press '
            + 'K. The space bar still works — it forces keyboard mode and stops.';
  } else if (f.armed_without_feedback) {
    level = 'warn';
    message = 'No steering feedback — traction was released on the timeout, '
            + 'not on measured wheel angles. The pose is not confirmed.';
  } else if (f.pose_reachable === false) {
    level = 'warn';
    message = 'That pose is outside the steering limits — the manoeuvre will '
            + 'not arm.';
  } else if (f.silent_sec !== null && f.silent_sec !== undefined
             && !f.link_fresh && holder === SESSION) {
    level = 'warn';
    message = 'Your input has not reached the node for '
            + f.silent_sec.toFixed(1) + ' s — the robot has been stopped.';
  }

  if (!level) {
    /* Nothing wrong. The strip stays, and carries the calm summary -- the
     * space is reserved either way, so it may as well say something true. */
    level = 'calm';
    message = (f.mode === 'keyboard'
                ? 'Keyboard mode'
                : 'Mode: ' + (f.mode || 'unknown') + ' — the space bar still stops')
            + ' · ' + (f.manoeuvre_label || '')
            + (f.drive_allowed ? ' · drive released' : ' · drive withheld');
  }
  el.className = 'banner ' + level + (level === 'alarm' ? ' blink' : '');
  text($('banner-icon'), level === 'alarm' ? '\u26a0' :
                         level === 'warn'  ? '\u26a0' :
                         level === 'calm'  ? '\u2022' : 'i');
  text($('banner-text'), message);
}

function renderState(f) {
  const chip = $('state-chip');
  const state = f.guard_state || '—';
  let cls = 'state';
  if (f.pose_reachable === false) cls += ' bad';
  else if (state === 'armed') cls += f.armed_without_feedback ? ' aligning' : ' armed';
  else if (state === 'aligning') cls += ' aligning';
  else if (state === 'releasing') cls += ' releasing';
  if (chip.className !== cls) chip.className = cls;

  text($('state-name'), state === 'armed'
    ? (f.drive_allowed ? 'drive released' : 'armed')
    : state);

  /* The node already writes one operator-facing sentence per state; show that
   * rather than inventing a second wording that could drift from it. */
  const status = f.status || '';
  const note = status.includes('-') ? status.slice(status.indexOf('-') + 1).trim() : status;
  text($('state-note'), note || 'waiting for the node');
}

function renderReadouts(f) {
  const lim = (f.limits || {});
  text($('rd-manoeuvre'), f.manoeuvre_label || '—');

  const steer = f.steer_deg || 0;
  text($('rd-steer'), steer.toFixed(1) + '°');
  const limit = lim.steer_limit_deg || 35;
  const bar = $('rd-steer-bar');
  const frac = Math.max(-1, Math.min(1, steer / limit));
  bar.style.width = Math.abs(frac) * 50 + '%';
  /* A POSITIVE steering angle turns the wheels to the robot's LEFT, so the bar
   * has to grow leftwards to read as the direction it is. It used to grow
   * right, which put the indicator on the opposite side to the turn. */
  bar.style.left = frac >= 0 ? (50 - Math.abs(frac) * 50) + '%' : '50%';
  text($('rd-steer-limit'), '± ' + limit.toFixed(0) + '°');
  if (lim.steer_return_rate_deg_s) {
    // Straight ahead is the resting state now, so how fast it gets back there
    // is a number the operator should be able to see, not infer.
    text($('rd-return'),
      '· back to straight at ' + lim.steer_return_rate_deg_s.toFixed(0) + '°/s');
  }

  const psi = $('rd-psi');
  if (f.crab_psi_deg === null || f.crab_psi_deg === undefined) {
    text(psi, 'no crab active');
    psi.className = 'dim';
  } else {
    text(psi, f.crab_psi_deg.toFixed(1) + '\u00b0 from straight ahead'
              + (f.crab_psi_snap ? '  \u00b7 jumps the gaps'
                                 : '  \u00b7 stops at the gap'));
    psi.className = '';
  }

  const c = f.cmd || { vx: 0, vy: 0, wz: 0 };
  text($('rd-cmd'),
    `vx ${c.vx.toFixed(3)}  vy ${c.vy.toFixed(3)}  ω ${c.wz.toFixed(3)}`);

  const source = f.measured_source;
  text($('lg-measured-label'),
    source === 'joint_states' ? 'measured (joint_states)'
    : source === 'hw' ? 'measured servo'
    : 'measured — no source');

  const align = $('rd-align');
  const err = alignmentError(f);
  if (err === null) {
    text(align, 'no steering feedback');
    align.className = 'warn';
  } else {
    const tol = lim.align_tolerance_deg || 6;
    text(align, 'max ' + err.toFixed(1) + '° off  (tol ' + tol.toFixed(0) + '°)'
               + (source === 'joint_states' ? '  · from joint_states' : ''));
    align.className = err <= tol ? 'ok' : 'warn';
  }

  const gate = $('rd-gate');
  if (!f.gate) { text(gate, 'unknown'); gate.className = 'dim'; }
  else if (f.gate.armed) {
    text(gate, `ARMED ${f.gate.seconds_remaining.toFixed(0)} s left`
               + (f.gate.armed_by ? ' · ' + f.gate.armed_by : ''));
    gate.className = 'warn';
  } else { text(gate, 'disarmed'); gate.className = 'ok'; }

  const link = $('rd-link');
  if (f.link_fresh) {
    const window = lim.deadman_window_sec !== undefined
      ? lim.deadman_window_sec : lim.drive_hold_sec;
    text(link, 'beats arriving · dead-man ' + (window || 0.6).toFixed(2) + ' s');
    link.className = 'ok';
  } else {
    text(link, f.silent_sec === null || f.silent_sec === undefined
      ? 'no input yet' : 'silent ' + f.silent_sec.toFixed(1) + ' s');
    link.className = 'warn';
  }
}

function alignmentError(f) {
  if (!f.target_deg || !f.measured_deg) return null;
  let worst = 0;
  for (let i = 0; i < 4; i++) {
    let diff = f.target_deg[i] - f.measured_deg[i];
    diff = ((diff + 180) % 360 + 360) % 360 - 180;
    worst = Math.max(worst, Math.abs(diff));
  }
  return worst;
}

/* ── Robot top view ─────────────────────────────────────────────────────── */

const SVG_NS = 'http://www.w3.org/2000/svg';
const PX_PER_M = 470;
/* Joint order, everywhere in this stack: FL, FR, BL, BR. The signs below turn
 * that into screen coordinates: robot +x is forward (screen up), robot +y is
 * left (screen left), so screen_x = -y*S and screen_y = -x*S. */
const WHEELS = [
  { name: 'FL', sx: -1, sy: -1 },
  { name: 'FR', sx: +1, sy: -1 },
  { name: 'BL', sx: -1, sy: +1 },
  { name: 'BR', sx: +1, sy: +1 },
];
let builtFor = null;

function el(tag, attrs) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  return node;
}

function buildRobot(geom) {
  const key = `${geom.a}|${geom.b}|${geom.wheel_radius}`;
  if (builtFor === key) return;
  builtFor = key;

  const halfLen = geom.a * PX_PER_M;          // king-pin to king-pin / 2
  const halfWid = geom.b * PX_PER_M;
  const wheelLen = geom.wheel_radius * 2 * PX_PER_M;
  const wheelWid = Math.max(12, wheelLen * 0.34);

  const chassis = $('chassis');
  chassis.querySelector('.body').setAttribute('x', -halfWid * 0.86);
  chassis.querySelector('.body').setAttribute('y', -halfLen * 1.02);
  chassis.querySelector('.body').setAttribute('width', halfWid * 1.72);
  chassis.querySelector('.body').setAttribute('height', halfLen * 2.04);
  const noseY = -halfLen * 1.02;
  chassis.querySelector('.nose')
         .setAttribute('d', `M-14,${noseY - 3} L0,${noseY - 20} L14,${noseY - 3} Z`);
  chassis.querySelector('.front-label').setAttribute('y', noseY - 27);

  const group = $('modules');
  group.replaceChildren();
  group.appendChild(el('line', {
    class: 'axle', x1: -halfWid, y1: -halfLen, x2: halfWid, y2: -halfLen }));
  group.appendChild(el('line', {
    class: 'axle', x1: -halfWid, y1: halfLen, x2: halfWid, y2: halfLen }));

  for (const wheel of WHEELS) {
    const x = wheel.sx * halfWid;
    const y = wheel.sy * halfLen;
    // The class is what renderRobot() selects on. It must NOT select on
    // `g[transform]`: from the second frame onwards the inner target/measured
    // groups carry a transform too, so that selector returns 12 elements
    // instead of 4 and everything after the first wheel is the wrong node.
    const module = el('g', { class: 'module', transform: `translate(${x},${y})` });

    const rect = (cls) => el('rect', {
      class: cls, x: -wheelWid / 2, y: -wheelLen / 2,
      width: wheelWid, height: wheelLen, rx: wheelWid / 2,
    });

    const measured = el('g', { class: 'measured-g', 'data-role': 'measured' });
    measured.appendChild(rect('measured'));
    const target = el('g', { class: 'target-g', 'data-role': 'target' });
    target.appendChild(rect('target'));

    module.appendChild(measured);
    module.appendChild(target);
    module.appendChild(el('circle', { class: 'hub', r: 3 }));

    const label = el('text', { class: 'wheel-label', x: 0,
      y: wheel.sy < 0 ? -wheelLen / 2 - 7 : wheelLen / 2 + 14 });
    label.textContent = wheel.name;
    module.appendChild(label);

    group.appendChild(module);
  }
}

/* Where a heading psi lands on screen. Same projection as the wheels: robot
 * +x is forward (screen up) and +y is left (screen left), so psi = 0 points at
 * the top of the dial and psi grows counter-clockwise. */
function dialPoint(psiDeg, radius) {
  const psi = psiDeg * Math.PI / 180;
  return [-Math.sin(psi) * radius, -Math.cos(psi) * radius];
}

const DIAL_R = 128;

function renderDial(f) {
  const group = $('dial');
  const arcs = f.translation_arcs_deg;
  if (!arcs) { group.replaceChildren(); return; }

  /* The arcs come out of the steering calibration and only change when that
   * does, so draw them once and then only move the needle. */
  const key = JSON.stringify(arcs);
  if (group.dataset.builtFor !== key) {
    group.dataset.builtFor = key;
    group.replaceChildren();
    // The whole circle first, as the "no pose exists" ground...
    group.appendChild(el('circle', { class: 'dial-dead', r: DIAL_R }));
    // ...then the reachable arcs painted over it.
    for (const [low, high] of arcs) {
      const [x1, y1] = dialPoint(low, DIAL_R);
      const [x2, y2] = dialPoint(high, DIAL_R);
      const large = (high - low) > 180 ? 1 : 0;
      // psi increasing runs counter-clockwise on screen, which is sweep 0.
      group.appendChild(el('path', {
        class: 'dial-reach',
        d: `M${x1.toFixed(2)},${y1.toFixed(2)} A${DIAL_R},${DIAL_R} 0 ${large},0 `
           + `${x2.toFixed(2)},${y2.toFixed(2)}`,
      }));
    }
    const needle = el('g', { class: 'dial-needle' });
    needle.appendChild(el('line', { x1: 0, y1: -DIAL_R + 18, x2: 0, y2: -DIAL_R - 9 }));
    group.appendChild(needle);
  }

  const needle = group.querySelector('.dial-needle');
  const psi = f.crab_psi_deg;
  if (psi === null || psi === undefined) {
    needle.style.display = 'none';
  } else {
    needle.style.display = '';
    // Drawn pointing at psi = 0, so it only has to be turned. Negated because
    // SVG rotates clockwise with y down.
    needle.setAttribute('transform', `rotate(${-psi})`);
  }
}

function renderRobot(f) {
  if (!f.geometry) return;
  buildRobot(f.geometry);

  const modules = $('modules').querySelectorAll('.module');
  const target = f.target_deg;
  const measured = f.measured_deg;
  const tol = (f.limits || {}).align_tolerance_deg || 6;

  modules.forEach((module, i) => {
    const targetG = module.querySelector('[data-role="target"]');
    const measuredG = module.querySelector('[data-role="measured"]');
    /* SVG rotates clockwise with y down; a positive joint angle turns the
     * wheel toward the robot's left, which is counter-clockwise on screen. */
    if (target) targetG.setAttribute('transform', `rotate(${-target[i]})`);
    targetG.style.display = target ? '' : 'none';

    if (measured) {
      measuredG.setAttribute('transform', `rotate(${-measured[i]})`);
      measuredG.style.display = '';
      const err = target ? Math.abs(target[i] - measured[i]) : 99;
      measuredG.querySelector('rect')
               .classList.toggle('aligned', err <= tol && !!f.drive_allowed);
    } else {
      measuredG.style.display = 'none';
    }
  });

  renderTwist(f);
  renderDial(f);
}

function renderTwist(f) {
  const group = $('twist');
  group.replaceChildren();
  const c = f.cmd || { vx: 0, vy: 0, wz: 0 };
  const lim = f.limits || {};
  const vmax = Math.max(0.05, lim.linear_vel_m_s || 0.5, lim.crab_speed_m_s || 0.25);
  const wmax = Math.max(0.05, lim.spin_speed_rad_s || 1.4);

  const speed = Math.hypot(c.vx, c.vy);
  if (speed > 1e-4) {
    const len = 34 + Math.min(1, speed / vmax) * 44;
    // screen_x = -vy, screen_y = -vx  (see WHEELS above)
    const ux = -c.vy / speed, uy = -c.vx / speed;
    group.appendChild(el('line', {
      x1: ux * 14, y1: uy * 14, x2: ux * len, y2: uy * len,
      'marker-end': 'url(#arrowhead)',
    }));
  }

  if (Math.abs(c.wz) > 1e-4) {
    const r = 30;
    const sweep = Math.min(1, Math.abs(c.wz) / wmax) * 3.6 + 0.8;  // radians of arc
    // Positive omega is counter-clockwise in the robot frame and, with this
    // projection, counter-clockwise on screen too.
    const dir = c.wz > 0 ? -1 : 1;
    const a0 = -Math.PI / 2;
    const a1 = a0 + dir * sweep;
    const p = (ang) => `${(r * Math.cos(ang)).toFixed(2)},${(r * Math.sin(ang)).toFixed(2)}`;
    const large = sweep > Math.PI ? 1 : 0;
    const clockwise = dir > 0 ? 1 : 0;
    group.appendChild(el('path', {
      d: `M${p(a0)} A${r},${r} 0 ${large},${clockwise} ${p(a1)}`,
      'marker-end': 'url(#arrowhead)',
    }));
  }

  if (!f.measured_deg) {
    const warn = el('text', { class: 'no-feedback', x: 0, y: 158 });
    warn.textContent = 'no steer feedback';
    group.appendChild(warn);
  }
}

/* ── Event log ──────────────────────────────────────────────────────────── */

function renderLog(events) {
  const list = $('log');
  for (const item of events) {
    if (item.id <= lastEventId) continue;
    lastEventId = item.id;
    const li = document.createElement('li');
    li.className = item.level || 'info';
    const stamp = document.createElement('time');
    stamp.textContent = item.wall;
    li.appendChild(stamp);
    li.appendChild(document.createTextNode(item.text));
    list.insertBefore(li, list.firstChild);
  }
  while (list.children.length > 60) list.removeChild(list.lastChild);
}

/* ── Go ─────────────────────────────────────────────────────────────────── */

wireKeycaps();
connect();
schedule();
beat(true);
