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
  o: 'open_gripper', i: 'home', j: 'leader', u: 'arm_gate', l: 'disarm_gate',
};

const BEAT_ACTIVE_MS = 50;    // while anything is held
const BEAT_IDLE_MS = 250;     // just keeping the control lease alive
const POST_TIMEOUT_MS = 1000; // abort a hanging beat (TO-VERIFY)

const held = new Set();       // node key names, e.g. "w", "left"
const queue = [];             // one-shot events {name, id, t} waiting for the next beat
/* Arm-motion one-shots are NEVER re-sent after a failed beat (audit LA1 H1): the
 * request may have reached the node with only the response lost, so a resend can
 * turn a cancel (J) into a fresh start, and a press made while the node was down
 * would land long after the operator stopped expecting it. Space IS re-sent. */
const NO_RETRY = new Set(['leader', 'pick', 'home', 'open_gripper']);
let nextEventId = 1;
let localNotice = null;       // {text, until}: page-side message, e.g. "J not delivered"
let hasControl = false;
let holder = null;
let wantControl = true;       // false once we have decided to observe
let inFlight = false;
let beatTimer = null;
let lastFrameAt = 0;
let lastEventId = 0;
let eventsEpoch = null;      // the node's event-id epoch; a new one = node restarted
/* How long a refused/aborted/unavailable leader alignment (J) stays in the banner. A display
 * choice, not a safety value: the text also stays under the keys and in the log. */
const LEADER_NOTICE_SEC = 10;
let renderFailures = 0;       // consecutive render() faults, surfaced as a banner
let pendingKick = false;      // a beat came in while one was already in flight
let claimForce = false;       // the operator pressed "Take control"

const $ = (id) => document.getElementById(id);

/* ── Transport ──────────────────────────────────────────────────────────── */

/* A free seat is taken automatically only by the page the operator is looking at.
 * A stale lease reads as a free seat, so without this any hidden or unfocused tab
 * (or one on another device) would take the seat from a driver who merely stalled
 * for a couple of seconds, and keep it. Such a page waits for focus, or for Take
 * control. The holder is not affected: it renews its lease whatever its focus. */
function mayAutoClaim() {
  return document.visibilityState === 'visible' && document.hasFocus();
}

async function beat(force) {
  if (inFlight) { if (force) pendingKick = true; return; }
  inFlight = true;
  const pending = queue.splice(0, queue.length);
  /* name|id|age_ms: the node drops duplicates (same id) and stale events. Age is the
   * page's own elapsed time, so no clock has to agree with the node's. */
  const events = pending.map(
    (e) => e.name + '|' + e.id + '|' + Math.round(performance.now() - e.t));
  const body = {
    session: SESSION,
    keys: [...held],
    events,
    claim: wantControl && !hasControl && mayAutoClaim(),
  };
  if (claimForce) { body.claim = true; body.force = true; claimForce = false; }
  try {
    /* A POST must not hang (audit LA2 T4): inFlight would block every later beat,
     * Space included, and the stuck request could arrive seconds late. 1 s, TO-VERIFY. */
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), POST_TIMEOUT_MS);
    let res;
    try {
      res = await fetch('/api/input', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
        signal: ctl.signal,
      });
    } finally {
      clearTimeout(timer);
    }
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
    const retry = pending.filter((e) => !NO_RETRY.has(e.name));
    const lost = pending.filter((e) => NO_RETRY.has(e.name)).map((e) => e.name);
    if (retry.length) queue.unshift(...retry);     // retry non-motion one-shots, Space
    if (lost.length) {
      /* The request may or may not have reached the node (aborted, or the response
       * lost): never claim it was not delivered (audit LA3). */
      localNotice = { text: 'Delivery unknown (request failed or timed out): ' + lost.join(', ')
                            + ' - check the J / arm state before pressing again',
                      until: performance.now() + 4000 };
    }
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
  if (refuseWhileQuit()) return;
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
  if (event !== 'estop' && event !== 'quit' && refuseWhileQuit()) return;
  queue.push({ name: event, id: nextEventId++, t: performance.now() });
  kick();
}

/* ── Keyboard ───────────────────────────────────────────────────────────── */

window.addEventListener('keydown', (ev) => {
  if (ev.ctrlKey || ev.metaKey || ev.altKey) return;

  /* While the quit dialog is open nothing may drive or fire: only Space (stop),
   * Escape (cancel), Tab (kept inside the dialog); Enter goes to the focused
   * dialog button untouched. */
  if (quitOpen() || clearOpen()) {
    const dialog = quitOpen() ? quitDialog : clearDialog;
    if (ev.key === ' ' || ev.key === 'Spacebar') {
      ev.preventDefault();
      if (!ev.repeat) estop();
    } else if (ev.key === 'Escape') {
      ev.preventDefault();
      if (dialog === quitDialog) closeQuit(); else closeClear();
    } else if (ev.key === 'Tab') {
      ev.preventDefault();
      trapQuitFocus(ev.shiftKey, dialog);
    }
    return;
  }

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
/* Closing or reloading the tab gives the seat back at once. Without it a reload
 * mints a new session id while the old one still holds the lease, and the new
 * page waits out the takeover interval. The server only honours a release from
 * the holder itself; a missed one (crash, lost link) is covered by that interval. */
window.addEventListener('pagehide', () => {
  try {
    navigator.sendBeacon('/api/input', new Blob(
      [JSON.stringify({ session: SESSION, release: true })], { type: 'application/json' }));
  } catch (err) { /* best effort */ }
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden) releaseAll(); else kick();
});
window.addEventListener('focus', () => kick());

/* ── Pointer / touch on the drawn keys ──────────────────────────────────── */

function wireKeycaps() {
  for (const cap of document.querySelectorAll('.keycap')) {
    const nodeKey = NODE_HOLD_KEYS.has(cap.dataset.key) ? cap.dataset.key : null;
    const event = cap.dataset.event;

    if (nodeKey) {
      cap.addEventListener('pointerdown', (ev) => {
        ev.preventDefault();
        if (refuseWhileQuit()) return;
        cap.setPointerCapture(ev.pointerId);
        press(nodeKey);
      });
      for (const evt of ['pointerup', 'pointercancel', 'lostpointercapture']) {
        cap.addEventListener(evt, () => release(nodeKey));
      }
    } else if (event === 'quit') {
      cap.addEventListener('click', requestQuit);
    } else if (event) {
      cap.addEventListener('click', () => {
        if (refuseWhileQuit()) return;
        flash(cap.dataset.key); fire(event);
      });
    }
  }

  wireEstopButton($('btn-estop'));
  $('btn-takeover').addEventListener('click', () => {
    if (refuseWhileQuit()) return;
    claimForce = true; wantControl = true; kick();
  });
}

/* An emergency-stop button fires on pointerdown (no wait for the release) AND on
 * keyboard activation. Enter on a focused button arrives as a click with
 * detail 0; a mouse/touch click has detail >= 1 and was already served by its
 * pointerdown, so it must not stop a second time. Space is the global handler's;
 * its key-up would click the button once more, hence the swallow. */
function wireEstopButton(btn) {
  btn.addEventListener('pointerdown', (ev) => { ev.preventDefault(); estop(); });
  btn.addEventListener('click', (ev) => { if (ev.detail === 0) estop(); });
  btn.addEventListener('keyup', (ev) => {
    if (ev.key === ' ' || ev.key === 'Spacebar') ev.preventDefault();
  });
}

function estop() {
  releaseAll();
  fire('estop');
  const btn = $('btn-estop');
  btn.classList.add('on');
  setTimeout(() => btn.classList.remove('on'), 180);
}

/* In-page confirmation, NOT window.confirm: a native dialog blocks this page's
 * event loop, the beats stop, the node declares the link lost after
 * client_timeout_sec and then (correctly) drops the quit that arrives on the
 * first beat afterwards. A plain <dialog> keeps the loop, and so the beats,
 * running. It is opened NON-modally (show(), not showModal()) because a modal
 * dialog makes the rest of the page inert -- including btn-estop; instead the
 * backdrop div blocks the pointer, the global keydown handler gates the keys,
 * and the dialog carries its own emergency stop. */
const quitDialog = $('quit-dialog');
const quitBackdrop = $('quit-backdrop');

function quitOpen() { return quitDialog.hasAttribute('open'); }

/* While the dialog is open every other child of <body> (top bar, the main
 * container with all keycaps, btn-estop and btn-takeover) is `inert`: no focus,
 * no click, no Enter on a keycap reached by Tab. The dialog's own estop and the
 * window-level Space cover btn-estop. The click handlers are gated as well, in
 * depth (refuseWhileQuit). */
function setPageInert(on, keep = quitDialog) {
  for (const el of document.body.children) {
    if (el !== keep && el.tagName !== 'SCRIPT') el.inert = on;
  }
}

/* Tab ring of the dialog: without this Tab walks out into the browser chrome
 * (the page behind is inert), and Enter in the URL bar reloads the page and
 * drops the dialog without quitting. */
function trapQuitFocus(backwards, dialog = quitDialog) {
  const ring = [...dialog.querySelectorAll('button')];
  const at = ring.indexOf(document.activeElement);
  const next = at < 0 ? (backwards ? ring.length - 1 : 0)
                      : (at + (backwards ? ring.length - 1 : 1)) % ring.length;
  ring[next].focus();
}

function closeQuit() {
  quitDialog.close();
  quitBackdrop.hidden = true;
  setPageInert(false);
}

/* True (and the caller must return) while the quit or the clear dialog is open. */
function refuseWhileQuit() { return quitOpen() || clearOpen(); }

function requestQuit() {
  if (quitOpen()) return;
  releaseAll();
  /* Launched from gripperx_desk.sh (GripperX Desktop launcher), quitting here
   * now tears the WHOLE desk session down -- RViz, the local mapping stack,
   * everything that script started -- not just this page; the node signals it
   * through a flag file the launcher's own wait loop watches for, AFTER this
   * same stop/straighten has published (scripts/gripperx_desk.sh, keyword
   * "Teardown from the teleop's own Q"). Run standalone (a bare
   * `ros2 launch ... web_teleop.launch.py`), there is no such launcher to
   * signal and only this node stops -- the page cannot tell which case it is
   * in, so the dialog text names both rather than assuming one. */
  quitBackdrop.hidden = false;
  setPageInert(true);
  quitDialog.show();
  $('btn-quit-confirm').focus();         // Enter confirms (user decision 2026-10-22)
}

function wireQuitDialog() {
  $('btn-quit-cancel').addEventListener('click', closeQuit);
  $('btn-quit-confirm').addEventListener('click', () => {
    if (!quitOpen()) return;             // a repeated Enter must not send a second quit
    closeQuit();
    fire('quit');
  });
  wireEstopButton($('btn-quit-estop'));
  /* Space is the emergency stop (global handler), but on a focused button its
   * key-UP would also "click" it: swallow that. */
  quitDialog.addEventListener('keyup', (ev) => {
    if (ev.key === ' ' || ev.key === 'Spacebar') ev.preventDefault();
  });
}

/* ── Operator clear of the whole-drive fault (SR-18 amendment 2026-11-09) ──
 * Same in-page, non-modal pattern as the quit dialog (and for the same reasons),
 * with ONE difference: CANCEL is focused, so Enter does nothing harmful. A clear
 * RELEASES THE DRIVE: a drive command still held anywhere drives off at once.
 * The reason shown is sent back as expected_reason, so a dialog that went stale
 * (the fault changed meanwhile) is refused by the controller, not obeyed. */
const clearDialog = $('clear-dialog');
const clearBackdrop = $('clear-backdrop');
let clearExpected = 0;
let clearFaultId = 0;
let lastFrame = null;

function clearOpen() { return clearDialog.hasAttribute('open'); }

function closeClear() {
  clearDialog.close();
  clearBackdrop.hidden = true;
  setPageInert(false, clearDialog);
}

function requestClear() {
  const f = lastFrame;
  if (quitOpen() || clearOpen() || !f || !f.drive_fault_active || !f.drive_fault_clearable
      || !f.drive_fault_id) return;
  releaseAll();
  clearExpected = f.drive_fault_reason_code;
  clearFaultId = f.drive_fault_id;      // the controller refuses if the fault changed
  text($('clear-reason'), f.drive_fault_reason || 'reason unknown');
  text($('clear-where'), f.drive_fault_where || 'wheel not identified');
  text($('clear-other'), f.drive_fault_other ? ' (other wheel: ' + f.drive_fault_other + ')' : '');
  text($('clear-detail'), (f.drive_fault_detail || '') + ' (fault #' + f.drive_fault_id + ')');
  clearBackdrop.hidden = false;
  setPageInert(true, clearDialog);
  clearDialog.show();
  $('btn-clear-cancel').focus();         // Cancel is the default (SR-18 amendment item 7)
}

function wireClearDialog() {
  $('btn-drive-fault-clear').addEventListener('click', requestClear);
  $('btn-clear-cancel').addEventListener('click', closeClear);
  $('btn-clear-confirm').addEventListener('click', () => {
    if (!clearOpen()) return;            // a repeated click must not send a second clear
    closeClear();
    fire('clear_drive_fault:' + clearExpected + ':' + clearFaultId);
  });
  wireEstopButton($('btn-clear-estop'));
  clearDialog.addEventListener('keyup', (ev) => {
    if (ev.key === ' ' || ev.key === 'Spacebar') ev.preventDefault();
  });
}

function clearNote(f) {
  const r = f.drive_fault_clear_result;
  if (r && r.pending) return 'Drive fault clear: requested, waiting for the controller...';
  if (r && r.message) return 'Drive fault clear ' + (r.success ? 'ACCEPTED: ' : 'REJECTED: ') + r.message;
  const last = f.drive_fault_last_clear;
  if (last) {
    return 'Last drive fault clear: ' + new Date(last.stamp_sec * 1000).toLocaleTimeString()
         + ', ' + last.reason + ', by ' + last.requester + ' (' + f.drive_fault_clear_count
         + ' since bringup)';
  }
  return '';
}

function flash(key) {
  if (refuseWhileQuit()) return;
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
  renderLog(f.events || [], f.events_epoch);
  renderCamera(f.camera);
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
  document.querySelector('.deck').classList.toggle('leader-on', !!f.leader_enabled);
  const lead = $('key-leader');
  lead.classList.toggle('on', !!f.leader_enabled);
  text($('leader-state'), f.leader_enabled ? 'ON' : (f.leader_aligning ? 'ALIGNING' : 'OFF'));
  lead.title = 'Leader alignment: ' + (f.leader_align_state || '—');
  const align = f.leader_align_state || '';
  const note = $('leader-align');
  text(note, align && align !== 'idle' ? 'Leader (J): ' + align : '');
  note.className = 'leader-align' + (leaderAlignBad(align) ? ' warn' : '');
  const wall = f.leader_wall_state || '';
  const wnote = $('leader-wall');
  text(wnote, leaderWallShown(wall) ? 'Leader wall: ' + wall : '');
  wnote.className = 'leader-align' + (leaderWallBad(wall) ? ' warn' : '');
  /* Whole-drive fault (2026-10-16): latched until the stack restarts, so it stays
   * on the deck for as long as it is true -- not only for a notice period. */
  const dnote = $('drive-fault');
  text(dnote, f.drive_fault_active ? driveFaultMessage(f) : '');
  dnote.className = 'leader-align' + (f.drive_fault_active ? ' warn' : '');
  /* The clear button exists ONLY for a clearable reason (SR-18 amendment item 7). */
  lastFrame = f;
  $('btn-drive-fault-clear').hidden = !(f.drive_fault_active && f.drive_fault_clearable);
  if (clearOpen() && !(f.drive_fault_active && f.drive_fault_clearable
                       && f.drive_fault_id === clearFaultId)) closeClear();
  const cnote = $('drive-fault-clear-note');
  text(cnote, clearNote(f));
  const result = f.drive_fault_clear_result;
  cnote.className = 'leader-align' + (result && result.success === false ? ' warn' : '');
}

/* drive_fault_where is composed by the node: 'wheel <name>', or the feedback link
 * for FEEDBACK_LOST, whose wheel_index is -1 and names no wheel on purpose. */
function driveFaultMessage(f) {
  const until = f.drive_fault_clearable
    ? 'until the stack restarts, or until cleared with the button below after checking the robot. '
    : 'until the stack restarts: stack restart required (Mode R). ';
  return 'DRIVE FAULT (' + (f.drive_fault_reason || 'reason unknown') + ', '
       + (f.drive_fault_where || 'wheel not identified') + '): all four drive motors held '
       + 'at zero ' + until + 'Nav2 goals are cancelled. '
       + (f.drive_fault_detail || '');
}

/* One line at 1366 px, with one short tag after it (the page test measures the worst
 * case). Only the first-episode part may be shortened, never the instruction. */
function yawAlarmMessage(f, compact) {
  const n = (f.yaw_alarm_episodes === null || f.yaw_alarm_episodes === undefined)
            ? '?' : f.yaw_alarm_episodes;
  /* compact (live suffix AND a tag, the longest line): date, seconds and the episode
   * count are dropped (the instruction and the zone stay). */
  const when = (f.yaw_alarm_first_utc || 'time unknown');
  return 'YAW-RATE DIVERGENCE ALARM: first episode '
       + (compact ? when.replace(/^\d{4}-\d{2}-\d{2} /, '').replace(/^(\d\d:\d\d):\d\d/, '$1') : when)
       + (compact ? '' : ', ' + n + 'x') + '. Stop. Restart the stack before moving again. '
       + 'Use hardware switch S1, not the software stop.' + yawLiveSuffix(f);
}

/* Latched ERROR stage (D17): the page also shows whether the divergence still persists.
 * 'unknown' (monitor silent or blind) is said so; empty only when the field is absent. */
function yawLiveSuffix(f) {
  if (f.yaw_alarm_live === 'persists') return ' Now: persists.';
  if (f.yaw_alarm_live === 'cleared') return ' Now: cleared.';
  if (f.yaw_alarm_live === 'unknown') return ' Now: unknown.';
  return '';
}

/* Non-latched WARNING stage (D17): yellow banner while it lasts, then gone. */
function yawWarnMessage(f) {
  return 'YAW-RATE WARNING: encoder and gyro yaw rate disagree. Stop and wait until it '
       + 'clears; if it does not clear or comes back, the alarm will latch.';
}

function yawUnknownLabel(f) {
  return f.yaw_alarm_state === 'unknown'
    ? 'YAW MONITOR NOT MONITORING' + (f.yaw_alarm_live_state ? ' (' + f.yaw_alarm_live_state + ')' : '')
    : 'YAW MONITOR SILENT';
}

function yawUnknownMessage(f) {
  return yawUnknownLabel(f) + ': the encoder-vs-gyro cross-check gives no verdict, so a yaw '
       + 'divergence would go unnoticed. Do not start or continue; do not rely on the pose.';
}

function leaderAlignBad(state) {
  return /^(refused|aborted|unavailable)/.test(state || '');
}

/* Held / outside joints, releases and an unavailable wall are shown; idle, disabled and
 * a plain 'armed' (nothing touching the wall) are not. */
function leaderWallShown(state) {
  return /held:|outside:|^released|^unavailable/.test(state || '');
}

function leaderWallBad(state) {
  return /wall fault|NOT confirmed/.test(state || '');
}

function renderBannerChain(f) {
  let level = null, message = '', tag = '';

  if (f.rivals && f.rivals.length) {
    /* Above the emergency stop on purpose: this is the only state on this page
     * where hitting the stop does not necessarily stop the robot. */
    level = 'alarm'; tag = 'RIVAL TELEOP';
    message = 'ANOTHER TELEOP IS RUNNING (' + f.rivals.join(', ') + '). It '
            + 'publishes the same cmd_vel as this page, so your emergency stop '
            + 'does NOT stop what it commands. Shut one of them down.';
  } else if (localNotice && performance.now() < localNotice.until) {
    level = 'alarm'; tag = 'NOTICE';
    message = localNotice.text;
  } else if (renderFailures >= 3) {
    level = 'warn'; tag = 'PAGE STALE';
    message = 'This page is failing to draw the telemetry it is receiving '
            + '(' + renderFailures + ' frames). What you see may be stale — '
            + 'check the browser console and do not drive on it.';
  } else if (f.estop_latched) {
    /* Order (audit LW1 L6): the latched stop first, then a leader wall fault, then a
     * refused/aborted alignment, then the standing leader-mode / aligning notices - a
     * fault during leader mode must not hide behind "LEADER MODE". */
    level = 'alarm'; tag = 'E-STOP';
    message = 'EMERGENCY STOP LATCHED — release every key; input stays ignored '
            + 'until nothing is held.';
  } else if (f.drive_fault_active) {
    /* Standing and latched: no age limit. Below the latched stop only because
     * that one ends as soon as the keys are released; this one does not. */
    level = 'alarm'; tag = 'DRIVE FAULT';
    message = driveFaultMessage(f);
  } else if (leaderWallBad(f.leader_wall_state)
             && f.leader_wall_age_sec !== null && f.leader_wall_age_sec !== undefined
             && f.leader_wall_age_sec < LEADER_NOTICE_SEC) {
    level = 'alarm'; tag = 'LEADER WALL';
    message = 'LEADER WALL: ' + f.leader_wall_state;
  } else if (leaderAlignBad(f.leader_align_state)
             && f.leader_align_age_sec !== null && f.leader_align_age_sec !== undefined
             && f.leader_align_age_sec < LEADER_NOTICE_SEC) {
    level = 'alarm'; tag = 'LEADER ALIGN';
    message = 'LEADER (J) ' + f.leader_align_state.replace(/^(\w+):\s*/, (m, w) =>
                w.toUpperCase() + ': ');
  } else if (f.leader_aligning) {
    level = 'warn'; tag = 'LEADER ALIGNING';
    message = 'LEADER ALIGNING: the leader arm moves itself to the follower pose, then '
            + 'leader mode is requested. Hands clear. J or the space bar cancels.';
  } else if (f.leader_enabled) {
    level = 'warn'; tag = 'LEADER MODE';
    message = 'LEADER MODE: drive blocked. The arm follows the leader; drive '
            + 'keys send a zero twist only. J or the space bar leaves it.';
  } else if (!hasControl && holder && holder !== SESSION) {
    level = 'info'; tag = 'OBSERVER';
    message = 'Observer mode. You can watch and you can hit the emergency stop, '
            + 'but the drive keys belong to ' + holder + '.';
  } else if (f.psi_jump_ago_sec !== null && f.psi_jump_ago_sec !== undefined
             && f.psi_jump_ago_sec < 2.5) {
    /* A transient warning outranks the standing mode notice below: the mode is
     * still true in three seconds, the dead-band crossing is not. */
    level = 'warn'; tag = 'CRAB JUMP';
    message = 'Crab heading jumped a dead band — no pose exists in that 45° '
            + 'gap, so all four modules are swinging across it. Traction is '
            + 'NOT withheld for this: the teleop guard only covers a change of '
            + 'manoeuvre.';
  } else if (f.mode && f.mode !== 'keyboard') {
    level = 'info'; tag = 'MODE';
    message = 'Mode is ' + f.mode + ': teleop_mux is forwarding Nav2, not this '
            + 'page. W/S, A/D and the manoeuvre keys do nothing until you press '
            + 'K. The space bar still works — it forces keyboard mode and stops.';
  } else if (f.armed_without_feedback) {
    level = 'warn'; tag = 'NO STEER FB';
    message = 'No steering feedback — traction was released on the timeout, '
            + 'not on measured wheel angles. The pose is not confirmed.';
  } else if (f.pose_reachable === false) {
    level = 'warn'; tag = 'POSE LIMIT';
    message = 'That pose is outside the steering limits — the manoeuvre will '
            + 'not arm.';
  } else if (f.silent_sec !== null && f.silent_sec !== undefined
             && !f.link_fresh && holder === SESSION) {
    level = 'warn'; tag = 'INPUT LOST';
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
  return {level: level, message: message, tag: tag};
}

/* Priority: latched yaw alarm > (yaw WARNING, not latched, D17: tag on a motion warning, else
 * its own yellow banner) > every banner that was a safety/motion warning before the
 * yaw monitor existed (it stays, with a short tag) > yaw monitor silent/unknown > info
 * banners > calm summary. The monitor runs only in the localization launch, so a desk or
 * arm session without it must keep its arm-motion warnings. */
function composeBanner(f, base) {
  let {level, message} = base;
  const shown = level === 'alarm' || level === 'warn';
  if (f.yaw_alarm_state === 'alarm') {
    /* Latched until the stack restarts (user decision 2026-11-05). Display only. */
    return {level: 'alarm',
            message: yawAlarmMessage(f, !!(shown && base.tag && yawLiveSuffix(f)))
                     + (shown && base.tag ? ' | ' + base.tag : '')};
  }
  if (f.yaw_alarm_state === 'warn') {
    /* Not latched (D17): a motion warning that was already shown keeps its text and gets
     * a short tag; otherwise this is the yellow banner. It disappears when cleared. */
    if (shown) return {level: level, message: message + ' | YAW WARNING'};
    return {level: 'warn', message: yawWarnMessage(f)};
  }
  if (f.yaw_alarm_state !== 'ok') {
    if (shown) return {level: level, message: message + ' | ' + yawUnknownLabel(f)};
    return {level: 'warn', message: yawUnknownMessage(f)};
  }
  return {level: level, message: message};
}

function renderBanner(f) {
  const el = $('banner');
  const {level, message} = composeBanner(f, renderBannerChain(f));
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

function renderLog(events, epoch) {
  const list = $('log');
  if (epoch !== eventsEpoch) {          // node restarted: its ids start at 1 again
    eventsEpoch = epoch;
    lastEventId = 0;
  }
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
wireQuitDialog();
wireClearDialog();
connect();
schedule();
beat(true);


/* ── Camera tile ────────────────────────────────────────────────────────── */
/* The stream is one HTTP connection held open by the <img>. Off = no src = no
 * connection = the node drops its ROS subscription. Nothing here is in the
 * drive path; a failure only changes the tile. */

let camOn = false;
let camRetry = null;

function camStart() {
  $('cam-img').src = '/api/camera/stream?t=' + Date.now();
}

function camStop() {
  clearTimeout(camRetry);
  $('cam-img').removeAttribute('src');   // aborts the request
}

function camSetOn(on) {
  camOn = on;
  if (on) camStart(); else camStop();
  $('cam-power').blur();
}

async function camProfile(name) {
  $('cam-' + name).blur();
  try {
    const ctl = new AbortController();
    const timer = setTimeout(() => ctl.abort(), 2000);
    const res = await fetch('/api/camera/profile', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ profile: name }),
      signal: ctl.signal,
    });
    clearTimeout(timer);
    const r = await res.json();
    if (!r.ok) text($('cam-hint'), 'mode change failed: ' + (r.detail || r.error));
  } catch (e) {
    text($('cam-hint'), 'mode change failed: no answer');
  }
}

$('cam-power').addEventListener('click', () => camSetOn(!camOn));
$('cam-view').addEventListener('click', () => camProfile('view'));
$('cam-detail').addEventListener('click', () => camProfile('detail'));
$('cam-img').addEventListener('error', () => {
  if (!camOn) return;
  clearTimeout(camRetry);
  camRetry = setTimeout(() => { if (camOn) camStart(); }, 2000);   // dropped: try again
});

function renderCamera(c) {
  const frame = $('cam-frame');
  const img = $('cam-img');
  const msg = $('cam-msg');
  const power = $('cam-power');
  text(power, camOn ? 'On' : 'Off');
  power.classList.toggle('active', camOn);
  power.setAttribute('aria-pressed', camOn ? 'true' : 'false');
  const profile = c ? c.profile : null;
  $('cam-view').classList.toggle('active', profile === 'view');
  $('cam-detail').classList.toggle('active', profile === 'detail');
  $('cam-view').disabled = !camOn;
  $('cam-detail').disabled = !camOn;
  const state = !camOn ? 'off' : (c && c.on ? c.state : 'connecting');
  frame.classList.toggle('stale', camOn && state === 'stale');
  frame.classList.toggle('dead', camOn && (state === 'no camera node' || state === 'connecting'));
  img.hidden = !(camOn && c && c.bytes);
  const texts = {
    'off': 'camera off',
    'connecting': 'connecting ...',
    'no camera node': 'no camera node (nothing publishes the image topic)',
    'waiting for frames': 'waiting for frames ...',
    'stale': 'STALE: no new frame',
    'live': '',
  };
  text(msg, texts[state] || state);
  msg.hidden = (state === 'live');
  if (!camOn) { text($('cam-hint'), 'off'); text($('cam-stats'), ''); return; }
  if (c && c.note) text($('cam-hint'), c.note); else text($('cam-hint'), state);
  if (c && c.on && c.bytes) {
    text($('cam-stats'), c.fps.toFixed(1) + ' fps \u00b7 '
      + (c.age_sec === null ? '-' : c.age_sec.toFixed(1) + ' s old') + ' \u00b7 '
      + Math.round(c.bytes / 1000) + ' kB');
  } else {
    text($('cam-stats'), '');
  }
}
