// Smoke test for the SVG Basestation panel: loads ../dist/extension.js under a
// tiny DOM stub and a fake Foxglove panel context, feeds it a commander status
// snapshot + odometry + velocity commands, clicks Start, Apply (CBF alpha and
// safety radius) and Send (formation), and checks the rendered text. Catches
// runtime errors the syntax check cannot.
//
// Run (no local Node needed):
//   docker run --rm -v "$PWD/robot/ros_ws/src/svg_ground_control/foxglove/svg-basestation:/p" -w /p node:20-alpine \
//     sh -c "node --check dist/extension.js && node test/smoke.js"
"use strict";
const fs = require("fs");
const path = require("path");

// ─── DOM stub ───────────────────────────────────────────────────────────────
function makeNode(tag) {
  const node = {
    tagName: tag, children: [], listeners: {}, style: {}, hidden: false,
    title: "", value: "", disabled: false, parentNode: null, _text: "",
    className: "",
    classList: {
      add(...c) { for (const x of c) if (!node.className.split(/\s+/).includes(x)) node.className = (node.className + " " + x).trim(); },
      remove(...c) { node.className = node.className.split(/\s+/).filter((x) => x && !c.includes(x)).join(" "); },
      toggle(c, on) { if (on === undefined) on = !node.classList.contains(c); on ? node.classList.add(c) : node.classList.remove(c); },
      contains(c) { return node.className.split(/\s+/).includes(c); },
    },
    appendChild(ch) { if (ch.parentNode) ch.parentNode.removeChild(ch); ch.parentNode = node; node.children.push(ch); return ch; },
    append(...chs) { for (const ch of chs) node.appendChild(typeof ch === "string" ? makeText(ch) : ch); },
    removeChild(ch) { const i = node.children.indexOf(ch); if (i >= 0) node.children.splice(i, 1); ch.parentNode = null; return ch; },
    addEventListener(t, fn) { (node.listeners[t] ??= []).push(fn); },
    removeEventListener() {},
    click() { for (const fn of node.listeners.click ?? []) fn({}); },
    fire(t, ev) { for (const fn of node.listeners[t] ?? []) fn(ev ?? {}); },
    get textContent() { return node._text + node.children.map((c) => c.textContent).join(""); },
    set textContent(v) { node.children = []; node._text = String(v); },
    set innerHTML(v) { node.children = []; node._text = String(v); },
    get innerHTML() { return node.textContent; },
  };
  return node;
}
function makeText(s) { const n = makeNode("#text"); n._text = String(s); return n; }
function* walk(n) { yield n; for (const c of n.children) yield* walk(c); }
function findAll(root, pred) { return [...walk(root)].filter(pred); }
function findButton(root, label) {
  return findAll(root, (n) => n.tagName === "button" && n.textContent.trim() === label)[0];
}

global.document = {
  createElement: makeNode, createTextNode: makeText, activeElement: null, hidden: false,
  addEventListener() {}, removeEventListener() {},
};
global.module = { exports: {} };
const timers = [];
global.setInterval = (fn) => { timers.push(fn); return timers.length; };
global.clearInterval = () => {};

// ─── load the extension ───────────────────────────────────────────────────────
const src = fs.readFileSync(path.join(__dirname, "..", "dist", "extension.js"), "utf8");
new Function("module", "require", src)(global.module, require);
const { activate } = global.module.exports;
if (typeof activate !== "function") throw new Error("activate not exported");

let initPanel = null;
activate({ registerPanel: (def) => { initPanel = def.initPanel; } });

// ─── fake Foxglove panel context ──────────────────────────────────────────────
const calls = [];
let saved = null;
let subscribed = [];
let onRender = null;
let settingsNodes = null;
// The commander's runtime parameters, as get/set_parameters see them.
const params = { cbf_alpha: 2.5, cbf_safety_radius_m: 0.55, cbf_max_speed_mps: 1.2,
  teleop_max_speed_mps: 2.0, goal_accel_mps2: 3.0, goal_settle_s: 0.3 };
// safe_teleop's parameters — max_speed_mps must always equal the commander's
// teleop_max_speed_mps, so the panel sets both.
const padParams = { max_speed_mps: 2.0 };
const published = [];
const missionState = { active: false, seq: 0, last: null };

const panelContext = {
  initialState: {},
  panelElement: makeNode("div"),
  saveState: (s) => { saved = s; },
  subscribe: (subs) => { subscribed = subs.map((s) => s.topic); },
  watch() {},
  setDefaultPanelTitle() {},
  updatePanelSettingsEditor: (ed) => { settingsNodes = ed.nodes; },
  advertise() {}, publish(topic, message) { published.push({ topic, message }); },
  set onRender(fn) { onRender = fn; },
  get onRender() { return onRender; },
  callService: async (service, req) => {
    calls.push({ service, req });
    if (service.endsWith("/start")) {
      missionState.active = true; missionState.seq += 1;
      missionState.last = { seq: missionState.seq, name: "start", success: true, message: 'scenario "antipodal" running', stamp: 1.7e9 + 5 };
      return { success: true, message: 'scenario "antipodal" running' };
    }
    if (service.endsWith("/hold")) {
      missionState.active = false; missionState.seq += 1;
      missionState.last = { seq: missionState.seq, name: "hold", success: true, message: "holding: drone_1", stamp: 1.7e9 + 9 };
      return { success: true, message: "holding: drone_1" };
    }
    const store = service.startsWith("/safe_teleop/") ? padParams : params;
    if (service.endsWith("/get_parameters")) {
      return { values: req.names.map((n) => (n in store ? { type: 3, double_value: store[n] } : { type: 0 })) };
    }
    if (service.endsWith("/set_parameters")) {
      const { name, value } = req.parameters[0];
      const v = value.double_value;
      if (!(name in store)) return { results: [{ successful: false, reason: `unknown parameter ${name}` }] };
      if (name === "goal_settle_s" ? !(v >= 0) : !(v > 0)) return { results: [{ successful: false, reason: `${name} must be > 0` }] };
      if (name === "cbf_max_speed_mps" && v > 5) return { results: [{ successful: false, reason: "too fast for indoors" }] };
      store[name] = v;
      return { results: [{ successful: true, reason: "" }] };
    }
    throw new Error("unknown service " + service);
  },
};

const dispose = initPanel(panelContext);
const root = panelContext.panelElement;
const render = timers[0];
if (!render) throw new Error("no render interval registered");

// ─── helpers to feed data ─────────────────────────────────────────────────────
let t = 1.7e9;
const rxTime = (s) => ({ sec: Math.floor(s), nsec: Math.round((s % 1) * 1e9) });
function statusMsg(overrides = {}) {
  return {
    stamp: t, node: "/swarm_commander", scenario: "antipodal",
    mission_active: missionState.active, mission_ever_started: missionState.active,
    mission_started_at: missionState.active ? 1.7e9 + 5 : null,
    fence_enabled: true, fence_breached: false,
    cbf: { alpha: params.cbf_alpha, safety_radius_m: params.cbf_safety_radius_m, max_speed_mps: params.cbf_max_speed_mps,
      external_velocity_gain: 1.0, active: ["drone_2"], emergency: false },
    tuning: { teleop_max_speed_mps: params.teleop_max_speed_mps, goal_accel_mps2: params.goal_accel_mps2,
      goal_settle_s: params.goal_settle_s, scenario_speed_mps: 1.2 },
    command_seq: missionState.seq, last_command: missionState.last,
    drones: [
      { name: "drone_1", role: "auto", mode: "sim", commanded: true, cbf_exempt: false, state: "ACTIVE",
        position_offset: [-2, 0, 0], position: [-1.234, 0.5, 1.2],
        odom_rx_total: Math.round((t - 1.7e9) * 30), odom_lost_total: Math.round((t - 1.7e9) * 30 / 20), odom_loss_counter: "dds", speed_mps: 0.1, hold_target: [-1.2, 0.5, 1.2], odom_fresh: true, odom_age_s: 0.02,
        cbf_active: false, robot_command: { label: "arm", result: "ok", message: "", stamp: t - 20 } },
      { name: "drone_2", role: "auto", mode: "sim", commanded: true, cbf_exempt: false, state: "ACTIVE",
        position_offset: [0, 0, 0], position: [0.0, 0.0, 1.21],
        odom_rx_total: Math.round((t - 1.7e9) * 30), odom_lost_total: 0, odom_loss_counter: "dds", speed_mps: 0.6, hold_target: null, odom_fresh: true, odom_age_s: 0.03,
        cbf_active: true, robot_command: { label: "arm", result: "rejected", message: "interface returned success=False", stamp: t - 20 } },
      { name: "drone_3", role: "auto", mode: "sim", commanded: true, cbf_exempt: false, state: "IDLE",
        position: null, speed_mps: null, hold_target: null, odom_fresh: false, odom_age_s: null,
        cbf_active: false, robot_command: null },
    ],
    ...overrides,
  };
}
function frame(events) {
  onRender({ topics: subscribed.map((name) => ({ name })), currentFrame: events }, () => {});
}
function feedStatus(overrides) {
  frame([{ topic: "/svg/commander_status", receiveTime: rxTime(t), message: { data: JSON.stringify(statusMsg(overrides)) } }]);
}
function feedOdom(name, x, y, z) {
  frame([{ topic: `/${name}/odometry_conversion/odometry`, receiveTime: rxTime(t),
    message: { header: { stamp: rxTime(t - 0.01) }, pose: { pose: { position: { x, y, z } } }, twist: { twist: { linear: { x: 0.1, y: 0, z: 0 } } } } }]);
}
function feedCmd(name) {
  frame([{ topic: `/${name}/interface/velocity_command`, receiveTime: rxTime(t),
    message: { header: { stamp: rxTime(t) }, twist: { linear: { x: 0, y: 0, z: 0 } } } }]);
}
const assert = (cond, msg) => { if (!cond) { console.error("FAIL:", msg); process.exitCode = 1; } else console.log("ok  ", msg); };
const text = () => root.textContent;

(async () => {
  // Subscriptions include the new topics.
  assert(subscribed.includes("/svg/commander_status"), "subscribes to the commander status topic");
  assert(subscribed.includes("/drone_1/interface/velocity_command"), "subscribes to sim velocity commands");
  assert(subscribed.includes("/drone_1/fmu/velocity_command"), "subscribes to real velocity commands");
  assert(settingsNodes.swarm.fields.statusTopic && settingsNodes.swarm.fields.cbfAlphaMax, "settings editor exposes statusTopic + cbfAlphaMax");
  assert(settingsNodes.swarm.fields.cbfRadiusMax && settingsNodes.swarm.fields.cbfSpeedMax, "settings editor exposes the radius + max-speed slider maxima");
  assert(settingsNodes.swarm.fields.teleopSpeedMax && settingsNodes.swarm.fields.goalAccelMax
    && settingsNodes.swarm.fields.goalSettleMax, "settings editor exposes the teleop / goal slider maxima");
  assert(settingsNodes.swarm.fields.teleopNs && settingsNodes.swarm.fields.teleopNs.value === "/safe_teleop",
    "settings editor exposes the safe_teleop namespace");

  // Before any data.
  render();
  assert(text().includes("NO COMMANDER"), "mission chip reads NO COMMANDER before any snapshot");
  assert(text().includes("No commands sent from this panel yet."), "empty command log");

  // Odometry only: position falls back to odom (+ offsets = 0).
  feedOdom("drone_1", 1.5, -0.25, 1.1);
  render();
  assert(text().includes("1.50") && text().includes("-0.25") && text().includes("odom"), "position from odometry with 'odom' source tag");

  // Commander snapshot: READY, positions from commander, per-drone details.
  for (let i = 0; i < 3; i++) { t += 0.2; feedStatus(); feedCmd("drone_1"); feedCmd("drone_2"); }
  render();
  const txt = text();
  assert(txt.includes("NOT READY") && txt.includes("on ground: drone_3"), "mission chip NOT READY while one commanded drone is still IDLE");
  {
    const allActive = statusMsg();
    allActive.drones[2].state = "ACTIVE";
    t += 0.2;
    frame([{ topic: "/svg/commander_status", receiveTime: rxTime(t), message: { data: JSON.stringify(allActive) } }]);
    render();
    assert(text().includes("READY TO START"), "mission chip READY when all commanded drones hold and never started");
  }
  assert(txt.includes("-1.23") && txt.includes("cmdr"), "position from the commander snapshot with 'cmdr' source tag");
  assert(txt.includes("ACTIVE") && txt.includes("IDLE"), "flight state chips rendered");
  assert(txt.includes("correcting drone_2"), "CBF live readout names the corrected drone");
  assert(txt.includes("live 2.50"), "CBF alpha live value shown from snapshot");
  assert(txt.includes("arm ✓") && txt.includes("arm ✗"), "interface (arm) results rendered per drone");
  {
    // Measured drop: several snapshots spanning > 5 samples -> dds-tagged cells.
    for (let i = 0; i < 5; i++) { t += 0.2; feedStatus(); feedCmd("drone_1"); }
    render();
    const dropCells = findAll(root, (n) => n.tagName === "td" && n.title.startsWith("Measured:"));
    assert(dropCells.length >= 2, "Drop column uses the commander's DDS counters (tagged dds)");
    const d1 = dropCells[0].textContent, d2 = dropCells[1].textContent;
    assert(/4\.\d\d %dds|5\.\d\d %dds/.test(d1), `drone_1 measured drop ~4.8 % from counters (got "${d1}")`);
    assert(d2.startsWith("0.00 %"), `drone_2 measured drop 0 % (got "${d2}")`);
    // Before the counters were flowing, the same odometry stream fed the
    // arrival estimate; an estimate must never grade the link as DEGRADED.
    const estCells = findAll(root, (n) => n.tagName === "td" && n.title.startsWith("ESTIMATE"));
    assert(estCells.every((n) => n.className.includes("sb-muted")), "estimated drop cells are muted, not graded");
  }
  assert(/\d+ Hz/.test(txt), "velocity command stream rate rendered");
  findButton(root, "Use Current").click();
  const goalX = findAll(root, (n) => n.tagName === "input" && n.classList.contains("sb-goal-in"))[0];
  assert(goalX && goalX.value === "-1.23", "Use Current copies the commander's own position (goal frame) into x");
  assert(text().includes("same frame as the Agent State positions"), "goal note confirms the frame matches the commander");
  const numBox = (ph) => findAll(root, (n) => n.tagName === "input" && n.type === "number" && n.placeholder === ph)[0];
  const alphaBox = numBox("alpha");
  assert(alphaBox && alphaBox.value === "2.50", "alpha draft seeded from the live value");
  // One slider row; the dropdown picks the gain.
  const cbfSel = findAll(root, (n) => n.tagName === "select" && n.children.some((o) => o.value === "radius"))[0];
  assert(cbfSel, "CBF gain dropdown lists alpha / radius / speed");
  for (const id of ["teleop", "accel", "settle"]) {
    assert(cbfSel.children.some((o) => o.value === id), `gain dropdown lists ${id}`);
  }
  assert(findAll(root, (n) => n.tagName === "button" && n.textContent.trim() === "Apply").length === 1, "a single Apply button for the CBF row");
  const pickCbf = (id) => { cbfSel.value = id; cbfSel.fire("change"); render(); };
  pickCbf("radius");
  assert(numBox("radius") && numBox("radius").value === "0.55", "selecting radius seeds its draft from the live value");
  assert(text().includes("live 0.55 m"), "radius live readout carries its unit");
  pickCbf("speed");
  assert(numBox("vmax") && numBox("vmax").value === "1.20", "selecting max speed seeds its draft from the live value");
  assert(text().includes("live 1.20 m/s"), "max speed live readout carries its unit");
  pickCbf("alpha");
  assert(numBox("alpha") && numBox("alpha").value === "2.50", "back to alpha, draft restored");

  // Click Start -> confirm dialog -> Confirm.
  findButton(root, "Start").click();
  const confirmBtn = findButton(root, "Confirm");
  assert(confirmBtn, "confirm dialog shown for Start");
  confirmBtn.click();
  await new Promise((r) => setTimeout(r, 10));
  assert(calls.some((c) => c.service === "/swarm_commander/start"), "Start called /swarm_commander/start");
  render();
  assert(text().includes("reply: accepted") && text().includes("confirming"), "log shows accepted reply, awaiting confirmation");
  // Snapshot arrives showing the mission running.
  t += 0.3; feedStatus();
  render();
  const after = text();
  assert(after.includes("RUNNING"), "mission chip RUNNING after the snapshot confirms it");
  assert(after.includes("confirmed by commander"), "start entry confirmed by the commander snapshot");
  assert(after.includes("✓ start"), "last-command chip shows the commander's own record of start");

  // Apply a new alpha.
  alphaBox.value = "4";
  alphaBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 10));
  const setCall = calls.find((c) => c.service === "/swarm_commander/set_parameters");
  assert(setCall && setCall.req.parameters[0].name === "cbf_alpha" && setCall.req.parameters[0].value.double_value === 4
    && setCall.req.parameters[0].value.type === 3, "Apply sends rcl_interfaces SetParameters with a double cbf_alpha");
  t += 0.3; feedStatus();
  render();
  assert(text().includes("live 4.00 ✓"), "live alpha updates to the applied value and is ticked");

  // Rejected set.
  alphaBox.value = "-1"; alphaBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 10));
  render();
  assert(text().includes("must be a positive number"), "negative alpha rejected client-side");

  // Apply a new safety radius via the dropdown; alpha must be untouched.
  pickCbf("radius");
  const radiusBox = numBox("radius");
  radiusBox.value = "0.8";
  radiusBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 10));
  const radiusCall = calls.filter((c) => c.service === "/swarm_commander/set_parameters").pop();
  assert(radiusCall && radiusCall.req.parameters[0].name === "cbf_safety_radius_m"
    && radiusCall.req.parameters[0].value.double_value === 0.8, "radius Apply sets cbf_safety_radius_m only");
  assert(params.cbf_alpha === 4, "alpha unchanged by the radius Apply");
  t += 0.3; feedStatus();
  render();
  assert(text().includes("live 0.80 m ✓"), "live radius updates to the applied value and is ticked");
  pickCbf("alpha");
  assert(text().includes("live 4.00") && numBox("alpha").value === "4.00", "switching back to alpha shows its own live value");

  // A commander-side rejection shows ✗ in the readout and the reason in the status line.
  pickCbf("speed");
  const speedBox = numBox("vmax");
  speedBox.value = "9"; speedBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 10));
  render();
  assert(text().includes("live 1.20 m/s ✗"), "rejected set marks the readout with ✗ but keeps the live value");
  assert(text().includes("REJECTED") && text().includes("too fast for indoors"), "rejection reason shown in the status line");
  assert(params.cbf_max_speed_mps === 1.2, "rejected value not applied");

  // Teleop max speed: one Apply sets the commander AND safe_teleop.
  pickCbf("teleop");
  const teleopBox = numBox("teleop");
  assert(teleopBox && teleopBox.value === "2.00", "teleop cap draft seeded from the snapshot's tuning block");
  assert(text().includes("live 2.00 m/s"), "teleop cap live readout carries its unit");
  assert(text().includes("pad 2.00 ✓"), "safe_teleop's max_speed_mps read back and ticked as equal");
  teleopBox.value = "3"; teleopBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 20));
  const teleopCalls = calls.filter((c) => c.service.endsWith("/set_parameters")).slice(-2);
  assert(teleopCalls[0].service === "/swarm_commander/set_parameters"
    && teleopCalls[0].req.parameters[0].name === "teleop_max_speed_mps"
    && teleopCalls[0].req.parameters[0].value.double_value === 3, "teleop Apply sets the commander's teleop_max_speed_mps first");
  assert(teleopCalls[1].service === "/safe_teleop/set_parameters"
    && teleopCalls[1].req.parameters[0].name === "max_speed_mps"
    && teleopCalls[1].req.parameters[0].value.double_value === 3, "then safe_teleop's max_speed_mps with the same number");
  assert(params.teleop_max_speed_mps === 3 && padParams.max_speed_mps === 3, "both nodes now hold 3.0");
  t += 0.3; feedStatus();
  render();
  assert(text().includes("live 3.00 m/s ✓") && text().includes("pad 3.00 ✓"), "both copies confirmed at 3.00");
  // Someone changes the pad behind the panel's back: the readout flags it.
  padParams.max_speed_mps = 1.0;
  findButton(root, "↻").click();
  await new Promise((r) => setTimeout(r, 20));
  t += 11; feedStatus();      // past the 10 s "asked" window
  render();
  assert(text().includes("pad 1.00 ✗"), "a pad value that differs from the commander's is crossed");
  assert(text().includes("lower one wins on the sticks"), "the mismatch note says why it matters");
  padParams.max_speed_mps = 3.0;
  findButton(root, "↻").click();
  await new Promise((r) => setTimeout(r, 20));
  render();
  assert(text().includes("pad 3.00 ✓"), "back in agreement after a refresh");
  // A rejection by the commander never touches the pad.
  teleopBox.value = "-2"; teleopBox.fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 20));
  assert(padParams.max_speed_mps === 3 && params.teleop_max_speed_mps === 3, "client-side rejection leaves both untouched");

  // Goal accel / settle live in the snapshot's tuning block; settle may be 0.
  pickCbf("accel");
  assert(numBox("accel") && numBox("accel").value === "3.00" && text().includes("live 3.00 m/s²"), "goal accel seeded and shown with its unit");
  numBox("accel").value = "6"; numBox("accel").fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 20));
  assert(params.goal_accel_mps2 === 6, "goal accel Apply sets goal_accel_mps2");
  pickCbf("settle");
  assert(numBox("settle") && numBox("settle").value === "0.30" && text().includes("live 0.30 s"), "goal settle seeded and shown with its unit");
  numBox("settle").value = "0"; numBox("settle").fire("input");
  findButton(root, "Apply").click();
  await new Promise((r) => setTimeout(r, 20));
  assert(params.goal_settle_s === 0, "a zero settle time is allowed and applied");
  t += 0.3; feedStatus();
  render();
  assert(text().includes("live 0.00 s ✓"), "zero settle confirmed by the snapshot");
  assert(!calls.some((c) => c.service === "/safe_teleop/set_parameters" && c.req.parameters[0].name !== "max_speed_mps"),
    "only the teleop cap is mirrored to safe_teleop");
  pickCbf("alpha");

  // Formation: dropdown + Send only — no free-text box, no Next.
  assert(!findButton(root, "Next"), "no Next button");
  assert(!findAll(root, (n) => n.tagName === "input" && n.type === "text" && /next/i.test(n.placeholder ?? "")).length,
    "no free-text formation box");
  const formSel = findAll(root, (n) => n.tagName === "select" && n.children.some((o) => o.value === "line"))[0];
  assert(formSel, "formation dropdown lists the configured profiles");
  formSel.value = "line";
  formSel.fire("change");
  findButton(root, "Send").click();
  const pub = published.find((p) => p.topic === "/svg/formation_command");
  assert(pub && pub.message.data === "line", "Send publishes the selected profile on /svg/formation_command");
  assert(saved && saved.formation === "line", "selected profile persisted");
  formSel.value = "";
  formSel.fire("change");
  findButton(root, "Send").click();
  assert(text().includes("Pick a formation profile first"), "Send with nothing selected is refused");

  // Stale commander -> NO COMMANDER, positions fall back.
  t += 5; feedOdom("drone_1", 9.87, 0, 1);
  render();
  assert(text().includes("NO COMMANDER") && text().includes("stale"), "stale snapshot flagged");
  assert(text().includes("7.87") && !text().includes("9.87"),
    "odometry fallback uses the offsets adopted from the commander (9.87 - 2), so it stays in the goal frame");

  // Hold with the reply lost (timeout) but the snapshot showing the effect.
  const realCall = panelContext.callService;
  panelContext.callService = (service, req) => { realCall(service, req); return new Promise(() => {}); };
  // Need a fresh snapshot first so seqBefore is known.
  t += 0.1; feedStatus({ mission_active: true, mission_ever_started: true, command_seq: missionState.seq, last_command: missionState.last });
  findButton(root, "Hold All").click();
  await new Promise((r) => setTimeout(r, 10));   // let the (never-replying) call go out
  render();
  assert(text().includes("hold") && text().includes("awaiting reply"), "hold logged as sent");
  // Commander shows it executed (snapshot), reply never comes.
  {
    const held = statusMsg({ mission_active: false, mission_ever_started: true });
    held.drones[2].state = "ACTIVE";
    t += 0.5;
    frame([{ topic: "/svg/commander_status", receiveTime: rxTime(t), message: { data: JSON.stringify(held) } }]);
  }
  render();
  assert(text().includes("HOLDING"), "mission chip HOLDING after hold");
  assert(/hold\s+sent, awaiting reply\s+✓ confirmed by commander/.test(text()), "hold confirmed from the snapshot even without a service reply");

  // Power-only instance (the panel under the 3D view in the shipped layout).
  const powerCtx = { ...panelContext, initialState: { view: "power" }, panelElement: makeNode("div"), callService: realCall };
  let powerOnRender = null;
  Object.defineProperty(powerCtx, "onRender", { set(fn) { powerOnRender = fn; }, get() { return powerOnRender; } });
  const disposePower = initPanel(powerCtx);
  const proot = powerCtx.panelElement;
  timers[timers.length - 1]();
  const hiddenClasses = findAll(proot, (n) => n.hidden).map((n) => n.className);
  assert(hiddenClasses.some((c) => c.includes("sb-safety")) && hiddenClasses.some((c) => c.includes("sb-columns")),
    "power view hides the safety bar and the main columns");
  const pcard = findAll(proot, (n) => n.className === "sb-card" && n.textContent.startsWith("Battery & Power Management"))[0];
  assert(pcard && pcard.parentNode === proot && !pcard.hidden, "power view shows Battery & Power directly under the banner");
  assert(!findAll(proot, (n) => n.className.includes("sb-mission") && !n.hidden).length || findAll(proot, (n) => n.className === "sb-card" && !n.hidden && n.textContent.includes("Swarm Command")).length === 0,
    "power view has no Swarm Command card visible");
  disposePower();

  dispose();
  console.log(process.exitCode ? "SMOKE FAILED" : "SMOKE PASSED");
})().catch((e) => { console.error(e); process.exitCode = 1; });
