// Smoke test for the SVG Basestation panel: loads ../dist/extension.js under a
// tiny DOM stub and a fake Foxglove panel context, feeds it a commander status
// snapshot + odometry + velocity commands, clicks Start and Apply (CBF alpha),
// and checks the rendered text. Catches runtime errors the syntax check cannot.
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
let paramValue = 2.5;
const missionState = { active: false, seq: 0, last: null };

const panelContext = {
  initialState: {},
  panelElement: makeNode("div"),
  saveState: (s) => { saved = s; },
  subscribe: (subs) => { subscribed = subs.map((s) => s.topic); },
  watch() {},
  setDefaultPanelTitle() {},
  updatePanelSettingsEditor: (ed) => { settingsNodes = ed.nodes; },
  advertise() {}, publish() {},
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
    if (service.endsWith("/get_parameters")) return { values: [{ type: 3, double_value: paramValue }] };
    if (service.endsWith("/set_parameters")) {
      const v = req.parameters[0].value.double_value;
      if (!(v > 0)) return { results: [{ successful: false, reason: "cbf_alpha must be > 0" }] };
      paramValue = v;
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
    cbf: { alpha: paramValue, safety_radius_m: 0.55, max_speed_mps: 1.2, external_velocity_gain: 1.0, active: ["drone_2"], emergency: false },
    command_seq: missionState.seq, last_command: missionState.last,
    drones: [
      { name: "drone_1", role: "auto", mode: "sim", commanded: true, cbf_exempt: false, state: "ACTIVE",
        position_offset: [-2, 0, 0], position: [-1.234, 0.5, 1.2], speed_mps: 0.1, hold_target: [-1.2, 0.5, 1.2], odom_fresh: true, odom_age_s: 0.02,
        cbf_active: false, robot_command: { label: "arm", result: "ok", message: "", stamp: t - 20 } },
      { name: "drone_2", role: "auto", mode: "sim", commanded: true, cbf_exempt: false, state: "ACTIVE",
        position_offset: [0, 0, 0], position: [0.0, 0.0, 1.21], speed_mps: 0.6, hold_target: null, odom_fresh: true, odom_age_s: 0.03,
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
  assert(/\d+ Hz/.test(txt), "velocity command stream rate rendered");
  findButton(root, "Use Current").click();
  const goalX = findAll(root, (n) => n.tagName === "input" && n.classList.contains("sb-goal-in"))[0];
  assert(goalX && goalX.value === "-1.23", "Use Current copies the commander's own position (goal frame) into x");
  assert(text().includes("same frame as the Agent State positions"), "goal note confirms the frame matches the commander");
  const alphaBox = findAll(root, (n) => n.tagName === "input" && n.type === "number" && n.placeholder === "alpha")[0];
  assert(alphaBox && alphaBox.value === "2.50", "alpha draft seeded from the live value");

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
