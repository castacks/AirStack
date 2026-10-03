(() => {
"use strict";

// ─────────────────────────── SVG Basestation ──────────────────────────────────
//
// Ground-station panel for the SVG counter-UAS demonstration. It is the SVG
// analogue of the DTC "Robot Control Panel" that anchors the foxglove_ws
// basestation layout: one panel that owns agent selection, the swarm-wide
// safety command, and the operator's health picture.
//
// Three things drive the whole panel:
//
//   1. MODE (sim | real) is the agent's *wiring*, mirroring swarm_commander's
//      drone_modes parameter. "sim" routes through the MAVROS interface
//      (/{name}/interface/...); "real" routes through px4_interface over
//      uXRCE-DDS (/{name}/fmu/...) and additionally carries mocap + EKF
//      telemetry. Mode is auto-detected from the topics actually on the wire
//      and can be pinned per agent in the settings.
//
//   2. TOPIC DISCOVERY drives visibility. Every section declares which topics
//      it needs; a section that has no source is not rendered at all, so a
//      sim-only run does not show empty mocap/EKF columns and a run with no
//      cellular reporter does not show an empty cellular table. The detected
//      task set (goal / formation / teleop / mocap) is shown in the banner so
//      the operator can see what the panel decided.
//
//   3. Anything with no source reads "--" rather than a fabricated number.
//
// Transport model: there is no Wi-Fi mesh. Each agent is reachable over the
// local Wi-Fi / LAN (the DDS path used in the lab) and, in the field, over
// 4G/5G with a Tailscale VPN carrying the same traffic. Both paths are drawn
// in the topology; only the ones actually provisioned are shown live.
//
// Wiring matches robot/ros_ws/src/svg_ground_control (swarm_commander.py,
// mocap_bridge.py):
//   state     /{name}/odometry_conversion/odometry       nav_msgs/Odometry
//   battery   /{name}/fmu/out/battery_status             px4_msgs/BatteryStatus   (real)
//             /{name}/interface/mavros/battery           sensor_msgs/BatteryState (sim)
//   mocap     /{name}/pose                               geometry_msgs/PoseStamped (real)
//   ekf       /{name}/fmu/out/estimator_status_flags     px4_msgs/EstimatorStatusFlags
//             /{name}/fmu/out/vehicle_local_position     px4_msgs/VehicleLocalPosition
//   ping      /{name}/fmu/out/timesync_status            px4_msgs/TimesyncStatus
//   cellular  /{name}/comms/cellular                     std_msgs/String, JSON
//   lifecycle /swarm_commander/{takeoff,start,hold,land,reset_fence}  std_srvs/Trigger
//   formation /svg/formation_command                     std_msgs/String

// ─────────────────────────── constants ────────────────────────────────────────

// Normal-operations commands. Stopping the swarm lives in the safety bar.
const LIFECYCLE = [
  { id: "takeoff",     label: "Takeoff",     color: "#2563eb", confirm: true,
    hint: "Arm + offboard, ascend everyone to the scenario's initial positions, then HOLD" },
  { id: "start",       label: "Start",       color: "#10b981", confirm: true,
    hint: "Begin the scenario — nominal policies go live" },
  { id: "reset_fence", label: "Reset Fence", color: "#6b7280", confirm: false,
    hint: "Clear a latched geofence breach" },
];

// Link health states, worst last — the swarm banner reports the worst one.
const LINK_STATE = {
  HEALTHY:  { rank: 0, label: "HEALTHY",  color: "#10b981" },
  FAILOVER: { rank: 1, label: "ON VPN",   color: "#3b82f6" },
  DEGRADED: { rank: 2, label: "DEGRADED", color: "#f59e0b" },
  LOST:     { rank: 3, label: "LOST",     color: "#dc2626" },
  NO_DATA:  { rank: 4, label: "NO DATA",  color: "#6b7280" },
};

// EKF / state-estimate verdicts, worst last.
const EKF_STATE = {
  EV_FUSED: { rank: 0, label: "EV FUSED", color: "#10b981" },
  VALID:    { rank: 1, label: "VALID",    color: "#10b981" },
  NO_EV:    { rank: 2, label: "NO VISION", color: "#f59e0b" },
  DEAD_REC: { rank: 3, label: "DEAD RECK", color: "#dc2626" },
  NO_POS:   { rank: 4, label: "NO POSITION", color: "#dc2626" },
  NO_DATA:  { rank: 5, label: "NO DATA",  color: "#6b7280" },
};

// RTB verdicts, worst last.
const RTB_STATE = {
  NOMINAL:  { rank: 0, label: "NOMINAL",       color: "#10b981" },
  RTB_NOW:  { rank: 1, label: "RTB NOW",       color: "#3b82f6" },
  GATED:    { rank: 2, label: "MANEUVER GATED", color: "#f59e0b" },
  FAILSAFE: { rank: 3, label: "FAILSAFE LAND", color: "#dc2626" },
  NO_DATA:  { rank: 4, label: "NO DATA",       color: "#6b7280" },
};

// Transport tiers. No mesh: the lab path is plain Wi-Fi/LAN, the field path is
// 4G/5G carrying the same DDS traffic inside a Tailscale VPN.
const TIERS = [
  { id: "lan", label: "Local Wi-Fi / LAN", sub: "primary DDS transport" },
  { id: "vpn", label: "4G / 5G · Tailscale", sub: "VPN transport" },
];

const MODES = {
  sim:  { id: "sim",  label: "SIM",  color: "#2563eb" },
  real: { id: "real", label: "REAL", color: "#b45309" },
};

const METRIC_WINDOW_S = 30;     // sliding window for drop rate / derived RTT.
                                // At 25 Hz a 10 s window quantises the drop rate
                                // to 0.4 % per sample, so a 1 % target had two
                                // representable steps of headroom; 30 s gives 0.13 %.
const CLOCK_WINDOW_S = 120;     // sliding window for clock-offset drift slope
const CLOCK_MIN_BUCKETS = 30;   // never fit a drift slope to less than this many
                                // per-second minima — a short window fits queueing
                                // noise, not clock separation
const LINK_LOSS_TIMEOUT_S = 0.5;  // = swarm_commander's state_timeout_s. That is the
                                  // moment the commander stops commanding the drone
                                  // and holds zero velocity, so the operator's LOST
                                  // indicator must not lag it.
const MAX_TRANSITIONS = 60;
const SOC_SLOPE_WINDOW_S = 60;  // sliding window for the burn-rate estimate
const STALE_REPORT_S = 10;      // JSON reports older than this stop overriding
const UI_REFRESH_MS = 200;
const SAFETY_ARM_S = 4;         // safety button stays armed this long

// ─────────────────────────── defaults ─────────────────────────────────────────

const DEFAULTS = {
  drones: "drone_1,drone_2,drone_3",
  // Wiring per agent, mirroring swarm_commander's drone_modes. Blank = detect
  // each agent from the topics on the wire.
  modes: "",
  commanderNs: "/swarm_commander",
  formationTopic: "/svg/formation_command",

  // shared
  stateTopicTemplate: "/{name}/odometry_conversion/odometry",

  // sim wiring (MAVROS interface)
  simBatteryTopicTemplate: "/{name}/interface/mavros/battery",
  simCommandTopicTemplate: "/{name}/interface/velocity_command",
  simRobotCommandTemplate: "/{name}/interface/robot_command",

  // real wiring (px4_interface / uXRCE-DDS)
  realBatteryTopicTemplate: "/{name}/fmu/out/battery_status",
  realCommandTopicTemplate: "/{name}/fmu/velocity_command",
  realRobotCommandTemplate: "/{name}/fmu/robot_command",
  mocapTopicTemplate: "/{name}/pose",
  ekfFlagsTopicTemplate: "/{name}/fmu/out/estimator_status_flags",
  localPositionTopicTemplate: "/{name}/fmu/out/vehicle_local_position",
  timesyncTopicTemplate: "/{name}/fmu/out/timesync_status",
  // MAVROS runs its own MAVLink TIMESYNC exchange and publishes a genuinely
  // measured round-trip plus a smoothed clock offset. Subscribing to it gives
  // sim agents a real ping instead of the rx-minus-stamp fallback, which on
  // this wiring compares two timestamps that both originate on the GCS.
  // Requires timesync_mode: MAVLINK in px4_config.yaml (SYSTEM_TIME is not a
  // valid MAVROS enum member and silently disables timesync).
  simTimesyncTopicTemplate: "/{name}/interface/mavros/timesync_status",

  // transports
  lanTopicTemplate: "/{name}/odometry_conversion/odometry",
  vpnTopicTemplate: "/{name}/cellular/odometry",
  cellularTopicTemplate: "/{name}/comms/cellular",
  linkStatusTopicTemplate: "/{name}/comms/link_status",

  // task detection only (the panel never publishes these)
  teleopTopicTemplate: "/svg/{name}/teleop_command",
  goalTopicTemplate: "/svg/{name}/goal_command",

  // link-safety targets
  // 20 ms is not arbitrary: with cbf_max_speed_mps 1.2 two agents close at
  // 2.4 m/s, so 20 ms of loop latency costs 4.8 cm of the 0.55 m
  // cbf_safety_radius_m (8.7 %), on top of the 12 cm the 20 Hz control period
  // already costs. Loosening it spends safety margin, so it stays.
  pingTargetMs: 20,
  dropTargetPct: 1.0,
  // The gate that actually maps to danger. A loss percentage cannot tell an
  // evenly-spread 1 % (harmless at 25 Hz) from one 400 ms blackout (100 ms
  // from the commander zeroing velocity). This is the longest single gap in
  // the window; 250 ms is half of state_timeout_s, i.e. warn at half the budget.
  stateGapTargetMs: 250,
  // = EKF2_EV_DELAY (~50 ms), the external-vision latency EKF2 is tuned to
  // compensate. Past it, PX4 is fusing data staler than its own delay model.
  mocapAgeTargetMs: 50,
  mocapTimeoutS: 0.5,
  vpnPingTargetMs: 120,

  // power / RTB
  padPosition: "0,0,0",
  positionOffsets: "",
  // Return-budget speeds mirror swarm_commander. Cruise is scenario_speed_mps
  // (0.6), NOT the cbf_max_speed_mps ceiling of 1.2: over-estimating cruise
  // shortens the predicted return and fires RTB later, which is the unsafe
  // direction. Descent is land_speed_mps.
  cruiseSpeedMps: 0.6,
  landSpeedMps: 0.5,
  reservePct: 8,
  rtbNominalPct: 30,
  rtbGatedPct: 20,

  // "auto" hides sections with no topic source; "all" forces everything on.
  sections: "auto",
};

// ─────────────────────────── helpers ──────────────────────────────────────────

function splitList(s) {
  return String(s ?? "").split(",").map((x) => x.trim()).filter(Boolean);
}

function tpl(template, name) {
  return template ? String(template).replace("{name}", name) : null;
}

function toSec(t) {
  if (t == null) return null;
  if (typeof t === "number") return t;
  const nanos = t.nanosec ?? t.nsec ?? 0;
  if (t.sec == null) return null;
  return Number(t.sec) + Number(nanos) * 1e-9;
}

function num(v) {
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

function clamp(v, lo, hi) { return Math.min(hi, Math.max(lo, v)); }

function median(arr) {
  if (!arr.length) return null;
  const s = [...arr].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2;
}

// Least-squares slope of y over x (used for clock drift and SoC burn rate).
// x is centred first: these are epoch seconds (~1.7e9) and the uncentred normal
// equations lose the whole signal to float64 cancellation.
function slope(xs, ys) {
  const n = xs.length;
  if (n < 3) return null;
  const mx = xs.reduce((a, b) => a + b, 0) / n;
  const my = ys.reduce((a, b) => a + b, 0) / n;
  let sxx = 0, sxy = 0;
  for (let i = 0; i < n; i++) {
    const dx = xs[i] - mx;
    sxx += dx * dx;
    sxy += dx * (ys[i] - my);
  }
  if (sxx < 1e-9) return null;
  return sxy / sxx;
}

function fmt(v, digits, unit) {
  if (v == null || !Number.isFinite(v)) return "--";
  return v.toFixed(digits) + (unit ?? "");
}

function fmtDuration(sec) {
  if (sec == null || !Number.isFinite(sec) || sec < 0) return "--";
  const s = Math.round(sec);
  const m = Math.floor(s / 60), h = Math.floor(m / 60);
  const mm = String(m % 60).padStart(2, "0"), ss = String(s % 60).padStart(2, "0");
  return h > 0 ? `${h}:${mm}:${ss}` : `${m}:${ss}`;
}

function clockStamp(t) {
  return new Date(t * 1000).toLocaleTimeString();
}

// Worst (highest-ranked) state in the list; NO_DATA only when the list is empty.
function worst(states, table) {
  let out = null;
  for (const s of states) if (s && (out == null || s.rank > out.rank)) out = s;
  return out ?? table.NO_DATA;
}

function parseVec3(s, fallback) {
  const p = splitList(s).map(Number);
  if (p.length !== 3 || p.some((x) => !Number.isFinite(x))) return fallback;
  return p;
}

// A JSON report only overrides derived values while it is fresh — a reporter
// that died must not pin the panel to its last-known-good numbers forever.
function freshReport(report, at, now) {
  return report != null && at != null && now - at <= STALE_REPORT_S ? report : null;
}

// ─────────────────────────── stream statistics ────────────────────────────────
//
// One instance per (agent, stream). Everything is derived from the arrival
// pattern of the messages themselves:
//
//   drop rate  — the nominal publish period P is the median inter-arrival gap;
//                a gap of k*P is counted as k-1 missed samples out of k expected.
//                Gaps longer than the loss timeout are outages, not loss, and
//                are excluded (the state machine reports those instead).
//   age        — rx_time - header.stamp: the one-way delay of the newest sample.
//                For the mocap stream this is exactly the "how stale is the
//                state estimate PX4 is fusing" number.
//   derived RTT— 2 x the mean of that offset. The identity holds only while the
//                publisher and the GCS share a timebase, which the clock-drift
//                column is there to falsify; a measured ping (PX4 timesync or
//                the Tailscale reporter) always wins over it.

function newStream(topic) {
  return {
    topic,
    lastRx: null,
    period: null,        // median inter-arrival, seconds
    gaps: [],            // [t, expected, missed]
    allGaps: [],         // [t, dt] for EVERY gap, outages included — feeds the
                         // consecutive-blackout metric
    offsets: [],         // [t, offset_sec] (rx - stamp)
    dts: [],             // recent inter-arrival gaps for the median
    everSeen: false,
  };
}

function streamOnMessage(st, rxSec, stampSec) {
  st.everSeen = true;
  if (st.lastRx != null) {
    const dt = rxSec - st.lastRx;
    if (dt > 0) {
      st.dts.push(dt);
      if (st.dts.length > 200) st.dts.shift();
      // Robust period estimate from the gaps that look un-dropped.
      const med = median(st.dts) ?? dt;
      const clean = st.dts.filter((d) => d < 2.5 * med);
      st.period = median(clean) ?? med;
      st.allGaps.push([rxSec, dt]);
      if (dt <= LINK_LOSS_TIMEOUT_S) {
        // floor, not round. A genuine loss lands at ~2.0x the period, but this
        // publisher is not metronomic (measured 31-65 ms on a 39 ms median), and
        // rounding scored every 1.5x jitter spike as a lost packet. ROS 2 headers
        // carry no sequence number, so timing is all we have — it must at least
        // not invent losses the stream never had.
        const expected = Math.max(1, Math.floor(dt / st.period));
        st.gaps.push([rxSec, expected, expected - 1]);
      }
      // A gap longer than the loss timeout is an outage, not packet loss: it is
      // already reported as LOST/ON VPN and logged as a state transition, so it
      // is deliberately left out of the drop-rate window.
    }
  }
  st.lastRx = rxSec;
  if (stampSec != null) st.offsets.push([rxSec, rxSec - stampSec]);
  // Prune by age, not by sample count — a count cap would silently shorten the
  // clock window on a fast topic.
  prune(st.gaps, rxSec - METRIC_WINDOW_S * 2);
  prune(st.allGaps, rxSec - METRIC_WINDOW_S * 2);
  prune(st.offsets, rxSec - CLOCK_WINDOW_S * 1.5);
}

// Drop leading entries older than `cutoff` from an ascending [t, ...] list.
function prune(list, cutoff) {
  let i = 0;
  while (i < list.length && list[i][0] < cutoff) i++;
  if (i > 0) list.splice(0, i);
}

function streamFresh(st, now, timeout) {
  return st.lastRx != null && now - st.lastRx <= (timeout ?? LINK_LOSS_TIMEOUT_S);
}

function streamRateHz(st) {
  return st.period ? 1 / st.period : null;
}

function streamDropRatePct(st, now) {
  const cutoff = now - METRIC_WINDOW_S;
  let expected = 0, missed = 0;
  for (let i = st.gaps.length - 1; i >= 0; i--) {
    if (st.gaps[i][0] < cutoff) break;
    expected += st.gaps[i][1];
    missed += st.gaps[i][2];
  }
  if (expected < 5) return null;
  return (missed / expected) * 100;
}

// Longest single gap in the window, in ms. Unlike the drop percentage this is
// not an average, so it survives being diluted by a long healthy window — and it
// is the quantity the commander reacts to (state_timeout_s).
function streamMaxGapMs(st, now) {
  const cutoff = now - METRIC_WINDOW_S;
  let worst = null;
  for (let i = st.allGaps.length - 1; i >= 0; i--) {
    if (st.allGaps[i][0] < cutoff) break;
    if (worst == null || st.allGaps[i][1] > worst) worst = st.allGaps[i][1];
  }
  // An ongoing silence counts too: the gap is still open and already this long.
  if (st.lastRx != null) {
    const open = now - st.lastRx;
    if (worst == null || open > worst) worst = open;
  }
  return worst == null ? null : worst * 1000;
}

// Age of the newest sample, in ms: how far behind wall time the data is.
function streamAgeMs(st) {
  if (!st.offsets.length) return null;
  return st.offsets[st.offsets.length - 1][1] * 1000;
}

// Fallback ping when nothing measured one: 2 x the mean one-way delay.
function streamDerivedRttMs(st, now) {
  const cutoff = now - METRIC_WINDOW_S;
  const win = [];
  for (let i = st.offsets.length - 1; i >= 0; i--) {
    if (st.offsets[i][0] < cutoff) break;
    win.push(st.offsets[i][1]);
  }
  if (win.length < 5) return null;
  const mean = win.reduce((a, b) => a + b, 0) / win.length;
  return Math.max(0, mean * 2 * 1000);
}

// Returns { offsetMs, driftMsPerMin } — the clock-sync error and its rate.
function streamClock(st, now) {
  const cutoff = now - CLOCK_WINDOW_S;
  const xs = [], ys = [];
  for (let i = st.offsets.length - 1; i >= 0; i--) {
    if (st.offsets[i][0] < cutoff) break;
    xs.push(st.offsets[i][0]);
    ys.push(st.offsets[i][1]);
  }
  if (xs.length < 5) return { offsetMs: null, driftMsPerMin: null };
  // Bucket to per-second minima so queueing jitter doesn't pollute the slope.
  const buckets = new Map();
  for (let i = 0; i < xs.length; i++) {
    const k = Math.floor(xs[i]);
    const prev = buckets.get(k);
    if (prev == null || ys[i] < prev) buckets.set(k, ys[i]);
  }
  const keys = [...buckets.keys()].sort((a, b) => a - b);
  const by = keys.map((k) => buckets.get(k));
  const offsetMs = by[by.length - 1] * 1000;
  // Measured on a link with no real drift: the same fit reads +1.98 ms/min with
  // 10 s of history and +0.007 ms/min with 120 s. Reporting the former as
  // "clock drift" is reporting startup noise, so wait for the window to fill.
  if (keys.length < CLOCK_MIN_BUCKETS) return { offsetMs, driftMsPerMin: null };
  const s = slope(keys, by);
  return { offsetMs, driftMsPerMin: s == null ? null : s * 60 * 1000 };
}

// ─────────────────────────── battery normalisation ────────────────────────────
//
// Accepts px4_msgs/BatteryStatus (real, uXRCE-DDS) or sensor_msgs/BatteryState
// (sim, MAVROS) and flattens them to one shape.

function normaliseBattery(msg) {
  if (msg == null || typeof msg !== "object") return null;
  const out = { soc: null, voltage: null, vFiltered: null, current: null,
                timeRemaining: null, cells: null, warning: null };

  if (msg.voltage_v !== undefined || msg.remaining !== undefined) {
    // px4_msgs/BatteryStatus — "unknown" is encoded as -1 / 0 / NaN.
    const rem = num(msg.remaining);
    out.soc = rem != null && rem >= 0 ? rem * 100 : null;
    const v = num(msg.voltage_v);
    out.voltage = v != null && v > 0 ? v : null;
    const vf = num(msg.voltage_filtered_v);
    out.vFiltered = vf != null && vf > 0 ? vf : out.voltage;
    const c = num(msg.current_filtered_a ?? msg.current_a);
    out.current = c != null && c >= 0 ? c : null;
    const tr = num(msg.time_remaining_s);
    out.timeRemaining = tr != null && tr > 0 ? tr : null;
    const cc = num(msg.cell_count);
    out.cells = cc != null && cc > 0 ? cc : null;
    out.warning = num(msg.warning);
    return out;
  }

  if (msg.percentage !== undefined || msg.voltage !== undefined) {
    // sensor_msgs/BatteryState — current is negative while discharging.
    const p = num(msg.percentage);
    out.soc = p != null && p >= 0 ? (p <= 1.0001 ? p * 100 : p) : null;
    const v = num(msg.voltage);
    out.voltage = v != null && v > 0 ? v : null;
    out.vFiltered = out.voltage;
    const c = num(msg.current);
    out.current = c != null ? Math.abs(c) : null;
    // cell_voltage is only a cell count if it actually holds CELL voltages.
    // MAVROS/PX4 SITL publishes the whole pack as a single element (16.2 V in a
    // 1-element array for a 4S), which would render as "16.2 V per cell" and a
    // pack sag mislabelled per-cell. Count only plausible cell voltages, and
    // report nothing rather than a number that is wrong by the cell count.
    const cv = Array.isArray(msg.cell_voltage) ? msg.cell_voltage.filter(
      (x) => Number.isFinite(Number(x)) && Number(x) > 1.0 && Number(x) < 5.0) : [];
    out.cells = cv.length ? cv.length : null;
    return out;
  }
  return null;
}

// ─────────────────────────── per-agent runtime ────────────────────────────────

function newAgent(name) {
  return {
    name,
    mode: "sim",                // resolved wiring: "sim" | "real"
    modeSource: "default",      // "config" | "detected" | "default"
    tiers: { lan: newStream(null), vpn: newStream(null) },
    mocap: newStream(null),
    reported: null,             // {linkStatusTopicTemplate} JSON
    reportedAt: null,
    cellular: null,             // {cellularTopicTemplate} JSON
    cellularAt: null,
    linkState: LINK_STATE.NO_DATA,
    linkSince: null,
    activeTier: null,
    transitions: [],            // [{t, from, to, tier}]
    // state estimate
    offset: [0, 0, 0],          // drone_position_offsets entry, added to odometry
    pos: null,                  // [x, y, z] world ENU
    speed: null,
    posAt: null,
    ekfFlags: null, ekfAt: null,
    localPos: null, localPosAt: null,
    timesync: null, timesyncAt: null,
    // battery
    batt: null,
    battAt: null,
    vRest: null,                // open-circuit baseline for the sag calculation
    sagPeak: 0,
    socHist: [],                // [t, soc] for the burn-rate estimate
  };
}

function noteLinkState(agent, next, tier, now) {
  if (agent.linkState === next) return;
  agent.transitions.push({
    t: now, from: agent.linkState.label, to: next.label, tier: tier ?? "--",
  });
  if (agent.transitions.length > MAX_TRANSITIONS) agent.transitions.shift();
  agent.linkState = next;
  agent.linkSince = now;
}

// Fold measured + derived + reported link data into the agent's health state.
function evaluateLink(agent, cfg, now) {
  const r = freshReport(agent.reported, agent.reportedAt, now);
  const cell = freshReport(agent.cellular, agent.cellularAt, now);
  const lan = agent.tiers.lan;
  const vpn = agent.tiers.vpn;

  const freshLan = streamFresh(lan, now);
  const freshVpn = streamFresh(vpn, now);

  let tier = r?.active_tier ?? (freshLan ? "lan" : freshVpn ? "vpn" : null);
  if (tier !== "lan" && tier !== "vpn") tier = freshLan ? "lan" : freshVpn ? "vpn" : null;
  agent.activeTier = tier;

  const st = tier ? agent.tiers[tier] : lan;
  const clock = streamClock(st, now);

  // Ping, best source first: an explicit link report, then PX4's own
  // uXRCE-DDS timesync round-trip (a genuinely measured RTT), then the
  // Tailscale reporter when the VPN is the live path, then the arrival-time
  // estimate. The source is surfaced so the operator knows which they got.
  let pingMs = null, pingSource = null;
  const reportedPing = num(r?.rtt_ms);
  // Two message shapes carry a measured round-trip:
  //   px4_msgs/TimesyncStatus    round_trip_time      microseconds (real, uXRCE-DDS)
  //   mavros_msgs/TimesyncStatus round_trip_time_ms   milliseconds (sim, MAVLink)
  const timesyncRtt = agent.timesync == null ? null
    : num(agent.timesync.round_trip_time_ms) != null
      ? Number(agent.timesync.round_trip_time_ms)
      : num(agent.timesync.round_trip_time) != null
        ? Number(agent.timesync.round_trip_time) / 1000
        : null;
  const cellPing = num(cell?.rtt_ms);
  if (reportedPing != null) { pingMs = reportedPing; pingSource = "report"; }
  else if (timesyncRtt != null && timesyncRtt > 0) { pingMs = timesyncRtt; pingSource = "timesync"; }
  else if (tier === "vpn" && cellPing != null) { pingMs = cellPing; pingSource = "tailscale"; }
  else {
    const derived = streamDerivedRttMs(st, now);
    if (derived != null) { pingMs = derived; pingSource = "derived"; }
  }

  // Clock offset: PX4's timesync estimate is authoritative when present.
  //   px4_msgs    estimated_offset      microseconds
  //   mavros_msgs  estimated_offset_ns   nanoseconds
  const timesyncOffsetMs = agent.timesync == null ? null
    : num(agent.timesync.estimated_offset_ns) != null
      ? Number(agent.timesync.estimated_offset_ns) / 1e6
      : num(agent.timesync.estimated_offset) != null
        ? Number(agent.timesync.estimated_offset) / 1000
        : null;

  agent.metrics = {
    dropPct: num(r?.drop_rate) != null ? Number(r.drop_rate) : streamDropRatePct(st, now),
    pingMs, pingSource,
    clockOffsetMs: timesyncOffsetMs != null ? timesyncOffsetMs
      : num(r?.clock_offset_ms) != null ? Number(r.clock_offset_ms) : clock.offsetMs,
    clockDriftMsPerMin: num(r?.clock_drift_ms) != null ? Number(r.clock_drift_ms)
      : clock.driftMsPerMin,
    lanFresh: freshLan,
    vpnFresh: freshVpn,
    vpnProvisioned: vpn.everSeen,
    rateHz: streamRateHz(st),
    maxGapMs: streamMaxGapMs(st, now),
    stateAgeMs: streamAgeMs(st),
  };

  let next;
  if (!lan.everSeen && !vpn.everSeen) {
    next = LINK_STATE.NO_DATA;
  } else if (!freshLan && !freshVpn) {
    next = LINK_STATE.LOST;
  } else {
    const m = agent.metrics;
    const badDrop = m.dropPct != null && m.dropPct > Number(cfg.dropTargetPct);
    // One long blackout is worse than the same loss spread thinly, and only
    // this gate can see it.
    const badGap = m.maxGapMs != null && m.maxGapMs > Number(cfg.stateGapTargetMs);
    // On the VPN path the acceptable ping is the (looser) cellular target.
    const pingTarget = tier === "vpn" ? Number(cfg.vpnPingTargetMs) : Number(cfg.pingTargetMs);
    const badPing = m.pingMs != null && m.pingMs > pingTarget;
    // Quality outranks routing: a link that failed over AND is out of spec is
    // reported as DEGRADED, with the tier column showing which path it took.
    next = badDrop || badPing || badGap ? LINK_STATE.DEGRADED
      : !freshLan && freshVpn ? LINK_STATE.FAILOVER
      : LINK_STATE.HEALTHY;
  }
  noteLinkState(agent, next, tier, now);
}

// Mocap freshness + PX4 EKF fusion state → one state-estimate verdict.
//
// This is the indoor arm-blocker made visible: with no GPS, PX4 only holds
// position while EKF2 is fusing the mocap stream as external vision. A mocap
// dropout shows up here (stale mocap, then cs_ev_pos clearing, then dead
// reckoning) long before the drone visibly drifts.
function evaluateEstimate(agent, cfg, now) {
  const mocapTimeout = Math.max(0.05, Number(cfg.mocapTimeoutS) || 0.5);
  const mocap = agent.mocap;
  const est = {
    mocapSeen: mocap.everSeen,
    mocapFresh: streamFresh(mocap, now, mocapTimeout),
    mocapAgeMs: streamAgeMs(mocap),
    mocapRateHz: streamRateHz(mocap),
    mocapSilentS: mocap.lastRx == null ? null : now - mocap.lastRx,
    detail: [],
  };

  const f = agent.ekfFlags;
  const lp = agent.localPos;
  const fEV = f ? Boolean(f.cs_ev_pos || f.cs_ev_hgt || f.cs_ev_vel) : null;

  if (f == null && lp == null) {
    est.state = EKF_STATE.NO_DATA;
  } else if (lp != null && (lp.xy_valid === false || lp.z_valid === false)) {
    est.state = EKF_STATE.NO_POS;
  } else if (f != null && (f.cs_fake_pos || f.cs_inertial_dead_reckoning)) {
    est.state = EKF_STATE.DEAD_REC;
  } else if (f != null && fEV === false && agent.mode === "real") {
    // Real drone flying indoors with no external-vision fusion: it is on
    // whatever else EKF2 found, which indoors is nothing good.
    est.state = EKF_STATE.NO_EV;
  } else if (fEV) {
    est.state = EKF_STATE.EV_FUSED;
  } else {
    est.state = EKF_STATE.VALID;
  }

  if (f) {
    if (f.cs_ev_pos) est.detail.push("ev-pos");
    if (f.cs_ev_yaw) est.detail.push("ev-yaw");
    if (f.cs_ev_hgt) est.detail.push("ev-hgt");
    if (f.cs_gps) est.detail.push("gps");
    if (f.cs_inertial_dead_reckoning) est.detail.push("dead-reckoning");
    if (f.cs_fake_pos) est.detail.push("fake-pos");
  }
  if (lp) {
    est.detail.push(`xy ${lp.xy_valid ? "ok" : "BAD"}`);
    est.detail.push(`z ${lp.z_valid ? "ok" : "BAD"}`);
    if (lp.heading_good_for_control === false) est.detail.push("heading BAD");
  }
  agent.estimate = est;
}

// Battery + distance-to-pad energy budget → RTB verdict.
function evaluatePower(agent, cfg, now) {
  const b = agent.batt;
  if (!b || b.soc == null) {
    agent.power = { state: RTB_STATE.NO_DATA };
    return;
  }

  // Burn rate (%/s) from the SoC slope; fall back to the autopilot estimate.
  const cutoff = now - SOC_SLOPE_WINDOW_S;
  const xs = [], ys = [];
  for (let i = agent.socHist.length - 1; i >= 0; i--) {
    if (agent.socHist[i][0] < cutoff) break;
    xs.push(agent.socHist[i][0]); ys.push(agent.socHist[i][1]);
  }
  let burnPctPerSec = null;
  const s = slope(xs, ys);
  if (s != null && s < 0) burnPctPerSec = -s;
  if (burnPctPerSec == null && b.timeRemaining) burnPctPerSec = b.soc / b.timeRemaining;

  // Dynamic remaining mission time.
  let missionTime = b.timeRemaining;
  if (missionTime == null && burnPctPerSec) missionTime = b.soc / burnPctPerSec;

  // Voltage sag: baseline is the highest voltage seen at low draw.
  const sag = agent.vRest != null && b.voltage != null
    ? Math.max(0, agent.vRest - b.voltage) : null;
  const sagPerCell = sag != null && b.cells ? sag / b.cells : null;

  // Distance-to-pad energy: cruise home, then descend.
  const pad = parseVec3(cfg.padPosition, [0, 0, 0]);
  let distance = null, returnTime = null, returnPct = null;
  if (agent.pos) {
    const dx = agent.pos[0] - pad[0], dy = agent.pos[1] - pad[1], dz = agent.pos[2] - pad[2];
    distance = Math.hypot(dx, dy);
    const cruise = Math.max(0.05, Number(cfg.cruiseSpeedMps) || 1);
    const land = Math.max(0.05, Number(cfg.landSpeedMps) || 0.3);
    returnTime = distance / cruise + Math.max(0, dz) / land;
    if (burnPctPerSec != null) returnPct = returnTime * burnPctPerSec + Number(cfg.reservePct);
  }

  const nominal = Number(cfg.rtbNominalPct);
  const gated = Number(cfg.rtbGatedPct);
  let state;
  if (b.soc < gated) state = RTB_STATE.FAILSAFE;
  else if (b.soc < nominal) state = RTB_STATE.GATED;
  else if (returnPct != null && b.soc <= returnPct) state = RTB_STATE.RTB_NOW;
  else state = RTB_STATE.NOMINAL;

  agent.power = {
    state, soc: b.soc, voltage: b.voltage, cells: b.cells,
    sag, sagPerCell, sagPeak: agent.sagPeak || null,
    current: b.current, missionTime, burnPctPerSec,
    distance, returnTime, returnPct,
    margin: returnPct != null ? b.soc - returnPct : null,
  };
}

// ─────────────────────────── styles ───────────────────────────────────────────
//
// Theme-neutral: colours inherit from Foxglove so the panel reads correctly in
// both the light and dark studio themes.

const STYLES = `
/* Section visibility is driven by the hidden property, and the author display
   rules below would otherwise beat the user-agent [hidden] rule. */
[hidden] { display: none !important; }

.sb-root {
  font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  font-size: 12px; color: inherit; height: 100%; box-sizing: border-box;
  padding: 8px; overflow-y: auto; overflow-x: hidden;
  display: flex; flex-direction: column; gap: 8px; position: relative;
}
.sb-card {
  background: rgba(127,127,127,0.08); border: 1px solid rgba(127,127,127,0.28);
  border-radius: 6px; padding: 8px;
}
.sb-title {
  font-size: 11px; font-weight: 700; letter-spacing: 0.06em; text-transform: uppercase;
  opacity: 0.75; margin-bottom: 6px; padding-bottom: 4px;
  border-bottom: 1px solid rgba(127,127,127,0.28);
}
.sb-sub { font-size: 10px; opacity: 0.6; font-weight: 500; text-transform: none; letter-spacing: 0; }

/* banner */
.sb-banner { display: flex; align-items: center; gap: 8px; flex-wrap: wrap; }
.sb-banner .sb-spacer { flex: 1; }
.sb-chip {
  display: inline-flex; align-items: center; gap: 5px; padding: 3px 8px;
  border-radius: 999px; font-size: 11px; font-weight: 700; color: #fff; white-space: nowrap;
}
.sb-chip.sb-quiet { background: transparent; border: 1px solid rgba(127,127,127,0.4); color: inherit; font-weight: 600; }
.sb-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }

/* safety bar */
.sb-safety { display: flex; gap: 8px; align-items: stretch; }
.sb-safety-stop {
  flex: 1; padding: 13px 14px; border: 2px solid #7f1d1d; border-radius: 6px;
  background: #dc2626; color: #fff; cursor: pointer;
  font-size: 14px; font-weight: 800; letter-spacing: 0.08em; text-transform: uppercase;
}
.sb-safety-stop.sb-armed { background: #7f1d1d; border-color: #fca5a5; animation: sb-pulse 0.7s ease-in-out infinite alternate; }
.sb-safety-stop:disabled { background: #6b7280; border-color: rgba(127,127,127,0.5); cursor: not-allowed; opacity: 0.7; }
@keyframes sb-pulse { from { box-shadow: 0 0 0 0 rgba(220,38,38,0.75); } to { box-shadow: 0 0 0 7px rgba(220,38,38,0); } }
.sb-safety-hold {
  padding: 13px 16px; border: 2px solid rgba(245,158,11,0.7); border-radius: 6px;
  background: #f59e0b; color: #1f2937; cursor: pointer;
  font-size: 13px; font-weight: 800; letter-spacing: 0.06em; text-transform: uppercase;
}
.sb-safety-hold:disabled { opacity: 0.55; cursor: not-allowed; }

/* command strip */
.sb-cmd-row { display: flex; gap: 6px; flex-wrap: wrap; align-items: center; }
.sb-btn {
  padding: 7px 12px; border: none; border-radius: 5px; color: #fff; cursor: pointer;
  font-size: 12px; font-weight: 700; letter-spacing: 0.02em;
}
.sb-btn:active { transform: scale(0.98); }
.sb-btn:disabled { opacity: 0.45; cursor: not-allowed; }
.sb-input {
  padding: 5px 7px; border-radius: 4px; border: 1px solid rgba(127,127,127,0.5);
  background: transparent; color: inherit; font-size: 12px; min-width: 0;
}
.sb-status { font-family: ui-monospace, monospace; font-size: 11px; opacity: 0.8; min-height: 14px; }

/* layout */
.sb-columns { display: grid; grid-template-columns: minmax(190px, 240px) minmax(0, 1fr); gap: 8px; align-items: start; }
@media (max-width: 680px) { .sb-columns { grid-template-columns: 1fr; } }
.sb-col { min-width: 0; display: flex; flex-direction: column; gap: 8px; }

/* roster */
.sb-agent {
  display: flex; flex-direction: column; gap: 4px; padding: 6px; border-radius: 5px;
  border: 1px solid rgba(127,127,127,0.3); cursor: pointer; background: transparent;
  color: inherit; text-align: left; width: 100%; box-sizing: border-box;
}
.sb-agent + .sb-agent { margin-top: 5px; }
.sb-agent.sb-selected { border-color: #10b981; box-shadow: inset 0 0 0 1px #10b981; }
.sb-agent-top { display: flex; align-items: center; gap: 6px; }
.sb-agent-name { font-weight: 700; font-size: 12px; flex: 1; }
.sb-mode { font-size: 9px; font-weight: 800; letter-spacing: 0.06em; padding: 1px 5px; border-radius: 3px; color: #fff; }
.sb-mode.sim { background: #2563eb; }
.sb-mode.real { background: #b45309; }

/* wiring */
.sb-wire { font-family: ui-monospace, monospace; font-size: 10.5px; line-height: 1.6; }
.sb-wire-row { display: flex; gap: 6px; }
.sb-wire-key { opacity: 0.55; min-width: 52px; flex-shrink: 0; }
.sb-wire-val { word-break: break-all; }

/* bars */
.sb-bar { position: relative; height: 8px; border-radius: 4px; background: rgba(127,127,127,0.25); overflow: hidden; }
.sb-bar-fill { position: absolute; left: 0; top: 0; bottom: 0; border-radius: 4px; transition: width 0.2s linear; }
.sb-bar-tick { position: absolute; top: -2px; bottom: -2px; width: 1px; background: rgba(255,255,255,0.75); box-shadow: 0 0 0 1px rgba(0,0,0,0.35); }
.sb-bar-lg { height: 14px; border-radius: 4px; }

/* metrics table */
.sb-scroll { overflow-x: auto; }
.sb-table { width: 100%; border-collapse: collapse; font-size: 11px; font-variant-numeric: tabular-nums; }
.sb-table th {
  text-align: right; font-weight: 600; opacity: 0.6; padding: 3px 6px; white-space: nowrap;
  border-bottom: 1px solid rgba(127,127,127,0.3); font-size: 10px; text-transform: uppercase; letter-spacing: 0.04em;
}
.sb-table th:first-child, .sb-table td:first-child { text-align: left; }
.sb-table td { text-align: right; padding: 3px 6px; white-space: nowrap; border-bottom: 1px solid rgba(127,127,127,0.14); }
.sb-table tr.sb-selected td { background: rgba(16,185,129,0.12); }
.sb-ok { color: #10b981; }
.sb-warn { color: #f59e0b; font-weight: 700; }
.sb-bad { color: #dc2626; font-weight: 700; }
.sb-muted { opacity: 0.45; }
.sb-src { font-size: 9px; opacity: 0.5; margin-left: 3px; }

/* transitions log */
.sb-log {
  font-family: ui-monospace, monospace; font-size: 10.5px; line-height: 1.5;
  max-height: 108px; overflow-y: auto; background: rgba(0,0,0,0.16);
  border: 1px solid rgba(127,127,127,0.28); border-radius: 4px; padding: 5px 7px; white-space: pre-wrap;
}

/* power cards */
.sb-power-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(230px, 1fr)); gap: 8px; }
.sb-power { border: 1px solid rgba(127,127,127,0.3); border-radius: 5px; padding: 7px; display: flex; flex-direction: column; gap: 6px; }
.sb-power.sb-selected { border-color: #10b981; }
.sb-kv { display: flex; justify-content: space-between; gap: 8px; font-size: 11px; font-variant-numeric: tabular-nums; }
.sb-kv span:first-child { opacity: 0.6; }
.sb-note { font-size: 10px; opacity: 0.55; line-height: 1.45; }

/* confirm dialog */
.sb-overlay {
  position: absolute; inset: 0; background: rgba(0,0,0,0.55); display: flex;
  align-items: center; justify-content: center; z-index: 20; padding: 16px;
}
.sb-modal {
  background: #1f2937; color: #f9fafb; border-radius: 8px; padding: 14px;
  max-width: 320px; display: flex; flex-direction: column; gap: 10px;
  box-shadow: 0 8px 28px rgba(0,0,0,0.5);
}
.sb-modal-title { font-weight: 700; font-size: 13px; }
.sb-modal-row { display: flex; gap: 8px; justify-content: flex-end; }
`;

// ─────────────────────────── topology drawing ─────────────────────────────────
//
// GCS/DDS domain → {local Wi-Fi/LAN | 4G/5G Tailscale VPN} → agents. The path
// actually carrying each agent's telemetry is drawn solid and coloured by link
// health; a provisioned-but-standby path is dashed, and a path nothing has ever
// used is not drawn at all.

function topologySvg(agents, lanes) {
  const n = Math.max(1, agents.length);
  const colW = 118;
  const w = Math.max(430, n * colW + 40);
  const h = 208;
  const gcsY = 24, laneY = 92, agentY = 176;
  const spread = lanes.length > 1 ? 118 : 0;
  const laneCx = lanes.map((_, i) =>
    lanes.length > 1 ? w / 2 + (i === 0 ? -spread : spread) : w / 2);
  const ax = (i) => (w - (n - 1) * colW) / 2 + i * colW;

  const parts = [];
  parts.push(`<svg viewBox="0 0 ${w} ${h}" width="100%" height="${h}" style="max-width:100%;overflow:visible">`);
  parts.push(`<style>
    .tp-lbl{font:600 10px Inter,sans-serif;fill:currentColor;opacity:.85;text-anchor:middle}
    .tp-sub{font:500 8.5px Inter,sans-serif;fill:currentColor;opacity:.5;text-anchor:middle}
    .tp-box{fill:rgba(127,127,127,.12);stroke:rgba(127,127,127,.55);stroke-width:1}
  </style>`);

  // GCS / DDS domain
  parts.push(`<rect x="${w / 2 - 92}" y="${gcsY - 15}" width="184" height="30" rx="6" class="tp-box"/>`);
  parts.push(`<text class="tp-lbl" x="${w / 2}" y="${gcsY - 1}">GCS · ROS 2 / DDS domain</text>`);
  parts.push(`<text class="tp-sub" x="${w / 2}" y="${gcsY + 10}">time sync · QoS · dynamic routing</text>`);

  // Transport lanes
  lanes.forEach((tier, ti) => {
    const anyActive = agents.some((a) => a.activeTier === tier.id);
    const cx = laneCx[ti];
    const stroke = anyActive ? "#10b981" : "rgba(127,127,127,.55)";
    parts.push(`<rect x="${cx - 88}" y="${laneY - 15}" width="176" height="30" rx="15"
      fill="${anyActive ? "rgba(16,185,129,.14)" : "rgba(127,127,127,.08)"}" stroke="${stroke}" stroke-width="${anyActive ? 1.6 : 1}"/>`);
    parts.push(`<text class="tp-lbl" x="${cx}" y="${laneY - 1}">${tier.label}</text>`);
    parts.push(`<text class="tp-sub" x="${cx}" y="${laneY + 10}">${tier.sub}</text>`);
    parts.push(`<path d="M ${w / 2} ${gcsY + 15} C ${w / 2} ${laneY - 40}, ${cx} ${laneY - 45}, ${cx} ${laneY - 15}"
      fill="none" stroke="${stroke}" stroke-width="${anyActive ? 2 : 1}" ${anyActive ? "" : 'stroke-dasharray="4 3" opacity="0.5"'}/>`);
  });

  // Agents + their links
  agents.forEach((a, i) => {
    const x = ax(i);
    const color = a.linkState.color;
    lanes.forEach((tier, ti) => {
      const active = a.activeTier === tier.id;
      const fresh = tier.id === "lan" ? a.metrics?.lanFresh : a.metrics?.vpnFresh;
      const provisioned = a.tiers[tier.id].everSeen;
      const cx = laneCx[ti];
      let stroke, width, dash, opacity;
      if (active) { stroke = color; width = 2.4; dash = ""; opacity = 1; }
      else if (fresh || provisioned) { stroke = "#9ca3af"; width = 1.2; dash = 'stroke-dasharray="5 4"'; opacity = 0.55; }
      else { stroke = "#6b7280"; width = 1; dash = 'stroke-dasharray="2 4"'; opacity = 0.28; }
      parts.push(`<path d="M ${cx} ${laneY + 15} C ${cx} ${agentY - 42}, ${x} ${agentY - 46}, ${x} ${agentY - 16}"
        fill="none" stroke="${stroke}" stroke-width="${width}" ${dash} opacity="${opacity}"/>`);
    });
    const modeColor = MODES[a.mode].color;
    parts.push(`<rect x="${x - 46}" y="${agentY - 16}" width="92" height="32" rx="5"
      fill="rgba(127,127,127,.12)" stroke="${color}" stroke-width="1.6"/>`);
    parts.push(`<circle cx="${x - 34}" cy="${agentY - 4}" r="4" fill="${modeColor}"/>`);
    parts.push(`<text class="tp-lbl" x="${x + 5}" y="${agentY - 1}">${a.name}</text>`);
    parts.push(`<text class="tp-sub" x="${x}" y="${agentY + 11}">${MODES[a.mode].label} · ${a.linkState.label}</text>`);
  });

  parts.push(`</svg>`);
  return parts.join("");
}

// ─────────────────────────── panel ────────────────────────────────────────────

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "SVG Basestation",
    initPanel: (panelContext) => {

      // ── state ────────────────────────────────────────────────────────────
      const persisted = panelContext.initialState ?? {};
      const cfg = { ...DEFAULTS };
      for (const k of Object.keys(DEFAULTS)) {
        if (persisted[k] !== undefined && persisted[k] !== null) cfg[k] = persisted[k];
      }
      let selected = persisted.selected ?? null;
      let formation = persisted.formation ?? "";

      let agents = [];                 // rebuilt whenever the roster config changes
      const byTopic = new Map();       // topic → [{agent, kind}]
      let available = new Set();       // topic names seen on the data source
      let topicsKey = "";              // change detector for the topic list
      let caps = {};                   // which sections have a source
      let statusText = "";
      let lastTopoKey = null;
      let safetyArmedUntil = 0;
      // "Now" is anchored on the newest receive time and advanced by real
      // elapsed time. Live, that is just the wall clock; during playback it
      // tracks bag time, and it keeps advancing when the data stops so link
      // staleness is still detected.
      let clockRx = null, clockWall = null;

      function nowSec() {
        const wall = Date.now() / 1000;
        return clockRx == null ? wall : clockRx + (wall - clockWall);
      }
      function persist() {
        panelContext.saveState({ ...cfg, selected, formation });
      }

      // ── wiring: mode → topics ────────────────────────────────────────────
      //
      // An agent's mode IS its wiring. "sim" talks to the MAVROS interface,
      // "real" talks to px4_interface over uXRCE-DDS and additionally carries
      // the mocap + EKF + timesync streams that only exist on hardware.

      function wiringFor(agent) {
        const n = agent.name;
        const real = agent.mode === "real";
        return {
          state: tpl(cfg.stateTopicTemplate, n),
          battery: tpl(real ? cfg.realBatteryTopicTemplate : cfg.simBatteryTopicTemplate, n),
          command: tpl(real ? cfg.realCommandTopicTemplate : cfg.simCommandTopicTemplate, n),
          robotCommand: tpl(real ? cfg.realRobotCommandTemplate : cfg.simRobotCommandTemplate, n),
          mocap: real ? tpl(cfg.mocapTopicTemplate, n) : null,
          ekfFlags: real ? tpl(cfg.ekfFlagsTopicTemplate, n) : null,
          localPosition: real ? tpl(cfg.localPositionTopicTemplate, n) : null,
          timesync: real ? tpl(cfg.timesyncTopicTemplate, n)
                         : tpl(cfg.simTimesyncTopicTemplate, n),
          lan: tpl(cfg.lanTopicTemplate, n),
          vpn: tpl(cfg.vpnTopicTemplate, n),
          cellular: tpl(cfg.cellularTopicTemplate, n),
        };
      }

      // px4_interface publishes under /{name}/fmu/..., the MAVROS interface
      // under /{name}/interface/... — so the topics on the wire tell us how
      // this agent is actually wired, with no extra configuration.
      function detectMode(name) {
        let real = false, sim = false;
        for (const t of available) {
          if (t.startsWith(`/${name}/fmu/`)) real = true;
          else if (t.startsWith(`/${name}/interface/`)) sim = true;
        }
        return real ? "real" : sim ? "sim" : null;
      }

      function resolveModes() {
        const configured = splitList(cfg.modes).map((m) => m.toLowerCase());
        agents.forEach((a, i) => {
          const want = configured[i];
          if (want === "sim" || want === "real") {
            a.mode = want;
            a.modeSource = "config";
            return;
          }
          const detected = detectMode(a.name);
          if (detected) {
            a.mode = detected;
            a.modeSource = "detected";
          } else {
            a.mode = "sim";
            a.modeSource = "default";
          }
        });
      }

      // ── roster / subscriptions ───────────────────────────────────────────
      function rebuildAgents() {
        const names = splitList(cfg.drones);
        const prev = new Map(agents.map((a) => [a.name, a]));
        // drone_position_offsets, flat x,y,z per agent (see swarm_sim.yaml).
        const offsets = splitList(cfg.positionOffsets).map(Number);
        agents = names.map((name, i) => {
          const at = (k) => (Number.isFinite(offsets[i * 3 + k]) ? offsets[i * 3 + k] : 0);
          const agent = prev.get(name) ?? newAgent(name);
          agent.offset = [at(0), at(1), at(2)];
          return agent;
        });
        if (!agents.some((a) => a.name === selected)) selected = agents[0]?.name ?? null;
        resolveModes();
        rebuildSubscriptions();
        buildRoster();
        recomputeCaps();
      }

      function rebuildSubscriptions() {
        byTopic.clear();
        const add = (topic, agent, kind) => {
          if (!topic) return;
          if (!byTopic.has(topic)) byTopic.set(topic, []);
          byTopic.get(topic).push({ agent, kind });
        };
        for (const a of agents) {
          const n = a.name;
          a.tiers.lan.topic = tpl(cfg.lanTopicTemplate, n);
          a.tiers.vpn.topic = tpl(cfg.vpnTopicTemplate, n);
          a.mocap.topic = tpl(cfg.mocapTopicTemplate, n);
          add(tpl(cfg.stateTopicTemplate, n), a, "state");
          add(a.tiers.lan.topic, a, "lan");
          add(a.tiers.vpn.topic, a, "vpn");
          add(a.mocap.topic, a, "mocap");
          // Both battery shapes are subscribed regardless of mode: a wrong
          // mode guess must not blank the power picture. normaliseBattery
          // accepts either message.
          add(tpl(cfg.simBatteryTopicTemplate, n), a, "battery");
          add(tpl(cfg.realBatteryTopicTemplate, n), a, "battery");
          add(tpl(cfg.ekfFlagsTopicTemplate, n), a, "ekfflags");
          add(tpl(cfg.localPositionTopicTemplate, n), a, "localpos");
          add(tpl(cfg.timesyncTopicTemplate, n), a, "timesync");
          add(tpl(cfg.simTimesyncTopicTemplate, n), a, "timesync");
          add(tpl(cfg.cellularTopicTemplate, n), a, "cellular");
          add(tpl(cfg.linkStatusTopicTemplate, n), a, "linkstatus");
        }
        panelContext.subscribe([...byTopic.keys()].map((topic) => ({ topic })));
      }

      // ── section visibility ───────────────────────────────────────────────
      //
      // A section is rendered only when something is actually publishing what
      // it needs. Before any topic list has arrived (a fresh connection, or a
      // data source that does not advertise topics) everything is shown —
      // an empty panel would be a worse failure than an over-full one.

      function recomputeCaps() {
        const showAll = cfg.sections === "all" || available.size === 0;
        const has = (t) => Boolean(t) && available.has(t);
        const any = (fn) => agents.some(fn);
        caps = {
          discovered: available.size > 0,
          forced: showAll,
          mocap: showAll || any((a) => has(tpl(cfg.mocapTopicTemplate, a.name))),
          ekf: showAll || any((a) => has(tpl(cfg.ekfFlagsTopicTemplate, a.name))
            || has(tpl(cfg.localPositionTopicTemplate, a.name))),
          timesync: showAll || any((a) => has(tpl(cfg.timesyncTopicTemplate, a.name))),
          cellular: showAll || any((a) => has(tpl(cfg.cellularTopicTemplate, a.name))
            || has(tpl(cfg.vpnTopicTemplate, a.name))),
          vpnLane: showAll || any((a) => has(tpl(cfg.vpnTopicTemplate, a.name))),
          battery: showAll || any((a) => has(tpl(cfg.simBatteryTopicTemplate, a.name))
            || has(tpl(cfg.realBatteryTopicTemplate, a.name))),
          formation: showAll || has(cfg.formationTopic),
          teleop: any((a) => has(tpl(cfg.teleopTopicTemplate, a.name))),
          goal: any((a) => has(tpl(cfg.goalTopicTemplate, a.name))),
        };
      }

      // The task set the panel inferred, shown in the banner so the operator
      // can see why a section is or is not there.
      function detectedTasks() {
        const tasks = [];
        if (caps.goal) tasks.push("goal-tracking");
        if (caps.formation) tasks.push("formation");
        if (caps.teleop) tasks.push("teleop");
        if (caps.mocap || caps.ekf) tasks.push("mocap/hardware");
        if (caps.cellular) tasks.push("cellular");
        return tasks;
      }

      // ── message handling ─────────────────────────────────────────────────
      function handleState(a, msg, rx) {
        const p = msg?.pose?.pose?.position;
        if (p && num(p.x) != null) {
          const o = a.offset;
          a.pos = [Number(p.x) + o[0], Number(p.y) + o[1], Number(p.z) + o[2]];
          a.posAt = rx;
        }
        const v = msg?.twist?.twist?.linear;
        if (v && num(v.x) != null) a.speed = Math.hypot(Number(v.x), Number(v.y), Number(v.z));
      }

      function handleBattery(a, msg, rx) {
        const b = normaliseBattery(msg);
        if (!b) return;
        a.batt = b;
        a.battAt = rx;
        // Open-circuit baseline: the highest terminal voltage seen this session
        // is the least-loaded sample we have, so it is the best OCV proxy.
        if (b.voltage != null) {
          if (a.vRest == null || b.voltage > a.vRest) a.vRest = b.voltage;
          const sag = Math.max(0, a.vRest - b.voltage);
          if (sag > a.sagPeak) a.sagPeak = sag;
        }
        if (b.soc != null) {
          a.socHist.push([rx, b.soc]);
          prune(a.socHist, rx - SOC_SLOPE_WINDOW_S * 2);
        }
      }

      function handleJson(msg) {
        const raw = msg?.data;
        if (typeof raw === "string") return JSON.parse(raw);
        return raw ?? msg;
      }

      panelContext.onRender = (renderState, done) => {
        if (renderState.topics) {
          const key = renderState.topics.map((t) => t.name).join(" ");
          if (key !== topicsKey) {
            topicsKey = key;
            available = new Set(renderState.topics.map((t) => t.name));
            resolveModes();
            recomputeCaps();
            buildRoster();
          }
        }
        const frame = renderState.currentFrame;
        if (frame) {
          for (const evt of frame) {
            const entries = byTopic.get(evt.topic);
            if (!entries) continue;
            const rx = toSec(evt.receiveTime) ?? Date.now() / 1000;
            if (clockRx == null || rx > clockRx) { clockRx = rx; clockWall = Date.now() / 1000; }
            const stamp = toSec(evt.message?.header?.stamp);
            for (const { agent, kind } of entries) {
              switch (kind) {
                case "state": handleState(agent, evt.message, rx); break;
                case "battery": handleBattery(agent, evt.message, rx); break;
                case "ekfflags": agent.ekfFlags = evt.message; agent.ekfAt = rx; break;
                case "localpos": agent.localPos = evt.message; agent.localPosAt = rx; break;
                case "timesync": agent.timesync = evt.message; agent.timesyncAt = rx; break;
                case "linkstatus":
                  // A malformed report leaves the derived metrics in place.
                  try { agent.reported = handleJson(evt.message); agent.reportedAt = rx; } catch { /* ignore */ }
                  break;
                case "cellular":
                  try { agent.cellular = handleJson(evt.message); agent.cellularAt = rx; } catch { /* ignore */ }
                  break;
                case "mocap": streamOnMessage(agent.mocap, rx, stamp); break;
                default: streamOnMessage(agent.tiers[kind], rx, stamp); break;
              }
            }
          }
        }
        done();
      };
      panelContext.watch("currentFrame");
      panelContext.watch("topics");

      // ── DOM ──────────────────────────────────────────────────────────────
      const root = panelContext.panelElement;
      root.classList.add("sb-root");
      const styleEl = document.createElement("style");
      styleEl.textContent = STYLES;
      root.appendChild(styleEl);

      const el = (tag, cls, text) => {
        const n = document.createElement(tag);
        if (cls) n.className = cls;
        if (text != null) n.textContent = text;
        return n;
      };
      const show = (node, visible) => { node.hidden = !visible; };

      // Banner
      const banner = el("div", "sb-card sb-banner");
      const linkChip = el("span", "sb-chip");
      const estChip = el("span", "sb-chip");
      const powerChip = el("span", "sb-chip");
      const modeChip = el("span", "sb-chip sb-quiet");
      const taskChip = el("span", "sb-chip sb-quiet");
      const clockEl = el("span", "sb-status");
      banner.append(linkChip, estChip, powerChip, modeChip, taskChip,
        el("div", "sb-spacer"), clockEl);
      root.appendChild(banner);

      // Safety bar — the one control an operator must be able to hit without
      // reading anything. Two clicks (arm, then fire) rather than a modal:
      // fast under stress, but an accidental brush cannot land the swarm.
      const safetyBar = el("div", "sb-safety");
      const holdBtn = el("button", "sb-safety-hold", "Hold All");
      holdBtn.title = "Freeze every commanded drone at its current position (scenario stops)";
      holdBtn.addEventListener("click", () => callLifecycle("hold"));
      const stopBtn = el("button", "sb-safety-stop");
      stopBtn.addEventListener("click", onSafetyClick);
      safetyBar.append(holdBtn, stopBtn);
      root.appendChild(safetyBar);

      // Command strip
      const cmdCard = el("div", "sb-card");
      cmdCard.appendChild(el("div", "sb-title", "Swarm Command"));
      const cmdRow = el("div", "sb-cmd-row");
      for (const item of LIFECYCLE) {
        const b = el("button", "sb-btn", item.label);
        b.style.background = item.color;
        b.title = item.hint;
        b.addEventListener("click", () => {
          if (item.confirm) {
            askConfirm(item.label, `${item.hint}.\n\nSend "${item.id}" to ${cfg.commanderNs}?`,
              () => callLifecycle(item.id));
          } else {
            callLifecycle(item.id);
          }
        });
        cmdRow.appendChild(b);
      }
      cmdCard.appendChild(cmdRow);

      const formRow = el("div", "sb-cmd-row");
      formRow.style.marginTop = "6px";
      const formLabel = el("span", null, "Formation:");
      formLabel.style.opacity = "0.65";
      const formInput = el("input", "sb-input");
      formInput.type = "text";
      formInput.placeholder = "profile name, or 'next'";
      formInput.value = formation;
      formInput.style.flex = "1";
      formInput.addEventListener("change", () => { formation = formInput.value.trim(); persist(); });
      const formBtn = el("button", "sb-btn", "Send");
      formBtn.style.background = "#4f46e5";
      formBtn.title = `Publish the profile name on ${cfg.formationTopic} (std_msgs/String) to retarget the swarm`;
      formBtn.addEventListener("click", () => sendFormation(formInput.value.trim()));
      formRow.append(formLabel, formInput, formBtn);
      cmdCard.appendChild(formRow);

      const statusEl = el("div", "sb-status");
      statusEl.style.marginTop = "5px";
      cmdCard.appendChild(statusEl);
      root.appendChild(cmdCard);

      // Columns
      const columns = el("div", "sb-columns");
      const leftCol = el("div", "sb-col");
      const rightCol = el("div", "sb-col");
      columns.append(leftCol, rightCol);
      root.appendChild(columns);

      // Roster
      const rosterCard = el("div", "sb-card");
      rosterCard.appendChild(el("div", "sb-title", "Agents"));
      const rosterBody = el("div");
      rosterCard.appendChild(rosterBody);
      leftCol.appendChild(rosterCard);
      const rosterRows = new Map();

      function buildRoster() {
        rosterBody.textContent = "";
        rosterRows.clear();
        if (!agents.length) {
          rosterBody.appendChild(el("div", "sb-note", "No agents configured — set the agent list in the panel settings."));
          return;
        }
        for (const a of agents) {
          const row = el("button", "sb-agent");
          const top = el("div", "sb-agent-top");
          const dot = el("span", "sb-dot");
          const name = el("span", "sb-agent-name", a.name);
          const mode = el("span", `sb-mode ${a.mode}`, MODES[a.mode].label);
          top.append(dot, name, mode);
          const bar = el("div", "sb-bar");
          const fill = el("div", "sb-bar-fill");
          bar.appendChild(fill);
          const meta = el("div", "sb-note");
          row.append(top, bar, meta);
          row.addEventListener("click", () => { selected = a.name; persist(); render(); });
          rosterBody.appendChild(row);
          rosterRows.set(a.name, { row, dot, fill, meta, mode });
        }
      }

      // Wiring card — makes "mode = wiring" concrete for the selected agent.
      const wireCard = el("div", "sb-card");
      const wireTitle = el("div", "sb-title");
      wireTitle.append(document.createTextNode("Wiring "));
      wireTitle.appendChild(el("span", "sb-sub", "— topics this mode uses"));
      wireCard.appendChild(wireTitle);
      const wireBody = el("div", "sb-wire");
      wireCard.appendChild(wireBody);
      const wireNote = el("div", "sb-note");
      wireNote.style.marginTop = "6px";
      wireCard.appendChild(wireNote);
      leftCol.appendChild(wireCard);

      // Link safety section
      const commCard = el("div", "sb-card");
      const commTitle = el("div", "sb-title");
      commTitle.append(document.createTextNode("Link Safety "));
      const commSub = el("span", "sb-sub");
      commTitle.appendChild(commSub);
      commCard.appendChild(commTitle);
      const topoBox = el("div");
      topoBox.style.cssText = "margin-bottom:8px;overflow-x:auto;";
      commCard.appendChild(topoBox);
      const metricsScroll = el("div", "sb-scroll");
      const metricsTable = el("table", "sb-table");
      const metricsHead = el("thead");
      const metricsHeadRow = el("tr");
      metricsHead.appendChild(metricsHeadRow);
      const metricsBody = el("tbody");
      metricsTable.append(metricsHead, metricsBody);
      metricsScroll.appendChild(metricsTable);
      commCard.appendChild(metricsScroll);
      const targetsNote = el("div", "sb-note");
      targetsNote.style.marginTop = "5px";
      commCard.appendChild(targetsNote);
      commCard.appendChild(el("div", "sb-title", "Link health state transitions"));
      const logBox = el("div", "sb-log");
      commCard.appendChild(logBox);
      rightCol.appendChild(commCard);

      // Cellular / Tailscale section
      const cellCard = el("div", "sb-card");
      const cellTitle = el("div", "sb-title");
      cellTitle.append(document.createTextNode("Cellular · Tailscale VPN "));
      cellTitle.appendChild(el("span", "sb-sub", "— 4G/5G transport delay"));
      cellCard.appendChild(cellTitle);
      const cellScroll = el("div", "sb-scroll");
      const cellTable = el("table", "sb-table");
      const cellHead = el("thead");
      const cellCols = ["Agent", "Network", "VPN ping", "Path", "Signal", "Interface", "Report age"];
      const cellHeadRow = el("tr");
      for (const c of cellCols) cellHeadRow.appendChild(el("th", null, c));
      cellHead.appendChild(cellHeadRow);
      const cellBody = el("tbody");
      cellTable.append(cellHead, cellBody);
      cellScroll.appendChild(cellTable);
      cellCard.appendChild(cellScroll);
      const cellNote = el("div", "sb-note");
      cellNote.style.marginTop = "5px";
      cellCard.appendChild(cellNote);
      rightCol.appendChild(cellCard);

      // Power section
      const powerCard = el("div", "sb-card");
      const powerTitle = el("div", "sb-title");
      powerTitle.append(document.createTextNode("Battery & Power Management "));
      powerTitle.appendChild(el("span", "sb-sub", "— SoC · voltage sag · mission time · RTB gating"));
      powerCard.appendChild(powerTitle);
      const powerGrid = el("div", "sb-power-grid");
      powerCard.appendChild(powerGrid);
      const powerNote = el("div", "sb-note");
      powerNote.style.marginTop = "6px";
      powerCard.appendChild(powerNote);
      rightCol.appendChild(powerCard);

      // Confirmation dialog
      let overlay = null;
      function askConfirm(title, message, onConfirm) {
        closeConfirm();
        overlay = el("div", "sb-overlay");
        const modal = el("div", "sb-modal");
        modal.appendChild(el("div", "sb-modal-title", title));
        const body = el("div", null, message);
        body.style.cssText = "font-size:12px;white-space:pre-wrap;line-height:1.45;";
        modal.appendChild(body);
        const row = el("div", "sb-modal-row");
        const cancel = el("button", "sb-btn", "Cancel");
        cancel.style.background = "#4b5563";
        cancel.addEventListener("click", closeConfirm);
        const ok = el("button", "sb-btn", "Confirm");
        ok.style.background = "#dc2626";
        ok.addEventListener("click", () => { closeConfirm(); onConfirm(); });
        row.append(cancel, ok);
        modal.appendChild(row);
        overlay.appendChild(modal);
        root.appendChild(overlay);
      }
      function closeConfirm() {
        if (overlay && overlay.parentNode) overlay.parentNode.removeChild(overlay);
        overlay = null;
      }

      // ── commands ─────────────────────────────────────────────────────────
      function setStatus(text) {
        statusText = `[${new Date().toLocaleTimeString()}] ${text}`;
        statusEl.textContent = statusText;
      }

      function servicesAvailable() {
        return typeof panelContext.callService === "function";
      }

      function callLifecycle(id) {
        const service = `${String(cfg.commanderNs).replace(/\/$/, "")}/${id}`;
        if (!servicesAvailable()) {
          setStatus(`Service calls unavailable in this data source (wanted ${service})`);
          return;
        }
        setStatus(`Calling ${service} ...`);
        Promise.resolve()
          .then(() => panelContext.callService(service, {}))
          .then((res) => {
            const okFlag = res?.success;
            const msg = res?.message ? ` — ${res.message}` : "";
            setStatus(`${service}: ${okFlag === false ? "REJECTED" : "ok"}${msg}`);
          })
          .catch((err) => setStatus(`${service} failed: ${err?.message ?? err}`));
      }

      function onSafetyClick() {
        const t = Date.now() / 1000;
        if (t < safetyArmedUntil) {
          safetyArmedUntil = 0;
          callLifecycle("land");
          setStatus("SAFETY STOP — landing all commanded drones");
        } else {
          safetyArmedUntil = t + SAFETY_ARM_S;
        }
        renderSafety();
      }

      function renderSafety() {
        const remaining = safetyArmedUntil - Date.now() / 1000;
        const armed = remaining > 0;
        stopBtn.classList.toggle("sb-armed", armed);
        stopBtn.textContent = armed
          ? `Confirm — Land All (${Math.ceil(remaining)})`
          : "Safety Stop · Land All";
        stopBtn.title = armed
          ? "Click again to land every commanded drone now"
          : "Two clicks: arm, then confirm. Lands every commanded drone and disarms on touchdown.";
        const ok = servicesAvailable();
        stopBtn.disabled = !ok;
        holdBtn.disabled = !ok;
        if (!armed && safetyArmedUntil !== 0 && remaining <= 0) safetyArmedUntil = 0;
      }

      function sendFormation(nameArg) {
        const value = nameArg || formInput.value.trim();
        if (!value) { setStatus("Enter a formation profile name first"); return; }
        try {
          panelContext.advertise(cfg.formationTopic, "std_msgs/msg/String");
          panelContext.publish(cfg.formationTopic, { data: value });
          formation = value;
          persist();
          setStatus(`Formation "${value}" → ${cfg.formationTopic}`);
        } catch (err) {
          setStatus(`Formation publish failed: ${err?.message ?? err}`);
        }
      }

      // ── render ───────────────────────────────────────────────────────────
      function socColor(soc) {
        if (soc == null) return "#6b7280";
        if (soc < Number(cfg.rtbGatedPct)) return "#dc2626";
        if (soc < Number(cfg.rtbNominalPct)) return "#f59e0b";
        return "#10b981";
      }

      function gradeCell(td, value, target, digits, unit) {
        td.textContent = fmt(value, digits, unit);
        td.className = "";
        if (value == null) { td.classList.add("sb-muted"); return; }
        if (value <= target) td.classList.add("sb-ok");
        else if (value <= target * 2) td.classList.add("sb-warn");
        else td.classList.add("sb-bad");
      }

      function chipCell(state) {
        const td = el("td");
        const chip = el("span", "sb-chip", state.label);
        chip.style.background = state.color;
        chip.style.fontSize = "10px";
        td.appendChild(chip);
        return td;
      }

      function renderWiring() {
        const a = agents.find((x) => x.name === selected);
        wireBody.textContent = "";
        if (!a) {
          wireNote.textContent = "";
          wireBody.appendChild(el("div", "sb-note", "No agent selected."));
          return;
        }
        const w = wiringFor(a);
        const rows = [
          ["mode", `${MODES[a.mode].label}  (${a.modeSource})`],
          ["state", w.state],
          ["cmd", w.command],
          ["service", w.robotCommand],
          ["battery", w.battery],
        ];
        if (a.mode === "real") {
          rows.push(["mocap", w.mocap], ["ekf", w.ekfFlags], ["ping", w.timesync]);
        }
        if (caps.cellular) rows.push(["cellular", w.cellular]);
        for (const [k, v] of rows) {
          if (!v) continue;
          const row = el("div", "sb-wire-row");
          row.append(el("span", "sb-wire-key", k));
          const val = el("span", "sb-wire-val", v);
          if (caps.discovered && k !== "mode" && !available.has(v)) {
            val.classList.add("sb-muted");
            val.title = "not present on this data source";
          }
          row.appendChild(val);
          wireBody.appendChild(row);
        }
        wireNote.textContent = a.modeSource === "detected"
          ? "Mode detected from the topics on the wire. Pin it per agent with the Modes setting."
          : a.modeSource === "config"
            ? "Mode pinned in the panel settings (mirrors swarm_commander's drone_modes)."
            : "No agent topics discovered yet — assuming sim wiring.";
      }

      function renderLinkTable(now, lanes) {
        const cols = ["Agent", "Mode", "Path", "Rate", "Ping", "Drop", "Max gap"];
        if (caps.mocap) cols.push("Mocap age");
        if (caps.ekf) cols.push("EKF");
        cols.push("Clock drift", "State");
        metricsHeadRow.textContent = "";
        for (const c of cols) metricsHeadRow.appendChild(el("th", null, c));

        metricsBody.textContent = "";
        if (!agents.length) {
          const tr = el("tr");
          const td = el("td", "sb-note", "No agents configured.");
          td.colSpan = cols.length;
          tr.appendChild(td);
          metricsBody.appendChild(tr);
          return;
        }

        for (const a of agents) {
          const tr = el("tr");
          if (a.name === selected) tr.className = "sb-selected";
          const m = a.metrics ?? {};

          const tdName = el("td", null, a.name);

          const tdMode = el("td");
          const modeSpan = el("span", `sb-mode ${a.mode}`, MODES[a.mode].label);
          tdMode.appendChild(modeSpan);

          const tdTier = el("td");
          if (a.activeTier === "vpn") {
            tdTier.textContent = "4G/5G VPN";
            tdTier.className = "sb-warn";
          } else if (a.activeTier === "lan") {
            tdTier.textContent = "Wi-Fi / LAN";
            tdTier.className = "sb-ok";
          } else {
            tdTier.textContent = "--";
            tdTier.className = "sb-muted";
          }

          const tdRate = el("td", m.rateHz == null ? "sb-muted" : null, fmt(m.rateHz, 1, " Hz"));

          const tdPing = el("td");
          const pingTarget = a.activeTier === "vpn"
            ? Number(cfg.vpnPingTargetMs) : Number(cfg.pingTargetMs);
          gradeCell(tdPing, m.pingMs, pingTarget, 1, " ms");
          if (m.pingSource) tdPing.appendChild(el("span", "sb-src", m.pingSource));

          const tdDrop = el("td");
          gradeCell(tdDrop, m.dropPct, Number(cfg.dropTargetPct), 2, " %");

          const tdGap = el("td");
          gradeCell(tdGap, m.maxGapMs, Number(cfg.stateGapTargetMs), 0, " ms");
          tdGap.title =
            "Longest single gap in the window. The commander holds zero velocity once "
            + "odometry is older than state_timeout_s, so this is the number that maps "
            + "to the drone actually stopping — a loss percentage cannot see it.";

          tr.append(tdName, tdMode, tdTier, tdRate, tdPing, tdDrop, tdGap);

          if (caps.mocap) {
            const td = el("td");
            const est = a.estimate;
            if (a.mode !== "real" || !est?.mocapSeen) {
              td.textContent = "--";
              td.className = "sb-muted";
              td.title = a.mode !== "real" ? "sim agent: PX4 SITL estimates its own state"
                : "no mocap samples received";
            } else if (!est.mocapFresh) {
              td.textContent = `LOST ${fmt(est.mocapSilentS, 1, " s")}`;
              td.className = "sb-bad";
              td.title = "no mocap sample inside the timeout — PX4 has no external position source";
            } else {
              gradeCell(td, est.mocapAgeMs, Number(cfg.mocapAgeTargetMs), 1, " ms");
              td.appendChild(el("span", "sb-src", fmt(est.mocapRateHz, 0, " Hz")));
              td.title = "rx time minus mocap header stamp — the delay of the state estimate PX4 fuses";
            }
            tr.appendChild(td);
          }

          if (caps.ekf) {
            const est = a.estimate;
            const td = chipCell(est?.state ?? EKF_STATE.NO_DATA);
            if (est?.detail?.length) td.title = est.detail.join(" · ");
            tr.appendChild(td);
          }

          const tdDrift = el("td");
          const drift = m.clockDriftMsPerMin;
          tdDrift.textContent = drift == null ? "--" : `${drift >= 0 ? "+" : ""}${drift.toFixed(2)} ms/min`;
          tdDrift.className = drift == null ? "sb-muted"
            : Math.abs(drift) <= 1 ? "sb-ok" : Math.abs(drift) <= 5 ? "sb-warn" : "sb-bad";
          // A steadily ramping offset on a SITL agent is usually not a clock fault:
          // PX4's boot clock is the simulator's, so a sim running below real time
          // separates from the GCS at exactly (1 - RTF). Measured here at RTF 0.83,
          // which is ~10 s/min of separation and entirely expected.
          tdDrift.title =
            (m.clockOffsetMs != null ? `offset ${m.clockOffsetMs.toFixed(2)} ms. ` : "")
            + (a.mode === "sim"
                ? "On a sim agent a large steady drift is the simulator's real-time "
                  + "factor (PX4's clock is sim time), not a link fault. RTF \u2248 "
                  + "1 - drift/60000 per minute."
                : "Clock separation between the autopilot and the ground station. "
                  + "A ramping value invalidates any ping derived from arrival times.");
          tr.appendChild(tdDrift);

          tr.appendChild(chipCell(a.linkState));

          tr.addEventListener("click", () => { selected = a.name; persist(); render(); });
          tr.style.cursor = "pointer";
          metricsBody.appendChild(tr);
        }

        const lanesLabel = lanes.map((l) => l.label).join(" + ");
        const vpnCount = agents.filter((a) => a.tiers.vpn.everSeen).length;
        targetsNote.textContent =
          `Targets: ping < ${cfg.pingTargetMs} ms on Wi-Fi/LAN, < ${cfg.vpnPingTargetMs} ms on the ` +
          `4G/5G VPN, packet drop < ${cfg.dropTargetPct}%, longest state gap < ${cfg.stateGapTargetMs} ms` +
          (caps.mocap ? `, mocap age < ${cfg.mocapAgeTargetMs} ms` : "") + ". " +
          "Ping prefers a measured round-trip — PX4's uXRCE-DDS timesync on real agents, the " +
          "Tailscale reporter on the VPN path — and falls back to twice the mean arrival delay, " +
          "which is only valid while the clocks agree (read it against the drift column). " +
          `Drop is over a ${METRIC_WINDOW_S}s window and counts a gap only from ≥2x the ` +
          `nominal period, so publisher jitter is not reported as loss; the max-gap column is the ` +
          `blackout measure and LINK goes LOST after ${LINK_LOSS_TIMEOUT_S}s of silence, matching ` +
          `state_timeout_s. Transports in use: ${lanesLabel || "none"}` +
          (caps.vpnLane ? ` · VPN provisioned on ${vpnCount}/${agents.length} agents.` : ".");
      }

      function renderCellular(now) {
        cellBody.textContent = "";
        let anyReport = false;
        for (const a of agents) {
          const c = freshReport(a.cellular, a.cellularAt, now);
          if (c) anyReport = true;
          const tr = el("tr");
          if (a.name === selected) tr.className = "sb-selected";

          const tdName = el("td", null, a.name);

          const tier = c?.tier ?? c?.network ?? null;
          const tdTier = el("td", tier ? null : "sb-muted", tier ?? "--");

          // Prefer the reporter's own measurement; fall back to the arrival
          // statistics of whatever DDS traffic is riding the VPN topic.
          const tdPing = el("td");
          const reported = num(c?.rtt_ms);
          const derived = streamDerivedRttMs(a.tiers.vpn, now);
          const value = reported != null ? reported : derived;
          gradeCell(tdPing, value, Number(cfg.vpnPingTargetMs), 1, " ms");
          if (value != null) {
            tdPing.appendChild(el("span", "sb-src", reported != null ? "tailscale" : "derived"));
          }

          const path = c?.state ?? c?.path ?? null;
          const tdPath = el("td", null, path ?? "--");
          if (path == null) tdPath.className = "sb-muted";
          else if (String(path).toLowerCase() === "direct") tdPath.className = "sb-ok";
          else if (String(path).toLowerCase() === "relay") {
            tdPath.className = "sb-warn";
            tdPath.title = "Tailscale is relaying via DERP — expect the higher latency";
          } else tdPath.className = "sb-bad";

          const signal = num(c?.rsrp_dbm) ?? num(c?.rssi_dbm);
          const tdSignal = el("td", signal == null ? "sb-muted" : null,
            signal == null ? "--" : `${signal.toFixed(0)} dBm`);

          const iface = c?.interface ?? c?.iface ?? null;
          const tdIface = el("td", iface ? null : "sb-muted", iface ?? "--");

          const age = a.cellularAt == null ? null : now - a.cellularAt;
          const tdAge = el("td", age == null ? "sb-muted" : null,
            age == null ? "--" : fmt(age, 1, " s"));
          if (age != null && age > STALE_REPORT_S) tdAge.className = "sb-bad";

          tr.append(tdName, tdTier, tdPing, tdPath, tdSignal, tdIface, tdAge);
          tr.addEventListener("click", () => { selected = a.name; persist(); render(); });
          tr.style.cursor = "pointer";
          cellBody.appendChild(tr);
        }
        if (!agents.length) {
          const tr = el("tr");
          const td = el("td", "sb-note", "No agents configured.");
          td.colSpan = cellCols.length;
          tr.appendChild(td);
          cellBody.appendChild(tr);
        }

        cellNote.textContent = anyReport
          ? `Reported on ${cfg.cellularTopicTemplate} (std_msgs/String, JSON). ` +
            "\"relay\" means Tailscale could not hole-punch and is bouncing through a DERP " +
            "server — the extra hop is the latency you are seeing."
          : `No cellular reporter publishing yet. Have each agent publish JSON on ` +
            `${cfg.cellularTopicTemplate} with any of: tier (\"5G\"/\"LTE\"), rtt_ms ` +
            `(a tailscale ping to the GCS), state (\"direct\"/\"relay\"), rsrp_dbm, interface. ` +
            "VPN ping falls back to the arrival statistics of DDS traffic on " +
            `${cfg.vpnTopicTemplate} when no reporter is running.`;
      }

      function renderPower() {
        powerGrid.textContent = "";
        for (const a of agents) {
          const p = a.power ?? { state: RTB_STATE.NO_DATA };
          const card = el("div", "sb-power");
          if (a.name === selected) card.classList.add("sb-selected");

          const head = el("div", "sb-agent-top");
          head.append(
            el("span", "sb-agent-name", a.name),
            el("span", `sb-mode ${a.mode}`, MODES[a.mode].label),
          );
          const rtbChip = el("span", "sb-chip", p.state.label);
          rtbChip.style.background = p.state.color;
          rtbChip.style.fontSize = "10px";
          head.appendChild(rtbChip);
          card.appendChild(head);

          // SoC bar with the RTB threshold ticks marked on it.
          const bar = el("div", "sb-bar sb-bar-lg");
          const fill = el("div", "sb-bar-fill");
          fill.style.width = `${clamp(p.soc ?? 0, 0, 100)}%`;
          fill.style.background = socColor(p.soc);
          bar.appendChild(fill);
          for (const pct of [Number(cfg.rtbGatedPct), Number(cfg.rtbNominalPct)]) {
            const tick = el("div", "sb-bar-tick");
            tick.style.left = `${clamp(pct, 0, 100)}%`;
            tick.title = `${pct}% threshold`;
            bar.appendChild(tick);
          }
          card.appendChild(bar);

          const kv = (k, v, cls) => {
            const row = el("div", "sb-kv");
            row.append(el("span", null, k), el("span", cls, v));
            card.appendChild(row);
          };
          kv("State of charge", p.soc == null ? "--" : `${p.soc.toFixed(1)} %`);
          kv("Voltage", p.voltage == null ? "--"
            : `${p.voltage.toFixed(2)} V${p.cells ? ` (${p.cells}S)` : ""}`);
          kv("Sag now / peak",
            p.sag == null ? "--"
              : `${p.sag.toFixed(2)} V / ${(p.sagPeak ?? 0).toFixed(2)} V` +
                (p.sagPerCell != null ? ` · ${p.sagPerCell.toFixed(3)} V/cell` : ""),
            p.sag != null && p.sagPerCell != null && p.sagPerCell > 0.15 ? "sb-warn" : null);
          kv("Draw", p.current == null ? "--" : `${p.current.toFixed(1)} A`);
          kv("Mission time left", fmtDuration(p.missionTime));
          kv("Distance to pad", p.distance == null ? "--" : `${p.distance.toFixed(1)} m`);
          kv("Return budget",
            p.returnPct == null ? "--"
              : `${p.returnPct.toFixed(1)} % (${fmtDuration(p.returnTime)})`);
          kv("Energy margin",
            p.margin == null ? "--" : `${p.margin >= 0 ? "+" : ""}${p.margin.toFixed(1)} %`,
            p.margin == null ? null : p.margin < 0 ? "sb-bad" : p.margin < 10 ? "sb-warn" : "sb-ok");

          if (p.state === RTB_STATE.FAILSAFE || p.state === RTB_STATE.GATED || p.state === RTB_STATE.RTB_NOW) {
            const act = el("button", "sb-btn", p.state === RTB_STATE.FAILSAFE ? "Failsafe: Land All" : "Land All");
            act.style.cssText = "background:" + p.state.color + ";padding:5px 10px;font-size:11px;";
            act.title = `${a.name}: ${p.state.label}. The commander lands the whole swarm — there is no per-agent land service.`;
            act.addEventListener("click", () =>
              askConfirm("Land All", `${a.name} is ${p.state.label}.\n\nLand every commanded drone?`,
                () => callLifecycle("land")));
            card.appendChild(act);
          }
          powerGrid.appendChild(card);
        }
        if (!agents.length) {
          powerGrid.appendChild(el("div", "sb-note", "No agents configured."));
        }

        powerNote.textContent =
          `RTB gating: nominal flight above ${cfg.rtbNominalPct}%, conservative maneuver gating ` +
          `${cfg.rtbGatedPct}-${cfg.rtbNominalPct}%, mandatory failsafe landing below ${cfg.rtbGatedPct}%. ` +
          `"RTB NOW" additionally fires when SoC drops to the distance-to-pad energy budget ` +
          `(cruise ${cfg.cruiseSpeedMps} m/s + descent ${cfg.landSpeedMps} m/s at the measured burn rate, ` +
          `plus a ${cfg.reservePct}% reserve). Sag is measured against the highest open-circuit voltage seen this session.`;
      }

      function render() {
        const now = nowSec();
        for (const a of agents) {
          evaluateLink(a, cfg, now);
          evaluateEstimate(a, cfg, now);
          evaluatePower(a, cfg, now);
        }

        renderSafety();

        // Section visibility follows the topics actually on the wire.
        show(cellCard, caps.cellular);
        show(powerCard, caps.battery);
        show(formRow, caps.formation);
        const lanes = TIERS.filter((t) => t.id === "lan" || caps.vpnLane);
        commSub.textContent = caps.mocap || caps.ekf
          ? "— ping · mocap delay · EKF status · transport"
          : "— ping · packet drop · max gap · transport";

        // Banner
        const worstLink = worst(agents.map((a) => a.linkState), LINK_STATE);
        linkChip.textContent = `LINK ${worstLink.label}`;
        linkChip.style.background = worstLink.color;

        show(estChip, caps.ekf || caps.mocap);
        if (caps.ekf || caps.mocap) {
          const realAgents = agents.filter((a) => a.mode === "real");
          const worstEst = worst(realAgents.map((a) => a.estimate?.state), EKF_STATE);
          estChip.textContent = `EKF ${worstEst.label}`;
          estChip.style.background = worstEst.color;
        }

        show(powerChip, caps.battery);
        if (caps.battery) {
          const worstPower = worst(agents.map((a) => a.power?.state), RTB_STATE);
          powerChip.textContent = `POWER ${worstPower.label}`;
          powerChip.style.background = worstPower.color;
        }

        const sims = agents.filter((a) => a.mode === "sim").length;
        modeChip.textContent = `${sims} sim · ${agents.length - sims} real`;
        const tasks = detectedTasks();
        show(taskChip, tasks.length > 0);
        taskChip.textContent = `Tasks: ${tasks.join(", ")}`;
        taskChip.title = caps.discovered
          ? "Inferred from the topics on this data source; sections with no source are hidden."
          : "No topic list from this data source — showing every section.";

        const airborne = agents.filter((a) => a.pos && a.pos[2] > 0.3).length;
        clockEl.textContent = `${airborne}/${agents.length} airborne · ${clockStamp(now)}`;

        // Roster
        for (const a of agents) {
          const r = rosterRows.get(a.name);
          if (!r) continue;
          r.row.classList.toggle("sb-selected", a.name === selected);
          r.dot.style.background = a.linkState.color;
          r.mode.className = `sb-mode ${a.mode}`;
          r.mode.textContent = MODES[a.mode].label;
          const soc = a.power?.soc;
          r.fill.style.width = `${clamp(soc ?? 0, 0, 100)}%`;
          r.fill.style.background = socColor(soc);
          const ping = a.metrics?.pingMs;
          const parts = [
            soc == null ? "-- %" : `${soc.toFixed(0)}%`,
            a.linkState.label,
            ping == null ? "-- ms" : `${ping.toFixed(0)} ms`,
          ];
          if (a.mode === "real" && caps.ekf && a.estimate?.state) {
            parts.push(a.estimate.state.label);
          }
          r.meta.textContent = parts.join(" · ");
        }

        renderWiring();

        // Topology — reparsing SVG markup 5x/s is wasteful, so only redraw when
        // the picture would actually change.
        const topoKey = agents.map((a) =>
          `${a.name}:${a.mode}:${a.linkState.label}:${a.activeTier}:` +
          `${a.tiers.lan.everSeen}${a.tiers.vpn.everSeen}`).join("|")
          + `#${lanes.length}`;
        if (topoKey !== lastTopoKey) {
          lastTopoKey = topoKey;
          topoBox.innerHTML = agents.length
            ? topologySvg(agents, lanes)
            : '<div class="sb-note">No agents configured.</div>';
        }

        renderLinkTable(now, lanes);

        // Transition log — newest first, across the whole swarm.
        const events = [];
        for (const a of agents) {
          for (const t of a.transitions) events.push({ ...t, name: a.name });
        }
        events.sort((x, y) => y.t - x.t);
        logBox.textContent = events.length
          ? events.slice(0, 40).map((e) =>
              `${clockStamp(e.t)}  ${e.name.padEnd(10)} ${e.from} → ${e.to}  [${e.tier}]`).join("\n")
          : "No link state transitions recorded yet.";

        if (caps.cellular) renderCellular(now);
        if (caps.battery) renderPower();

        statusEl.textContent = statusText;
      }

      // ── settings ─────────────────────────────────────────────────────────
      const NUMERIC = new Set([
        "cruiseSpeedMps", "landSpeedMps", "reservePct", "rtbNominalPct",
        "rtbGatedPct", "dropTargetPct", "stateGapTargetMs", "pingTargetMs", "vpnPingTargetMs",
        "mocapAgeTargetMs", "mocapTimeoutS",
      ]);
      const ROSTER_KEYS = new Set([
        "drones", "modes", "stateTopicTemplate", "lanTopicTemplate",
        "vpnTopicTemplate", "simBatteryTopicTemplate", "realBatteryTopicTemplate",
        "mocapTopicTemplate", "ekfFlagsTopicTemplate", "localPositionTopicTemplate",
        "timesyncTopicTemplate", "cellularTopicTemplate", "linkStatusTopicTemplate",
        "positionOffsets",
      ]);
      const CAPS_KEYS = new Set([
        "sections", "formationTopic", "teleopTopicTemplate", "goalTopicTemplate",
        "simCommandTopicTemplate", "realCommandTopicTemplate",
        "simRobotCommandTemplate", "realRobotCommandTemplate",
      ]);

      function updateSettingsEditor() {
        panelContext.updatePanelSettingsEditor({
          actionHandler: (action) => {
            if (action.action !== "update") return;
            const key = action.payload.path[action.payload.path.length - 1];
            if (!(key in DEFAULTS)) return;
            cfg[key] = NUMERIC.has(key) ? Number(action.payload.value) : String(action.payload.value ?? "");
            persist();
            if (ROSTER_KEYS.has(key)) rebuildAgents();
            else if (CAPS_KEYS.has(key)) recomputeCaps();
            updateSettingsEditor();
            render();
          },
          nodes: {
            swarm: {
              label: "Swarm",
              fields: {
                drones: { label: "Agents", input: "string", value: cfg.drones,
                  help: "Comma-separated agent names, in drone_names order" },
                modes: { label: "Modes (wiring)", input: "string", value: cfg.modes,
                  help: "Comma-separated sim|real, one per agent — mirrors swarm_commander's " +
                        "drone_modes. Blank entries are detected from the topics on the wire." },
                sections: { label: "Sections", input: "select", value: cfg.sections,
                  options: [
                    { label: "Auto (hide with no topic)", value: "auto" },
                    { label: "Show all", value: "all" },
                  ],
                  help: "Auto hides any section whose topics are not being published" },
                commanderNs: { label: "Commander namespace", input: "string", value: cfg.commanderNs,
                  help: "std_srvs/Trigger lifecycle services live under this namespace" },
                formationTopic: { label: "Formation topic", input: "string", value: cfg.formationTopic },
              },
            },
            simWiring: {
              label: "Wiring — sim",
              fields: {
                simCommandTopicTemplate: { label: "Velocity command", input: "string", value: cfg.simCommandTopicTemplate },
                simRobotCommandTemplate: { label: "Robot command", input: "string", value: cfg.simRobotCommandTemplate },
                simBatteryTopicTemplate: { label: "Battery (MAVROS)", input: "string", value: cfg.simBatteryTopicTemplate },
              },
            },
            realWiring: {
              label: "Wiring — real",
              fields: {
                realCommandTopicTemplate: { label: "Velocity command", input: "string", value: cfg.realCommandTopicTemplate },
                realRobotCommandTemplate: { label: "Robot command", input: "string", value: cfg.realRobotCommandTemplate },
                realBatteryTopicTemplate: { label: "Battery (PX4)", input: "string", value: cfg.realBatteryTopicTemplate },
                mocapTopicTemplate: { label: "Mocap pose", input: "string", value: cfg.mocapTopicTemplate,
                  help: "geometry_msgs/PoseStamped — mocap_bridge's input; its age is the state-estimate delay" },
                ekfFlagsTopicTemplate: { label: "EKF flags", input: "string", value: cfg.ekfFlagsTopicTemplate,
                  help: "px4_msgs/EstimatorStatusFlags — cs_ev_* tells you EKF2 is fusing mocap" },
                localPositionTopicTemplate: { label: "Local position", input: "string", value: cfg.localPositionTopicTemplate,
                  help: "px4_msgs/VehicleLocalPosition — xy_valid / z_valid / heading_good_for_control" },
                simTimesyncTopicTemplate: { label: "Timesync topic (sim)", input: "string",
                  value: cfg.simTimesyncTopicTemplate,
                  help: "mavros_msgs/TimesyncStatus — measured MAVLink round-trip. Needs timesync_mode: MAVLINK in px4_config.yaml." },
                timesyncTopicTemplate: { label: "Timesync (ping)", input: "string", value: cfg.timesyncTopicTemplate,
                  help: "px4_msgs/TimesyncStatus — round_trip_time is a measured link RTT" },
              },
            },
            transports: {
              label: "Transports",
              fields: {
                stateTopicTemplate: { label: "State", input: "string", value: cfg.stateTopicTemplate },
                lanTopicTemplate: { label: "Wi-Fi / LAN path", input: "string", value: cfg.lanTopicTemplate,
                  help: "A stamped message carried on the local network path" },
                vpnTopicTemplate: { label: "4G/5G VPN path", input: "string", value: cfg.vpnTopicTemplate,
                  help: "A stamped message carried over the Tailscale VPN path" },
                cellularTopicTemplate: { label: "Cellular report", input: "string", value: cfg.cellularTopicTemplate,
                  help: "std_msgs/String JSON: tier, rtt_ms, state (direct|relay), rsrp_dbm, interface" },
                linkStatusTopicTemplate: { label: "Link report (optional)", input: "string", value: cfg.linkStatusTopicTemplate,
                  help: "std_msgs/String JSON; drop_rate / rtt_ms / clock_offset_ms / clock_drift_ms / active_tier override the derived values" },
                teleopTopicTemplate: { label: "Teleop (detect only)", input: "string", value: cfg.teleopTopicTemplate },
                goalTopicTemplate: { label: "Goal (detect only)", input: "string", value: cfg.goalTopicTemplate },
              },
            },
            linkSafety: {
              label: "Link safety targets",
              fields: {
                pingTargetMs: { label: "Ping target, LAN (ms)", input: "number", value: cfg.pingTargetMs, step: 1 },
                vpnPingTargetMs: { label: "Ping target, VPN (ms)", input: "number", value: cfg.vpnPingTargetMs, step: 5 },
                dropTargetPct: { label: "Packet drop target (%)", input: "number", value: cfg.dropTargetPct, step: 0.1 },
                stateGapTargetMs: { label: "Max state gap (ms)", input: "number", value: cfg.stateGapTargetMs, step: 10,
                  help: "Longest tolerated single gap in the state stream. Half of swarm_commander's state_timeout_s by default, so it warns at half the budget before the commander zeroes velocity." },
                mocapAgeTargetMs: { label: "Mocap age target (ms)", input: "number", value: cfg.mocapAgeTargetMs, step: 5 },
                mocapTimeoutS: { label: "Mocap loss timeout (s)", input: "number", value: cfg.mocapTimeoutS, step: 0.1 },
              },
            },
            power: {
              label: "Power & RTB",
              fields: {
                padPosition: { label: "Landing pad (x,y,z)", input: "string", value: cfg.padPosition },
                positionOffsets: { label: "Position offsets", input: "string", value: cfg.positionOffsets,
                  help: "Flat x,y,z per agent, matching drone_position_offsets; blank = none" },
                cruiseSpeedMps: { label: "Cruise speed (m/s)", input: "number", value: cfg.cruiseSpeedMps, step: 0.1 },
                landSpeedMps: { label: "Land speed (m/s)", input: "number", value: cfg.landSpeedMps, step: 0.1 },
                reservePct: { label: "Reserve (%)", input: "number", value: cfg.reservePct, step: 1 },
                rtbNominalPct: { label: "Nominal above (%)", input: "number", value: cfg.rtbNominalPct, step: 1 },
                rtbGatedPct: { label: "Failsafe below (%)", input: "number", value: cfg.rtbGatedPct, step: 1 },
              },
            },
          },
        });
      }

      // ── boot ─────────────────────────────────────────────────────────────
      panelContext.setDefaultPanelTitle("SVG Basestation");
      rebuildAgents();
      updateSettingsEditor();
      render();

      const timer = setInterval(render, UI_REFRESH_MS);

      // Foxglove throttles JS while the browser tab is hidden, so latched /
      // TRANSIENT_LOCAL samples can be dropped from the queue during the gap.
      // Re-subscribing on resume replays them (same approach as robot-commands).
      const onVisibilityChange = () => {
        if (typeof document !== "undefined" && !document.hidden) {
          rebuildSubscriptions();
          render();
        }
      };
      if (typeof document !== "undefined") {
        document.addEventListener("visibilitychange", onVisibilityChange);
      }

      return () => {
        clearInterval(timer);
        if (typeof document !== "undefined") {
          document.removeEventListener("visibilitychange", onVisibilityChange);
        }
        closeConfirm();
        byTopic.clear();
        panelContext.subscribe([]);
        root.classList.remove("sb-root");
      };
    },
  });
}

module.exports = { activate };
})();
