(() => {
"use strict";

// ─────────────────────────── constants ────────────────────────────────────────

const GOAL_STATUS = {
  UNKNOWN: 0,
  ACCEPTED: 1,
  EXECUTING: 2,
  CANCELING: 3,
  SUCCEEDED: 4,
  CANCELED: 5,
  ABORTED: 6,
};

const TERMINAL_STATUSES = new Set([
  GOAL_STATUS.SUCCEEDED,
  GOAL_STATUS.CANCELED,
  GOAL_STATUS.ABORTED,
]);

// Topic the Waypoint Editor publishes its current list on (std_msgs/String JSON).
const EDITOR_LIST_TOPIC = "/gcs/waypoints/list";

// Topic the Polygon Editor publishes its current vertex list on.
const POLYGON_LIST_TOPIC = "/gcs/polygon/list";

// Module-level caches of latest editor data, refreshed in the panel's onRender.
// The "Grab from Editor" button on each polygon/path field copies these into
// the textbox.
let editorWaypointsCache = [];
let polygonVerticesCache = [];

// Per-tab runtime (feedback lines, last result, status, active flag) lives at
// module scope so it survives Foxglove panel re-instantiation (layout swaps,
// dock changes). It is only cleared explicitly by Execute via
// resetRuntimeForNewGoal.
// Keyed by `${robot}::${tabId}` so each robot's panel has its own runtime —
// otherwise a goal running for robot_1 would color robot_2's panel too.
const perTabRuntime = {};
function getPerTab(robot, tabId) {
  const key = `${robot}::${tabId}`;
  if (!perTabRuntime[key]) {
    perTabRuntime[key] = {
      feedbackLines: [],
      resultText: "",
      statusText: "Idle",
      statusCode: 0,  // GOAL_STATUS.UNKNOWN
      active: false,
    };
  }
  return perTabRuntime[key];
}

// JSON-encode the cached editor list as `[[x,y,z], ...]` for the textbox.
function editorCacheToJson(cache) {
  return JSON.stringify((cache ?? []).map((p) => [p.x, p.y, p.z]));
}

const TRAJECTORY_DEFAULTS = {
  Circle:    [["frame_id","base_link"],["radius","5.0"],["velocity","2.0"]],
  Figure8:   [["frame_id","base_link"],["length","10.0"],["width","5.0"],["height","0.0"],["velocity","2.0"],["max_acceleration","1.0"]],
  Racetrack: [["frame_id","base_link"],["length","20.0"],["width","10.0"],["height","0.0"],["velocity","2.0"],["turn_velocity","1.0"],["max_acceleration","1.0"]],
  Line:      [["frame_id","base_link"],["length","10.0"],["height","0.0"],["velocity","2.0"],["max_acceleration","1.0"]],
  Point:     [["frame_id","base_link"],["x","5.0"],["y","0.0"],["height","0.0"],["velocity","2.0"],["max_acceleration","1.0"]],
  Lawnmower: [["frame_id","base_link"],["length","20.0"],["width","5.0"],["height","10.0"],["velocity","2.0"],["vertical","0"]],
};

// Shared altitude/speed field sets used across several area tasks.
const altSpeedFields = [
  { name: "min_altitude_agl",  kind: "float", default: 3.0,  min: 0,   max: 500, step: 0.1 },
  { name: "max_altitude_agl",  kind: "float", default: 10.0, min: 0,   max: 500, step: 0.1 },
  { name: "min_flight_speed",  kind: "float", default: 1.0,  min: 0,   max: 50,  step: 0.1 },
  { name: "max_flight_speed",  kind: "float", default: 3.0,  min: 0,   max: 50,  step: 0.1 },
];

// Tab registry: drives rendering, goal packing, and feedback formatting.
// Each tab owns its default sub-state (built from `fields`) and a goal builder.
const TASK_TABS = [
  {
    id: "takeoff",
    label: "Takeoff",
    actionSuffix: "tasks/takeoff",
    goalSchema: "task_msgs/TakeoffTask_Goal",
    fields: [
      { name: "target_altitude_m", kind: "float", default: 10.0, min: 0, max: 500, step: 0.1 },
      { name: "velocity_m_s",      kind: "float", default: 1.0,  min: 0, max: 50,  step: 0.1 },
    ],
    buildGoal: (s) => ({
      target_altitude_m: numOr(s.target_altitude_m, 0),
      velocity_m_s:      numOr(s.velocity_m_s, 0),
    }),
    formatFeedback: (fb) =>
      `status: ${fb.status ?? ""} | alt: ${toFixed(fb.current_altitude_m)} / ${toFixed(fb.target_altitude_m)} m`,
  },
  {
    id: "land",
    label: "Land",
    actionSuffix: "tasks/land",
    goalSchema: "task_msgs/LandTask_Goal",
    fields: [
      { name: "velocity_m_s", kind: "float", default: 0.3, min: 0, max: 10, step: 0.1, hint: "0 = use config default" },
    ],
    buildGoal: (s) => ({ velocity_m_s: numOr(s.velocity_m_s, 0) }),
    formatFeedback: (fb) =>
      `status: ${fb.status ?? ""} | alt: ${toFixed(fb.current_altitude_m)} m`,
  },
  {
    id: "navigate",
    label: "Navigate",
    actionSuffix: "tasks/navigate",
    goalSchema: "task_msgs/NavigateTask_Goal",
    fields: [
      { name: "frame_id",         kind: "string", default: "map" },
      { name: "waypoints",        kind: "path",   default: "[[0.0, 0.0, 5.0]]",
        hint: "JSON array of [x, y, z] waypoints. Use 'Grab from Editor' to copy in the Waypoint Editor's current list." },
      { name: "goal_tolerance_m", kind: "float",  default: 1.0, min: 0, max: 100, step: 0.1 },
    ],
    buildGoal: (s) => ({
      global_plan: buildPath(s.frame_id, s.waypoints),
      goal_tolerance_m: numOr(s.goal_tolerance_m, 0),
    }),
    formatFeedback: genericAreaFeedback,
  },
  {
    id: "exploration",
    label: "Exploration",
    actionSuffix: "tasks/exploration",
    goalSchema: "task_msgs/ExplorationTask_Goal",
    fields: [
      { name: "search_bounds", kind: "polygon", default: "[]",
        hint: "JSON array of [x, y, z] vertices ([] = unbounded). Use 'Grab from Editor' to copy in the Polygon Editor's current list." },
      ...altSpeedFields,
      { name: "time_limit_sec", kind: "float", default: 120.0, min: 0, max: 86400, step: 1 },
    ],
    buildGoal: (s) => ({
      search_bounds: buildPolygon(s.search_bounds),
      min_altitude_agl: numOr(s.min_altitude_agl, 0),
      max_altitude_agl: numOr(s.max_altitude_agl, 0),
      min_flight_speed: numOr(s.min_flight_speed, 0),
      max_flight_speed: numOr(s.max_flight_speed, 0),
      time_limit_sec:   numOr(s.time_limit_sec, 0),
    }),
    formatFeedback: genericAreaFeedback,
  },
  {
    id: "coverage",
    label: "Coverage",
    actionSuffix: "tasks/coverage",
    goalSchema: "task_msgs/CoverageTask_Goal",
    fields: [
      { name: "coverage_area", kind: "polygon", default: "[]",
        hint: "JSON array of [x, y, z] vertices. Use 'Grab from Editor' to copy in the Polygon Editor's current list." },
      ...altSpeedFields,
      { name: "line_spacing_m", kind: "float", default: 5.0, min: 0.1, max: 1000, step: 0.1 },
      { name: "heading_deg",    kind: "float", default: 0.0, min: 0,   max: 360,  step: 1 },
    ],
    buildGoal: (s) => ({
      coverage_area: buildPolygon(s.coverage_area),
      min_altitude_agl: numOr(s.min_altitude_agl, 0),
      max_altitude_agl: numOr(s.max_altitude_agl, 0),
      min_flight_speed: numOr(s.min_flight_speed, 0),
      max_flight_speed: numOr(s.max_flight_speed, 0),
      line_spacing_m:   numOr(s.line_spacing_m, 0),
      heading_deg:      numOr(s.heading_deg, 0),
    }),
    formatFeedback: (fb) => {
      const base = genericAreaFeedback(fb);
      const cov = fb.coverage_percentage;
      return cov != null ? `${base} | coverage: ${toFixed(cov, 1)}%` : base;
    },
  },
  {
    id: "object_search",
    label: "Object Search",
    actionSuffix: "tasks/object_search",
    goalSchema: "task_msgs/ObjectSearchTask_Goal",
    fields: [
      { name: "object_class", kind: "string", default: "" },
      { name: "search_area",  kind: "polygon", default: "[]",
        hint: "JSON array of [x, y, z] vertices. Use 'Grab from Editor' to copy in the Polygon Editor's current list." },
      ...altSpeedFields,
      { name: "time_limit_sec", kind: "float", default: 120.0, min: 0, max: 86400, step: 1 },
      { name: "target_count",   kind: "int",   default: 1,     min: 0, max: 10000, step: 1,
        hint: "0 = find all within area/time" },
    ],
    buildGoal: (s) => ({
      object_class: String(s.object_class ?? ""),
      search_area:  buildPolygon(s.search_area),
      min_altitude_agl: numOr(s.min_altitude_agl, 0),
      max_altitude_agl: numOr(s.max_altitude_agl, 0),
      min_flight_speed: numOr(s.min_flight_speed, 0),
      max_flight_speed: numOr(s.max_flight_speed, 0),
      time_limit_sec: numOr(s.time_limit_sec, 0),
      target_count:   intOr(s.target_count, 0),
    }),
    formatFeedback: (fb) => {
      const base = genericAreaFeedback(fb);
      const n = fb.objects_found_so_far;
      return n != null ? `${base} | found: ${n}` : base;
    },
  },
  {
    id: "object_counting",
    label: "Object Counting",
    actionSuffix: "tasks/object_counting",
    goalSchema: "task_msgs/ObjectCountingTask_Goal",
    fields: [
      { name: "object_class", kind: "string",  default: "" },
      { name: "count_area",   kind: "polygon", default: "[]",
        hint: "JSON array of [x, y, z] vertices. Use 'Grab from Editor' to copy in the Polygon Editor's current list." },
      ...altSpeedFields,
    ],
    buildGoal: (s) => ({
      object_class: String(s.object_class ?? ""),
      count_area:   buildPolygon(s.count_area),
      min_altitude_agl: numOr(s.min_altitude_agl, 0),
      max_altitude_agl: numOr(s.max_altitude_agl, 0),
      min_flight_speed: numOr(s.min_flight_speed, 0),
      max_flight_speed: numOr(s.max_flight_speed, 0),
    }),
    formatFeedback: (fb) => {
      const base = genericAreaFeedback(fb);
      const n = fb.current_count;
      return n != null ? `${base} | count: ${n}` : base;
    },
  },
  {
    id: "semantic_search",
    label: "Semantic Search",
    actionSuffix: "tasks/semantic_search",
    goalSchema: "task_msgs/SemanticSearchTask_Goal",
    fields: [
      { name: "query",              kind: "string", default: "" },
      { name: "background_queries", kind: "string", default: "",
        hint: "comma-separated contrast classes, e.g. building,tree,ground" },
      { name: "search_area",        kind: "polygon", default: "[]",
        hint: "JSON array of [x, y, z] vertices. Use 'Grab from Editor' to copy in the Polygon Editor's current list." },
      { name: "min_altitude_agl",   kind: "float", default: 3.0,  min: 0, max: 500, step: 0.1 },
      { name: "max_altitude_agl",   kind: "float", default: 15.0, min: 0, max: 500, step: 0.1 },
      { name: "min_flight_speed",   kind: "float", default: 1.0,  min: 0, max: 50,  step: 0.1 },
      { name: "max_flight_speed",   kind: "float", default: 3.0,  min: 0, max: 50,  step: 0.1 },
      { name: "confidence_threshold", kind: "float", default: 0.95, min: 0, max: 1, step: 0.01 },
    ],
    buildGoal: (s) => ({
      query: String(s.query ?? ""),
      background_queries: String(s.background_queries ?? ""),
      search_area: buildPolygon(s.search_area),
      min_altitude_agl: numOr(s.min_altitude_agl, 0),
      max_altitude_agl: numOr(s.max_altitude_agl, 0),
      min_flight_speed: numOr(s.min_flight_speed, 0),
      max_flight_speed: numOr(s.max_flight_speed, 0),
      confidence_threshold: numOr(s.confidence_threshold, 0),
    }),
    formatFeedback: (fb) => {
      const base = genericAreaFeedback(fb);
      const c = fb.best_confidence_so_far;
      return c != null ? `${base} | best conf: ${toFixed(c, 2)}` : base;
    },
  },
  {
    id: "fixed_trajectory",
    label: "Fixed Trajectory",
    actionSuffix: "tasks/fixed_trajectory",
    goalSchema: "task_msgs/FixedTrajectoryTask_Goal",
    fields: [],  // custom renderer
    defaultState: () => ({
      type: "Circle",
      attributes: TRAJECTORY_DEFAULTS.Circle.map(([k,v]) => [k,v]),
      loop: false,
    }),
    buildGoal: (s) => ({
      trajectory_spec: {
        type: s.type,
        attributes: (s.attributes ?? []).map(([k,v]) => ({ key: String(k), value: String(v) })),
      },
      loop: Boolean(s.loop),
    }),
    formatFeedback: genericAreaFeedback,
  },
];

function tabById(id) {
  return TASK_TABS.find((t) => t.id === id) ?? TASK_TABS[0];
}

// ─────────────────────────── utilities ────────────────────────────────────────

function numOr(v, d) { const n = Number(v); return Number.isFinite(n) ? n : d; }
function intOr(v, d) { const n = parseInt(v, 10); return Number.isFinite(n) ? n : d; }
function toFixed(v, n = 1) { const x = Number(v); return Number.isFinite(x) ? x.toFixed(n) : "0.0"; }

function genericAreaFeedback(fb) {
  const p = fb.current_position ?? {};
  const prog = fb.progress != null ? ` | progress: ${toFixed(fb.progress, 2)}` : "";
  return `status: ${fb.status ?? ""}${prog} | pos: (${toFixed(p.x)}, ${toFixed(p.y)}, ${toFixed(p.z)})`;
}

function parseJsonPoints(text) {
  // Accepts "[[x,y,z], ...]" or "[]". Returns array of {x,y,z} or throws.
  const parsed = JSON.parse(text);
  if (!Array.isArray(parsed)) throw new Error("must be a JSON array");
  return parsed.map((pt, i) => {
    if (!Array.isArray(pt) || pt.length < 2) {
      throw new Error(`point ${i} must be [x, y] or [x, y, z]`);
    }
    return { x: Number(pt[0]) || 0, y: Number(pt[1]) || 0, z: Number(pt[2]) || 0 };
  });
}

function buildPolygon(text) {
  try {
    const pts = parseJsonPoints(text ?? "[]");
    return { points: pts.map((p) => ({ x: p.x, y: p.y, z: p.z })) };
  } catch {
    return { points: [] };
  }
}

function buildPath(frameId, text) {
  let pts = [];
  try { pts = parseJsonPoints(text ?? "[]"); } catch { pts = []; }
  const header = { stamp: { sec: 0, nanosec: 0 }, frame_id: String(frameId ?? "map") };
  return {
    header,
    poses: pts.map((p) => ({
      header,
      pose: {
        position: { x: p.x, y: p.y, z: p.z },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    })),
  };
}

function randomUuidBytes() {
  const bytes = new Uint8Array(16);
  (globalThis.crypto ?? window.crypto).getRandomValues(bytes);
  return Array.from(bytes);
}

function uuidEqual(a, b) {
  if (!a || !b || a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    if (a[i] !== b[i]) return false;
  }
  return true;
}

function actionTopic(robot, suffix, leaf) {
  return `/${robot}/${suffix}/${leaf}`;
}

function statusLabel(code) {
  switch (code) {
    case GOAL_STATUS.ACCEPTED:  return "Accepted";
    case GOAL_STATUS.EXECUTING: return "Running";
    case GOAL_STATUS.CANCELING: return "Cancelling";
    case GOAL_STATUS.SUCCEEDED: return "Succeeded";
    case GOAL_STATUS.CANCELED:  return "Canceled";
    case GOAL_STATUS.ABORTED:   return "Aborted";
    default:                    return "Unknown";
  }
}

function statusColor(code) {
  switch (code) {
    case GOAL_STATUS.SUCCEEDED: return "#16a34a";
    case GOAL_STATUS.ABORTED:   return "#dc2626";
    case GOAL_STATUS.CANCELED:  return "#ea580c";
    case GOAL_STATUS.CANCELING: return "#ea580c";
    default:                    return "#2563eb";
  }
}

// Build the default sub-state for a tab from its field list (or explicit defaultState).
function defaultTabState(tab) {
  if (tab.defaultState) return tab.defaultState();
  const s = {};
  for (const f of tab.fields) s[f.name] = f.default;
  return s;
}

// ─────────────────────────── panel ────────────────────────────────────────────

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "Robot Tasks",
    initPanel: (panelContext) => {
      // Build default state.
      const DEFAULT_STATE = { robot: "robot_1", activeTab: TASK_TABS[0].id };
      for (const tab of TASK_TABS) DEFAULT_STATE[tab.id] = defaultTabState(tab);

      // Merge persisted state (shallow per tab, so new fields added in code show up).
      const persisted = panelContext.initialState ?? {};
      const state = {
        robot: persisted.robot ?? DEFAULT_STATE.robot,
        activeTab: persisted.activeTab ?? DEFAULT_STATE.activeTab,
      };
      for (const tab of TASK_TABS) {
        state[tab.id] = { ...DEFAULT_STATE[tab.id], ...(persisted[tab.id] ?? {}) };
        // Deep-copy attributes array for fixed_trajectory to avoid sharing refs.
        if (tab.id === "fixed_trajectory" && Array.isArray(state[tab.id].attributes)) {
          state[tab.id].attributes = state[tab.id].attributes.map(([k,v]) => [k,v]);
        }
      }

      // Per-tab runtime is module-scoped (perTabRuntime) so it survives panel
      // re-mounts. Lookups read `state.robot` lazily so changing the robot
      // input switches the panel to that robot's runtime instantly.
      const perTab = new Proxy({}, {
        get(_, key) { return getPerTab(state.robot, String(key)); },
        has(_, key) {
          return Boolean(perTabRuntime[`${state.robot}::${String(key)}`]);
        },
      });
      // Pre-create entries for the current robot.
      for (const tab of TASK_TABS) getPerTab(state.robot, tab.id);
      const runtime = {
        subscribedTopics: [],
      };

      const MAX_FEEDBACK_LINES = 100;

      // Helpers to get the active tab's per-tab state
      function curTab() { return perTab[state.activeTab]; }
      function anyActive() { return TASK_TABS.some((t) => perTab[t.id].active); }

      // ── DOM ───────────────────────────────────────────────────────────────
      const root = panelContext.panelElement;
      root.style.cssText =
        "display:flex;flex-direction:column;height:100%;box-sizing:border-box;padding:8px;gap:8px;font-family:sans-serif;color:inherit;overflow-y:auto;overflow-x:hidden;";

      // Robot row
      const robotRow = document.createElement("div");
      robotRow.style.cssText = "display:flex;align-items:center;gap:6px;flex-shrink:0;";
      const robotLabel = document.createElement("span");
      robotLabel.textContent = "Robot:";
      robotLabel.style.fontWeight = "bold";
      const robotInput = document.createElement("input");
      robotInput.type = "text";
      robotInput.value = state.robot;
      robotInput.style.cssText = "flex:1;padding:4px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
      robotInput.addEventListener("change", () => {
        state.robot = robotInput.value.trim() || "robot_1";
        persist();
        for (const tab of TASK_TABS) getPerTab(state.robot, tab.id);
        rebuildSubscriptions();
        renderTabs();
      });
      robotRow.appendChild(robotLabel);
      robotRow.appendChild(robotInput);
      root.appendChild(robotRow);

      // Tab bar (horizontally scrollable so many tabs fit)
      const tabBar = document.createElement("div");
      tabBar.style.cssText = "display:flex;gap:2px;border-bottom:1px solid #444;overflow-x:auto;flex-shrink:0;";
      const tabButtons = {};
      for (const tab of TASK_TABS) {
        const b = document.createElement("button");
        b.textContent = tab.label;
        b.style.cssText =
          "padding:8px 10px;border:none;background:transparent;color:inherit;cursor:pointer;font-size:13px;border-bottom:2px solid transparent;white-space:nowrap;";
        b.addEventListener("click", () => {
          state.activeTab = tab.id;
          persist();
          renderTabs();
        });
        tabButtons[tab.id] = b;
        tabBar.appendChild(b);
      }
      root.appendChild(tabBar);

      // Tab bodies
      const tabBodies = {};
      for (const tab of TASK_TABS) {
        const body = document.createElement("div");
        body.style.cssText = "display:flex;flex-direction:column;gap:6px;flex-shrink:0;";

        if (tab.id === "fixed_trajectory") {
          body.appendChild(buildFixedTrajectoryForm(state.fixed_trajectory, persist));
        } else {
          // Render simple fields first, then polygon/path (collapsible) at the bottom.
          const isCollapsible = (f) => f.kind === "polygon" || f.kind === "path";
          const simple  = tab.fields.filter((f) => !isCollapsible(f));
          const complex = tab.fields.filter(isCollapsible);
          for (const field of [...simple, ...complex]) {
            body.appendChild(buildField(field, state[tab.id], persist));
          }
        }

        tabBodies[tab.id] = body;
        root.appendChild(body);
      }

      // Feedback area
      const feedbackLabel = document.createElement("div");
      feedbackLabel.textContent = "Feedback";
      feedbackLabel.style.cssText = "font-weight:bold;margin-top:4px;flex-shrink:0;";
      const feedbackBox = document.createElement("div");
      feedbackBox.style.cssText = "min-height:80px;max-height:180px;overflow-y:auto;background:rgba(0,0,0,0.2);border:1px solid #444;border-radius:4px;padding:6px;font-family:monospace;font-size:12px;white-space:pre-wrap;flex-shrink:0;";
      root.appendChild(feedbackLabel);
      root.appendChild(feedbackBox);

      const resultLabel = document.createElement("div");
      resultLabel.textContent = "Result";
      resultLabel.style.cssText = "font-weight:bold;flex-shrink:0;";
      const resultBox = document.createElement("div");
      resultBox.style.cssText = "min-height:32px;background:rgba(0,0,0,0.2);border:1px solid #444;border-radius:4px;padding:6px;font-family:monospace;font-size:12px;white-space:pre-wrap;flex-shrink:0;";
      root.appendChild(resultLabel);
      root.appendChild(resultBox);

      // Status + buttons
      const statusRow = document.createElement("div");
      statusRow.style.cssText = "display:flex;align-items:center;gap:8px;flex-shrink:0;";
      const statusLbl = document.createElement("span");
      statusLbl.style.cssText = "flex:1;font-weight:bold;";
      const cancelBtn = document.createElement("button");
      cancelBtn.textContent = "Cancel";
      cancelBtn.style.cssText = "padding:8px 16px;border-radius:4px;border:none;background:#6b7280;color:white;cursor:pointer;";
      const executeBtn = document.createElement("button");
      executeBtn.textContent = "Execute";
      executeBtn.style.cssText = "padding:8px 16px;border-radius:4px;border:none;background:#10b981;color:white;cursor:pointer;font-weight:bold;";
      statusRow.appendChild(statusLbl);
      statusRow.appendChild(cancelBtn);
      statusRow.appendChild(executeBtn);
      root.appendChild(statusRow);

      // ── helpers ──────────────────────────────────────────────────────────
      function persist() { panelContext.saveState(state); }

      function renderTabs() {
        for (const tab of TASK_TABS) {
          const isActive = tab.id === state.activeTab;
          tabBodies[tab.id].style.display = isActive ? "flex" : "none";
          tabButtons[tab.id].style.borderBottomColor = isActive ? "#10b981" : "transparent";
          tabButtons[tab.id].style.fontWeight = isActive ? "bold" : "normal";
          // Tab text color: yellow while running, green on success, yellow on
          // failure/cancel, inherit otherwise.
          const pt = perTab[tab.id];
          let color = "inherit";
          if (pt.active) {
            color = "#eab308";  // yellow — running
          } else if (pt.statusCode === GOAL_STATUS.SUCCEEDED) {
            color = "#10b981";  // green — succeeded
          } else if (pt.statusCode === GOAL_STATUS.ABORTED ||
                     pt.statusCode === GOAL_STATUS.CANCELED) {
            color = "#eab308";  // yellow — failed / canceled
          }
          tabButtons[tab.id].style.color = color;
        }
        renderStatus();
        renderFeedback();
      }

      function renderStatus() {
        const t = curTab();
        statusLbl.textContent = `Status: ${t.statusText}`;
        statusLbl.style.color = statusColor(t.statusCode);
        // Disable execute if ANY tab has an active goal
        const blocked = anyActive();
        executeBtn.disabled = blocked;
        executeBtn.style.opacity = blocked ? "0.5" : "1";
        cancelBtn.disabled = !t.active;
        cancelBtn.style.opacity = cancelBtn.disabled ? "0.5" : "1";
      }

      function renderFeedback() {
        const t = curTab();
        feedbackBox.textContent = t.feedbackLines.join("\n");
        feedbackBox.scrollTop = feedbackBox.scrollHeight;
        resultBox.textContent = t.resultText;
      }

      function appendFeedback(line, tabId) {
        const tid = tabId ?? state.activeTab;
        const t = perTab[tid];
        if (!t) return;
        const ts = new Date().toLocaleTimeString();
        t.feedbackLines.push(`[${ts}] ${line}`);
        if (t.feedbackLines.length > MAX_FEEDBACK_LINES) {
          t.feedbackLines.splice(0, t.feedbackLines.length - MAX_FEEDBACK_LINES);
        }
        // Only update display if this tab is currently visible
        if (tid === state.activeTab) renderFeedback();
      }

      function resetRuntimeForNewGoal() {
        const t = curTab();
        t.feedbackLines = [];
        t.resultText = "";
        t.statusText = "Sending...";
        t.statusCode = GOAL_STATUS.UNKNOWN;
        renderFeedback();
        renderStatus();
        renderTabs();
      }

      // ── subscriptions (relay uses plain std_msgs/String topics) ────────
      // Subscribe to ALL relay topics for all tabs so we catch results
      // even when the user switches tabs during execution.
      function rebuildSubscriptions() {
        const topics = [EDITOR_LIST_TOPIC, POLYGON_LIST_TOPIC];
        for (const tab of TASK_TABS) {
          if (perTab[tab.id].active) {
            topics.push(`/${state.robot}/${tab.actionSuffix}/relay_feedback`);
            topics.push(`/${state.robot}/${tab.actionSuffix}/relay_result`);
          }
        }
        runtime.subscribedTopics = topics;
        panelContext.subscribe(topics.map((topic) => ({ topic })));
      }
      // Subscribe to the editor topic immediately so the cache is warm before
      // the user clicks Execute on Navigate.
      rebuildSubscriptions();

      // ── execute / cancel ─────────────────────────────────────────────────
      executeBtn.addEventListener("click", async () => {
        // Block if ANY tab has an active goal
        if (anyActive()) return;
        const tab = tabById(state.activeTab);
        const t = perTab[tab.id];
        let goal;
        try {
          goal = tab.buildGoal(state[tab.id]);
        } catch (err) {
          t.statusText = "Invalid goal";
          t.statusCode = GOAL_STATUS.ABORTED;
          t.resultText = `buildGoal failed: ${err?.message ?? err}`;
          renderStatus();
          renderFeedback();
          return;
        }

        t.active = true;
        resetRuntimeForNewGoal();
        rebuildSubscriptions();

        const goalTopic = `/${state.robot}/${tab.actionSuffix}/goal`;
        try {
          panelContext.advertise(goalTopic, "std_msgs/msg/String");
          panelContext.publish(goalTopic, { data: JSON.stringify(goal) });
          t.statusText = "Goal sent";
          t.statusCode = GOAL_STATUS.EXECUTING;
          appendFeedback(`Sent goal: ${JSON.stringify(goal)}`);
          renderStatus();
        } catch (err) {
          t.statusText = "Publish failed";
          t.statusCode = GOAL_STATUS.ABORTED;
          t.resultText = String(err?.message ?? err);
          t.active = false;
          rebuildSubscriptions();
          renderStatus();
          renderFeedback();
        }
      });

      cancelBtn.addEventListener("click", async () => {
        const t = curTab();
        if (!t.active) return;
        const tab = tabById(state.activeTab);
        const cancelTopic = `/${state.robot}/${tab.actionSuffix}/cancel`;
        try {
          panelContext.advertise(cancelTopic, "std_msgs/msg/String");
          panelContext.publish(cancelTopic, { data: "cancel" });
          appendFeedback("Cancel requested");
        } catch (err) {
          appendFeedback(`Cancel failed: ${err?.message ?? err}`);
        }
      });

      // ── incoming messages (relay publishes std_msgs/String as JSON) ────
      // Find which tab owns a given relay topic
      function tabForTopic(topic) {
        for (const tab of TASK_TABS) {
          if (topic.includes(`/${tab.actionSuffix}/`)) return tab;
        }
        return null;
      }

      function handleRelayFeedback(topic, msg) {
        const tab = tabForTopic(topic);
        if (!tab) return;
        const t = perTab[tab.id];
        if (!t.active) return;
        const data = msg?.data;
        if (!data) return;
        try {
          const fb = JSON.parse(data);
          appendFeedback(tab.formatFeedback(fb), tab.id);
        } catch {
          appendFeedback(data, tab.id);
        }
      }

      function handleRelayResult(topic, msg) {
        const tab = tabForTopic(topic);
        if (!tab) return;
        const t = perTab[tab.id];
        if (!t.active) return;
        const data = msg?.data;
        if (!data) return;
        try {
          const result = JSON.parse(data);
          t.resultText = `success: ${result.success}\nmessage: ${result.message ?? ""}`;
          t.statusText = result.success ? "Succeeded" : "Failed";
          t.statusCode = result.success ? GOAL_STATUS.SUCCEEDED : GOAL_STATUS.ABORTED;
        } catch {
          t.resultText = data;
          t.statusText = "Done";
          t.statusCode = GOAL_STATUS.SUCCEEDED;
        }
        t.active = false;
        rebuildSubscriptions();
        renderTabs();
      }

      // ── render loop ──────────────────────────────────────────────────────
      panelContext.onRender = (renderState, done) => {
        const frame = renderState.currentFrame;
        if (frame) {
          for (const evt of frame) {
            if (!runtime.subscribedTopics.includes(evt.topic)) continue;
            if (evt.topic === EDITOR_LIST_TOPIC) {
              try {
                const data = JSON.parse(evt.message?.data ?? "{}");
                editorWaypointsCache = Array.isArray(data.waypoints) ? data.waypoints : [];
              } catch { /* ignore bad data */ }
            } else if (evt.topic === POLYGON_LIST_TOPIC) {
              try {
                const data = JSON.parse(evt.message?.data ?? "{}");
                polygonVerticesCache = Array.isArray(data.vertices) ? data.vertices : [];
              } catch { /* ignore bad data */ }
            } else if (evt.topic.endsWith("/relay_feedback")) {
              handleRelayFeedback(evt.topic, evt.message);
            } else if (evt.topic.endsWith("/relay_result")) {
              handleRelayResult(evt.topic, evt.message);
            }
          }
        }
        done();
      };
      panelContext.watch("currentFrame");
      panelContext.watch("topics");

      panelContext.updatePanelSettingsEditor({
        actionHandler: (action) => {
          if (action.action !== "update") return;
          const key = action.payload.path[1];
          if (key === "robot") {
            state.robot = String(action.payload.value || "robot_1");
            robotInput.value = state.robot;
            persist();
          }
        },
        nodes: {
          general: {
            label: "Settings",
            fields: {
              robot: { label: "Robot Name", input: "string", value: state.robot, placeholder: "robot_1" },
            },
          },
        },
      });

      panelContext.setDefaultPanelTitle("Robot Tasks");
      renderTabs();
      renderStatus();
      renderFeedback();

      return () => {
        runtime.subscribedTopics = [];
        panelContext.subscribe([]);
      };
    },
  });
}

// ─────────────────────────── field builders ───────────────────────────────────

function fieldRow(labelText, control, hint) {
  const wrap = document.createElement("div");
  wrap.style.cssText = "display:flex;flex-direction:column;gap:2px;";
  const row = document.createElement("div");
  row.style.cssText = "display:flex;align-items:center;gap:6px;";
  const label = document.createElement("span");
  label.textContent = labelText + ":";
  label.style.cssText = "flex:1;";
  row.appendChild(label);
  row.appendChild(control);
  wrap.appendChild(row);
  if (hint) {
    const h = document.createElement("span");
    h.textContent = hint;
    h.style.cssText = "font-size:11px;opacity:0.6;padding-left:4px;";
    wrap.appendChild(h);
  }
  return wrap;
}

function buildField(field, tabState, persist) {
  switch (field.kind) {
    case "float":
    case "int": {
      const input = document.createElement("input");
      input.type = "number";
      input.step = field.kind === "int" ? "1" : String(field.step ?? 0.1);
      if (field.min != null) input.min = String(field.min);
      if (field.max != null) input.max = String(field.max);
      input.value = String(tabState[field.name]);
      input.style.cssText = "width:120px;padding:4px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
      input.addEventListener("change", () => {
        const v = field.kind === "int" ? parseInt(input.value, 10) : Number(input.value);
        tabState[field.name] = Number.isFinite(v) ? v : 0;
        persist();
      });
      return fieldRow(field.name, input, field.hint);
    }
    case "string": {
      const input = document.createElement("input");
      input.type = "text";
      input.value = String(tabState[field.name] ?? "");
      input.style.cssText = "flex:1;min-width:120px;padding:4px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
      input.addEventListener("change", () => {
        tabState[field.name] = input.value;
        persist();
      });
      return fieldRow(field.name, input, field.hint);
    }
    case "bool": {
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = Boolean(tabState[field.name]);
      cb.addEventListener("change", () => {
        tabState[field.name] = cb.checked;
        persist();
      });
      return fieldRow(field.name, cb, field.hint);
    }
    case "polygon":
    case "path": {
      const openKey = `_${field.name}_open`;
      const details = document.createElement("details");
      details.style.cssText = "border:1px solid #444;border-radius:4px;padding:4px 6px;margin-top:2px;";
      details.open = Boolean(tabState[openKey]);
      const summary = document.createElement("summary");
      summary.style.cssText = "cursor:pointer;user-select:none;font-size:13px;";
      const countPoints = (text) => {
        try {
          const parsed = JSON.parse(text || "[]");
          return Array.isArray(parsed) ? parsed.length : 0;
        } catch { return null; }
      };
      const renderSummary = () => {
        const n = countPoints(tabState[field.name]);
        const label = n == null ? "invalid" : `${n} pt${n === 1 ? "" : "s"}`;
        summary.textContent = `${field.name} (${label})`;
      };
      renderSummary();
      details.appendChild(summary);

      details.addEventListener("toggle", () => {
        tabState[openKey] = details.open;
        persist();
      });

      const ta = document.createElement("textarea");
      ta.rows = 3;
      ta.value = String(tabState[field.name] ?? "[]");
      ta.style.cssText = "width:100%;box-sizing:border-box;margin-top:4px;padding:4px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;font-family:monospace;font-size:12px;resize:vertical;";
      const err = document.createElement("span");
      err.style.cssText = "display:block;font-size:11px;color:#dc2626;min-height:14px;";
      const validate = () => {
        try { JSON.parse(ta.value || "[]"); err.textContent = ""; }
        catch (e) { err.textContent = "invalid JSON: " + (e?.message ?? e); }
      };
      ta.addEventListener("input", () => {
        tabState[field.name] = ta.value;
        validate();
        renderSummary();
        persist();
      });
      validate();
      details.appendChild(ta);

      // "Grab from Editor" button — snapshots the matching editor cache
      // (waypoints for path fields, polygon vertices for polygon fields) into
      // the textbox so the user can capture, modify the editor, capture again
      // for another command/drone, etc.
      const grabBtn = document.createElement("button");
      grabBtn.textContent = "Grab from Editor";
      grabBtn.style.cssText = "margin-top:4px;padding:4px 10px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;cursor:pointer;font-size:12px;";
      grabBtn.addEventListener("click", () => {
        const cache = field.kind === "polygon" ? polygonVerticesCache : editorWaypointsCache;
        const json = editorCacheToJson(cache);
        ta.value = json;
        tabState[field.name] = json;
        validate();
        renderSummary();
        persist();
      });
      details.appendChild(grabBtn);

      if (field.hint) {
        const h = document.createElement("span");
        h.textContent = field.hint;
        h.style.cssText = "display:block;font-size:11px;opacity:0.6;margin-top:2px;";
        details.appendChild(h);
      }
      details.appendChild(err);
      return details;
    }
  }
  const fallback = document.createElement("div");
  fallback.textContent = `(unsupported field kind: ${field.kind})`;
  return fallback;
}

function buildFixedTrajectoryForm(tabState, persist) {
  const wrapper = document.createElement("div");
  wrapper.style.cssText = "display:flex;flex-direction:column;gap:6px;";

  // Type combo
  const typeRow = document.createElement("div");
  typeRow.style.cssText = "display:flex;align-items:center;gap:6px;";
  const typeLabel = document.createElement("span");
  typeLabel.textContent = "type:";
  typeLabel.style.flex = "1";
  const typeSelect = document.createElement("select");
  typeSelect.style.cssText = "padding:4px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
  for (const name of Object.keys(TRAJECTORY_DEFAULTS)) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    if (name === tabState.type) opt.selected = true;
    typeSelect.appendChild(opt);
  }
  typeRow.appendChild(typeLabel);
  typeRow.appendChild(typeSelect);
  wrapper.appendChild(typeRow);

  // Attributes table
  const table = document.createElement("div");
  table.style.cssText = "display:flex;flex-direction:column;gap:2px;";
  wrapper.appendChild(table);

  function renderAttributes() {
    table.replaceChildren();
    tabState.attributes.forEach(([k, v], idx) => {
      const row = document.createElement("div");
      row.style.cssText = "display:flex;gap:4px;";
      const keyIn = document.createElement("input");
      keyIn.type = "text";
      keyIn.value = k;
      keyIn.placeholder = "key";
      keyIn.style.cssText = "flex:1;padding:3px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
      keyIn.addEventListener("change", () => {
        tabState.attributes[idx][0] = keyIn.value;
        persist();
      });
      const valIn = document.createElement("input");
      valIn.type = "text";
      valIn.value = v;
      valIn.placeholder = "value";
      valIn.style.cssText = "flex:1;padding:3px 6px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;";
      valIn.addEventListener("change", () => {
        tabState.attributes[idx][1] = valIn.value;
        persist();
      });
      const delBtn = document.createElement("button");
      delBtn.textContent = "✕";
      delBtn.style.cssText = "width:28px;border-radius:4px;border:none;background:#6b7280;color:white;cursor:pointer;";
      delBtn.addEventListener("click", () => {
        tabState.attributes.splice(idx, 1);
        persist();
        renderAttributes();
      });
      row.appendChild(keyIn);
      row.appendChild(valIn);
      row.appendChild(delBtn);
      table.appendChild(row);
    });
  }
  renderAttributes();

  typeSelect.addEventListener("change", () => {
    tabState.type = typeSelect.value;
    tabState.attributes = (TRAJECTORY_DEFAULTS[tabState.type] ?? []).map(([k, v]) => [k, v]);
    persist();
    renderAttributes();
  });

  const addBtn = document.createElement("button");
  addBtn.textContent = "+ Add Attribute";
  addBtn.style.cssText = "align-self:flex-start;padding:4px 8px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;cursor:pointer;";
  addBtn.addEventListener("click", () => {
    tabState.attributes.push(["", ""]);
    persist();
    renderAttributes();
  });
  wrapper.appendChild(addBtn);

  // Loop checkbox
  const loopRow = document.createElement("label");
  loopRow.style.cssText = "display:flex;align-items:center;gap:6px;cursor:pointer;";
  const loopCb = document.createElement("input");
  loopCb.type = "checkbox";
  loopCb.checked = Boolean(tabState.loop);
  loopCb.addEventListener("change", () => {
    tabState.loop = loopCb.checked;
    persist();
  });
  const loopLabel = document.createElement("span");
  loopLabel.textContent = "loop";
  loopRow.appendChild(loopCb);
  loopRow.appendChild(loopLabel);
  wrapper.appendChild(loopRow);

  return wrapper;
}

module.exports = { activate };
})();
