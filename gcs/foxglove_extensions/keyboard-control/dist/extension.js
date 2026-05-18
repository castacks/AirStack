// AirStack Keyboard Control — Foxglove Extension Panel
// Publishes std_msgs/String to keyboard_input and std_msgs/Bool to keyboard_control_enable.
// Key processing logic mirrors keyboard_controller.cpp (ProcessKey).
(() => {
"use strict";

const KEY_INFO = {
  w: { label: "W", action: "Forward (+X)" },
  s: { label: "S", action: "Backward (−X)" },
  a: { label: "A", action: "Left (+Y)" },
  d: { label: "D", action: "Right (−Y)" },
  c: { label: "C", action: "Up (+Z)" },
  z: { label: "Z", action: "Down (−Z)" },
  q: { label: "Q", action: "Yaw Left" },
  e: { label: "E", action: "Yaw Right" },
  o: { label: "O", action: "Step −0.1 m" },
  p: { label: "P", action: "Step +0.1 m" },
  k: { label: "K", action: "Yaw step −" },
  l: { label: "L", action: "Yaw step +" },
};

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "Keyboard Control",
    initPanel: (ctx) => {
      // ── Persisted state ────────────────────────────────────────────────────
      const saved = ctx.initialState ?? {};
      const state = { robot: saved.robot ?? "robot_1" };
      let enabled = false; // always start disabled — safety first

      function persist() { ctx.saveState({ robot: state.robot }); }

      function inputTopic()  { return `/${state.robot}/keyboard_controller/keyboard_input`; }
      function enableTopic() { return `/${state.robot}/keyboard_controller/keyboard_control_enable`; }

      function publishKey(key) {
        ctx.advertise(inputTopic(), "std_msgs/msg/String");
        ctx.publish(inputTopic(), { data: key });
      }

      function publishEnable(val) {
        ctx.advertise(enableTopic(), "std_msgs/msg/Bool");
        ctx.publish(enableTopic(), { data: val });
      }

      // ── DOM ────────────────────────────────────────────────────────────────
      const root = ctx.panelElement;
      root.tabIndex = 0;
      root.style.cssText =
        "display:flex;flex-direction:column;height:100%;box-sizing:border-box;" +
        "padding:10px;gap:8px;font-family:sans-serif;color:inherit;overflow-y:auto;";

      // Robot name row
      const robotRow = document.createElement("div");
      robotRow.style.cssText = "display:flex;align-items:center;gap:8px;flex-shrink:0;";
      const robotLabel = document.createElement("span");
      robotLabel.textContent = "Robot:";
      robotLabel.style.fontWeight = "bold";
      const robotInput = document.createElement("input");
      robotInput.type = "text";
      robotInput.value = state.robot;
      robotInput.style.cssText =
        "flex:1;padding:4px 8px;border-radius:4px;border:1px solid #555;" +
        "background:transparent;color:inherit;font-size:13px;";
      robotInput.addEventListener("change", () => {
        state.robot = robotInput.value.trim() || "robot_1";
        persist();
        ctx.advertise(inputTopic(), "std_msgs/msg/String");
        ctx.advertise(enableTopic(), "std_msgs/msg/Bool");
      });
      robotRow.appendChild(robotLabel);
      robotRow.appendChild(robotInput);
      root.appendChild(robotRow);

      // Enable / Disable button
      const enableBtn = document.createElement("button");
      enableBtn.style.cssText =
        "padding:10px;border-radius:6px;border:none;cursor:pointer;" +
        "font-size:14px;font-weight:bold;flex-shrink:0;";

      // Status line
      const statusEl = document.createElement("div");
      statusEl.style.cssText =
        "font-size:12px;text-align:center;padding:4px;flex-shrink:0;";

      function applyEnabledStyle() {
        if (enabled) {
          enableBtn.textContent = "ENABLED — Click to Disable";
          enableBtn.style.background = "#2e7d32";
          enableBtn.style.color = "#fff";
          statusEl.textContent = "Click panel to focus, then press keys to fly.";
          statusEl.style.color = "#4caf50";
        } else {
          enableBtn.textContent = "DISABLED — Click to Enable";
          enableBtn.style.background = "#444";
          enableBtn.style.color = "#aaa";
          statusEl.textContent = "Keyboard control is off. Enable before flying.";
          statusEl.style.color = "#888";
        }
      }

      enableBtn.addEventListener("click", () => {
        enabled = !enabled;
        publishEnable(enabled);
        applyEnabledStyle();
        if (enabled) root.focus();
      });
      root.appendChild(enableBtn);
      root.appendChild(statusEl);
      applyEnabledStyle();

      // Active-key display
      const activeKeyEl = document.createElement("div");
      activeKeyEl.style.cssText =
        "font-size:22px;font-weight:bold;text-align:center;min-height:36px;" +
        "padding:4px;flex-shrink:0;";
      root.appendChild(activeKeyEl);

      // Key grid (3 columns)
      const grid = document.createElement("div");
      grid.style.cssText =
        "display:grid;grid-template-columns:repeat(4,1fr);gap:4px;flex-shrink:0;";
      const keyEls = {};
      for (const [k, info] of Object.entries(KEY_INFO)) {
        const cell = document.createElement("div");
        cell.style.cssText =
          "border:1px solid #555;border-radius:4px;padding:6px 2px;" +
          "text-align:center;transition:background 0.1s;";
        cell.innerHTML =
          `<div style="font-weight:bold;font-size:15px;">${info.label}</div>` +
          `<div style="color:#888;font-size:10px;line-height:1.2;">${info.action}</div>`;
        keyEls[k] = cell;
        grid.appendChild(cell);
      }
      root.appendChild(grid);

      // Hint
      const hint = document.createElement("div");
      hint.style.cssText = "font-size:11px;color:#666;text-align:center;flex-shrink:0;";
      hint.textContent = "Disable before typing elsewhere to avoid accidental commands.";
      root.appendChild(hint);

      // ── Key event handling ────────────────────────────────────────────────
      let flashTimeout = null;

      function flash(k) {
        activeKeyEl.textContent = `[ ${k.toUpperCase()} ]  ${KEY_INFO[k].action}`;
        if (keyEls[k]) {
          keyEls[k].style.background = "#1565c0";
          keyEls[k].style.color = "#fff";
          keyEls[k].style.borderColor = "#42a5f5";
        }
        clearTimeout(flashTimeout);
        flashTimeout = setTimeout(() => {
          activeKeyEl.textContent = "";
          for (const el of Object.values(keyEls)) {
            el.style.background = "";
            el.style.color = "";
            el.style.borderColor = "#555";
          }
        }, 400);
      }

      root.addEventListener("keydown", (e) => {
        if (!enabled) return;
        const k = e.key.toLowerCase();
        if (!KEY_INFO[k]) return;
        e.preventDefault();
        publishKey(k);
        flash(k);
      });

      root.addEventListener("blur", () => {
        activeKeyEl.textContent = "";
        for (const el of Object.values(keyEls)) {
          el.style.background = "";
          el.style.color = "";
          el.style.borderColor = "#555";
        }
      });

      // ── Render loop (required by Foxglove API) ────────────────────────────
      ctx.onRender = (_renderState, done) => { done(); };

      // Pre-advertise topics on init
      ctx.advertise(inputTopic(), "std_msgs/msg/String");
      ctx.advertise(enableTopic(), "std_msgs/msg/Bool");
    },
  });
}

module.exports = { activate };
})();
