(() => {
"use strict";

// ─────────────────────────── constants ────────────────────────────────────────

const CMD_TOPIC = "/gcs/polygon/command";
const LIST_TOPIC = "/gcs/polygon/list";
const CMD_SCHEMA = "std_msgs/msg/String";

// ─────────────────────────── panel ───────────────────────────────────────────

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "Polygon Editor",
    initPanel: (panelContext) => {

      // ── state ────────────────────────────────────────────────────────
      const persisted = panelContext.initialState ?? {};
      const state = {
        defaultZ: persisted.defaultZ ?? 0.0,
      };
      let vertices = [];  // [{x, y, z}, ...]
      let selectedIdx = -1;
      let enabled = false;  // synced from /gcs/polygon/list

      function persist() { panelContext.saveState(state); }
      function sendCmd(cmd) {
        try {
          panelContext.advertise(CMD_TOPIC, CMD_SCHEMA);
          panelContext.publish(CMD_TOPIC, { data: JSON.stringify(cmd) });
        } catch (err) {
          statusEl.textContent = "Cmd failed: " + (err?.message ?? err);
        }
      }

      // ── DOM ──────────────────────────────────────────────────────────
      const root = panelContext.panelElement;
      root.style.cssText =
        "display:flex;flex-direction:column;height:100%;box-sizing:border-box;" +
        "padding:8px;gap:6px;font-family:sans-serif;color:inherit;overflow-y:auto;";

      // Title row
      const titleRow = el("div", "display:flex;align-items:center;gap:8px;flex-shrink:0;");
      const title = el("span", "font-weight:bold;font-size:14px;flex:1;");
      title.textContent = "Polygon Editor";
      titleRow.appendChild(title);
      root.appendChild(titleRow);

      // Altitude + clear row
      const ctrlRow = el("div", "display:flex;align-items:center;gap:8px;flex-shrink:0;");

      const enableBtn = el("button",
        "padding:6px 14px;border-radius:4px;border:none;color:white;cursor:pointer;font-weight:bold;font-size:13px;");
      enableBtn.addEventListener("click", () => {
        sendCmd({ action: "set_enabled", enabled: !enabled });
      });
      ctrlRow.appendChild(enableBtn);

      const altLabel = el("span", "font-size:12px;");
      altLabel.textContent = "Z:";
      const altInput = el("input",
        "width:60px;padding:3px 5px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;");
      altInput.type = "number";
      altInput.step = "0.5";
      altInput.value = String(state.defaultZ);
      altInput.addEventListener("change", () => {
        state.defaultZ = Number(altInput.value) || 0.0;
        persist();
        sendCmd({ action: "set_altitude", z: state.defaultZ });
      });

      const clearBtn = el("button",
        "padding:6px 10px;border-radius:4px;border:none;background:#dc2626;color:white;cursor:pointer;font-size:12px;");
      clearBtn.textContent = "Clear All";
      clearBtn.addEventListener("click", () => { sendCmd({ action: "clear" }); });

      ctrlRow.appendChild(altLabel);
      ctrlRow.appendChild(altInput);
      ctrlRow.appendChild(el("span", "flex:1;"));  // spacer
      ctrlRow.appendChild(clearBtn);
      root.appendChild(ctrlRow);

      // Vertex count
      const countEl = el("div", "font-size:12px;opacity:0.8;flex-shrink:0;");
      root.appendChild(countEl);

      // Vertex list container
      const listContainer = el("div",
        "flex:1;overflow-y:auto;border:1px solid #444;border-radius:4px;min-height:60px;");
      root.appendChild(listContainer);

      // Add vertex manually row
      const addRow = el("div", "display:flex;align-items:center;gap:4px;flex-shrink:0;");
      const addXIn = numInput("X", "0");
      const addYIn = numInput("Y", "0");
      const addZIn = numInput("Z", String(state.defaultZ));
      const addBtn = el("button",
        "padding:4px 10px;border-radius:4px;border:none;background:#10b981;color:white;cursor:pointer;font-size:12px;");
      addBtn.textContent = "+ Add";
      addBtn.addEventListener("click", () => {
        sendCmd({
          action: "add",
          x: Number(addXIn.input.value) || 0,
          y: Number(addYIn.input.value) || 0,
          z: Number(addZIn.input.value) || state.defaultZ,
        });
      });
      addRow.appendChild(addXIn.wrap);
      addRow.appendChild(addYIn.wrap);
      addRow.appendChild(addZIn.wrap);
      addRow.appendChild(addBtn);
      root.appendChild(addRow);

      // Status
      const statusEl = el("div", "font-size:11px;opacity:0.6;flex-shrink:0;");
      root.appendChild(statusEl);

      // ── render helpers ───────────────────────────────────────────────
      function renderEnableBtn() {
        if (enabled) {
          enableBtn.textContent = "Capture: ON";
          enableBtn.style.background = "#10b981";
        } else {
          enableBtn.textContent = "Capture: OFF";
          enableBtn.style.background = "#dc2626";
        }
      }
      renderEnableBtn();

      function renderList() {
        countEl.textContent = vertices.length + " vert" + (vertices.length === 1 ? "ex" : "ices");
        listContainer.replaceChildren();

        if (vertices.length === 0) {
          const empty = el("div", "padding:12px;text-align:center;opacity:0.5;font-size:12px;");
          empty.textContent = "No vertices. Enable capture and click in the 3D view, or add manually.";
          listContainer.appendChild(empty);
          return;
        }

        for (let i = 0; i < vertices.length; i++) {
          const v = vertices[i];
          const row = el("div",
            "display:flex;align-items:center;gap:4px;padding:4px 6px;" +
            "border-bottom:1px solid #333;font-size:12px;font-family:monospace;" +
            (i === selectedIdx ? "background:rgba(220,38,38,0.18);" : ""));

          const idx = el("span", "width:20px;font-weight:bold;color:#dc2626;");
          idx.textContent = String(i);
          row.appendChild(idx);

          const xIn = coordInput(v.x, (val) => {
            sendCmd({ action: "move", index: i, x: val, y: v.y, z: v.z });
          });
          const yIn = coordInput(v.y, (val) => {
            sendCmd({ action: "move", index: i, x: v.x, y: val, z: v.z });
          });
          const zIn = coordInput(v.z, (val) => {
            sendCmd({ action: "move", index: i, x: v.x, y: v.y, z: val });
          });
          row.appendChild(xIn);
          row.appendChild(yIn);
          row.appendChild(zIn);

          if (i > 0) {
            const upBtn = smallBtn("▲", () => {
              sendCmd({ action: "reorder", from: i, to: i - 1 });
            });
            upBtn.title = "Move up";
            row.appendChild(upBtn);
          } else {
            row.appendChild(el("span", "width:24px;"));
          }

          if (i < vertices.length - 1) {
            const downBtn = smallBtn("▼", () => {
              sendCmd({ action: "reorder", from: i, to: i + 1 });
            });
            downBtn.title = "Move down";
            row.appendChild(downBtn);
          } else {
            row.appendChild(el("span", "width:24px;"));
          }

          const delBtn = smallBtn("✕", () => {
            sendCmd({ action: "delete", index: i });
          });
          delBtn.style.color = "#dc2626";
          delBtn.title = "Delete";
          row.appendChild(delBtn);

          row.addEventListener("click", (e) => {
            if (e.target.tagName === "INPUT" || e.target.tagName === "BUTTON") return;
            selectedIdx = (selectedIdx === i) ? -1 : i;
            renderList();
          });

          listContainer.appendChild(row);
        }
      }
      renderList();

      // ── subscriptions ────────────────────────────────────────────────
      panelContext.subscribe([{ topic: LIST_TOPIC }]);
      panelContext.watch("currentFrame");

      panelContext.onRender = (renderState, done) => {
        const frame = renderState.currentFrame;
        if (frame) {
          for (const evt of frame) {
            if (evt.topic === LIST_TOPIC) {
              try {
                const data = JSON.parse(evt.message.data);
                vertices = data.vertices ?? [];
                if (data.default_z != null) {
                  state.defaultZ = data.default_z;
                  altInput.value = String(data.default_z);
                }
                if (data.enabled != null) {
                  enabled = Boolean(data.enabled);
                  renderEnableBtn();
                }
                if (selectedIdx >= vertices.length) selectedIdx = -1;
                renderList();
              } catch { /* ignore bad data */ }
            }
          }
        }
        done();
      };

      panelContext.setDefaultPanelTitle("Polygon Editor");

      return () => {};
    },
  });
}

// ─────────────────────────── DOM helpers ─────────────────────────────────────

function el(tag, style) {
  const e = document.createElement(tag);
  if (style) e.style.cssText = style;
  return e;
}

function smallBtn(text, onClick) {
  const b = el("button",
    "width:24px;height:24px;padding:0;border:none;background:transparent;" +
    "color:inherit;cursor:pointer;font-size:12px;border-radius:3px;");
  b.textContent = text;
  b.addEventListener("mouseenter", () => { b.style.background = "rgba(255,255,255,0.1)"; });
  b.addEventListener("mouseleave", () => { b.style.background = "transparent"; });
  b.addEventListener("click", (e) => { e.stopPropagation(); onClick(); });
  return b;
}

function coordInput(value, onChange) {
  const inp = el("input",
    "width:55px;padding:2px 4px;border-radius:3px;border:1px solid #555;" +
    "background:transparent;color:inherit;font-family:monospace;font-size:11px;text-align:right;");
  inp.type = "number";
  inp.step = "0.5";
  inp.value = String(value);
  inp.addEventListener("change", () => {
    onChange(Number(inp.value) || 0);
  });
  return inp;
}

function numInput(label, defaultVal) {
  const wrap = el("div", "display:flex;align-items:center;gap:2px;");
  const lbl = el("span", "font-size:11px;opacity:0.7;");
  lbl.textContent = label + ":";
  const input = el("input",
    "width:50px;padding:2px 4px;border-radius:3px;border:1px solid #555;" +
    "background:transparent;color:inherit;font-size:11px;");
  input.type = "number";
  input.step = "0.5";
  input.value = defaultVal;
  wrap.appendChild(lbl);
  wrap.appendChild(input);
  return { wrap, input };
}

module.exports = { activate };
})();
