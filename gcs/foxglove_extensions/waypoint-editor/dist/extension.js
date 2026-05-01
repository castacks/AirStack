(() => {
"use strict";

// ─────────────────────────── constants ────────────────────────────────────────

const CMD_TOPIC = "/gcs/waypoints/command";
const LIST_TOPIC = "/gcs/waypoints/list";
const SAVES_TOPIC = "/gcs/waypoints/saves";
const CMD_SCHEMA = "std_msgs/msg/String";

// ─────────────────────────── panel ───────────────────────────────────────────

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "Waypoint Editor",
    initPanel: (panelContext) => {

      // ── state ────────────────────────────────────────────────────────
      const persisted = panelContext.initialState ?? {};
      const state = {
        defaultZ: persisted.defaultZ ?? 10.0,
      };
      let waypoints = [];  // [{x, y, z}, ...]
      let selectedIdx = -1;
      let enabled = false;  // synced from /gcs/waypoints/list
      let saves = [];  // [{name, color, count, saved, vertices}, ...] from /gcs/waypoints/saves

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
      title.textContent = "Waypoint Editor";
      titleRow.appendChild(title);
      root.appendChild(titleRow);

      // Altitude + clear row
      const ctrlRow = el("div", "display:flex;align-items:center;gap:8px;flex-shrink:0;");

      const enableBtn = el("button",
        "padding:8px 18px;border-radius:5px;border:none;color:white;cursor:pointer;font-weight:bold;font-size:15px;");
      enableBtn.addEventListener("click", () => {
        sendCmd({ action: "set_enabled", enabled: !enabled });
      });
      ctrlRow.appendChild(enableBtn);

      const altLabel = el("span", "font-size:14px;");
      altLabel.textContent = "Altitude:";
      const altInput = el("input",
        "width:80px;padding:6px 8px;border-radius:5px;border:1px solid #555;background:transparent;color:inherit;font-size:14px;");
      altInput.type = "number";
      altInput.step = "0.5";
      altInput.value = String(state.defaultZ);
      altInput.addEventListener("change", () => {
        state.defaultZ = Number(altInput.value) || 10.0;
        persist();
        sendCmd({ action: "set_altitude", z: state.defaultZ });
      });

      const clearBtn = el("button",
        "padding:8px 14px;border-radius:5px;border:none;background:#dc2626;color:white;cursor:pointer;font-size:14px;");
      clearBtn.textContent = "Clear All";
      clearBtn.addEventListener("click", () => { sendCmd({ action: "clear" }); });

      ctrlRow.appendChild(altLabel);
      ctrlRow.appendChild(altInput);
      ctrlRow.appendChild(el("span", "flex:1;"));  // spacer
      ctrlRow.appendChild(clearBtn);
      root.appendChild(ctrlRow);

      // Waypoint count
      const countEl = el("div", "font-size:12px;opacity:0.8;flex-shrink:0;");
      root.appendChild(countEl);

      // Waypoint list container
      const listContainer = el("div",
        "flex:1;overflow-y:auto;border:1px solid #444;border-radius:4px;min-height:60px;");
      root.appendChild(listContainer);

      // Add waypoint manually row
      const addRow = el("div", "display:flex;align-items:center;gap:4px;flex-shrink:0;");
      const addXIn = numInput("X", "0");
      const addYIn = numInput("Y", "0");
      const addZIn = numInput("Z", String(state.defaultZ));
      const addBtn = el("button",
        "padding:6px 14px;border-radius:5px;border:none;background:#10b981;color:white;cursor:pointer;font-size:14px;");
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

      // ── Saves section ────────────────────────────────────────────────
      const savesLabel = el("div",
        "font-size:11px;font-weight:bold;opacity:0.8;margin-top:6px;flex-shrink:0;");
      savesLabel.textContent = "Saves";
      root.appendChild(savesLabel);

      const saveAddRow = el("div", "display:flex;align-items:center;gap:6px;flex-shrink:0;");
      const saveNameIn = el("input",
        "flex:1;padding:6px 8px;border-radius:5px;border:1px solid #555;background:transparent;color:inherit;font-size:14px;");
      saveNameIn.type = "text";
      saveNameIn.placeholder = "save name…";
      const saveAddBtn = el("button",
        "padding:6px 14px;border-radius:5px;border:none;background:#10b981;color:white;cursor:pointer;font-size:14px;");
      saveAddBtn.textContent = "+ Add";
      saveAddBtn.addEventListener("click", () => {
        const name = saveNameIn.value.trim();
        if (!name) return;
        sendCmd({ action: "add_save", name });
      });
      saveAddRow.appendChild(saveNameIn);
      saveAddRow.appendChild(saveAddBtn);
      root.appendChild(saveAddRow);

      const saveList = el("div",
        "border:1px solid #333;border-radius:4px;min-height:0;flex-shrink:0;");
      root.appendChild(saveList);

      // Status
      const statusEl = el("div", "font-size:11px;opacity:0.6;flex-shrink:0;");
      root.appendChild(statusEl);

      function renderSaves() {
        saveList.replaceChildren();
        if (saves.length === 0) {
          const empty = el("div", "padding:6px 8px;opacity:0.5;font-size:11px;");
          empty.textContent = "No saves yet. Type a name and click + Add.";
          saveList.appendChild(empty);
          return;
        }
        for (const s of saves) {
          const row = el("div",
            "display:flex;align-items:center;gap:8px;padding:6px 8px;border-bottom:1px solid #333;font-size:14px;");
          const swatch = el("span",
            "display:inline-block;width:12px;height:12px;border-radius:50%;flex-shrink:0;");
          const [r, g, b] = s.color || [0.5, 0.5, 0.5];
          swatch.style.background =
            `rgb(${Math.round(r*255)},${Math.round(g*255)},${Math.round(b*255)})`;
          row.appendChild(swatch);

          const nameEl = el("span", "flex:1;font-family:monospace;");
          nameEl.textContent = `${s.name} (${s.count} wp)`;
          row.appendChild(nameEl);

          const loadBtn = el("button",
            "padding:4px 10px;border-radius:4px;border:1px solid #555;background:transparent;color:inherit;cursor:pointer;font-size:13px;");
          loadBtn.textContent = "Load";
          loadBtn.addEventListener("click", () => {
            sendCmd({ action: "load_save", name: s.name });
          });
          row.appendChild(loadBtn);

          const saveBtn = el("button", "");
          if (s.saved) {
            saveBtn.textContent = "✓ Saved";
            saveBtn.disabled = true;
            saveBtn.style.cssText =
              "padding:4px 10px;border-radius:4px;border:1px solid #10b981;background:rgba(16,185,129,0.1);color:#10b981;cursor:default;font-size:13px;";
          } else {
            saveBtn.textContent = "Save";
            saveBtn.style.cssText =
              "padding:4px 10px;border-radius:4px;border:none;background:#2563eb;color:white;cursor:pointer;font-size:13px;";
            saveBtn.addEventListener("click", () => {
              sendCmd({ action: "save_save", name: s.name });
            });
          }
          row.appendChild(saveBtn);

          const delBtn = el("button",
            "padding:4px 8px;border-radius:4px;border:none;background:transparent;color:#dc2626;cursor:pointer;font-size:15px;");
          delBtn.textContent = "✕";
          delBtn.title = "Delete save";
          delBtn.addEventListener("click", () => {
            sendCmd({ action: "delete_save", name: s.name });
          });
          row.appendChild(delBtn);

          saveList.appendChild(row);
        }
      }
      renderSaves();

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
        countEl.textContent = waypoints.length + " waypoint" + (waypoints.length === 1 ? "" : "s");
        listContainer.replaceChildren();

        if (waypoints.length === 0) {
          const empty = el("div", "padding:12px;text-align:center;opacity:0.5;font-size:12px;");
          empty.textContent = "No waypoints. Click in the 3D view or add manually below.";
          listContainer.appendChild(empty);
          return;
        }

        for (let i = 0; i < waypoints.length; i++) {
          const wp = waypoints[i];
          const row = el("div",
            "display:flex;align-items:center;gap:5px;padding:5px 8px;" +
            "border-bottom:1px solid #333;font-size:14px;font-family:monospace;" +
            (i === selectedIdx ? "background:rgba(16,185,129,0.15);" : ""));

          // Index label
          const idx = el("span", "width:24px;font-weight:bold;font-size:14px;color:#10b981;");
          idx.textContent = String(i);
          row.appendChild(idx);

          // Coordinates (editable)
          const xIn = coordInput(wp.x, (v) => {
            sendCmd({ action: "move", index: i, x: v, y: wp.y, z: wp.z });
          });
          const yIn = coordInput(wp.y, (v) => {
            sendCmd({ action: "move", index: i, x: wp.x, y: v, z: wp.z });
          });
          const zIn = coordInput(wp.z, (v) => {
            sendCmd({ action: "move", index: i, x: wp.x, y: wp.y, z: v });
          });
          row.appendChild(xIn);
          row.appendChild(yIn);
          row.appendChild(zIn);

          // Move up
          if (i > 0) {
            const upBtn = smallBtn("\u25B2", () => {
              sendCmd({ action: "reorder", from: i, to: i - 1 });
            });
            upBtn.title = "Move up";
            row.appendChild(upBtn);
          } else {
            row.appendChild(el("span", "width:32px;"));
          }

          // Move down
          if (i < waypoints.length - 1) {
            const downBtn = smallBtn("\u25BC", () => {
              sendCmd({ action: "reorder", from: i, to: i + 1 });
            });
            downBtn.title = "Move down";
            row.appendChild(downBtn);
          } else {
            row.appendChild(el("span", "width:32px;"));
          }

          // Duplicate
          const dupBtn = smallBtn("\u29c9", () => {
            sendCmd({ action: "duplicate", index: i });
          });
          dupBtn.title = "Duplicate";
          row.appendChild(dupBtn);

          // Delete
          const delBtn = smallBtn("\u2715", () => {
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
      panelContext.subscribe([
        { topic: LIST_TOPIC },
        { topic: SAVES_TOPIC },
      ]);
      panelContext.watch("currentFrame");

      panelContext.onRender = (renderState, done) => {
        const frame = renderState.currentFrame;
        if (frame) {
          for (const evt of frame) {
            if (evt.topic === LIST_TOPIC) {
              try {
                const data = JSON.parse(evt.message.data);
                waypoints = data.waypoints ?? [];
                if (data.default_z != null) {
                  state.defaultZ = data.default_z;
                  altInput.value = String(data.default_z);
                }
                if (data.enabled != null) {
                  enabled = Boolean(data.enabled);
                  renderEnableBtn();
                }
                // Clamp selected index
                if (selectedIdx >= waypoints.length) selectedIdx = -1;
                renderList();
              } catch { /* ignore bad data */ }
            } else if (evt.topic === SAVES_TOPIC) {
              try {
                const data = JSON.parse(evt.message.data);
                saves = Array.isArray(data.saves) ? data.saves : [];
                renderSaves();
              } catch { /* ignore bad data */ }
            }
          }
        }
        done();
      };

      panelContext.setDefaultPanelTitle("Waypoint Editor");

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
    "width:32px;height:32px;padding:0;border:none;background:transparent;" +
    "color:inherit;cursor:pointer;font-size:15px;border-radius:4px;");
  b.textContent = text;
  b.addEventListener("mouseenter", () => { b.style.background = "rgba(255,255,255,0.1)"; });
  b.addEventListener("mouseleave", () => { b.style.background = "transparent"; });
  b.addEventListener("click", (e) => { e.stopPropagation(); onClick(); });
  return b;
}

function coordInput(value, onChange) {
  const inp = el("input",
    "width:72px;padding:5px 7px;border-radius:4px;border:1px solid #555;" +
    "background:transparent;color:inherit;font-family:monospace;font-size:13px;text-align:right;");
  inp.type = "number";
  inp.step = "0.5";
  inp.value = String(value);
  inp.addEventListener("change", () => {
    onChange(Number(inp.value) || 0);
  });
  return inp;
}

function numInput(label, defaultVal) {
  const wrap = el("div", "display:flex;align-items:center;gap:4px;");
  const lbl = el("span", "font-size:13px;opacity:0.75;");
  lbl.textContent = label + ":";
  const input = el("input",
    "width:64px;padding:5px 7px;border-radius:4px;border:1px solid #555;" +
    "background:transparent;color:inherit;font-size:13px;");
  input.type = "number";
  input.step = "0.5";
  input.value = defaultVal;
  wrap.appendChild(lbl);
  wrap.appendChild(input);
  return { wrap, input };
}

module.exports = { activate };
})();
