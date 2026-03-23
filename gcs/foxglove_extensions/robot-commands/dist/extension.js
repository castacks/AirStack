(()=>{
"use strict";

const BT_COMMANDS_DATATYPES = new Map([
  [
    "behavior_tree_msgs/msg/BehaviorTreeCommands",
    {
      definitions: [
        {
          name: "commands",
          type: "behavior_tree_msgs/msg/BehaviorTreeCommand",
          isArray: true,
          isComplex: true,
        },
      ],
    },
  ],
  [
    "behavior_tree_msgs/msg/BehaviorTreeCommand",
    {
      definitions: [
        { name: "condition_name", type: "string", isArray: false, isComplex: false },
        { name: "status", type: "int8", isArray: false, isComplex: false },
      ],
    },
  ],
]);

const PRESET_COMMANDS = [
  { label: "Arm & Takeoff",        value: "Auto Takeoff Commanded" },
  { label: "Global Plan",          value: "Global Plan Commanded" },
  { label: "Fixed Trajectory",     value: "Fixed Trajectory Commanded" },
  { label: "Autonomously Explore", value: "Autonomously Explore Commanded" },
  { label: "Pause",                value: "Pause Commanded" },
  { label: "Rewind",               value: "Rewind Commanded" },
  { label: "Land",                 value: "Land Commanded" },
  { label: "Disarm",               value: "Disarm Commanded" },
  { label: "Custom",               value: "__custom__" },
];

const DEFAULT_STATE = {
  robot: "robot_1",
  command: "Auto Takeoff Commanded",
  custom_command: "",
  button_label: "Arm & Takeoff",
  button_color: "#10B981",
};

function getConditionName(state) {
  return state.command === "__custom__" ? state.custom_command : state.command;
}

function activate(extensionContext) {
  extensionContext.registerPanel({
    name: "Robot Command Button",
    initPanel: (panelContext) => {
      let state = Object.assign({}, DEFAULT_STATE, panelContext.initialState ?? {});
      let advertisedTopic = null;

      const root = panelContext.panelElement;
      root.style.cssText =
        "display:flex;align-items:center;justify-content:center;height:100%;padding:12px;box-sizing:border-box;";

      const btn = document.createElement("button");
      btn.style.cssText =
        "width:100%;height:100%;min-height:48px;border:none;border-radius:6px;" +
        "font-size:16px;font-weight:bold;color:white;cursor:pointer;transition:opacity 0.1s;";
      btn.addEventListener("mousedown", () => (btn.style.opacity = "0.75"));
      btn.addEventListener("mouseup", () => (btn.style.opacity = "1"));
      btn.addEventListener("mouseleave", () => (btn.style.opacity = "1"));
      btn.addEventListener("click", handleClick);
      root.appendChild(btn);

      function render() {
        btn.textContent = state.button_label || "Button";
        btn.style.backgroundColor = state.button_color || "#10B981";
        panelContext.setDefaultPanelTitle(state.button_label || "Robot Command Button");
      }

      function handleClick() {
        const conditionName = getConditionName(state);
        if (!conditionName) return;
        const topic = `/${state.robot}/behavior/behavior_tree_commands`;

        if (advertisedTopic !== topic) {
          if (advertisedTopic != null) {
            panelContext.unadvertise(advertisedTopic);
          }
          panelContext.advertise(topic, "behavior_tree_msgs/msg/BehaviorTreeCommands", {
            datatypes: BT_COMMANDS_DATATYPES,
          });
          advertisedTopic = topic;
        }

        // Send 2 (SUCCESS) for the selected command, 0 (IDLE) for all others.
        // Matches rqt behaviour exactly.
        const commands = [];
        for (const preset of PRESET_COMMANDS) {
          if (preset.value === "__custom__") continue;
          commands.push({
            condition_name: preset.value,
            status: preset.value === conditionName ? 2 : 0,
          });
        }
        if (state.command === "__custom__" && conditionName) {
          commands.push({ condition_name: conditionName, status: 2 });
        }
        panelContext.publish(topic, { commands });
      }

      function updateSettings() {
        const isCustom = state.command === "__custom__";
        const fields = {
          button_label: {
            label: "Label",
            input: "string",
            value: state.button_label,
          },
          button_color: {
            label: "Color",
            input: "rgb",
            value: state.button_color,
          },
          robot: {
            label: "Robot Name",
            input: "string",
            value: state.robot,
            placeholder: "robot_1",
          },
          command: {
            label: "Command",
            input: "select",
            value: state.command,
            options: PRESET_COMMANDS,
          },
        };

        if (isCustom) {
          fields.custom_command = {
            label: "Custom Command Name",
            input: "string",
            value: state.custom_command,
            placeholder: "e.g. My Custom Commanded",
          };
        }

        panelContext.updatePanelSettingsEditor({
          actionHandler: (action) => {
            if (action.action !== "update") return;
            const key = action.payload.path[1];
            state[key] = action.payload.value;
            panelContext.saveState(state);
            render();
            updateSettings();
          },
          nodes: {
            general: {
              label: "Button",
              fields,
            },
          },
        });
      }

      render();
      updateSettings();

      return () => {
        if (advertisedTopic != null) {
          panelContext.unadvertise(advertisedTopic);
        }
      };
    },
  });
}

module.exports = { activate };
})();
