// Copyright (c) 2024 Carnegie Mellon University
// MIT License

#include "rviz_tasks_panel/tasks_panel.hpp"

#include <QFileDialog>
#include <QHeaderView>
#include <QMetaObject>
#include <QTabBar>
#include <QTime>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <set>

namespace rviz_tasks_panel
{

// ─────────────────────────── static task registry ─────────────────────────────

std::vector<TaskTypeDef> TasksPanel::getTaskDefs()
{
  // default, min, max
  return {
    {"Takeoff", "tasks/takeoff", {
      {"target_altitude_m", "float32", 5.0, 0.0, 500.0},
      {"velocity_m_s", "float32", 1.0, 0.0, 50.0},
    }},
    {"Land", "tasks/land", {
      {"velocity_m_s", "float32", 0.3, 0.0, 10.0},
    }},
    {"Navigate", "tasks/navigate", {
      {"global_plan", "nav_msgs/Path", 0, 0, 0},
      {"goal_tolerance_m", "float32", 1.0, 0.0, 100.0},
    }},
    {"Exploration", "tasks/exploration", {
      {"search_bounds", "geometry_msgs/Polygon", 0, 0, 0},
      {"min_altitude_agl", "float32", 3.0, 0.0, 500.0},
      {"max_altitude_agl", "float32", 10.0, 0.0, 500.0},
      {"min_flight_speed", "float32", 1.0, 0.0, 50.0},
      {"max_flight_speed", "float32", 3.0, 0.0, 50.0},
      {"time_limit_sec", "float32", 120.0, 0.0, 86400.0},
    }},
    {"Coverage", "tasks/coverage", {
      {"coverage_area", "geometry_msgs/Polygon", 0, 0, 0},
      {"min_altitude_agl", "float32", 3.0, 0.0, 500.0},
      {"max_altitude_agl", "float32", 10.0, 0.0, 500.0},
      {"min_flight_speed", "float32", 1.0, 0.0, 50.0},
      {"max_flight_speed", "float32", 3.0, 0.0, 50.0},
      {"line_spacing_m", "float32", 5.0, 0.1, 1000.0},
      {"heading_deg", "float32", 0.0, 0.0, 360.0},
    }},
    {"Object Search", "tasks/object_search", {
      {"object_class", "string", 0, 0, 0},
      {"search_area", "geometry_msgs/Polygon", 0, 0, 0},
      {"min_altitude_agl", "float32", 3.0, 0.0, 500.0},
      {"max_altitude_agl", "float32", 10.0, 0.0, 500.0},
      {"min_flight_speed", "float32", 1.0, 0.0, 50.0},
      {"max_flight_speed", "float32", 3.0, 0.0, 50.0},
      {"time_limit_sec", "float32", 120.0, 0.0, 86400.0},
      {"target_count", "int32", 1, 0, 10000},
    }},
    {"Object Counting", "tasks/object_counting", {
      {"object_class", "string", 0, 0, 0},
      {"count_area", "geometry_msgs/Polygon", 0, 0, 0},
      {"min_altitude_agl", "float32", 3.0, 0.0, 500.0},
      {"max_altitude_agl", "float32", 10.0, 0.0, 500.0},
      {"min_flight_speed", "float32", 1.0, 0.0, 50.0},
      {"max_flight_speed", "float32", 3.0, 0.0, 50.0},
    }},
    {"Semantic Search", "tasks/semantic_search", {
      {"query", "string", 0, 0, 0},
      {"search_area", "geometry_msgs/Polygon", 0, 0, 0},
      {"min_altitude_agl", "float32", 3.0, 0.0, 500.0},
      {"max_altitude_agl", "float32", 10.0, 0.0, 500.0},
      {"min_flight_speed", "float32", 1.0, 0.0, 50.0},
      {"max_flight_speed", "float32", 3.0, 0.0, 50.0},
      {"time_limit_sec", "float32", 120.0, 0.0, 86400.0},
      {"confidence_threshold", "float32", 0.5, 0.0, 1.0},
    }},
    {"Fixed Trajectory", "tasks/fixed_trajectory", {
      {"trajectory_spec", "airstack_msgs/FixedTrajectory", 0, 0, 0},
      {"loop", "bool", 0, 0, 0},
    }},
  };
}

// ─────────────────────────── constructor / init ───────────────────────────────

TasksPanel::TasksPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  task_defs_ = getTaskDefs();
}

void TasksPanel::onInitialize()
{
  auto node_abs = getDisplayContext()->getRosNodeAbstraction().lock();
  raw_node_ = node_abs->get_raw_node();

  // Polygon selection service client
  polygon_client_ =
    raw_node_->create_client<rviz_polygon_selection_tool::srv::GetSelection>("get_selection");

  // Shared waypoint manager (singleton created by WaypointTool)
  waypoint_manager_ = waypoint_rviz2_plugin::WaypointManager::instance();
  if (!waypoint_manager_->isInitialized()) {
    // WaypointTool hasn't initialized yet; try again shortly
    QTimer::singleShot(1000, [this]() {
      if (waypoint_manager_ && !waypoint_manager_->isInitialized()) {
        waypoint_manager_->initialize(getDisplayContext(),
          getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
      }
    });
  }

  // Connect waypoint manager signals
  connect(waypoint_manager_.get(),
    &waypoint_rviz2_plugin::WaypointManager::waypointCountChanged,
    this, &TasksPanel::onWaypointCountChanged, Qt::QueuedConnection);
  connect(waypoint_manager_.get(),
    &waypoint_rviz2_plugin::WaypointManager::selectedMarkerChanged,
    this, &TasksPanel::onSelectedMarkerChanged, Qt::QueuedConnection);
  connect(waypoint_manager_.get(),
    &waypoint_rviz2_plugin::WaypointManager::waypointsCleared,
    this, [this]() {
      onWaypointCountChanged(0);
    }, Qt::QueuedConnection);

  // Build UI
  auto * main_layout = new QVBoxLayout();

  // Robot selector bar
  auto * top_bar = new QHBoxLayout();
  top_bar->addWidget(new QLabel("Robot:"));
  robot_combo_ = new QComboBox();
  robot_combo_->setEditable(true);
  robot_combo_->setMinimumWidth(120);
  top_bar->addWidget(robot_combo_);
  auto * refresh_btn = new QPushButton("Refresh");
  connect(refresh_btn, &QPushButton::clicked, this, &TasksPanel::onRefreshExecutors);
  top_bar->addWidget(refresh_btn);
  top_bar->addStretch();
  main_layout->addLayout(top_bar);

  // Tab widget
  tab_widget_ = new QTabWidget();
  tab_widget_->setFont(QFont("Noto Sans", 10));
  main_layout->addWidget(tab_widget_);

  setLayout(main_layout);

  buildTabs();

  // Discovery timer
  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &TasksPanel::onRefreshExecutors);
  refresh_timer_->start(5000);

  // Initial discovery
  QTimer::singleShot(500, this, &TasksPanel::onRefreshExecutors);
}

// ─────────────────────────── build tabs ───────────────────────────────────────

void TasksPanel::buildTabs()
{
  tab_states_.resize(task_defs_.size());

  for (size_t i = 0; i < task_defs_.size(); ++i) {
    const auto & def = task_defs_[i];
    auto & state = tab_states_[i];

    auto * tab = new QWidget();
    auto * splitter = new QSplitter(Qt::Horizontal);

    // ── LEFT: Goal Parameters ──
    auto * left_widget = new QWidget();
    auto * left_layout = new QVBoxLayout(left_widget);

    // Executor selector
    auto * exec_layout = new QHBoxLayout();
    exec_layout->addWidget(new QLabel("Executor:"));
    state.executor_combo = new QComboBox();
    state.executor_combo->setEditable(true);
    state.executor_combo->setMinimumWidth(200);
    exec_layout->addWidget(state.executor_combo);
    left_layout->addLayout(exec_layout);

    // Goal fields
    auto * form = new QFormLayout();
    for (const auto & field : def.goal_fields) {
      auto * widget = buildGoalFieldWidget(field, static_cast<int>(i));
      state.field_widgets[field.name] = widget;
      form->addRow(QString::fromStdString(field.name) + ":", widget);
    }
    left_layout->addLayout(form);

    left_layout->addStretch();

    // Cancel / Execute buttons
    auto * btn_layout = new QHBoxLayout();
    state.cancel_btn = new QPushButton("Cancel");
    state.cancel_btn->setEnabled(false);
    state.execute_btn = new QPushButton("Execute");
    btn_layout->addWidget(state.cancel_btn);
    btn_layout->addStretch();
    btn_layout->addWidget(state.execute_btn);
    left_layout->addLayout(btn_layout);

    int tab_idx = static_cast<int>(i);
    connect(state.execute_btn, &QPushButton::clicked, [this, tab_idx]() {
      // store which tab triggered it, then call handler
      tab_widget_->setCurrentIndex(tab_idx);
      onExecuteClicked();
    });
    connect(state.cancel_btn, &QPushButton::clicked, [this, tab_idx]() {
      tab_widget_->setCurrentIndex(tab_idx);
      onCancelClicked();
    });

    // ── RIGHT: Feedback & Result ──
    auto * right_widget = new QWidget();
    auto * right_layout = new QVBoxLayout(right_widget);

    right_layout->addWidget(new QLabel("Feedback:"));
    state.feedback_display = new QTextEdit();
    state.feedback_display->setReadOnly(true);
    state.feedback_display->setMaximumHeight(200);
    right_layout->addWidget(state.feedback_display);

    right_layout->addWidget(new QLabel("Result:"));
    state.result_display = new QTextEdit();
    state.result_display->setReadOnly(true);
    state.result_display->setMaximumHeight(100);
    right_layout->addWidget(state.result_display);

    state.status_label = new QLabel("Idle");
    right_layout->addWidget(state.status_label);
    right_layout->addStretch();

    splitter->addWidget(left_widget);
    splitter->addWidget(right_widget);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

    auto * tab_layout = new QVBoxLayout(tab);
    tab_layout->addWidget(splitter);

    tab_widget_->addTab(tab, QString::fromStdString(def.display_name));
  }
}

QWidget * TasksPanel::buildGoalFieldWidget(const GoalFieldDef & def, int tab_index)
{
  if (def.ros_type == "float32" || def.ros_type == "float64") {
    auto * spin = new QDoubleSpinBox();
    spin->setRange(def.min_value, def.max_value);
    spin->setValue(def.default_value);
    spin->setDecimals(2);
    spin->setSingleStep(0.1);
    return spin;
  }
  if (def.ros_type == "int32") {
    auto * spin = new QSpinBox();
    spin->setRange(static_cast<int>(def.min_value), static_cast<int>(def.max_value));
    spin->setValue(static_cast<int>(def.default_value));
    return spin;
  }
  if (def.ros_type == "string") {
    return new QLineEdit();
  }
  if (def.ros_type == "bool") {
    auto * cb = new QCheckBox();
    cb->setChecked(def.default_value != 0.0);
    return cb;
  }
  if (def.ros_type == "geometry_msgs/Polygon") {
    auto * container = new QWidget();
    auto * layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);
    auto * btn = new QPushButton("Get Polygon from RViz");
    auto * label = new QLabel("No polygon selected");
    layout->addWidget(btn);
    layout->addWidget(label);
    std::string field_name = def.name;
    connect(btn, &QPushButton::clicked, [this, tab_index, field_name]() {
      onGetPolygon(tab_index, field_name);
    });
    return container;
  }
  if (def.ros_type == "nav_msgs/Path") {
    auto * container = new QWidget();
    auto * layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);

    // Instruction label
    layout->addWidget(new QLabel("Activate Waypoint Tool (key: 1) to place waypoints"));

    // Height / count / selected row
    auto * info_row = new QHBoxLayout();
    info_row->addWidget(new QLabel("Height:"));
    wp_height_spin_ = new QDoubleSpinBox();
    wp_height_spin_->setRange(-100.0, 500.0);
    wp_height_spin_->setSingleStep(0.1);
    wp_height_spin_->setValue(waypoint_manager_ ? waypoint_manager_->defaultHeight() : 0.0);
    wp_height_spin_->setMaximumWidth(80);
    info_row->addWidget(wp_height_spin_);

    wp_count_label_ = new QLabel("Waypoints: 0");
    info_row->addWidget(wp_count_label_);

    wp_selected_label_ = new QLabel("Selected: none");
    info_row->addWidget(wp_selected_label_);
    info_row->addStretch();
    layout->addLayout(info_row);

    // X / Y / Z / Yaw spinboxes for editing selected waypoint
    auto * pose_row = new QHBoxLayout();

    pose_row->addWidget(new QLabel("X:"));
    wp_x_spin_ = new QDoubleSpinBox();
    wp_x_spin_->setRange(-1000.0, 1000.0);
    wp_x_spin_->setSingleStep(0.5);
    pose_row->addWidget(wp_x_spin_);

    pose_row->addWidget(new QLabel("Y:"));
    wp_y_spin_ = new QDoubleSpinBox();
    wp_y_spin_->setRange(-1000.0, 1000.0);
    wp_y_spin_->setSingleStep(0.5);
    pose_row->addWidget(wp_y_spin_);

    pose_row->addWidget(new QLabel("Z:"));
    wp_z_spin_ = new QDoubleSpinBox();
    wp_z_spin_->setRange(-1000.0, 1000.0);
    wp_z_spin_->setSingleStep(0.5);
    pose_row->addWidget(wp_z_spin_);

    pose_row->addWidget(new QLabel("Yaw:"));
    wp_yaw_spin_ = new QDoubleSpinBox();
    wp_yaw_spin_->setRange(-3.15, 3.15);
    wp_yaw_spin_->setSingleStep(0.1);
    pose_row->addWidget(wp_yaw_spin_);
    layout->addLayout(pose_row);

    // Action buttons
    auto * btn_row = new QHBoxLayout();
    auto * clear_btn = new QPushButton("Clear All");
    auto * save_btn = new QPushButton("Save");
    auto * load_btn = new QPushButton("Load");
    btn_row->addWidget(clear_btn);
    btn_row->addWidget(save_btn);
    btn_row->addWidget(load_btn);
    btn_row->addStretch();
    layout->addLayout(btn_row);

    // Connect height spinbox to manager
    connect(wp_height_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
      [this](double val) {
        if (waypoint_manager_) { waypoint_manager_->setDefaultHeight(val); }
      });

    // Connect pose spinboxes to update selected waypoint
    auto pose_changed = [this](double) { onWaypointPoseChanged(0.0); };
    connect(wp_x_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), pose_changed);
    connect(wp_y_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), pose_changed);
    connect(wp_z_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), pose_changed);
    connect(wp_yaw_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), pose_changed);

    // Connect buttons
    connect(clear_btn, &QPushButton::clicked, this, &TasksPanel::onClearWaypoints);
    connect(save_btn, &QPushButton::clicked, this, &TasksPanel::onSaveWaypoints);
    connect(load_btn, &QPushButton::clicked, this, &TasksPanel::onLoadWaypoints);

    return container;
  }
  if (def.ros_type == "airstack_msgs/FixedTrajectory") {
    auto * container = new QWidget();
    auto * layout = new QVBoxLayout(container);
    layout->setContentsMargins(0, 0, 0, 0);

    auto * type_combo = new QComboBox();
    type_combo->addItems({"Circle", "Figure8", "Racetrack", "Line", "Point", "Lawnmower"});
    layout->addWidget(type_combo);

    auto * table = new QTableWidget(0, 2);
    table->setHorizontalHeaderLabels({"Key", "Value"});
    table->horizontalHeader()->setStretchLastSection(true);
    table->setMaximumHeight(150);
    layout->addWidget(table);

    auto * add_btn = new QPushButton("+ Add Attribute");
    layout->addWidget(add_btn);

    connect(add_btn, &QPushButton::clicked, [table]() {
      int row = table->rowCount();
      table->insertRow(row);
      table->setItem(row, 0, new QTableWidgetItem(""));
      table->setItem(row, 1, new QTableWidgetItem(""));
    });

    // Store combo and table for later retrieval
    tab_states_[tab_index].field_data[def.name + "_type_combo"] = type_combo;
    tab_states_[tab_index].field_data[def.name + "_table"] = table;

    connect(type_combo, &QComboBox::currentTextChanged,
      [this, tab_index, field_name = def.name](const QString & type) {
        onTrajectoryTypeChanged(tab_index, type);
      });

    // Trigger initial population
    QTimer::singleShot(0, [type_combo]() {
      Q_EMIT type_combo->currentTextChanged(type_combo->currentText());
    });

    return container;
  }

  return new QLabel("(unsupported type: " + QString::fromStdString(def.ros_type) + ")");
}

// ─────────────────────────── executor discovery ──────────────────────────────

void TasksPanel::onRefreshExecutors()
{
  if (!raw_node_) {return;}

  auto topic_map = raw_node_->get_topic_names_and_types();

  // Collect robot namespaces and per-task executors
  std::set<std::string> robots;

  for (size_t i = 0; i < task_defs_.size(); ++i) {
    const std::string suffix = task_defs_[i].action_topic_suffix + "/_action/status";
    QStringList executors;

    for (const auto & [topic, types] : topic_map) {
      if (topic.size() > suffix.size() &&
        topic.substr(topic.size() - suffix.size()) == suffix)
      {
        // Extract action name (remove /_action/status)
        std::string action_name = topic.substr(0, topic.size() - std::string("/_action/status").size());
        executors.append(QString::fromStdString(action_name));

        // Extract robot namespace (everything before /tasks/...)
        std::string task_suffix = task_defs_[i].action_topic_suffix;
        size_t pos = action_name.rfind(task_suffix);
        if (pos != std::string::npos && pos > 1) {
          robots.insert(action_name.substr(0, pos - 1));  // -1 to remove trailing /
        }
      }
    }

    // Update executor combo preserving current selection
    auto & combo = tab_states_[i].executor_combo;
    QString current = combo->currentText();
    combo->clear();
    combo->addItems(executors);
    if (!current.isEmpty()) {
      int idx = combo->findText(current);
      if (idx >= 0) {
        combo->setCurrentIndex(idx);
      } else {
        combo->setCurrentText(current);
      }
    }
  }

  // Update robot combo
  QString current_robot = robot_combo_->currentText();
  robot_combo_->clear();
  for (const auto & r : robots) {
    robot_combo_->addItem(QString::fromStdString(r));
  }
  if (!current_robot.isEmpty()) {
    int idx = robot_combo_->findText(current_robot);
    if (idx >= 0) {
      robot_combo_->setCurrentIndex(idx);
    } else {
      robot_combo_->setCurrentText(current_robot);
    }
  }
}

// ─────────────────────────── special widgets ──────────────────────────────────

void TasksPanel::onGetPolygon(int tab_index, const std::string & field_name)
{
  if (!polygon_client_->wait_for_service(std::chrono::seconds(0))) {
    tab_states_[tab_index].status_label->setText("Polygon tool not active - press 'p' first");
    return;
  }

  auto request = std::make_shared<rviz_polygon_selection_tool::srv::GetSelection::Request>();
  polygon_client_->async_send_request(request,
    [this, tab_index, field_name](
      rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedFuture future)
    {
      auto response = future.get();
      if (response->selection.empty()) {
        QMetaObject::invokeMethod(this, [this, tab_index]() {
          tab_states_[tab_index].status_label->setText("No polygons drawn");
        }, Qt::QueuedConnection);
        return;
      }

      // Take first polygon
      geometry_msgs::msg::Polygon poly;
      for (const auto & pt : response->selection[0].polygon.points) {
        geometry_msgs::msg::Point32 p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        poly.points.push_back(p);
      }

      QMetaObject::invokeMethod(this, [this, tab_index, field_name, poly]() {
        tab_states_[tab_index].field_data[field_name] = poly;
        // Update label in the polygon widget
        auto * container = tab_states_[tab_index].field_widgets[field_name];
        auto * label = container->findChild<QLabel *>();
        if (label) {
          label->setText(QString("Polygon: %1 points").arg(poly.points.size()));
        }
      }, Qt::QueuedConnection);
    });
}

void TasksPanel::onTrajectoryTypeChanged(int tab_index, const QString & type)
{
  std::string key = "trajectory_spec_table";
  auto it = tab_states_[tab_index].field_data.find(key);
  if (it == tab_states_[tab_index].field_data.end()) {return;}

  auto * table = std::any_cast<QTableWidget *>(it->second);
  table->setRowCount(0);

  // Default attributes per trajectory type
  std::vector<std::pair<std::string, std::string>> attrs;
  if (type == "Circle") {
    attrs = {{"frame_id", "base_link"}, {"radius", "5.0"}, {"velocity", "2.0"}};
  } else if (type == "Figure8") {
    attrs = {{"frame_id", "base_link"}, {"length", "10.0"}, {"width", "5.0"},
             {"height", "0.0"}, {"velocity", "2.0"}, {"max_acceleration", "1.0"}};
  } else if (type == "Racetrack") {
    attrs = {{"frame_id", "base_link"}, {"length", "20.0"}, {"width", "10.0"},
             {"height", "0.0"}, {"velocity", "2.0"}, {"turn_velocity", "1.0"},
             {"max_acceleration", "1.0"}};
  } else if (type == "Line") {
    attrs = {{"frame_id", "base_link"}, {"length", "10.0"}, {"height", "0.0"},
             {"velocity", "2.0"}, {"max_acceleration", "1.0"}};
  } else if (type == "Point") {
    attrs = {{"frame_id", "base_link"}, {"x", "5.0"}, {"y", "0.0"},
             {"height", "0.0"}, {"velocity", "2.0"}, {"max_acceleration", "1.0"}};
  } else if (type == "Lawnmower") {
    attrs = {{"frame_id", "base_link"}, {"length", "20.0"}, {"width", "5.0"},
             {"height", "10.0"}, {"velocity", "2.0"}, {"vertical", "0"}};
  }

  for (const auto & [k, v] : attrs) {
    int row = table->rowCount();
    table->insertRow(row);
    table->setItem(row, 0, new QTableWidgetItem(QString::fromStdString(k)));
    table->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(v)));
  }
}

// ─────────────────────────── value extraction ─────────────────────────────────

double TasksPanel::getFloat(int tab_index, const std::string & field_name)
{
  auto * w = tab_states_[tab_index].field_widgets[field_name];
  if (auto * spin = qobject_cast<QDoubleSpinBox *>(w)) {return spin->value();}
  return 0.0;
}

int32_t TasksPanel::getInt(int tab_index, const std::string & field_name)
{
  auto * w = tab_states_[tab_index].field_widgets[field_name];
  if (auto * spin = qobject_cast<QSpinBox *>(w)) {return spin->value();}
  return 0;
}

std::string TasksPanel::getString(int tab_index, const std::string & field_name)
{
  auto * w = tab_states_[tab_index].field_widgets[field_name];
  if (auto * edit = qobject_cast<QLineEdit *>(w)) {return edit->text().toStdString();}
  return "";
}

bool TasksPanel::getBool(int tab_index, const std::string & field_name)
{
  auto * w = tab_states_[tab_index].field_widgets[field_name];
  if (auto * cb = qobject_cast<QCheckBox *>(w)) {return cb->isChecked();}
  return false;
}

geometry_msgs::msg::Polygon TasksPanel::getPolygon(int tab_index, const std::string & field_name)
{
  auto it = tab_states_[tab_index].field_data.find(field_name);
  if (it != tab_states_[tab_index].field_data.end()) {
    return std::any_cast<geometry_msgs::msg::Polygon>(it->second);
  }
  return geometry_msgs::msg::Polygon();
}

nav_msgs::msg::Path TasksPanel::getPath(int, const std::string &)
{
  if (waypoint_manager_ && waypoint_manager_->isInitialized()) {
    return waypoint_manager_->getPath();
  }
  return nav_msgs::msg::Path();
}

airstack_msgs::msg::FixedTrajectory TasksPanel::getFixedTrajectory(
  int tab_index, const std::string & field_name)
{
  airstack_msgs::msg::FixedTrajectory ft;

  auto combo_it = tab_states_[tab_index].field_data.find(field_name + "_type_combo");
  if (combo_it != tab_states_[tab_index].field_data.end()) {
    auto * combo = std::any_cast<QComboBox *>(combo_it->second);
    ft.type = combo->currentText().toStdString();
  }

  auto table_it = tab_states_[tab_index].field_data.find(field_name + "_table");
  if (table_it != tab_states_[tab_index].field_data.end()) {
    auto * table = std::any_cast<QTableWidget *>(table_it->second);
    for (int row = 0; row < table->rowCount(); ++row) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = table->item(row, 0)->text().toStdString();
      kv.value = table->item(row, 1)->text().toStdString();
      ft.attributes.push_back(kv);
    }
  }

  return ft;
}

// ─────────────────────────── UI state helpers ─────────────────────────────────

void TasksPanel::setGoalActive(int tab_index, bool active)
{
  auto & state = tab_states_[tab_index];
  state.goal_active = active;
  state.cancel_btn->setEnabled(active);

  if (active) {
    active_task_tab_ = tab_index;
    state.feedback_display->clear();
    state.result_display->clear();
    state.status_label->setText("Running...");
    state.status_label->setStyleSheet("color: blue;");
    setTabStatus(tab_index, QString::fromUtf8(u8"⏳"), Qt::blue); 
  } else {
    active_task_tab_ = -1;
  }

  // Disable all Execute buttons while any task is running
  for (size_t i = 0; i < tab_states_.size(); ++i) {
    tab_states_[i].execute_btn->setEnabled(active_task_tab_ == -1);
  }
}

void TasksPanel::setTabStatus(int tab_index, const QString & icon, const QColor & text_color)
{
  tab_widget_->tabBar()->setTabTextColor(tab_index, text_color);
  const auto & def = task_defs_[tab_index];
  tab_widget_->setTabText(tab_index, icon + " " + QString::fromStdString(def.display_name));
}

void TasksPanel::clearTabStatus(int tab_index)
{
  tab_widget_->tabBar()->setTabTextColor(tab_index, QColor());
  const auto & def = task_defs_[tab_index];
  tab_widget_->setTabText(tab_index, QString::fromStdString(def.display_name));
}

QString TasksPanel::currentRobot() const
{
  return robot_combo_->currentText();
}

// ─────────────────────────── template: send/cancel ────────────────────────────

template<typename ActionT>
void TasksPanel::doSendGoal(
  int tab_index,
  const typename ActionT::Goal & goal,
  std::function<QString(const typename ActionT::Feedback &)> fmt_feedback,
  std::function<QString(const typename ActionT::Result::SharedPtr)> fmt_result)
{
  auto & state = tab_states_[tab_index];
  std::string action_name = state.executor_combo->currentText().toStdString();
  if (action_name.empty()) {
    state.status_label->setText("No executor selected");
    state.status_label->setStyleSheet("color: red;");
    return;
  }

  auto client = rclcpp_action::create_client<ActionT>(raw_node_, action_name);
  state.action_client = client;

  if (!client->wait_for_action_server(std::chrono::seconds(2))) {
    state.status_label->setText("Action server not available");
    state.status_label->setStyleSheet("color: red;");
    return;
  }

  // Reset all tab colors from previous task results
  for (size_t i = 0; i < tab_states_.size(); ++i) {
    clearTabStatus(static_cast<int>(i));
  }
  setGoalActive(tab_index, true);

  auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();

  // Feedback callback
  send_goal_options.feedback_callback =
    [this, tab_index, fmt_feedback](
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr,
      const std::shared_ptr<const typename ActionT::Feedback> feedback)
    {
      QString text = fmt_feedback(*feedback);
      QMetaObject::invokeMethod(this, [this, tab_index, text]() {
        auto & state = tab_states_[tab_index];
        state.feedback_display->append(
          "[" + QTime::currentTime().toString("HH:mm:ss") + "] " + text);
        // Auto-scroll to bottom
        auto cursor = state.feedback_display->textCursor();
        cursor.movePosition(QTextCursor::End);
        state.feedback_display->setTextCursor(cursor);
      }, Qt::QueuedConnection);
    };

  // Result callback
  send_goal_options.result_callback =
    [this, tab_index, fmt_result](
      const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & wrapped_result)
    {
      QString result_text;
      QString status_text;
      QString color;
      QString tab_icon;
      QColor tab_text_color;

      switch (wrapped_result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          result_text = fmt_result(wrapped_result.result);
          status_text = "Succeeded";
          color = "color: darkGreen;";
          tab_icon = QString::fromUtf8(u8"✅");
          tab_text_color = Qt::darkGreen;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          result_text = fmt_result(wrapped_result.result);
          status_text = "Aborted";
          color = "color: red;";
          tab_icon = QString::fromUtf8(u8"❌");
          tab_text_color = Qt::red;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          result_text = "Goal canceled";
          status_text = "Canceled";
          color = "color: orange;";
          tab_icon = QString::fromUtf8(u8"🚫");
          tab_text_color = QColor("orange");
          break;
        default:
          result_text = "Unknown result";
          status_text = "Unknown";
          color = "color: gray;";
          tab_icon = "?";
          tab_text_color = Qt::gray;
          break;
      }

      QMetaObject::invokeMethod(this, [this, tab_index, result_text, status_text, color,
                                        tab_icon, tab_text_color]() {
        auto & state = tab_states_[tab_index];
        state.result_display->setText(result_text);
        state.status_label->setText(status_text);
        state.status_label->setStyleSheet(color);
        setTabStatus(tab_index, tab_icon, tab_text_color);
        setGoalActive(tab_index, false);
      }, Qt::QueuedConnection);
    };

  auto goal_future = client->async_send_goal(goal, send_goal_options);
}

template<typename ActionT>
void TasksPanel::doCancelGoal(int tab_index)
{
  auto & state = tab_states_[tab_index];
  try {
    auto client = std::any_cast<typename rclcpp_action::Client<ActionT>::SharedPtr>(
      state.action_client);
    client->async_cancel_all_goals();
    state.status_label->setText("Cancelling...");
    state.status_label->setStyleSheet("color: orange;");
  } catch (const std::bad_any_cast &) {
    state.status_label->setText("No active goal to cancel");
  }
}

// ─────────────────────────── execute dispatch ─────────────────────────────────

void TasksPanel::onExecuteClicked()
{
  int idx = tab_widget_->currentIndex();
  if (idx < 0 || idx >= static_cast<int>(tab_states_.size())) {return;}
  if (active_task_tab_ >= 0) {return;}  // only one task at a time

  switch (idx) {
    case 0: {  // Takeoff
      task_msgs::action::TakeoffTask::Goal goal;
      goal.target_altitude_m = getFloat(0, "target_altitude_m");
      goal.velocity_m_s = getFloat(0, "velocity_m_s");
      doSendGoal<task_msgs::action::TakeoffTask>(0, goal,
        [](const auto & fb) {
          return QString("status: %1 | alt: %2 / %3 m")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.current_altitude_m, 0, 'f', 1)
            .arg(fb.target_altitude_m, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message));
        });
      break;
    }
    case 1: {  // Land
      task_msgs::action::LandTask::Goal goal;
      goal.velocity_m_s = getFloat(1, "velocity_m_s");
      doSendGoal<task_msgs::action::LandTask>(1, goal,
        [](const auto & fb) {
          return QString("status: %1 | alt: %2 m")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.current_altitude_m, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message));
        });
      break;
    }
    case 2: {  // Navigate
      task_msgs::action::NavigateTask::Goal goal;
      goal.global_plan = getPath(2, "global_plan");
      goal.goal_tolerance_m = getFloat(2, "goal_tolerance_m");
      doSendGoal<task_msgs::action::NavigateTask>(2, goal,
        [](const auto & fb) {
          return QString("status: %1 | dist: %2 m | pos: (%3, %4, %5)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.distance_to_goal, 0, 'f', 1)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message));
        });
      break;
    }
    case 3: {  // Exploration
      task_msgs::action::ExplorationTask::Goal goal;
      goal.search_bounds = getPolygon(3, "search_bounds");
      goal.min_altitude_agl = getFloat(3, "min_altitude_agl");
      goal.max_altitude_agl = getFloat(3, "max_altitude_agl");
      goal.min_flight_speed = getFloat(3, "min_flight_speed");
      goal.max_flight_speed = getFloat(3, "max_flight_speed");
      goal.time_limit_sec = getFloat(3, "time_limit_sec");
      doSendGoal<task_msgs::action::ExplorationTask>(3, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | pos: (%3, %4, %5)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message));
        });
      break;
    }
    case 4: {  // Coverage
      task_msgs::action::CoverageTask::Goal goal;
      goal.coverage_area = getPolygon(4, "coverage_area");
      goal.min_altitude_agl = getFloat(4, "min_altitude_agl");
      goal.max_altitude_agl = getFloat(4, "max_altitude_agl");
      goal.min_flight_speed = getFloat(4, "min_flight_speed");
      goal.max_flight_speed = getFloat(4, "max_flight_speed");
      goal.line_spacing_m = getFloat(4, "line_spacing_m");
      goal.heading_deg = getFloat(4, "heading_deg");
      doSendGoal<task_msgs::action::CoverageTask>(4, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | coverage: %3% | pos: (%4, %5, %6)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.coverage_percentage, 0, 'f', 1)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2\ncoverage: %3%")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message))
            .arg(r->coverage_percentage, 0, 'f', 1);
        });
      break;
    }
    case 5: {  // Object Search
      task_msgs::action::ObjectSearchTask::Goal goal;
      goal.object_class = getString(5, "object_class");
      goal.search_area = getPolygon(5, "search_area");
      goal.min_altitude_agl = getFloat(5, "min_altitude_agl");
      goal.max_altitude_agl = getFloat(5, "max_altitude_agl");
      goal.min_flight_speed = getFloat(5, "min_flight_speed");
      goal.max_flight_speed = getFloat(5, "max_flight_speed");
      goal.time_limit_sec = getFloat(5, "time_limit_sec");
      goal.target_count = getInt(5, "target_count");
      doSendGoal<task_msgs::action::ObjectSearchTask>(5, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | found: %3 | pos: (%4, %5, %6)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.objects_found_so_far)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2\nobjects_found: %3")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message))
            .arg(r->objects_found);
        });
      break;
    }
    case 6: {  // Object Counting
      task_msgs::action::ObjectCountingTask::Goal goal;
      goal.object_class = getString(6, "object_class");
      goal.count_area = getPolygon(6, "count_area");
      goal.min_altitude_agl = getFloat(6, "min_altitude_agl");
      goal.max_altitude_agl = getFloat(6, "max_altitude_agl");
      goal.min_flight_speed = getFloat(6, "min_flight_speed");
      goal.max_flight_speed = getFloat(6, "max_flight_speed");
      doSendGoal<task_msgs::action::ObjectCountingTask>(6, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | count: %3 | pos: (%4, %5, %6)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.current_count)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2\ncount: %3")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message))
            .arg(r->count);
        });
      break;
    }
    case 7: {  // Semantic Search
      task_msgs::action::SemanticSearchTask::Goal goal;
      goal.query = getString(7, "query");
      goal.search_area = getPolygon(7, "search_area");
      goal.min_altitude_agl = getFloat(7, "min_altitude_agl");
      goal.max_altitude_agl = getFloat(7, "max_altitude_agl");
      goal.min_flight_speed = getFloat(7, "min_flight_speed");
      goal.max_flight_speed = getFloat(7, "max_flight_speed");
      goal.time_limit_sec = getFloat(7, "time_limit_sec");
      goal.confidence_threshold = getFloat(7, "confidence_threshold");
      doSendGoal<task_msgs::action::SemanticSearchTask>(7, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | best_conf: %3 | pos: (%4, %5, %6)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.best_confidence_so_far, 0, 'f', 3)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2\nconfidence: %3")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message))
            .arg(r->confidence, 0, 'f', 3);
        });
      break;
    }
    case 8: {  // Fixed Trajectory
      task_msgs::action::FixedTrajectoryTask::Goal goal;
      goal.trajectory_spec = getFixedTrajectory(8, "trajectory_spec");
      goal.loop = getBool(8, "loop");
      doSendGoal<task_msgs::action::FixedTrajectoryTask>(8, goal,
        [](const auto & fb) {
          return QString("status: %1 | progress: %2 | pos: (%3, %4, %5)")
            .arg(QString::fromStdString(fb.status))
            .arg(fb.progress, 0, 'f', 2)
            .arg(fb.current_position.x, 0, 'f', 1)
            .arg(fb.current_position.y, 0, 'f', 1)
            .arg(fb.current_position.z, 0, 'f', 1);
        },
        [](const auto & r) {
          return QString("success: %1\nmessage: %2")
            .arg(r->success ? "true" : "false")
            .arg(QString::fromStdString(r->message));
        });
      break;
    }
  }
}

// ─────────────────────────── cancel dispatch ──────────────────────────────────

void TasksPanel::onCancelClicked()
{
  int idx = tab_widget_->currentIndex();
  if (idx < 0 || idx >= static_cast<int>(tab_states_.size())) {return;}
  if (!tab_states_[idx].goal_active) {return;}

  switch (idx) {
    case 0: doCancelGoal<task_msgs::action::TakeoffTask>(0); break;
    case 1: doCancelGoal<task_msgs::action::LandTask>(1); break;
    case 2: doCancelGoal<task_msgs::action::NavigateTask>(2); break;
    case 3: doCancelGoal<task_msgs::action::ExplorationTask>(3); break;
    case 4: doCancelGoal<task_msgs::action::CoverageTask>(4); break;
    case 5: doCancelGoal<task_msgs::action::ObjectSearchTask>(5); break;
    case 6: doCancelGoal<task_msgs::action::ObjectCountingTask>(6); break;
    case 7: doCancelGoal<task_msgs::action::SemanticSearchTask>(7); break;
    case 8: doCancelGoal<task_msgs::action::FixedTrajectoryTask>(8); break;
  }
}

// ─────────────────────────── config persistence ──────────────────────────────

void TasksPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("robot", robot_combo_->currentText());
  for (size_t i = 0; i < tab_states_.size(); ++i) {
    config.mapSetValue(
      QString("executor_%1").arg(i),
      tab_states_[i].executor_combo->currentText());
  }
}

void TasksPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString robot;
  if (config.mapGetString("robot", &robot)) {
    robot_combo_->setCurrentText(robot);
  }
  for (size_t i = 0; i < tab_states_.size(); ++i) {
    QString executor;
    if (config.mapGetString(QString("executor_%1").arg(i), &executor)) {
      tab_states_[i].executor_combo->setCurrentText(executor);
    }
  }
}

// ─────────────────────────── waypoint slots ──────────────────────────────────

void TasksPanel::onWaypointCountChanged(int count)
{
  if (wp_count_label_) {
    wp_count_label_->setText(QString("Waypoints: %1").arg(count));
  }
}

void TasksPanel::onSelectedMarkerChanged(
  const QString & name, double x, double y, double z, double yaw)
{
  if (wp_selected_label_) {
    wp_selected_label_->setText(QString("Selected: %1").arg(name));
  }

  // Update spinboxes without triggering poseChanged
  wp_updating_spinboxes_ = true;
  if (wp_x_spin_) { wp_x_spin_->setValue(x); }
  if (wp_y_spin_) { wp_y_spin_->setValue(y); }
  if (wp_z_spin_) { wp_z_spin_->setValue(z); }
  if (wp_yaw_spin_) { wp_yaw_spin_->setValue(yaw); }
  wp_updating_spinboxes_ = false;
}

void TasksPanel::onClearWaypoints()
{
  if (waypoint_manager_) { waypoint_manager_->clearAll(); }
}

void TasksPanel::onSaveWaypoints()
{
  QString filename = QFileDialog::getSaveFileName(
    this, tr("Save Waypoints"), "waypoints", tr("Bag Files (*.db3)"));
  if (filename.isEmpty()) { return; }
  if (waypoint_manager_) { waypoint_manager_->saveBag(filename.toStdString()); }
}

void TasksPanel::onLoadWaypoints()
{
  QString filename = QFileDialog::getOpenFileName(
    this, tr("Load Waypoints"), "~/", tr("Bag Files (*.db3)"));
  if (filename.isEmpty()) { return; }
  if (waypoint_manager_) { waypoint_manager_->loadBag(filename.toStdString()); }
}

void TasksPanel::onWaypointPoseChanged(double)
{
  if (wp_updating_spinboxes_ || !waypoint_manager_) { return; }
  waypoint_manager_->updateSelectedPose(
    wp_x_spin_->value(), wp_y_spin_->value(),
    wp_z_spin_->value(), wp_yaw_spin_->value());
}

}  // namespace rviz_tasks_panel

PLUGINLIB_EXPORT_CLASS(rviz_tasks_panel::TasksPanel, rviz_common::Panel)
