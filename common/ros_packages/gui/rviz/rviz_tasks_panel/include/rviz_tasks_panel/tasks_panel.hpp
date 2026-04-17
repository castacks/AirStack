// Copyright (c) 2024 Carnegie Mellon University
// MIT License

#pragma once

#include <any>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <nav_msgs/msg/path.hpp>
#include <airstack_msgs/msg/fixed_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rviz_polygon_selection_tool/srv/get_selection.hpp>

#include <task_msgs/action/takeoff_task.hpp>
#include <task_msgs/action/land_task.hpp>
#include <task_msgs/action/navigate_task.hpp>
#include <task_msgs/action/exploration_task.hpp>
#include <task_msgs/action/coverage_task.hpp>
#include <task_msgs/action/semantic_search_task.hpp>
#include <task_msgs/action/fixed_trajectory_task.hpp>

#include <waypoint_rviz2_plugin/waypoint_manager.hpp>

namespace rviz_tasks_panel
{

// Goal field definition for auto-generating widgets
struct GoalFieldDef
{
  std::string name;
  std::string ros_type;  // "float32", "int32", "string", "bool",
                         // "geometry_msgs/Polygon", "nav_msgs/Path",
                         // "airstack_msgs/FixedTrajectory"
  double default_value{0.0};
  double min_value{0.0};
  double max_value{10000.0};
};

// Task type definition
struct TaskTypeDef
{
  std::string display_name;
  std::string action_topic_suffix;  // e.g. "tasks/takeoff"
  std::vector<GoalFieldDef> goal_fields;
  bool requires_airborne{false};  // if true, Execute is disabled while drone is on the ground
};

// Per-tab runtime state
struct TaskTabState
{
  QComboBox * executor_combo{nullptr};
  std::map<std::string, QWidget *> field_widgets;
  std::map<std::string, std::any> field_data;  // complex types (Polygon, Path)
  QPushButton * cancel_btn{nullptr};
  QPushButton * execute_btn{nullptr};
  QTextEdit * feedback_display{nullptr};
  QTextEdit * result_display{nullptr};
  QLabel * status_label{nullptr};
  std::any action_client;
  std::any goal_handle;
  bool goal_active{false};
};

class TasksPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit TasksPanel(QWidget * parent = nullptr);
  ~TasksPanel() override = default;
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onRefreshExecutors();
  void onExecuteClicked();
  void onCancelClicked();
  void onGetPolygon(int tab_index, const std::string & field_name);
  void onTrajectoryTypeChanged(int tab_index, const QString & type);

  // Waypoint UI slots
  void onWaypointCountChanged(int count);
  void onSelectedMarkerChanged(
    const QString & name, double x, double y, double z, double yaw);
  void onClearWaypoints();
  void onSaveWaypoints();
  void onLoadWaypoints();
  void onWaypointPoseChanged(double value);

private:
  rclcpp::Node::SharedPtr raw_node_;

  // Top bar
  QComboBox * robot_combo_{nullptr};

  // Tabs
  QTabWidget * tab_widget_{nullptr};
  std::vector<TaskTypeDef> task_defs_;
  std::vector<TaskTabState> tab_states_;

  // Polygon selection service
  rclcpp::Client<rviz_polygon_selection_tool::srv::GetSelection>::SharedPtr polygon_client_;

  // Shared waypoint manager (singleton, also held by WaypointTool)
  std::shared_ptr<waypoint_rviz2_plugin::WaypointManager> waypoint_manager_;

  // Waypoint UI widgets (Navigate tab)
  QLabel * wp_count_label_{nullptr};
  QLabel * wp_selected_label_{nullptr};
  QDoubleSpinBox * wp_height_spin_{nullptr};
  QDoubleSpinBox * wp_x_spin_{nullptr};
  QDoubleSpinBox * wp_y_spin_{nullptr};
  QDoubleSpinBox * wp_z_spin_{nullptr};
  QDoubleSpinBox * wp_yaw_spin_{nullptr};
  bool wp_updating_spinboxes_{false};  // guard against signal loops

  // Global task lock: only one task may run at a time
  int active_task_tab_{-1};  // -1 = no task running, else = tab index of active task

  // Airborne state (fed by is_airborne topic from takeoff_landing_planner)
  bool is_airborne_{false};
  QString is_airborne_robot_;  // robot namespace currently subscribed to
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_airborne_sub_;

  // Discovery timer
  QTimer * refresh_timer_{nullptr};

  static std::vector<TaskTypeDef> getTaskDefs();
  void buildTabs();
  QWidget * buildGoalFieldWidget(const GoalFieldDef & def, int tab_index);
  void updateAvailableExecutors();

  // Widget value extraction helpers
  double getFloat(int tab_index, const std::string & field_name);
  int32_t getInt(int tab_index, const std::string & field_name);
  std::string getString(int tab_index, const std::string & field_name);
  bool getBool(int tab_index, const std::string & field_name);
  geometry_msgs::msg::Polygon getPolygon(int tab_index, const std::string & field_name);
  nav_msgs::msg::Path getPath(int tab_index, const std::string & field_name);
  airstack_msgs::msg::FixedTrajectory getFixedTrajectory(int tab_index, const std::string & field_name);

  // Template for sending goals with type-specific callbacks
  template<typename ActionT>
  void doSendGoal(
    int tab_index,
    const typename ActionT::Goal & goal,
    std::function<QString(const typename ActionT::Feedback &)> fmt_feedback,
    std::function<QString(const typename ActionT::Result::SharedPtr)> fmt_result);

  template<typename ActionT>
  void doCancelGoal(int tab_index);

  void setGoalActive(int tab_index, bool active);
  void updateExecuteButtons();
  void renewAirborneSubscription();
  void setTabStatus(int tab_index, const QString & icon, const QColor & text_color);
  void clearTabStatus(int tab_index);
  QString currentRobot() const;
};

}  // namespace rviz_tasks_panel
