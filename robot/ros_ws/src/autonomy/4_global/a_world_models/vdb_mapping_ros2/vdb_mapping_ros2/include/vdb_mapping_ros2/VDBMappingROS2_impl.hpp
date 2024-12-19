// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2022-05-07
 *
 */
//----------------------------------------------------------------------


template <typename VDBMappingT>
VDBMappingROS2<VDBMappingT>::VDBMappingROS2(const rclcpp::NodeOptions& options)
  : Node("vdb_mapping_ros2", options)
{
  using namespace std::placeholders;

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  this->declare_parameter<double>("resolution", 0.1);
  this->get_parameter("resolution", m_resolution);
  m_vdb_map = std::make_shared<VDBMappingT>(m_resolution);

  this->declare_parameter<double>("max_range", 10.0);
  this->get_parameter("max_range", m_config.max_range);
  this->declare_parameter<double>("prob_hit", 0.7);
  this->get_parameter("prob_hit", m_config.prob_hit);
  this->declare_parameter<double>("prob_miss", 0.4);
  this->get_parameter("prob_miss", m_config.prob_miss);
  this->declare_parameter<double>("prob_thres_min", 0.12);
  this->get_parameter("prob_thres_min", m_config.prob_thres_min);
  this->declare_parameter<double>("prob_thres_max", 0.97);
  this->get_parameter("prob_thres_max", m_config.prob_thres_max);
  this->declare_parameter<std::string>("map_directory_path", "");
  this->get_parameter("map_directory_path", m_config.map_directory_path);
  this->declare_parameter<int>("two_dim_projection_threshold", 5);
  this->get_parameter("two_dim_projection_threshold", m_two_dim_projection_threshold);

  // Configuring the VDB map
  m_vdb_map->setConfig(m_config);

  this->declare_parameter<bool>("publish_pointcloud", true);
  this->get_parameter("publish_pointcloud", m_publish_pointcloud);
  this->declare_parameter<bool>("publish_vis_marker", true);
  this->get_parameter("publish_vis_marker", m_publish_vis_marker);
  this->declare_parameter<bool>("publish_updates", true);
  this->get_parameter("publish_updates", m_publish_updates);
  this->declare_parameter<bool>("publish_overwrites", true);
  this->get_parameter("publish_overwrites", m_publish_overwrites);
  this->declare_parameter<bool>("publish_sections", true);
  this->get_parameter("publish_sections", m_publish_sections);
  this->declare_parameter<bool>("apply_raw_sensor_data", true);
  this->get_parameter("apply_raw_sensor_data", m_apply_raw_sensor_data);


  this->declare_parameter<double>("z_limit_min", 0);
  this->get_parameter("z_limit_min", m_lower_visualization_z_limit);
  this->declare_parameter<double>("z_limit_max", 0);
  this->get_parameter("z_limit_max", m_upper_visualization_z_limit);

  m_param_sub = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto min_z_cb = [this](const rclcpp::Parameter &p) {
    m_lower_visualization_z_limit = p.as_double();
  };
  auto max_z_cb = [this](const rclcpp::Parameter &p) {
    m_upper_visualization_z_limit = p.as_double();
  };

  m_z_min_param_handle = m_param_sub->add_parameter_callback("z_limit_min", min_z_cb);
  m_z_min_param_handle = m_param_sub->add_parameter_callback("z_limit_max", max_z_cb);


  this->declare_parameter<std::string>("map_frame", "");
  this->get_parameter("map_frame", m_map_frame);
  if (m_map_frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No map frame specified");
  }
  m_vdb_map->getGrid()->insertMeta("ros/map_frame", openvdb::StringMetadata(m_map_frame));
  this->declare_parameter<std::string>("robot_frame", "");
  this->get_parameter("robot_frame", m_robot_frame);
  if (m_robot_frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No robot frame specified");
  }

  // Setting up remote sources
  std::vector<std::string> source_ids;
  this->declare_parameter<std::vector<std::string> >("remote_sources", std::vector<std::string>());
  this->get_parameter("remote_sources", source_ids);

  for (auto& source_id : source_ids)
  {
    std::string remote_namespace;
    this->declare_parameter<std::string>(source_id + ".namespace", "");
    this->get_parameter(source_id + ".namespace", remote_namespace);

    RemoteSource remote_source;
    this->declare_parameter<bool>(source_id + ".apply_remote_updates", false);
    this->get_parameter(source_id + ".apply_remote_updates", remote_source.apply_remote_updates);
    this->declare_parameter<bool>(source_id + ".apply_remote_overwrites", false);
    this->get_parameter(source_id + ".apply_remote_overwrites",
                        remote_source.apply_remote_overwrites);
    this->declare_parameter<bool>(source_id + ".apply_remote_sections", false);
    this->get_parameter(source_id + ".apply_remote_sections", remote_source.apply_remote_sections);
    if (remote_source.apply_remote_updates)
    {
      remote_source.map_update_sub =
        this->create_subscription<vdb_mapping_interfaces::msg::UpdateGrid>(
          remote_namespace + "/vdb_map_updates",
          rclcpp::QoS(10).durability_volatile().best_effort(),
          std::bind(&VDBMappingROS2::mapUpdateCallback, this, _1));
    }
    if (remote_source.apply_remote_overwrites)
    {
      remote_source.map_overwrite_sub =
        this->create_subscription<vdb_mapping_interfaces::msg::UpdateGrid>(
          remote_namespace + "/vdb_map_overwrites",
          rclcpp::QoS(10).durability_volatile().best_effort(),
          std::bind(&VDBMappingROS2::mapOverwriteCallback, this, _1));
    }
    if (remote_source.apply_remote_sections)
    {
      remote_source.map_section_sub =
        this->create_subscription<vdb_mapping_interfaces::msg::UpdateGrid>(
          remote_namespace + "/vdb_map_sections",
          rclcpp::QoS(10).durability_volatile().best_effort(),
          std::bind(&VDBMappingROS2::mapSectionCallback, this, _1));
    }
    remote_source.get_map_section_client =
      this->create_client<vdb_mapping_interfaces::srv::GetMapSection>(remote_namespace +
                                                                      "/get_map_section");
    m_remote_sources.insert(std::make_pair(source_id, remote_source));
  }

  if (m_publish_updates)
  {
    m_map_update_pub = this->create_publisher<vdb_mapping_interfaces::msg::UpdateGrid>(
      "~/vdb_map_updates", rclcpp::QoS(1).durability_volatile().best_effort());
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub = this->create_publisher<vdb_mapping_interfaces::msg::UpdateGrid>(
      "~/vdb_map_overwrites", rclcpp::QoS(1).durability_volatile().best_effort());
  }
  if (m_publish_sections)
  {
    m_map_section_pub = this->create_publisher<vdb_mapping_interfaces::msg::UpdateGrid>(
      "~/vdb_map_sections", rclcpp::QoS(1).durability_volatile().best_effort());

    double section_update_rate;
    this->declare_parameter<double>("section_update.rate", 1);
    this->get_parameter("section_update.rate", section_update_rate);
    m_section_timer =
      this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / section_update_rate)),
                              std::bind(&VDBMappingROS2::sectionTimerCallback, this));

    this->declare_parameter<double>("section_update.min_coord.x", -10);
    this->get_parameter("section_update.min_coord.x", m_section_min_coord.x());
    this->declare_parameter<double>("section_update.min_coord.y", -10);
    this->get_parameter("section_update.min_coord.y", m_section_min_coord.y());
    this->declare_parameter<double>("section_update.min_coord.z", -10);
    this->get_parameter("section_update.min_coord.z", m_section_min_coord.z());
    this->declare_parameter<double>("section_update.max_coord.x", 10);
    this->get_parameter("section_update.max_coord.x", m_section_max_coord.x());
    this->declare_parameter<double>("section_update.max_coord.y", 10);
    this->get_parameter("section_update.max_coord.y", m_section_max_coord.y());
    this->declare_parameter<double>("section_update.max_coord.z", 10);
    this->get_parameter("section_update.max_coord.z", m_section_max_coord.z());
    this->declare_parameter<std::string>("section_update.frame", m_robot_frame);
    this->get_parameter("section_update.frame", m_section_update_frame);
  }

  if (m_apply_raw_sensor_data)
  {
    this->declare_parameter<std::vector<std::string> >("sources", std::vector<std::string>());
    this->get_parameter("sources", source_ids);

    for (auto& source_id : source_ids)
    {
      SensorSource sensor_source;
      this->declare_parameter<std::string>(source_id + ".topic", "");
      this->get_parameter(source_id + ".topic", sensor_source.topic);
      this->declare_parameter<std::string>(source_id + ".sensor_origin_frame", "");
      this->get_parameter(source_id + ".sensor_origin_frame", sensor_source.sensor_origin_frame);
      this->declare_parameter<double>(source_id + ".max_range", 0);
      this->get_parameter(source_id + ".max_range", sensor_source.max_range);
      RCLCPP_INFO_STREAM(this->get_logger(), "Setting up source: " << source_id);

      if (sensor_source.topic.empty())
      {
        RCLCPP_ERROR_STREAM(this->get_logger(),
                            "No input topic specified for source: " << source_id);
        continue;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Topic: " << sensor_source.topic);
      if (sensor_source.sensor_origin_frame.empty())
      {
        RCLCPP_INFO(this->get_logger(), "Using frame id of topic as raycast origin");
      }
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Using " << sensor_source.sensor_origin_frame << " as raycast origin");
      }


      m_cloud_subs.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        sensor_source.topic,
        rclcpp::QoS(1).durability_volatile().best_effort(),
        [&, sensor_source](const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
          cloudCallback(cloud_msg, sensor_source);
        }));
    }
  }

  m_reset_map_service = this->create_service<std_srvs::srv::Trigger>(
    "~/reset_map", std::bind(&VDBMappingROS2::resetMapCallback, this, _1, _2));

  m_save_map_service = this->create_service<std_srvs::srv::Trigger>(
    "~/save_map", std::bind(&VDBMappingROS2::saveMap, this, _1, _2));

  m_load_map_service = this->create_service<vdb_mapping_interfaces::srv::LoadMap>(
    "~/load_map", std::bind(&VDBMappingROS2::loadMap, this, _1, _2));

  m_load_map_from_pcd_service = this->create_service<vdb_mapping_interfaces::srv::LoadMapFromPCD>(
    "~/load_map_from_pcd", std::bind(&VDBMappingROS2::loadMapFromPCD, this, _1, _2));

  m_get_map_section_service = this->create_service<vdb_mapping_interfaces::srv::GetMapSection>(
    "~/get_map_section", std::bind(&VDBMappingROS2::getMapSectionCallback, this, _1, _2));

  m_trigger_map_section_update_service =
    this->create_service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>(
      "~/trigger_map_section_update",
      std::bind(&VDBMappingROS2::triggerMapSectionUpdateCallback, this, _1, _2));

  m_raytrace_service = this->create_service<vdb_mapping_interfaces::srv::Raytrace>(
    "~/raytrace", std::bind(&VDBMappingROS2::raytraceCallback, this, _1, _2));

  m_add_points_to_grid_service = this->create_service<vdb_mapping_interfaces::srv::AddPointsToGrid>(
    "~/add_points_to_grid", std::bind(&VDBMappingROS2::addPointsToGridCallback, this, _1, _2));

  m_remove_points_from_grid_service =
    this->create_service<vdb_mapping_interfaces::srv::RemovePointsFromGrid>(
      "~/remove_points_from_grid",
      std::bind(&VDBMappingROS2::removePointsFromGridCallback, this, _1, _2));

  m_pointcloud_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("~/vdb_map_pointcloud", 1);
  m_visualization_marker_pub =
    this->create_publisher<visualization_msgs::msg::Marker>("~/vdb_map_visualization", 1);

  m_occupancy_grid_service = this->create_service<vdb_mapping_interfaces::srv::GetOccGrid>(
    "~/get_occupancy_grid", std::bind(&VDBMappingROS2::occGridGenCallback, this, _1, _2));

  double visualization_rate;
  this->declare_parameter<double>("visualization_rate", 1.0);
  this->get_parameter("visualization_rate", visualization_rate);
  if (visualization_rate > 0.0)
  {
    m_visualization_timer =
      this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / visualization_rate)),
                              std::bind(&VDBMappingROS2::visualizationTimerCallback, this));
  }

  this->declare_parameter<bool>("accumulate_updates", false);
  this->get_parameter("accumulate_updates", m_accumulate_updates);
  if (m_accumulate_updates)
  {
    double accumulation_period;
    this->declare_parameter<double>("accumulation_period", 1);
    this->get_parameter("accumulation_period", accumulation_period);
    m_accumulation_update_timer =
      this->create_wall_timer(std::chrono::milliseconds((int)(1000 * accumulation_period)),
                              std::bind(&VDBMappingROS2::accumulationUpdateTimerCallback, this));
  }

  // Load initial map file
  std::string initial_map_file;
  bool set_background;
  bool clear_map;
  this->declare_parameter<std::string>("map_server.initial_map_file", "");
  this->get_parameter("map_server.initial_map_file", initial_map_file);
  RCLCPP_INFO_STREAM(this->get_logger(), "Loading intial Map " << initial_map_file);
  this->declare_parameter<bool>("map_server.set_background", false);
  this->get_parameter("map_server.set_background", set_background);
  this->declare_parameter<bool>("map_server.clear_map", false);
  this->get_parameter("map_server.clear_map", clear_map);
  if (initial_map_file != "")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Loading intial Map " << initial_map_file);
    m_vdb_map->loadMapFromPCD(initial_map_file, set_background, clear_map);
    publishMap();
  }
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::resetMap()
{
  RCLCPP_INFO(this->get_logger(), "Resetting Map");
  m_vdb_map->resetMap();
  publishMap();
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::resetMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  resetMap();
  res->success = true;
  res->message = "Reset map successful.";
  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::saveMap(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Saving Map");
  res->success = m_vdb_map->saveMap();
  return res->success;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::saveMapToPCD(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  RCLCPP_INFO(this->get_logger(), "Saving Map to PCD");
  res->success = m_vdb_map->saveMapToPCD();
  return res->success;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::loadMap(
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Loading Map");
  bool success = m_vdb_map->loadMap(req->path);
  publishMap();
  res->success = success;
  return success;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::loadMapFromPCD(
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "Loading Map from PCD file");
  bool success = m_vdb_map->loadMapFromPCD(req->path, req->set_background, req->clear_map);
  publishMap();
  res->success = success;
  return success;
}


template <typename VDBMappingT>
const std::string& VDBMappingROS2<VDBMappingT>::getMapFrame() const
{
  return m_map_frame;
}

template <typename VDBMappingT>
std::shared_ptr<VDBMappingT> VDBMappingROS2<VDBMappingT>::getMap()
{
  return m_vdb_map;
}

template <typename VDBMappingT>
const std::shared_ptr<VDBMappingT> VDBMappingROS2<VDBMappingT>::getMap() const
{
  return m_vdb_map;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::getMapSectionCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Response> res)
{
  geometry_msgs::msg::TransformStamped source_to_map_tf;
  try
  {
    source_to_map_tf = m_tf_buffer->lookupTransform(
      m_map_frame, req->header.frame_id, rclcpp::Time(0), rclcpp::Duration(1, 0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 req->header.frame_id.c_str(),
                 ex.what());
    res->success = false;
    return true;
  }
  res->section.map = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(
    m_vdb_map->getMapSectionUpdateGrid(Eigen::Matrix<double, 3, 1>(req->bounding_box.min_corner.x,
                                                                   req->bounding_box.min_corner.y,
                                                                   req->bounding_box.min_corner.z),
                                       Eigen::Matrix<double, 3, 1>(req->bounding_box.max_corner.x,
                                                                   req->bounding_box.max_corner.y,
                                                                   req->bounding_box.max_corner.z),
                                       tf2::transformToEigen(source_to_map_tf).matrix()));
  res->section.header.frame_id = m_map_frame;
  res->section.header.stamp    = this->now();
  res->success                 = true;

  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::triggerMapSectionUpdateCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Response> res)
{
  auto remote_source = m_remote_sources.find(req->remote_source);
  if (remote_source == m_remote_sources.end())
  {
    std::stringstream ss;
    ss << "Key " << req->remote_source << " not found. Available sources are: ";
    for (auto& source : m_remote_sources)
    {
      ss << source.first << ", ";
    }
    RCLCPP_WARN(this->get_logger(), ss.str().c_str());
    res->success = false;
    return true;
  }

  auto request = std::make_shared<vdb_mapping_interfaces::srv::GetMapSection::Request>();

  request->header       = req->header;
  request->bounding_box = req->bounding_box;
  auto result           = remote_source->second.get_map_section_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result.get();
    if (response->success)
    {
      m_vdb_map->overwriteMap(
        m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(
          response->section.map));
    }
    res->success = response->success;
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to call servcie get_map_section");
    res->success = false;
  }

  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::raytraceCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Response> res)
{
  geometry_msgs::msg::TransformStamped reference_tf;
  try
  {
    reference_tf =
      m_tf_buffer->lookupTransform(m_map_frame, req->header.frame_id.c_str(), req->header.stamp);

    Eigen::Matrix<double, 4, 4> m = tf2::transformToEigen(reference_tf).matrix();
    Eigen::Matrix<double, 4, 1> origin, direction;
    origin << req->origin.x, req->origin.y, req->origin.z, 1;
    direction << req->direction.x, req->direction.y, req->direction.z, 0;

    origin    = m * origin;
    direction = m * direction;

    openvdb::Vec3d end_point;

    res->success = m_vdb_map->raytrace(openvdb::Vec3d(origin.x(), origin.y(), origin.z()),
                                       openvdb::Vec3d(direction.x(), direction.y(), direction.z()),
                                       req->max_ray_length,
                                       end_point);

    res->header.frame_id = m_map_frame;
    res->header.stamp    = req->header.stamp;
    res->end_point.x     = end_point.x();
    res->end_point.y     = end_point.y();
    res->end_point.z     = end_point.z();
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Transform to map frame failed: " << ex.what());
    res->success = false;
  }
  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::addPointsToGridCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Response> res)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(req->points, *cloud);
  res->success = m_vdb_map->addPointsToGrid(cloud);
  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::removePointsFromGridCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Response> res)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(req->points, *cloud);
  res->success = m_vdb_map->removePointsFromGrid(cloud);
  return true;
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::visualizationTimerCallback()
{
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::accumulationUpdateTimerCallback()
{
  typename VDBMappingT::UpdateGridT::Ptr update;
  typename VDBMappingT::UpdateGridT::Ptr overwrite;
  m_vdb_map->integrateUpdate(update, overwrite);

  publishUpdates(update, overwrite, this->now());
  m_vdb_map->resetUpdate();
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::sectionTimerCallback()
{
  geometry_msgs::msg::TransformStamped map_to_robot_tf;
  try
  {
    // Get sensor origin transform in map coordinates
    map_to_robot_tf = m_tf_buffer->lookupTransform(
      m_map_frame, m_section_update_frame, rclcpp::Time(0), rclcpp::Duration(1, 0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 m_section_update_frame.c_str(),
                 ex.what());
    return;
  }

  typename VDBMappingT::UpdateGridT::Ptr section = m_vdb_map->getMapSectionUpdateGrid(
    m_section_min_coord, m_section_max_coord, tf2::transformToEigen(map_to_robot_tf).matrix());
  vdb_mapping_interfaces::msg::UpdateGrid msg;
  msg.header.frame_id = m_map_frame;
  msg.header.stamp    = map_to_robot_tf.header.stamp;
  msg.map = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(section);
  m_map_section_pub->publish(msg);
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::mapUpdateCallback(
  const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg)
{
  m_vdb_map->updateMap(
    m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(update_msg->map));
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::mapOverwriteCallback(
  const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg)
{
  m_vdb_map->overwriteMap(
    m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(update_msg->map));
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::mapSectionCallback(
  const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg)
{
  m_vdb_map->applyMapSectionUpdateGrid(
    m_vdb_map->template byteArrayToGrid<typename VDBMappingT::UpdateGridT>(update_msg->map));
}

template <typename VDBMappingT>
bool VDBMappingROS2<VDBMappingT>::occGridGenCallback(
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetOccGrid::Request> req,
  const std::shared_ptr<vdb_mapping_interfaces::srv::GetOccGrid::Response> res)
{
  (void)req;
  nav_msgs::msg::OccupancyGrid grid;
  openvdb::CoordBBox curr_bbox = m_vdb_map->getGrid()->evalActiveVoxelBoundingBox();
  grid.header.frame_id         = m_map_frame;
  grid.header.stamp            = this->now();
  grid.info.height             = curr_bbox.dim().y();
  grid.info.width              = curr_bbox.dim().x();
  grid.info.resolution         = m_resolution;
  std::vector<int> voxel_projection_grid;
  grid.data.resize(grid.info.width * grid.info.height);
  voxel_projection_grid.resize(grid.info.width * grid.info.height);

  int x_offset = abs(curr_bbox.min().x());
  int y_offset = abs(curr_bbox.min().y());

  geometry_msgs::msg::Pose origin_pose;
  origin_pose.position.x = curr_bbox.min().x() * m_resolution;
  origin_pose.position.y = curr_bbox.min().y() * m_resolution;
  origin_pose.position.z = 0.00;

  grid.info.origin   = origin_pose;
  int world_to_index = 0;
  for (openvdb::FloatGrid::ValueOnCIter iter = m_vdb_map->getGrid()->cbeginValueOn(); iter; ++iter)
  {
    if (iter.isValueOn())
    {
      world_to_index =
        (iter.getCoord().y() + y_offset) * curr_bbox.dim().x() + (iter.getCoord().x() + x_offset);
      voxel_projection_grid[world_to_index] += 1;
    }
  }

  for (size_t i = 0; i < voxel_projection_grid.size(); i++)
  {
    if (voxel_projection_grid[i] > m_two_dim_projection_threshold)
    {
      grid.data[i] = 100;
    }
    else if (voxel_projection_grid[i] == 0)
    {
      grid.data[i] = -1;
    }
    else
    {
      grid.data[i] = 0;
    }
  }
  res->occupancy_grid = grid;
  return true;
}


template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::cloudCallback(
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg, const SensorSource& sensor_source)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  geometry_msgs::msg::TransformStamped cloud_origin_tf;

  std::string sensor_frame = sensor_source.sensor_origin_frame.empty()
                               ? cloud_msg->header.frame_id
                               : sensor_source.sensor_origin_frame;

  // Get the origin of the sensor used as a starting point of the ray cast
  try
  {
    cloud_origin_tf = m_tf_buffer->lookupTransform(
      m_map_frame, sensor_frame, cloud_msg->header.stamp, rclcpp::Duration(0, 100000000));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Could not transform %s to %s: %s",
                 m_map_frame.c_str(),
                 sensor_frame.c_str(),
                 ex.what());
    return;
  }
  // If aligned map is not already in correct map frame, transform it
  if (m_map_frame != cloud_msg->header.frame_id)
  {
    geometry_msgs::msg::TransformStamped origin_to_map_tf;
    try
    {
      origin_to_map_tf = m_tf_buffer->lookupTransform(
        m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Could not transform %s to %s: %s",
                   m_map_frame.c_str(),
                   cloud_msg->header.frame_id.c_str(),
                   ex.what());
      return;
    }
    pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(origin_to_map_tf).matrix());
    cloud->header.frame_id = m_map_frame;
  }
  m_vdb_map->accumulateUpdate(
    cloud, tf2::transformToEigen(cloud_origin_tf).translation(), sensor_source.max_range);
  if (!m_accumulate_updates)
  {
    typename VDBMappingT::UpdateGridT::Ptr update;
    typename VDBMappingT::UpdateGridT::Ptr overwrite;
    m_vdb_map->integrateUpdate(update, overwrite);
    m_vdb_map->resetUpdate();
    publishUpdates(update, overwrite, cloud_msg->header.stamp);
  }
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::insertPointCloud(
  const typename VDBMappingT::PointCloudT::Ptr cloud,
  const geometry_msgs::msg::TransformStamped transform)
{
  Eigen::Matrix<double, 3, 1> sensor_to_map_eigen = tf2::transformToEigen(transform).translation();
  typename VDBMappingT::UpdateGridT::Ptr update;
  typename VDBMappingT::UpdateGridT::Ptr overwrite;
  // Integrate data into vdb grid
  m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen, update, overwrite);
  publishUpdates(update, overwrite, transform.header.stamp);
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::publishUpdates(typename VDBMappingT::UpdateGridT::Ptr update,
                                                 typename VDBMappingT::UpdateGridT::Ptr overwrite,
                                                 rclcpp::Time stamp)
{
  std_msgs::msg::Header header;
  header.frame_id = m_map_frame;
  header.stamp    = stamp;
  if (m_publish_updates)
  {
    vdb_mapping_interfaces::msg::UpdateGrid msg;
    msg.map    = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(update);
    msg.header = header;
    m_map_update_pub->publish(msg);
  }
  if (m_publish_overwrites)
  {
    vdb_mapping_interfaces::msg::UpdateGrid msg;
    msg.map    = m_vdb_map->template gridToByteArray<typename VDBMappingT::UpdateGridT>(overwrite);
    msg.header = header;
    m_map_update_pub->publish(msg);
  }
}

template <typename VDBMappingT>
void VDBMappingROS2<VDBMappingT>::publishMap() const
{
  if (!(m_publish_pointcloud || m_publish_vis_marker))
  {
    return;
  }
  bool publish_vis_marker;
  publish_vis_marker =
    (m_publish_vis_marker && this->count_subscribers("~/vdb_map_visualization") > 0);
  bool publish_pointcloud;
  publish_pointcloud =
    (m_publish_pointcloud && this->count_subscribers("~/vdb_map_pointcloud") > 0);

  visualization_msgs::msg::Marker visualization_marker_msg;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  VDBMappingTools<VDBMappingT>::createMappingOutput(m_vdb_map->getGrid(),
                                                    m_map_frame,
                                                    visualization_marker_msg,
                                                    cloud_msg,
                                                    m_publish_vis_marker,
                                                    m_publish_pointcloud,
                                                    m_lower_visualization_z_limit,
                                                    m_upper_visualization_z_limit);
  if (publish_vis_marker)
  {
    visualization_marker_msg.header.stamp = this->now();
    m_visualization_marker_pub->publish(visualization_marker_msg);
  }
  if (publish_pointcloud)
  {
    cloud_msg.header.stamp = this->now();
    m_pointcloud_pub->publish(cloud_msg);
  }
}
