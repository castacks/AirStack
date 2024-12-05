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
 * \date    2022-05-09
 *
 */
//----------------------------------------------------------------------
template <typename VDBMappingT>
void VDBMappingTools<VDBMappingT>::createMappingOutput(const typename VDBMappingT::GridT::Ptr grid,
                                                       const std::string& frame_id,
                                                       visualization_msgs::msg::Marker& marker_msg,
                                                       sensor_msgs::msg::PointCloud2& cloud_msg,
                                                       const bool create_marker,
                                                       const bool create_pointcloud,
                                                       double lower_z_limit,
                                                       double upper_z_limit)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
  double min_z, max_z;
  openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
  openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());
  min_z                          = min_world_coord.z();
  max_z                          = max_world_coord.z();

  if (lower_z_limit != upper_z_limit && lower_z_limit < upper_z_limit)
  {
    min_z = min_z < lower_z_limit ? lower_z_limit : min_z;
    max_z = max_z > upper_z_limit ? upper_z_limit : max_z;
  }

  for (typename VDBMappingT::GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    openvdb::Vec3d world_coord = grid->indexToWorld(iter.getCoord());

    if( world_coord.z() < min_z || world_coord.z() > max_z)
    {
      continue;
    }

    if (create_marker)
    {
      geometry_msgs::msg::Point cube_center;
      cube_center.x = world_coord.x();
      cube_center.y = world_coord.y();
      cube_center.z = world_coord.z();
      marker_msg.points.push_back(cube_center);
      // Calculate the relative height of each voxel.
      double h = (1.0 - ((world_coord.z() - min_z) / (max_z - min_z)));
      marker_msg.colors.push_back(heightColorCoding(h));
    }
    if (create_pointcloud)
    {
      cloud->points.push_back(
        typename VDBMappingT::PointT(world_coord.x(), world_coord.y(), world_coord.z()));
    }
  }
  if (create_marker)
  {
    double size                = grid->transform().voxelSize()[0];
    marker_msg.header.frame_id = frame_id;
    // marker_msg.header.stamp       = ros::Time::now();
    marker_msg.id                 = 0;
    marker_msg.type               = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_msg.scale.x            = size;
    marker_msg.scale.y            = size;
    marker_msg.scale.z            = size;
    marker_msg.color.a            = 1.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.frame_locked       = true;
    if (marker_msg.points.size() > 0)
    {
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
    }
    else
    {
      marker_msg.action = visualization_msgs::msg::Marker::DELETE;
    }
  }
  if (create_pointcloud)
  {
    cloud->width  = cloud->points.size();
    cloud->height = 1;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = frame_id;
    // cloud_msg.header.stamp    = ros::Time::now();
  }
}
// Conversion from Hue to RGB Value
template <typename VDBMappingT>
std_msgs::msg::ColorRGBA VDBMappingTools<VDBMappingT>::heightColorCoding(const double height)
{
  // The factor of 0.8 is only for a nicer color range
  double h = height * 0.8;
  int i    = (int)(h * 6.0);
  double f = (h * 6.0) - i;
  double q = (1.0 - f);
  i %= 6;
  auto toMsg = [](double v1, double v2, double v3) {
    std_msgs::msg::ColorRGBA rgba;
    rgba.a = 1.0;
    rgba.r = v1;
    rgba.g = v2;
    rgba.b = v3;
    return rgba;
  };
  switch (i)
  {
    case 0:
      return toMsg(1.0, f, 0.0);
      break;
    case 1:
      return toMsg(q, 1.0, 0.0);
      break;
    case 2:
      return toMsg(0.0, 1.0, f);
      break;
    case 3:
      return toMsg(0.0, q, 1.0);
      break;
    case 4:
      return toMsg(f, 0.0, 1.0);
      break;
    case 5:
      return toMsg(1.0, 0.0, q);
      break;
    default:
      return toMsg(1.0, 0.5, 0.5);
      break;
  }
}
