/**
 * @file rolling_occupancy_grid.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling occupancy grid
 * @version 0.1
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 * @modified_by Huazhang Zhu (22210720332@m.fudan.edu.cn)
 * @date 2023-11-05
 */

#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

namespace rolling_occupancy_grid_ns
{
RollingOccupancyGrid::RollingOccupancyGrid(std::shared_ptr<rclcpp::Node>& nh) : initialized_(false), dimension_(3)
{
  ament_occupancy_point = nh->create_publisher<sensor_msgs::msg::PointCloud2>("ament_occupancy_point",10);//可视化
  ament_occupancy_point_all = nh->create_publisher<sensor_msgs::msg::PointCloud2>("ament_occupancy_point_all",10);//可视化
  double pointcloud_cell_size = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 18);
  double pointcloud_cell_height = misc_utils_ns::getParam<double>(nh, "kPointCloudCellHeight", 1.8);
  int pointcloud_cell_neighbor_number = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
  range_.x() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
  range_.y() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
  range_.z() = pointcloud_cell_height * pointcloud_cell_neighbor_number;

  resolution_.x() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_x", 0.3);
  resolution_.y() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_y", 0.3);
  resolution_.z() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_z", 0.3);

  rollover_range_.x() = pointcloud_cell_size;
  rollover_range_.y() = pointcloud_cell_size;
  rollover_range_.z() = pointcloud_cell_height;

  for (int i = 0; i < dimension_; i++)
  {
    grid_size_(i) = static_cast<int>(range_(i) / resolution_(i));
    rollover_step_size_(i) = static_cast<int>(rollover_range_(i) / resolution_(i));
    origin_(i) = -range_(i) / 2;
  }

  rolling_grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(grid_size_);
  occupancy_array_ = std::make_unique<grid_ns::Grid<CellState>>(grid_size_, UNKNOWN, origin_, resolution_);

  robot_position_ = Eigen::Vector3d(0, 0, 0);

  occupancy_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  gather_points_from_occupancy_array_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  local_free_points = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
  local_occupy_points = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
  local_non_free_points = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
  local_unknown_points = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);
}

void RollingOccupancyGrid::InitializeOrigin(const Eigen::Vector3d& origin)
{
  if (!initialized_)
  {
    initialized_ = true;
    origin_ = origin;
    occupancy_array_->SetOrigin(origin_);
  }
}

bool RollingOccupancyGrid::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  if (!initialized_)
  {
    return false;
  }
  robot_position_ = robot_position;
  Eigen::Vector3i robot_grid_sub;
  Eigen::Vector3d diff = robot_position_ - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < dimension_; i++)
  {
    robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (rollover_step_size_(i) * resolution_(i))) : -1;
  }

  Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
  for (int i = 0; i < dimension_; i++)
  {
    sub_diff(i) = (grid_size_(i) / rollover_step_size_(i)) / 2 - robot_grid_sub(i);
  }
  gather_points_from_occupancy_array_->clear();
  for (int ind=0;ind<rolling_grid_->size_int;ind++)
  {
    int array_ind = rolling_grid_->GetArrayInd(ind);
    if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 0;
      gather_points_from_occupancy_array_->points.push_back(point);
    }
    else if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      gather_points_from_occupancy_array_->points.push_back(point);
    }
  }
  if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
  {
    return false;
  }

  Eigen::Vector3i rollover_step(0, 0, 0);
  for (int i = 0; i < dimension_; i++)
  {
    rollover_step(i) =
        std::abs(sub_diff(i)) > 0 ? rollover_step_size_(i) * ((sub_diff(i) > 0) ? 1 : -1) * std::abs(sub_diff(i)) : 0;
  }

  std::vector<int> rolled_out_grid_indices;
  rolling_grid_->GetRolledOutIndices(rollover_step, rolled_out_grid_indices);

  // Get rolled out occupancy cloud
  occupancy_cloud_->clear();
  
  for (const auto& ind : rolled_out_grid_indices)
  {
    int array_ind = rolling_grid_->GetArrayInd(ind);
    if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 0;
      occupancy_cloud_->points.push_back(point);
    }
    else if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      occupancy_cloud_->points.push_back(point);
    }
  }

  



  rolling_grid_->Roll(rollover_step);

  // Update origin
  for (int i = 0; i < dimension_; i++)
  {
    origin_(i) -= rollover_step(i) * resolution_(i);
  }
  occupancy_array_->SetOrigin(origin_);

  std::vector<int> updated_grid_indices;
  rolling_grid_->GetUpdatedArrayIndices(updated_grid_indices);

  for (const auto& ind : updated_grid_indices)
  {
    occupancy_array_->SetCellValue(ind, CellState::UNKNOWN);
  }

  return true;
}

void RollingOccupancyGrid::UpdateOccupancyStatus(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (!initialized_)
  {
    return;
  }
  for (const auto& point : cloud->points)
  {
    Eigen::Vector3i sub = occupancy_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
    if (!occupancy_array_->InRange(sub))
    {
      continue;
    }
    int array_ind = rolling_grid_->GetArrayInd(sub);
    if (point.intensity < 0.1)
    {
      occupancy_array_->SetCellValue(array_ind, CellState::FREE);
    }
    else if (point.intensity > 0.9)
    {
      occupancy_array_->SetCellValue(array_ind, CellState::OCCUPIED);
    }
  }
}

void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  // Eigen::Vector3i sub_max = occupancy_array_->GetSize() - Eigen::Vector3i::Ones();
  // Eigen::Vector3i sub_min = Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);
  int ray_trace_count = 0;
  if (!occupancy_array_->InRange(origin_sub))
  {
    // ROS_WARN("RollingOccupancyGrid::RayTrace(), robot not in range");
    RCLCPP_WARN(rclcpp::get_logger("my_logger"), "RollingOccupancyGrid::RayTrace(), robot not in range");
    return;
  }

  misc_utils_ns::UniquifyIntVector(updated_grid_indices_);

  for (const auto& ind : updated_grid_indices_)
  {
    if (occupancy_array_->InRange(ind))
    {
      Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind);
      if (!occupancy_array_->InRange(cur_sub))
      {
        // ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
        RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
        continue;
      }
      int array_ind = rolling_grid_->GetArrayInd(ind);
      if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
      {
        // ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
        RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
        continue;
      }
      ray_trace_count++;
      std::vector<Eigen::Vector3i> ray_cast_cells;
      RayTraceHelper(origin_sub, cur_sub, ray_cast_cells);
      for (int i = 0; i < ray_cast_cells.size(); i++)
      {
        Eigen::Vector3i ray_sub = ray_cast_cells[i];
        int array_ind = rolling_grid_->GetArrayInd(ray_sub);
        if (occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
        {
          break;
        }
        else
        {
          if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
          {
            occupancy_array_->SetCellValue(array_ind, FREE);
          }
        }
      }
    }
  }
}

void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d& origin)
{
  if (!initialized_)
  {
    return;
  }
  RayTrace(origin, range_);
}

void RollingOccupancyGrid::RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                          std::vector<Eigen::Vector3i>& cells)
{
  cells.clear();
  MY_ASSERT(occupancy_array_->InRange(start_sub));
  MY_ASSERT(occupancy_array_->InRange(end_sub));

  if (start_sub == end_sub)
  {
    cells.push_back(start_sub);
    return;
  }
  Eigen::Vector3i diff_sub = end_sub - start_sub;
  double max_dist = diff_sub.squaredNorm();
  int step_x = misc_utils_ns::signum(diff_sub.x());
  int step_y = misc_utils_ns::signum(diff_sub.y());
  int step_z = misc_utils_ns::signum(diff_sub.z());
  double t_max_x = step_x == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.x(), diff_sub.x());
  double t_max_y = step_y == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.y(), diff_sub.y());
  double t_max_z = step_z == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.z(), diff_sub.z());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
  double dist = 0;
  Eigen::Vector3i cur_sub = start_sub;

  while (occupancy_array_->InRange(cur_sub))
  {
    cells.push_back(cur_sub);
    dist = (cur_sub - start_sub).squaredNorm();
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (cur_sub == end_sub || dist > max_dist || occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
    {
      return;
    }
    if (t_max_x < t_max_y)
    {
      if (t_max_x < t_max_z)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
    else
    {
      if (t_max_y < t_max_z)
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
  }
}
void RollingOccupancyGrid::RayTraceHelper_for_point_2_bound(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                          std::vector<Eigen::Vector3i>& cells)
{
  cells.clear();
  MY_ASSERT(occupancy_array_->InRange(start_sub));
  // MY_ASSERT(occupancy_array_->InRange(end_sub));

  if (start_sub == end_sub)
  {
    cells.push_back(start_sub);
    return;
  }
  Eigen::Vector3i diff_sub = end_sub - start_sub;
  double max_dist = diff_sub.squaredNorm();
  int step_x = misc_utils_ns::signum(diff_sub.x());
  int step_y = misc_utils_ns::signum(diff_sub.y());
  int step_z = misc_utils_ns::signum(diff_sub.z());
  double t_max_x = step_x == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.x(), diff_sub.x());
  double t_max_y = step_y == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.y(), diff_sub.y());
  double t_max_z = step_z == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.z(), diff_sub.z());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
  double dist = 0;
  Eigen::Vector3i cur_sub = start_sub;

  while (occupancy_array_->InRange(cur_sub))
  {
    cells.push_back(cur_sub);
    dist = (cur_sub - start_sub).squaredNorm();
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (cur_sub == end_sub || dist > max_dist)
    {
      return;
    }
    if (t_max_x < t_max_y)
    {
      if (t_max_x < t_max_z)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
    else
    {
      if (t_max_y < t_max_z)
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
  }
}
void RollingOccupancyGrid::RayTraceHelper_for_viewpoint_connected(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                          std::vector<Eigen::Vector3i>& cells)
{
  std::vector<std::vector<Eigen::Vector3i>> idx_addon;
  idx_addon.resize(4);
  
  

  idx_addon[0].push_back(Eigen::Vector3i(-1,0,0));//0是平面 1是
  idx_addon[0].push_back(Eigen::Vector3i(1,0,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,1,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,0,0));
  idx_addon[0].push_back(Eigen::Vector3i(-2,0,0));//0是平面 1是
  idx_addon[0].push_back(Eigen::Vector3i(2,0,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,-2,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,2,0));
  idx_addon[0].push_back(Eigen::Vector3i(-1,1,0));
  idx_addon[0].push_back(Eigen::Vector3i(1,-1,0));
  idx_addon[0].push_back(Eigen::Vector3i(-1,-1,0));
  idx_addon[0].push_back(Eigen::Vector3i(1,1,0));



  idx_addon[1].push_back(Eigen::Vector3i(-1,0,0));//0是平面 1是
  idx_addon[1].push_back(Eigen::Vector3i(1,0,0));
  idx_addon[1].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[1].push_back(Eigen::Vector3i(0,0,+1));
  idx_addon[1].push_back(Eigen::Vector3i(0,0,0));

  idx_addon[2].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[2].push_back(Eigen::Vector3i(0,1,0));
  idx_addon[2].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[2].push_back(Eigen::Vector3i(0,0,1));
  idx_addon[2].push_back(Eigen::Vector3i(0,0,0));

  idx_addon[3].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[3].push_back(Eigen::Vector3i(0,1,0));
  idx_addon[3].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[3].push_back(Eigen::Vector3i(0,0,1));
  idx_addon[3].push_back(Eigen::Vector3i(0,0,0));
  idx_addon[3].push_back(Eigen::Vector3i(-1,0,0));
  idx_addon[3].push_back(Eigen::Vector3i(1,0,0));
  cells.clear();
  MY_ASSERT(occupancy_array_->InRange(start_sub));
  MY_ASSERT(occupancy_array_->InRange(end_sub));

  if (start_sub == end_sub)
  {
    cells.push_back(start_sub);
    return;
  }
  Eigen::Vector3i diff_sub = end_sub - start_sub;
  double max_dist = diff_sub.squaredNorm();
  int step_x = misc_utils_ns::signum(diff_sub.x());
  int step_y = misc_utils_ns::signum(diff_sub.y());
  int step_z = misc_utils_ns::signum(diff_sub.z());
  double t_max_x = step_x == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.x(), diff_sub.x());
  double t_max_y = step_y == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.y(), diff_sub.y());
  double t_max_z = step_z == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.z(), diff_sub.z());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
  double dist = 0;
  Eigen::Vector3i cur_sub = start_sub;
  Eigen::Vector3i temp_sub;
  // cells.push_back(cur_sub);
  for(int idx_addon_ind=0;idx_addon_ind<idx_addon[3].size();idx_addon_ind++)//从邻居点开始，邻居点要严格一点，6个方向
  {
    temp_sub = cur_sub + idx_addon[3][idx_addon_ind];
    cells.push_back(temp_sub);
  }
  while (occupancy_array_->InRange(cur_sub))
  {
    
    dist = (cur_sub - start_sub).squaredNorm();
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (cur_sub == end_sub || dist > max_dist || occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
    {
      //这里原本想删除掉cells中相同元素再返回，现在看不是特别有必要，因为重复的不多，然后为删除重复而进行的算法反而计算大一些
      return;
    }
    if (t_max_x < t_max_y)
    {
      if (t_max_x < t_max_z)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
        for(int idx_addon_ind=0;idx_addon_ind<idx_addon[2].size();idx_addon_ind++)
        {
          temp_sub = cur_sub + idx_addon[2][idx_addon_ind];
          cells.push_back(temp_sub);
        }

      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
        for(int idx_addon_ind=0;idx_addon_ind<idx_addon[0].size();idx_addon_ind++)
        {
          temp_sub = cur_sub + idx_addon[0][idx_addon_ind];
          cells.push_back(temp_sub);
        }
      }
    }
    else
    {
      if (t_max_y < t_max_z)
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
        for(int idx_addon_ind=0;idx_addon_ind<idx_addon[1].size();idx_addon_ind++)
        {
          temp_sub = cur_sub + idx_addon[1][idx_addon_ind];
          cells.push_back(temp_sub);
        }
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
        for(int idx_addon_ind=0;idx_addon_ind<idx_addon[0].size();idx_addon_ind++)
        {
          temp_sub = cur_sub + idx_addon[0][idx_addon_ind];
          cells.push_back(temp_sub);
        }
      }
    }
  }
}

void RollingOccupancyGrid::GetFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud,
                                       const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  if (!initialized_)
  {
    return;
  }
  frontier_cloud->points.clear();
  Eigen::Vector3i sub_max = occupancy_array_->Pos2Sub(origin + range);
  Eigen::Vector3i sub_min = occupancy_array_->Pos2Sub(origin - range);
  Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);

  if (!occupancy_array_->InRange(origin_sub))
  {
    // ROS_WARN("RollingOccupancyGrid::GetFrontierInRange(), robot not in range");
    RCLCPP_WARN(rclcpp::get_logger("my_logger"), "RollingOccupancyGrid::GetFrontierInRange(), robot not in range");
    return;
  }
  int ray_trace_count = 0;

  int cell_num = occupancy_array_->GetCellNumber();
  for (int ind = 0; ind < cell_num; ind++)
  {
    Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind);
    if (!occupancy_array_->InRange(cur_sub))
    {
      continue;
    }
    if (!InRange(cur_sub, sub_min, sub_max))
    {
      continue;
    }
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (occupancy_array_->GetCellValue(array_ind) == UNKNOWN)
    {
      bool z_free = false;
      bool xy_free = false;
      // If the unknown cell has neighboring free cells in xy but not z direction
      cur_sub(2)--;
      if (occupancy_array_->InRange(cur_sub))
      {
        array_ind = rolling_grid_->GetArrayInd(cur_sub);
        if (occupancy_array_->GetCellValue(array_ind) == FREE)
        {
          z_free = true;
          continue;
        }
      }
      cur_sub(2) += 2;
      if (occupancy_array_->InRange(cur_sub))
      {
        array_ind = rolling_grid_->GetArrayInd(cur_sub);
        if (occupancy_array_->GetCellValue(array_ind) == FREE)
        {
          z_free = true;
          continue;
        }
      }
      cur_sub(2)--;

      for (int i = 0; i < 2; i++)
      {
        cur_sub(i)--;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            xy_free = true;
            cur_sub(i)++;
            break;
          }
        }
        cur_sub(i) += 2;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            xy_free = true;
            cur_sub(i)--;
            break;
          }
        }
        cur_sub(i)--;
      }
      if (xy_free && !z_free)
      {
        Eigen::Vector3d position = occupancy_array_->Sub2Pos(cur_sub);
        pcl::PointXYZI point;
        point.x = position.x();
        point.y = position.y();
        point.z = position.z();
        point.intensity = 0;
        frontier_cloud->points.push_back(point);
      }
    }
  }
}

void RollingOccupancyGrid::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->clear();
  int cell_number = occupancy_array_->GetCellNumber();
  for (int i = 0; i < cell_number; i++)
  {
    int array_ind = rolling_grid_->GetArrayInd(i);
    if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED ||
        occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
      {
        point.intensity = 0.0;
      }
      else if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
      {
        point.intensity = 1.0;
      }
      else
      {
        point.intensity = 2.0;
      }
      vis_cloud->points.push_back(point);
    }
  }
}
void RollingOccupancyGrid::Get_free_occupy_pointcloud_fromoccupancy_array_()
{
  // std::cout<<"-----------"<<std::endl;
  local_free_points->clear();
  local_occupy_points->clear();
  local_non_free_points->clear();
  local_unknown_points->clear();
  // Eigen::Vector3i tem_3i = occupancy_array_->GetSize();
  // std::cout<<tem_3i.x()*tem_3i.y()*tem_3i.z()<<std::endl;
  // std::cout<<"-----------1111"<<std::endl;
  // std::cout<<occupancy_array_->GetCellNumber()<<std::endl;
  for (int i=0;i<occupancy_array_->GetCellNumber();i++)//跨界的时候对离开的5*5区域遍历选定,把其中所有的最小单位根据其xyz坐标 通过grid01_找到对应的 occupancy_array_的cells_索引获取状态值
  {
    int array_ind = rolling_grid_->GetArrayInd(i);  

    if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 0;
      local_free_points->points.push_back(point);
    }
    else if(occupancy_array_->GetCellValue(array_ind) == CellState::UNKNOWN)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      local_unknown_points->points.push_back(point);
      local_non_free_points->points.push_back(point);
    }
    else if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      local_occupy_points->points.push_back(point);
      local_non_free_points->points.push_back(point);
    }
    else
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      local_non_free_points->points.push_back(point);
    }
  }
  
}
void RollingOccupancyGrid::Return_free_occupy_pointcloud(pcl::PointCloud<pcl::PointXYZI>&free_cloud ,pcl::PointCloud<pcl::PointXYZI>&occupy_cloud ,pcl::PointCloud<pcl::PointXYZI>&non_free_cloud ,pcl::PointCloud<pcl::PointXYZI>& unknow_cloud)
{
  free_cloud = *local_free_points;
  occupy_cloud = *local_occupy_points;
  non_free_cloud = *local_non_free_points;
  unknow_cloud = *local_unknown_points;
}
// void RollingOccupancyGrid::Return_free_occupy_pointcloud(pcl::PointCloud<pcl::PointXYZI>&free_cloud ,pcl::PointCloud<pcl::PointXYZI>&occupy_cloud )
// {
//   free_cloud = *local_free_points;
//   occupy_cloud = *local_occupy_points;
//   // non_free_cloud = *local_non_free_points;
// }
bool RollingOccupancyGrid::InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& sub_min,
                                   const Eigen::Vector3i& sub_max)
{
  bool in_range = true;
  for (int i = 0; i < 3; i++)
  {
    in_range &= (sub(i) >= sub_min(i) && sub(i) <= sub_max(i));
  }
  return in_range;
}
void RollingOccupancyGrid::Amend_occupancy_array_()//未被使用，计算量实在过大，且原本用于防止viewpoint穿模连接的作用有限
{
  std::vector<std::vector<Eigen::Vector3i>> idx_addon;
  idx_addon.resize(4);
  idx_addon[0].push_back(Eigen::Vector3i(-1,0,0));//0是平面 1是
  idx_addon[0].push_back(Eigen::Vector3i(1,0,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[0].push_back(Eigen::Vector3i(0,1,0));

  idx_addon[1].push_back(Eigen::Vector3i(-1,0,0));//0是平面 1是
  idx_addon[1].push_back(Eigen::Vector3i(1,0,0));
  idx_addon[1].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[1].push_back(Eigen::Vector3i(0,0,+1));

  idx_addon[2].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[2].push_back(Eigen::Vector3i(0,1,0));
  idx_addon[2].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[2].push_back(Eigen::Vector3i(0,0,1));

  idx_addon[3].push_back(Eigen::Vector3i(-1,0,0));
  idx_addon[3].push_back(Eigen::Vector3i(1,0,0));
  idx_addon[3].push_back(Eigen::Vector3i(0,-1,0));
  idx_addon[3].push_back(Eigen::Vector3i(0,1,0));
  idx_addon[3].push_back(Eigen::Vector3i(0,0,-1));
  idx_addon[3].push_back(Eigen::Vector3i(0,0,1));

  std::vector<bool> check; //用于标记已经查询过的点
  //TODO 记得给点赋true
  check.resize(occupancy_array_->GetCellNumber(),false);
  //先提取
  int temp_array_ind;
  int temp_check_ind;
  Eigen::Vector3i temp_occupy_sub;
  Eigen::Vector3i temp_occupy_nieghbour_sub;
  Eigen::Vector3i temp_free_nieghbour_sub;
  bool fixed_flag;
  pcl::PointCloud<pcl::PointXYZ> ament_cloud;//可视化
  sensor_msgs::msg::PointCloud2 viewpoints_msg;//可视化
  pcl::PointXYZ pt;//可视化
  Eigen::Vector3d A;//可视化
  for(int ind=0;ind<occupancy_array_->GetCellNumber();ind++)
  {
    temp_array_ind = rolling_grid_->GetArrayInd(ind);
    if(occupancy_array_->GetCell(temp_array_ind) == CellState::OCCUPIED)
    {
      temp_occupy_sub = occupancy_array_->Ind2Sub(ind);
      for(int i=0;i<idx_addon[3].size();i++)//对于每个邻居（寻找空闲邻居）
      {
        temp_occupy_nieghbour_sub = temp_occupy_sub + idx_addon[3][i];
        if(!rolling_grid_->InRange(temp_occupy_nieghbour_sub))//放错机制，对于游走在边界上不在occupancy范围的邻居点
        {
          continue;
        }
        temp_check_ind = occupancy_array_->Sub2Ind(temp_occupy_nieghbour_sub);
        if(!check[temp_check_ind])//对于已经判断过的邻居点（空闲或者占据）不再判断
        {
          if(occupancy_array_->GetCell(rolling_grid_->GetArrayInd(temp_occupy_nieghbour_sub))==CellState::FREE)
          {//free的话正式进入邻居判断
            for(int idx_addon_ind=0;idx_addon_ind<=2;idx_addon_ind++)//三个面,一个面封上就break；
            {
              fixed_flag = true;
              for(int idx_addon_ind2=0;idx_addon_ind2<idx_addon[idx_addon_ind].size();idx_addon_ind2++)//一个点没封上直接break
              {
                temp_free_nieghbour_sub = temp_occupy_nieghbour_sub + idx_addon[idx_addon_ind][idx_addon_ind2];
                if(!rolling_grid_->InRange(temp_free_nieghbour_sub))//放错机制，对于游走在边界上不在occupancy范围的空闲邻居的邻居点
                {
                  continue;
                }
                if(occupancy_array_->GetCell(rolling_grid_->GetArrayInd(temp_free_nieghbour_sub)) != CellState::OCCUPIED)
                {
                  fixed_flag = false;
                  break;
                }
              }
              if(fixed_flag)
              {
                occupancy_array_->GetCell(rolling_grid_->GetArrayInd(temp_occupy_nieghbour_sub)) = CellState::OCCUPIED;
                // std::cout<<"查漏补缺！！"<<std::endl;
                A = occupancy_array_->Sub2Pos(temp_occupy_nieghbour_sub);
                pt.x = A.x();
                pt.y = A.y();
                pt.z = A.z();
                ament_cloud.points.push_back(pt);
                break;
              }
              
            }
          }
          check[temp_check_ind]=true;//对于没判断过的邻居点，这里判断过后无论空闲占据直接设true
        }
      }
    }
  }
  pcl::toROSMsg(ament_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  ament_occupancy_point->publish(viewpoints_msg);

  ament_cloud_all += ament_cloud;
  pcl::toROSMsg(ament_cloud_all,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  ament_occupancy_point_all->publish(viewpoints_msg);
  // for(int i=0;i<idx_addon.size();i++)//上下左右前后的点都遍历
  // {
  //   for(int h=0;h<idx_addon[i].size();h++)
  //   {
  //     temp_occupy_nieghbour_sub = temp_occupy_sub+idx_addon[i][h];

  //     if()//一面的存在一个失去那么就直接
  //   }
      
  // }
}
bool RollingOccupancyGrid::Visible_in_RayTraceHelper_for_viewpoint_connected(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos, bool start_pos_inrange, int connected_premise)
{

  //connected_premise  0:默认不能出现未知或者障碍 1:允许出现未知
  Eigen::Vector3i start_point_sub_in_occupancy 
                = occupancy_array_->Pos2Sub(start_pos);
  Eigen::Vector3i end_point_sub_in_occupancy;
  std::vector<Eigen::Vector3i> ray_cast_cells;
  int array_ind_for_occupancy;
  bool connected_flag=true;
  if(start_pos_inrange || occupancy_array_->InRange(start_point_sub_in_occupancy))
  {
    end_point_sub_in_occupancy = occupancy_array_->Pos2Sub(end_pos);
    // std::cout<<"--------------"<<std::endl;
    if( !occupancy_array_->InRange(end_point_sub_in_occupancy))
    {//邻居不在其中，那么直接下一轮
      return false;
    }
    RayTraceHelper_for_viewpoint_connected(start_point_sub_in_occupancy, end_point_sub_in_occupancy, ray_cast_cells);//获取occupancy从一点到另一点 路径上经过的cells的 sub值；从后往前，因为判断不过的地方一般在后面
    for(int h=0;h<ray_cast_cells.size();h++)//对于每个cells确定它是否是占据的
    {
      if(!rolling_grid_->InRange(ray_cast_cells[h]))
      {
        connected_flag=false;
        break;
      }
      array_ind_for_occupancy = rolling_grid_->GetArrayInd(ray_cast_cells[h]);
      if(connected_premise == 0)
      {
        if(occupancy_array_->GetCell(array_ind_for_occupancy) != CellState::FREE)
        {//如果不是free的，那么就不能视作连接
          connected_flag=false;
          break;
        }
      }
      else if(connected_premise == 1)
      {
        if(occupancy_array_->GetCell(array_ind_for_occupancy) == CellState::OCCUPIED)
        {//如果不是free的，那么就不能视作连接
          connected_flag=false;
          break;
        }
      }

    }
    return connected_flag;
  }
  return false;
}


int RollingOccupancyGrid::Get_collision_unkown_in_path(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos)
{

  //connected_premise  0:默认不能出现未知或者障碍 1:允许出现未知
  Eigen::Vector3i start_point_sub_in_occupancy 
                = occupancy_array_->Pos2Sub(start_pos);
  Eigen::Vector3i end_point_sub_in_occupancy;
  std::vector<Eigen::Vector3i> ray_cast_cells;
  int array_ind_for_occupancy;
  bool connected_flag=true;
  int non_free_count=0;
  if(occupancy_array_->InRange(start_point_sub_in_occupancy))
  {
    end_point_sub_in_occupancy = occupancy_array_->Pos2Sub_for_outside_point(end_pos);
    RayTraceHelper_for_point_2_bound(start_point_sub_in_occupancy, end_point_sub_in_occupancy, ray_cast_cells);//获取occupancy从一点到另一点 路径上经过的cells的 sub值；从后往前，因为判断不过的地方一般在后面
    for(int h=0;h<ray_cast_cells.size();h++)//对于每个cells确定它是否是占据的
    {
      if(!rolling_grid_->InRange(ray_cast_cells[h]))
      {
        std::cout<<"直接不在范围内"<<std::endl;
        break;
      }

      array_ind_for_occupancy = rolling_grid_->GetArrayInd(ray_cast_cells[h]);
      
      if(occupancy_array_->GetCell(array_ind_for_occupancy) != CellState::FREE)
      {
        non_free_count++;
      }
      
    }
  }
  return non_free_count;
}

std::vector<Eigen::Vector3d> RollingOccupancyGrid::Get_near_free_position_in_occupancy(Eigen::Vector3d position, double radius_thr)
{
  int radius_size = radius_thr/resolution_.x();
  Eigen::Vector3i position_sub_in_occupancy 
                = occupancy_array_->Pos2Sub(position);
  std::vector<Eigen::Vector3d> free_pose;
  std::vector<Eigen::Vector3i> free_pose_sub_inoccupancy;

  int array_ind_for_occupancy;
  for(int x=-radius_size; x<=radius_size; x++)
  {
    for(int y=-radius_size; y<=radius_size; y++)
    {
      for(int z=-radius_size/3; z<=radius_size/3; z++)
      {
        Eigen::Vector3i temp = position_sub_in_occupancy;
        temp.x() += x; 
        temp.y() += y; 
        temp.z() += z; 
        if(rolling_grid_->InRange(temp))
        {
          array_ind_for_occupancy = rolling_grid_->GetArrayInd(temp);
          if(occupancy_array_->GetCell(array_ind_for_occupancy) == CellState::FREE)
          {
            free_pose_sub_inoccupancy.push_back(temp);
          }
        }
        
      }
    }
  }
  for(auto sub: free_pose_sub_inoccupancy)
  {
    free_pose.push_back(occupancy_array_->Sub2Pos(sub));
  }
  return free_pose;

}


}  // namespace rolling_occupancy_grid_ns