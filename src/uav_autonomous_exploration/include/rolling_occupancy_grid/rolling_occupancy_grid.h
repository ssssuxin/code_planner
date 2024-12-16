/**
 * @file rolling_occupancy_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling occupancy grid
 * @version 0.1
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "grid/grid.h"
#include "rolling_grid/rolling_grid.h"
#include "utils/misc_utils.h"

namespace rolling_occupancy_grid_ns
{
class RollingOccupancyGrid
{
public:
  enum CellState : char
  {
    UNKNOWN = 0,
    OCCUPIED = 1,
    FREE = 2,
    NOT_FRONTIER = 3
  };

  explicit RollingOccupancyGrid(std::shared_ptr<rclcpp::Node>& nh);
  ~RollingOccupancyGrid() = default;

  Eigen::Vector3d GetResolution()
  {
    return resolution_;
  }

  void InitializeOrigin(const Eigen::Vector3d& origin);
  bool UpdateRobotPosition(const Eigen::Vector3d& robot_position);
  template <class PointType>
  void UpdateOccupancy(typename pcl::PointCloud<PointType>::Ptr& cloud)
  {
    if (!initialized_)
    {
      return;
    }
    updated_grid_indices_.clear();
    for (const auto& point : cloud->points)
    {
      Eigen::Vector3i sub = occupancy_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
      if (occupancy_array_->InRange(sub))
      {
        int ind = occupancy_array_->Sub2Ind(sub);
        int array_ind = rolling_grid_->GetArrayInd(ind);
        occupancy_array_->SetCellValue(array_ind, OCCUPIED);
        updated_grid_indices_.push_back(ind);
      }
    }
  }
  void UpdateOccupancyStatus(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range);
  void RayTrace(const Eigen::Vector3d& origin);
  void RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                      std::vector<Eigen::Vector3i>& cells);
  void RayTraceHelper_for_viewpoint_connected(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                      std::vector<Eigen::Vector3i>& cells);
  void RayTraceHelper_for_point_2_bound(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                      std::vector<Eigen::Vector3i>& cells);
  void GetFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud, const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& range);
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetRolledOutOccupancyCloud()
  {
    return occupancy_cloud_;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetRolledOut_gather_points_from_occupancy_array_()
  {
    return gather_points_from_occupancy_array_;
  }
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);
  void Get_free_occupy_pointcloud_fromoccupancy_array_();
  void Return_free_occupy_pointcloud(pcl::PointCloud<pcl::PointXYZI>&free_cloud ,pcl::PointCloud<pcl::PointXYZI>&occupy_cloud ,pcl::PointCloud<pcl::PointXYZI>&non_free_cloud,pcl::PointCloud<pcl::PointXYZI>& unknow_cloud);
  int Get_collision_unkown_in_path(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);
  std::vector<Eigen::Vector3d> Get_near_free_position_in_occupancy(Eigen::Vector3d position, double radius_thr);
  // void Return_free_occupy_pointcloud(pcl::PointCloud<pcl::PointXYZI>&free_cloud ,pcl::PointCloud<pcl::PointXYZI>&occupy_cloud );

  bool return_initialized_()
  {
    return initialized_;
  }
  std::unique_ptr<grid_ns::Grid<CellState>>& Get_occupancy_array_()
  {
    return occupancy_array_;
  }

  std::unique_ptr<rolling_grid_ns::RollingGrid>& Get_rolling_grid_()
  {
    return rolling_grid_;
  }
  void Amend_occupancy_array_();
  bool Visible_in_RayTraceHelper_for_viewpoint_connected(Eigen::Vector3d start_pos, Eigen::Vector3d end_pos, bool start_pos_inrange = false, int connected_premise = 0);
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ament_occupancy_point;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ament_occupancy_point_all;//可视化
  pcl::PointCloud<pcl::PointXYZ> ament_cloud_all;//可视化
private:
  bool initialized_;
  int dimension_;
  Eigen::Vector3d range_;
  Eigen::Vector3i grid_size_;
  Eigen::Vector3d rollover_range_;
  Eigen::Vector3i rollover_step_size_;
  Eigen::Vector3d resolution_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d robot_position_;
  std::unique_ptr<rolling_grid_ns::RollingGrid> rolling_grid_;
  std::unique_ptr<grid_ns::Grid<CellState>> occupancy_array_;
  std::vector<int> updated_grid_indices_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr gather_points_from_occupancy_array_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_free_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_occupy_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_non_free_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_unknown_points;
  bool InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& sub_min, const Eigen::Vector3i& sub_max);

  // void InitializeOrigin();
};
}  // namespace rolling_occupancy_grid_ns
