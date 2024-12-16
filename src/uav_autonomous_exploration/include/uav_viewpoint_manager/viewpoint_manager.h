/**
 * @file viewpoint_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <cmath>

#include <Eigen/Core>
// ROS
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PointIndices.h>

#include <grid/grid.h>
#include <rolling_grid/rolling_grid.h>
#include <viewpoint/viewpoint.h>
#include <utils/misc_utils.h>
#include <grid_world/grid_world.h>
#include <exploration_path/exploration_path.h>
#include <std_msgs/msg/string.hpp>
#include <pcl/filters/crop_box.h>
namespace uav_viewpoint_manager_ns
{
struct ViewPointManagerParameter
{
  //big_box_param
  bool disable_outsider_viewpoints;
  bool smaller_visited_radius;
  bool show_time;
  Eigen::Vector3i big_box_size_;
  Eigen::Vector3d big_box_origin_;//未使用
  Eigen::Vector3d big_box_resolution;
  Eigen::Vector3d length_size;
  //mid_box_param
  Eigen::Vector3i middle_box_size_;
  Eigen::Vector3d middle_box_resolution;
  Eigen::Vector3i middle_box_side_;
  double middle_box_bigger_side;
  double big_box_bigger_side;
  int incollision_barrier_level;
  int viewpoint_num;
  int middle_box_viewpoint_num;
  // Layout
  bool kUseFrontier;
  int dimension_;
  int kViewPointNumber;
  Eigen::Vector3i kNumber;
  Eigen::Vector3d kRolloverStepsize;
  Eigen::Vector3d kResolution;
  bool first_frame_grace;
  // Collision check
  double kConnectivityHeightDiffThr;

  // Collision grid
  Eigen::Vector3i kCollisionGridSize;
  Eigen::Vector3d kCollisionGridResolution;
  double kViewPointCollisionMargin;
  double kViewPointCollisionMarginZPlus;
  double kViewPointCollisionMarginZMinus;
  double kCollisionGridZScale;
  int kCollisionPointThr;

  // Line of Sight Check
  bool kLineOfSightStopAtNearestObstacle;
  bool kCheckDynamicObstacleCollision;
  int kCollisionFrameCountMax;

  // Terrain height
  double kViewPointHeightFromTerrain;
  double kViewPointHeightFromTerrainChangeThreshold;

  // Coverage
  double kCoverageOcclusionThr;
  double kCoverageDilationRadius;
  double kCoveragePointCloudResolution;

  // Distances
  double kSensorRange;
  double kVisitRange;
  double kNeighborRange;
  double kHeightFromTerrain;
  double kDistanceToIntConst;

  // FOV
  double kVerticalFOVRatio;
  double kDiffZMax;
  double kInFovXYDistThreshold;
  double kInFovZDiffThreshold;
  //
  double visited_rad;
  double visited_rad_z;
  double visited_rad_copy;
  bool ReadParameters(std::shared_ptr<rclcpp::Node>& nh);
};

class ViewPointManager
{
public:
  int Robot_Hier_ind_;
  bool update_Robot_Hier_ind_;
  bool use_frontier_viewpoint;
  bool Print_visited_points_bool;//可视化
  int check_collision_margin;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr print_visited_points;
  void Print_visited_points(const std_msgs::msg::String::SharedPtr   start_msg)//可视化
  {
    Print_visited_points_bool=true;
  }
  std::vector<int> candidate_indices_;
  std::vector<int> ind_of_connected_viewpoints;//它存的就是Hier_ind
  std::vector<int> ind_of_visited_viewpoints;//它存的就是Hier_ind
  std::vector<int> ind_of_connected_viewpoints_in_noncovered_area;//它存的就是Hier_ind
  std::vector<int> Hier_ind_of_cantidate_frontier_viewpoints;//从 viewpoint_num 到 viewpoint_num + frontiersize()-1
  std::vector<int> last_ind_of_connected_viewpoints;
  std::vector<Eigen::Vector3d> last_viewpoint_pos;
  std::vector<int> boundary_index;
  explicit ViewPointManager(std::shared_ptr<rclcpp::Node>& nh);
  ~ViewPointManager() = default;

  int GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind = false) const;
  int GetViewPointInd(int viewpoint_array_ind) const;
  
  bool& Get_disable_outsider_viewpoints()
  {
    return vp_.disable_outsider_viewpoints;
  }
  bool& Get_relocation_()
  {
    return relocation_;
  }
  bool InRange_Hier_sub(Eigen::Vector3i sub)
  {

    return sub.x() >= 0 && sub.x() < vp_.big_box_size_.x()*vp_.middle_box_size_.x() && sub.y() >= 0 && sub.y() < vp_.big_box_size_.y()*vp_.middle_box_size_.y() &&  sub.z() >= 0 && sub.z() < vp_.big_box_size_.z()*vp_.middle_box_size_.z();
  }
  bool InRange_Hier_sub_for_keypose(Eigen::Vector3i sub)//因为边界地方容易出问题所以我特意规避下viewpoint边界的判断
  {

    return sub.x() >= vp_.big_box_size_.x()*vp_.middle_box_size_.x()/check_collision_margin && sub.x() < vp_.big_box_size_.x()*vp_.middle_box_size_.x()*(check_collision_margin-1)/check_collision_margin && 
           sub.y() >= vp_.big_box_size_.y()*vp_.middle_box_size_.y()/check_collision_margin && sub.y() < vp_.big_box_size_.y()*vp_.middle_box_size_.y()*(check_collision_margin-1)/check_collision_margin &&  
           sub.z() >= vp_.big_box_size_.z()*vp_.middle_box_size_.z()/check_collision_margin && sub.z() < vp_.big_box_size_.z()*vp_.middle_box_size_.z()*(check_collision_margin-1)/check_collision_margin;
  }
  bool InRange_Hier_ind(int ind)
  {
    return InRange_Hier_sub(Hierarchy_ind_2_Hierarchy_sub(ind));
  }
  inline bool InRange(int viewpoint_ind)
  {
    return grid_->InRange(viewpoint_ind);
  }
  inline bool InRange(const Eigen::Vector3i& sub)
  {
    return grid_->InRange(sub);
  }
  inline int GetViewPointNum()
  {
    return vp_.viewpoint_num;
  }
  Eigen::Vector3d GetResolution()
  {
    return vp_.middle_box_resolution;
  }
  inline void UpdateViewPointBoundary(const geometry_msgs::msg::Polygon& polygon)
  {
    viewpoint_boundary_ = polygon;
  }

  inline void UpdateNogoBoundary(const std::vector<geometry_msgs::msg::Polygon>& nogo_boundary)
  {
    nogo_boundary_ = nogo_boundary;
  }
  bool UpdateRobotPosition(const Eigen::Vector3d& robot_position);
  void UpdateOrigin();
  Eigen::Vector3i GetViewPointSub(Eigen::Vector3d position);
  int GetViewPointInd(Eigen::Vector3d position);
  Eigen::Vector3d GetOrigin()
  {
    return origin_;
  }
  double& Retrun_visited_rad()
  {
    return vp_.visited_rad;
  }
  double& Retrun_visited_rad_z()
  {
    return vp_.visited_rad_z;
  }
  

  double& Retrun_visited_rad_copy()
  {
    return vp_.visited_rad_copy;
  }
  int Get_middle_box_viewpoint_num()
  {
    return vp_.middle_box_viewpoint_num;
  }
  bool InCollision(const Eigen::Vector3d& position);
  bool InCurrentFrameLineOfSight(const Eigen::Vector3d& position,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_);
  void CheckViewPointBoundaryCollision();
  void CheckViewPointCollisionWithCollisionGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                          double collision_threshold);
  void CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub);
  void CheckViewPointLineOfSight();
  void CheckViewPointInFOV();
  bool InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position);
  bool InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position);
  bool InRobotFOV(const Eigen::Vector3d& position);
  void UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions,const std::unique_ptr<grid_world_ns::GridWorld>& grid_world);
  void UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);
  void SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                     double terrain_height_threshold = DBL_MAX);
  void Get_connected_Hier_ind_vec_in_exploring_area(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world_);
  void Push_boundary_index(int Hier);
  void Clear_boundary_index();
  template <class PCLPointType>
  void UpdateViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    // std::cout << "update cloud size: " << cloud->points.size() << std::endl;
    int update_viewpoint_count = 0;
    for (auto& viewpoint : viewpoints_)
    {
      if (viewpoint.InCollision())
      {
        continue;
      }
      update_viewpoint_count++;
    }
    // std::cout << "update viewpoint num: " << update_viewpoint_count << std::endl;
    for (const auto& point : cloud->points)
    {
      for (int i = 0; i < viewpoints_.size(); i++)
      // for (auto& viewpoint : viewpoints_)
      {
        if (viewpoints_[i].InCollision())
        {
          continue;
        }
        geometry_msgs::msg::Point viewpoint_position = viewpoints_[i].GetPosition();
        if (misc_utils_ns::InFOVSimple(
                Eigen::Vector3d(point.x, point.y, point.z),
                Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
        {
          viewpoints_[i].UpdateCoverage<PCLPointType>(point);
        }
      }
    }
  }

  template <class PCLPointType>
  void UpdateViewPointCoverage_for_frontier_viewpoint(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud, viewpoint_ns::ViewPoint& viewpoint)
  {


    for (const auto& point : cloud->points)
    {


      geometry_msgs::msg::Point viewpoint_position = viewpoint.GetPosition();
      if (misc_utils_ns::InFOVSimple(
              Eigen::Vector3d(point.x, point.y, point.z),
              Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
              vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
      {
        viewpoint.UpdateCoverage<PCLPointType>(point);
      }
      
    }
  }




  
  template <class PCLPointType>
  void Updateuav_ViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud,bool invalidate_viewpoint_and_point_in_covered_boxes)
  {
    if(invalidate_viewpoint_and_point_in_covered_boxes)
    {
      for (const auto& point : cloud->points)
      {
        for (int i = 0; i < ind_of_connected_viewpoints_in_noncovered_area.size(); i++)//仅仅对于 非 covered的 进行更新壁垒工作
        {
          geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints_in_noncovered_area[i])->GetPosition();
          if (misc_utils_ns::InFOVSimple(
                  Eigen::Vector3d(point.x, point.y, point.z),
                  Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                  vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
          {
            Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints_in_noncovered_area[i])->UpdateCoverage<PCLPointType>(point);
          }
        }
      }
    }
    else{
      for (const auto& point : cloud->points)
      {
        for (int i = 0; i < ind_of_connected_viewpoints.size(); i++)//对于每个viewpoints
        {
          geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetPosition();
          if (misc_utils_ns::InFOVSimple(
                  Eigen::Vector3d(point.x, point.y, point.z),
                  Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                  vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
          {
            Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->UpdateCoverage<PCLPointType>(point);
          }
        }
      }
    }
    
  }

  template <class PCLPointType>
  void UpdateRolledOverViewPointCoverage_for_frontier_viewpoint(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud,viewpoint_ns::ViewPoint& viewpoint)
  {
    for (const auto& point : cloud->points)
    {
      
        geometry_msgs::msg::Point viewpoint_position = viewpoint.GetPosition();
        if (misc_utils_ns::InFOVSimple(
                Eigen::Vector3d(point.x, point.y, point.z),
                Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
        {
          viewpoint.UpdateCoverage<PCLPointType>(point);
        }
      
    }
  }

  template <class PCLPointType>
  void UpdateRolledOverViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    for (const auto& point : cloud->points)
    {
      for (const auto& viewpoint_ind : updated_viewpoint_indices_)
      {
        int array_ind = grid_->GetArrayInd(viewpoint_ind);
        if (viewpoints_[array_ind].InCollision())
        {
          continue;
        }
        geometry_msgs::msg::Point viewpoint_position = viewpoints_[array_ind].GetPosition();
        if (misc_utils_ns::InFOVSimple(
                Eigen::Vector3d(point.x, point.y, point.z),
                Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
        {
          viewpoints_[array_ind].UpdateCoverage<PCLPointType>(point);
        }
      }
    }
  }

  template <class PCLPointType>
  void UpdateRolledOveruav_ViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud,bool invalidate_viewpoint_and_point_in_covered_boxes)
  {
    int ind_of_middle_box,ind_of_little_box;
    if(invalidate_viewpoint_and_point_in_covered_boxes)
    {
      for (const auto& point : cloud->points)
      {
        for (int i = 0; i < ind_of_connected_viewpoints_in_noncovered_area.size(); i++)    //仅仅对 非covered 里面的更新壁垒
        {
          
          Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_connected_viewpoints_in_noncovered_area[i]);
          if(updated_viewpoint_indices_.end() == std::find(updated_viewpoint_indices_.begin(), updated_viewpoint_indices_.end(), ind_of_middle_box))//不在新进来的点的集合内；就是确认该viewpoint点的大方格是不是刚刚更新的，如果是则continue
          {
            continue;
          }
          geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints_in_noncovered_area[i])->GetPosition();
          if (misc_utils_ns::InFOVSimple(
                  Eigen::Vector3d(point.x, point.y, point.z),
                  Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                  vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
          {
            Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints_in_noncovered_area[i])->UpdateCoverage<PCLPointType>(point);
          }
        }
      }
    }
    else{
      for (const auto& point : cloud->points)
      {
        for (int i = 0; i < ind_of_connected_viewpoints.size(); i++)    //对于每个新进来的viewpoint对象
        {
          
          Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_connected_viewpoints[i]);
          if(updated_viewpoint_indices_.end() == std::find(updated_viewpoint_indices_.begin(), updated_viewpoint_indices_.end(), ind_of_middle_box))//不在新进来的点的集合内；就是确认该viewpoint点的大方格是不是刚刚更新的，如果是则continue
          {
            continue;
          }
          geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetPosition();
          if (misc_utils_ns::InFOVSimple(
                  Eigen::Vector3d(point.x, point.y, point.z),
                  Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                  vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
          {
            Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->UpdateCoverage<PCLPointType>(point);
          }
        }
      }
    }
    
  }

  inline double GetSensorRange() const
  {
    return vp_.kSensorRange;
  }
  inline double GetCoverageOcclusionThr() const
  {
    return vp_.kCoverageOcclusionThr;
  }
  inline double GetCoverageDilationRadius() const
  {
    return vp_.kCoverageDilationRadius;
  }

  template <class PointType>
  bool VisibleByViewPoint(const PointType& point, int viewpoint_ind)
  {
    MY_ASSERT(grid_->InRange(viewpoint_ind));
    int array_ind = grid_->GetArrayInd(viewpoint_ind);
    geometry_msgs::msg::Point viewpoint_position = viewpoints_[array_ind].GetPosition();
    if (std::abs(point.z - viewpoint_position.z) > vp_.kDiffZMax)
    {
      return false;
    }
    if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                    Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                                    vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
                                    vp_.kInFovZDiffThreshold))
    {
      return false;
    }
    bool visible = viewpoints_[array_ind].CheckVisibility<PointType>(point, vp_.kCoverageOcclusionThr);

    return visible;
  }
  template <class PointType>
  bool VisibleByViewPoint_uav(const PointType& point, int viewpoint_ind)
  {
    // MY_ASSERT(grid_->InRange(viewpoint_ind));
    // int array_ind = grid_->GetArrayInd(viewpoint_ind);
    geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->GetPosition();
    if (std::abs(point.z - viewpoint_position.z) > vp_.kDiffZMax)
    {
      return false;
    }
    if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                    Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                                    vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
                                    vp_.kInFovZDiffThreshold))
    {
      return false;
    }
    bool visible = Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->CheckVisibility<PointType>(point, vp_.kCoverageOcclusionThr);

    return visible;
  }

  // Viewpoint management
  void ResetViewPoint(int viewpoint_ind, bool use_array_ind = false);
  void ResetViewPointCoverage();

  bool ViewPointInCollision(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind = false);

  bool ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind = false);

  bool ViewPointConnected(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind = false);

  bool ViewPointVisited(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind = false);

  bool ViewPointSelected(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind = false);

  bool IsViewPointCandidate(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind = false);

  bool ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind = false);

  bool ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind = false);

  double GetViewPointHeight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind = false);

  bool ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                             bool use_array_ind = false);

  geometry_msgs::msg::Point GetViewPointPosition(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointPosition(int viewpoint_ind, geometry_msgs::msg::Point position, bool use_array_ind = false);

  int GetViewPointCellInd(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind = false);

  int GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);
  void AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);
  void ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);

  void ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind = false);
  void AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind = false);
  void AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind = false);
  const std::vector<int>& GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind = false) const;
  const std::vector<int>& GetViewPointCoveredFrontierPointList(int viewpoint_ind, bool use_array_ind = false) const;

  int GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind = false);
  int GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind = false);
  int GetViewPointCoveredPointNum(const std::vector<bool>& point_list, int viewpoint_Hier_ind, bool use_array_ind = false);
  int GetViewPointCoveredFrontierPointNum(const std::vector<bool>& frontier_point_list, int viewpoint_index,
                                          bool use_array_ind = false);
  void UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index, bool use_array_ind = false);
  void UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                           bool use_array_ind = false);

  void UpdateViewPointGraph(bool add_frontier_viewpoint=false);
  std::vector<int> GetViewPointCandidateIndices() const
  {
    return candidate_indices_;
  }
  nav_msgs::msg::Path GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind, bool use_frontier_viewpoint=false);
  nav_msgs::msg::Path GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                          const Eigen::Vector3d& target_position);
  nav_msgs::msg::Path GetViewPointShortestPath_no_warn(const Eigen::Vector3d& start_position,
                                          const Eigen::Vector3d& target_position);
                                          
  bool GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                             const Eigen::Vector3d& target_position, double max_path_length,
                                             nav_msgs::msg::Path& path);

  void UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);

  int GetNearestCandidateViewPointInd(const Eigen::Vector3d& position);
  bool InLocalPlanningHorizon(const Eigen::Vector3d& position);
  
  bool UseFrontier()
  {
    return vp_.kUseFrontier;
  }
  // For visualization
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);
  void GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void visualize_all_viewpoints_();//可视化
  void visualize_free_viewpoints_();//可视化
  void visualize_in_collision_viewpoints_();//可视化
  void visualize_connected_viewpoints_();//可视化
  void visualize_connected_viewpoints_in_exploringarea();//可视化
  void visualize_visited_viewpoints_();//可视化
  void visualize_visited_positions_(const std::vector<Eigen::Vector3d>& positions);//可视化
  std::shared_ptr<viewpoint_ns::ViewPoint>& Get_littlebox_from_Hierarchy_ind(int ind_of_free_viewpoints);
  typedef std::unique_ptr<ViewPointManager> Ptr;
  // std::unique_ptr<grid_ns::Grid<std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>>>> &Get_big_box()
  // {
  //   return big_box;
  // }
  std::unique_ptr<grid_ns::Grid<std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>>>> &Get_big_box()//可视化疑似
  {
    return big_box;
  }
  std::unique_ptr<rolling_grid_ns::RollingGrid> &Get_grid_()//可视化疑似
  {
    return grid_;
  }
  ViewPointManagerParameter &Get_vp_()//可视化疑似
  {
    return vp_;
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pos;//可视化
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_sub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_all_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_free_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_collision_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_real_collision_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_connected_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_connected_inexploringarea_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoint_visited_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr viewpoints_in_lineofsight_pub;//可视化
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visited_positions_pub;//可视化
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr robot_point;//可视化
  void sub_robot_pos_deal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);//可视化
  void sub_deal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);//可视化
  void test_()//可视化
  {
    std::cout<<"测试"<<std::endl;
  }
  visualization_msgs::msg::Marker Return_connected_viewpoint_marker_()//可视化
  {
    return connected_viewpoint_marker_;
  }
  visualization_msgs::msg::Marker connected_viewpoint_marker_;//可视化
  void Two_index_to_Hierarchy_ind(const int& ind_of_middle_box, const int& ind_of_little_box, int &index_in_one);
  void Hierarchy_ind_to_into_Two(int &ind_of_middle_box, int &ind_of_little_box,const int& index_in_one);
  Eigen::Vector3i Hierarchy_ind_2_Hierarchy_sub(int Hierarchy_ind);
  int Hierarchy_sub_2_Hierarchy_ind(const Eigen::Vector3i& Hierarchy_sub);
  Eigen::Vector3i Hierarchy_Pos_2_Hierarchy_sub(const Eigen::Vector3d& pos);
  Eigen::Vector3d Hierarchy_sub_2Hierarchy_Pos_(const Eigen::Vector3i& Hierarchy_sub);
  Eigen::Vector3d Hierarchy_ind_2Hierarchy_Pos_(int Hierarchy_ind);
  void Abstract_free_viewpoints(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_);
  bool check_viewpoint_in_real_collision(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_, viewpoint_ns::ViewPoint& viewpoint);
  void Abstract_connected_viewpoints(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_);
  bool Hierarchy_InRange(const Eigen::Vector3i& sub);
  void Hierarchy_sub_2_midbox_ind_and_litbox_ind(const Eigen::Vector3i& sub,int& mid_box_ind,int& lit_box_ind);
  void midbox_ind_and_litbox_ind_2_Hierarchy_sub_( Eigen::Vector3i& Hierarchy_sub,const int& mid_box_ind,const int& lit_box_ind); 
  bool Get_nearest_connected_viewpoint(const Eigen::Vector3d& pos, int& Get_ind);


  int Get_Hier_ind_of_frontier_viewpoints(int i)
  {
    return i+vp_.viewpoint_num;
  }
  int Hier_ind_2_frontier_viewpoint_ind(int Hier_ind)
  {
    return Hier_ind-vp_.viewpoint_num;
  }
  bool Is_frontier_viewpoint_Hier_ind(int Hier_ind)
  {
    bool flag=false;
    if(Hier_ind >= vp_.viewpoint_num)
    {
      if(Hier_ind_2_frontier_viewpoint_ind(Hier_ind)<Hier_ind_of_cantidate_frontier_viewpoints.size())
      {
        flag=true;
      }
    }
    return flag;
  }
  std::shared_ptr<viewpoint_ns::ViewPoint>& Get_Frontier_viewpoint_from_Hier_ind(int Hier_ind)
  {
    return frontier_viewpoints_cantidate[Hier_ind_2_frontier_viewpoint_ind(Hier_ind)];
  }
  bool In_local_planner_region(Eigen::Vector3d pos)
  {
    // std::cout<<"左下：("<<(origin_.x()-vp_.length_size.x()/2)<<","<<(origin_.y()-vp_.length_size.y()/2)<<","<< (origin_.z()-2*vp_.length_size.z())<<")"<<std::endl;
    // std::cout<<"右上：("<<(origin_.x()+3*vp_.length_size.x()/2)<<","<<(origin_.y()+3*vp_.length_size.y()/2)<<","<< (origin_.z()+3*vp_.length_size.z())<<")"<<std::endl;
    if(!relocation_)
    return pos.x() >  origin_.x()                        &&   pos.y() >  origin_.y()                        &&   pos.z() >  origin_.z() &&
           pos.x() < (origin_.x()+vp_.length_size.x())   &&   pos.y() < (origin_.y()+vp_.length_size.y())   &&   pos.z() < (origin_.z()+vp_.length_size.z());
    else{
      return pos.x() >  origin_.x()+vp_.middle_box_side_.x()                      &&   pos.y() >  origin_.y()+vp_.middle_box_side_.y()                        &&   pos.z() >  origin_.z() &&
           pos.x() < (origin_.x()+vp_.length_size.x()-vp_.middle_box_side_.x())   &&   pos.y() < (origin_.y()+vp_.length_size.y()-vp_.middle_box_side_.y())   &&   pos.z() < (origin_.z()+vp_.length_size.z());
    }
  }
  bool In_2size_of_local_planner_region(Eigen::Vector3d pos)
  {
    // std::cout<<"左下：("<<(origin_.x()-vp_.length_size.x()/2)<<","<<(origin_.y()-vp_.length_size.y()/2)<<","<< (origin_.z()-2*vp_.length_size.z())<<")"<<std::endl;
    // std::cout<<"右上：("<<(origin_.x()+3*vp_.length_size.x()/2)<<","<<(origin_.y()+3*vp_.length_size.y()/2)<<","<< (origin_.z()+3*vp_.length_size.z())<<")"<<std::endl;
    return pos.x() > (origin_.x()-vp_.length_size.x()/2)   &&   pos.y() > (origin_.y()-vp_.length_size.y()/2)   &&   pos.z() > (origin_.z()-2*vp_.length_size.z()) &&
           pos.x() < (origin_.x()+3*vp_.length_size.x()/2)   &&   pos.y() < (origin_.y()+3*vp_.length_size.y()/2)   &&   pos.z() < (origin_.z()+3*vp_.length_size.z());
  }
  // bool In_the_middle_of_middle_boxes(Eigen::Vector3d pos)
  // {
  //   // std::cout<<"左下：("<<(origin_.x()-vp_.length_size.x()/2)<<","<<(origin_.y()-vp_.length_size.y()/2)<<","<< (origin_.z()-2*vp_.length_size.z())<<")"<<std::endl;
  //   // std::cout<<"右上：("<<(origin_.x()+3*vp_.length_size.x()/2)<<","<<(origin_.y()+3*vp_.length_size.y()/2)<<","<< (origin_.z()+3*vp_.length_size.z())<<")"<<std::endl;
  //   return pos.x() > (origin_.x()-vp_.length_size.x()/2)   &&   pos.y() > (origin_.y()-vp_.length_size.y()/2)   &&   pos.z() > (origin_.z()-2*vp_.length_size.z()) &&
  //          pos.x() < (origin_.x()+3*vp_.length_size.x()/2)   &&   pos.y() < (origin_.y()+3*vp_.length_size.y()/2)   &&   pos.z() < (origin_.z()+3*vp_.length_size.z());
  // }

  template <class PlannerCloudPointType>
  void Get_frontier_viewpoint(const std::unique_ptr<planning_env_ns::PlanningEnv> & planning_env_, const std::vector<Eigen::Vector3d> & visited_positions_, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph_)
  {
    for(auto frontier_viewpoint : frontier_viewpoints)
    {
      frontier_viewpoint->Set_first_added(false);
    }


    frontier_viewpoints_cantidate.clear();
    misc_utils_ns::Timer Get_frontier_viewpoint_timer("Get_frontier_viewpoint2");
    misc_utils_ns::Timer timmer1("collision visited 刪除          ");
    misc_utils_ns::Timer timmer2("濾波          ");
    misc_utils_ns::Timer timmer3("更新壁壘計算數量          ");
    misc_utils_ns::Timer timmer4("地鐵          ");
    misc_utils_ns::Timer timmer5("微調位置          ");
    Get_frontier_viewpoint_timer.Start();
    // timmer1.Start();
    
    // double intervel_radius = 2;
    // double para_visited_radius = 2.5;
    // int least_visible_unknown_num = 30;
    // double in_collision_viewpoint_get_connected_viewpoint_dis = 1.2;
    // double search_radius_thr=1;
    
    // int least_visible_free_num = 100;
    geometry_msgs::msg::Point temp_point1,temp_point2;
    double temp_dist;
    bool addmision_flag;
    std::shared_ptr<viewpoint_ns::ViewPoint> temp_ptr;
    for(auto new_point:  planning_env_->frontier_center_point->cloud_->points)
    {
      addmision_flag=true;
      //加入
      if(frontier_viewpoints.size()==0)//没有的时候直接进
      {
        frontier_viewpoints.push_back(temp_ptr);
        frontier_viewpoints[0] = std::shared_ptr<viewpoint_ns::ViewPoint> (new viewpoint_ns::ViewPoint(new_point.x,new_point.y,new_point.z));
        temp_point1.x = new_point.x;
        temp_point1.y = new_point.y;
        temp_point1.z = new_point.z;
        frontier_viewpoints[0]->SetinitiaPosition(temp_point1);
        frontier_viewpoints[0]->Reset();
        frontier_viewpoints[0]->Set_first_added(true);
        continue;
      }
      //判断，距离够大的留下来
      for(auto viewpoint: frontier_viewpoints)
      {
        temp_point1.x = new_point.x;
        temp_point1.y = new_point.y;
        temp_point1.z = new_point.z;
        temp_point2 = viewpoint->GetPosition();
        temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(temp_point1, temp_point2);
        if(temp_dist < intervel_radius)
        {
          addmision_flag=false;
          break;
        }
      }
      
      if(addmision_flag)
      {
        frontier_viewpoints.push_back(temp_ptr);
        frontier_viewpoints[frontier_viewpoints.size()-1] = std::shared_ptr<viewpoint_ns::ViewPoint> (new viewpoint_ns::ViewPoint(new_point.x,new_point.y,new_point.z));
        temp_point1.x = new_point.x;
        temp_point1.y = new_point.y;
        temp_point1.z = new_point.z;
        frontier_viewpoints[frontier_viewpoints.size()-1]->SetinitiaPosition(temp_point1);
        frontier_viewpoints[frontier_viewpoints.size()-1]->Reset();
        frontier_viewpoints[frontier_viewpoints.size()-1]->Set_first_added(true);
      }
      //加入
    }
    // timmer1.Stop(true,"ms");//11
    // timmer1.Start();
    //更新下面判断点使命是否完成的依据   ，收集所有半径内的unknow点，让下面判断是否还能够覆盖到这些点（只收集了有限范围内的点）
    pcl::PointXYZI tem_point;
    std::vector<int> in_range_indices;
    std::vector<float> in_range_sqdist;
    planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points.clear();
    planning_env_->Local_free_points_for_frontier_viewpoint->cloud_->points.clear();

    Eigen::Vector4f minPoint(origin_.x()-vp_.kNeighborRange, origin_.y()-vp_.kNeighborRange, origin_.z()-vp_.kNeighborRange,1.0);
    Eigen::Vector4f maxPoint(origin_.x() + vp_.length_size.x() +vp_.kNeighborRange, origin_.y() + vp_.length_size.y()+vp_.kNeighborRange, origin_.z() + vp_.length_size.z()+vp_.kNeighborRange,1.0);
    // std::cout<<"origin_:("<<origin_.x()<<","<<origin_.y()<<","<<origin_.z()<<std::endl;
    // std::cout<<"origin_:("<<origin_.x() + vp_.length_size.x()<<","<<origin_.y() + vp_.length_size.y()<<","<<origin_.z() + vp_.length_size.z()<<std::endl;
    cropBoxFilter_unknown_point.setMin(minPoint);
    cropBoxFilter_unknown_point.setMax(maxPoint);
    cropBoxFilter_unknown_point.setInputCloud(planning_env_->Occupancy_unknow_cloud->cloud_);
    cropBoxFilter_unknown_point.filter(*planning_env_->Occupancy_unknow_cloud->cloud_);
    cropBoxFilter_unknown_point.setInputCloud(planning_env_->Local_free_points_->cloud_);
    cropBoxFilter_unknown_point.filter(*planning_env_->Local_free_points_->cloud_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    // kdtree_occupy_unknow_->setInputCloud(planning_env_->Occupancy_unknow_cloud->cloud_);
    planning_env_->Cantidate_frontier_viewpoint->cloud_->points.clear();
    planning_env_->In_field_frontier_viewpoint->cloud_->points.clear();
    planning_env_->All_frontier_viewpoint->cloud_->points.clear();
    timmer2.Start();
    
    for(auto viewpoint: frontier_viewpoints)
    {
      
      temp_point1 = viewpoint->GetPosition();
      tem_point.x = temp_point1.x;
      tem_point.y = temp_point1.y;
      tem_point.z = temp_point1.z;

      Eigen::Vector3d frontier_viewpoint_position;
      frontier_viewpoint_position.x() = temp_point1.x;
      frontier_viewpoint_position.y() = temp_point1.y;
      frontier_viewpoint_position.z() = temp_point1.z;
      if(!InRange_Hier_sub(Hierarchy_Pos_2_Hierarchy_sub(frontier_viewpoint_position)))
      {
        continue;
      }
      // kdtree_occupy_unknow_->radiusSearch(tem_point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
      // for(auto ind : in_range_indices)
      // {
      //   planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points.push_back(planning_env_->Occupancy_unknow_cloud->cloud_->points[ind]);
      // }
      Eigen::Vector4f minPoint(tem_point.x-vp_.kNeighborRange, tem_point.y-vp_.kNeighborRange, tem_point.z-vp_.kNeighborRange, 1.0);
      Eigen::Vector4f maxPoint(tem_point.x+vp_.kNeighborRange, tem_point.y+vp_.kNeighborRange, tem_point.z+vp_.kNeighborRange, 1.0);
      cropBoxFilter_unknown_point.setMin(minPoint);
      cropBoxFilter_unknown_point.setMax(maxPoint);
      cropBoxFilter_unknown_point.setInputCloud(planning_env_->Occupancy_unknow_cloud->cloud_);
      cropBoxFilter_unknown_point.filter(*temp_cloud);
      *planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_ += *temp_cloud;

      cropBoxFilter_unknown_point.setInputCloud(planning_env_->Local_free_points_->cloud_);
      cropBoxFilter_unknown_point.filter(*temp_cloud);
      *planning_env_->Local_free_points_for_frontier_viewpoint->cloud_ += *temp_cloud;
    }
    //更新下面判断点使命是否完成的依据
    planning_env_->Occupancy_unknow_cloud->Publish();//可视化
    planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->Publish();//可视化
    planning_env_->Local_free_points_->Publish();
    planning_env_->Local_free_points_for_frontier_viewpoint->Publish();
    timmer2.Stop(vp_.show_time);
    // timmer1.Stop(true,"ms");//22
    // timmer1.Start();
    //EXPL 上面做加入点工作
    
    //EXPL 下面做删除点工作

    //踢出去  没被踢出去的都连接地铁//工作
    bool erase_flag = false;
    Eigen::Vector3d frontier_viewpoint_position, frontier_initi_viewpoint_position;
    int visible_unknow_point_count=0;
    int visible_free_point_count=0;
    Eigen::Vector3d start_position,end_position;
    for(int i=0; i<frontier_viewpoints.size(); i++)
    {
      timmer1.Start();
      frontier_viewpoint_position.x() = frontier_viewpoints[i]->GetPosition().x;
      frontier_viewpoint_position.y() = frontier_viewpoints[i]->GetPosition().y;
      frontier_viewpoint_position.z() = frontier_viewpoints[i]->GetPosition().z;
      frontier_initi_viewpoint_position.x() = frontier_viewpoints[i]->GetintiPosition().x;
      frontier_initi_viewpoint_position.y() = frontier_viewpoints[i]->GetintiPosition().y;
      frontier_initi_viewpoint_position.z() = frontier_viewpoints[i]->GetintiPosition().z;
      pcl::PointXYZI point_pcl;
      point_pcl.x = frontier_viewpoint_position.x();
      point_pcl.y = frontier_viewpoint_position.y();
      point_pcl.z = frontier_viewpoint_position.z();
      planning_env_->All_frontier_viewpoint->cloud_->points.push_back(point_pcl);
      if(!InRange_Hier_sub(Hierarchy_Pos_2_Hierarchy_sub(frontier_viewpoint_position)))
      {
        continue;
      }
      //（real collision）
      if(check_viewpoint_in_real_collision(planning_env_->Get_rolling_occupancy_grid_(),*frontier_viewpoints[i]))
      {
        erase_flag = true;

        temp_point2 = frontier_viewpoints[i]->GetPosition();
        for(auto connected_viewpoint_ind : ind_of_connected_viewpoints)
        {
          temp_point1 = Get_littlebox_from_Hierarchy_ind(connected_viewpoint_ind)->GetPosition();
          temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(temp_point1, temp_point2);
          if(temp_dist < in_collision_viewpoint_get_connected_viewpoint_dis)
          {
            // std::cout<<"拯救一个frontier_viewpoint"<<std::endl;
            frontier_viewpoints[i]->SetPosition(temp_point1);
            frontier_viewpoints[i]->SetinitiaPosition(temp_point1);
            frontier_viewpoints[i]->Reset();
            erase_flag = false;
          }
        }
        if(erase_flag)
        {
          keypose_graph_->erase_node_edge_for_frontier_viewpoint(frontier_viewpoints[i]->Get_keypose_station_ind());;
          if(InRange_Hier_ind(frontier_viewpoints[i]->conntedcomviewpointHierind()))
          Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->SetSelected(false);
          frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
          i--;
          continue;
        }
      }
      //（real collision）
      //机器人访问到了也踢出去
      erase_flag = false;
      for ( auto position : visited_positions_)
      {
        temp_point1 = frontier_viewpoints[i]->GetPosition();
        // temp_point2.x = robot_position_.x();
        // temp_point2.y = robot_position_.y();
        // temp_point2.z = robot_position_.z();
        temp_point2.x = position.x();
        temp_point2.y = position.y();
        temp_point2.z = temp_point1.z;//为了让车提供便利，车太矮了
        position.z() += position_add_high_for_guv;
        temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(temp_point1, temp_point2);

        double delete_radius_ = para_visited_radius;
        bool frontier_viewpoint_notcandidate = false;//对于那些非candidate点，在机器人旁边又不可见因而不能消除它，此时无需多言 不用可见，直接弄他
        if(!frontier_viewpoints[i]->IsCandidate() && !frontier_viewpoints[i]->first_added())
        {
          delete_radius_ = para_visited_radius*5;
          frontier_viewpoint_notcandidate = true;
        }
        if(temp_dist < delete_radius_ && abs(position.z()-temp_point1.z)<para_visited_radius_z)//在足够小的范围内 
        {
          if(planning_env_->Get_rolling_occupancy_grid_()->Visible_in_RayTraceHelper_for_viewpoint_connected(frontier_viewpoint_position,position,false,0)||frontier_viewpoint_notcandidate)
          {//并且 可见
            erase_flag = true;
            break;
          }
        }
      }

      if(erase_flag)
      {
        keypose_graph_->erase_node_edge_for_frontier_viewpoint(frontier_viewpoints[i]->Get_keypose_station_ind());
        if(InRange_Hier_ind(frontier_viewpoints[i]->conntedcomviewpointHierind()))
        Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->SetSelected(false);
        frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
        i--;
        continue;
      }
      //机器人访问到了也踢出去
      timmer1.Stop(vp_.show_time);
      timmer3.Start();
      // 只针对在 范围内 的这些点踢出去，不然超出occupancy的范围，这样的剔除是无意义的
      visible_unknow_point_count = 0;
      visible_free_point_count = 0;
        //更新壁垒
      UpdateViewPointCoverage_for_frontier_viewpoint<PlannerCloudPointType>(planning_env_->GetDiffCloud(),*frontier_viewpoints[i]);
      UpdateRolledOverViewPointCoverage_for_frontier_viewpoint<PlannerCloudPointType>(planning_env_->GetStackedCloud(),*frontier_viewpoints[i]);
      //计算能看见的unknow数量
      for(auto point : planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points)
      {
        if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                    frontier_viewpoint_position,
                                    vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
                                    vp_.kInFovZDiffThreshold))//0.2125 12  3.5294 1.5
        {
          if(frontier_viewpoints[i]->CheckVisibility<pcl::PointXYZI>(point, vp_.kCoverageOcclusionThr))
          {
            visible_unknow_point_count++;
            if(visible_unknow_point_count > least_visible_unknown_num+1)
            {
              break;
            }
          }
        }
        
      }
      for(auto point : planning_env_->Local_free_points_for_frontier_viewpoint->cloud_->points)
      {
        if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                    frontier_viewpoint_position,
                                    vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
                                    vp_.kInFovZDiffThreshold))//0.2125 12  3.5294 1.5
        {
          if(frontier_viewpoints[i]->CheckVisibility<pcl::PointXYZI>(point, vp_.kCoverageOcclusionThr))
          {
            visible_free_point_count++;
            if(visible_free_point_count > least_visible_free_num+1)
            {
              break;
            }
          }
        }
        
      }
      // std::cout<<"visible_free_point_count: "<<visible_free_point_count<<std::endl;

      if(visible_unknow_point_count <= least_visible_unknown_num || visible_free_point_count < least_visible_free_num)
      {
        
        keypose_graph_->erase_node_edge_for_frontier_viewpoint(frontier_viewpoints[i]->Get_keypose_station_ind());;
        if(InRange_Hier_ind(frontier_viewpoints[i]->conntedcomviewpointHierind()))
        Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->SetSelected(false);
        frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
        i--;
        continue;
      }
      //TODO std::cout<<"visible_unknow_point_count"<<visible_unknow_point_count<<std::endl;
      //半径之内不再能够照射到unknow点也踢出去
      timmer3.Stop(vp_.show_time);


      //EXPL 上面是删除点
      
      planning_env_->In_field_frontier_viewpoint->cloud_->points.push_back(point_pcl);
      //EXPL 下面是更新点的环境参数
      

      timmer4.Start();
      if(!frontier_viewpoints[i]->connectedtograph())//没连接过的地铁去连接
      {

        std::vector<int> near_node_ind_vec = keypose_graph_->Get_near_pose(10,frontier_viewpoints[i]->GetPosition());//最多10个，
        // std::cout<<near_node_ind_vec.size()<<std::endl;
        reverse(near_node_ind_vec.begin(),near_node_ind_vec.end());//现在小的在前面
        // std::cout<<near_node_ind_vec.size()<<std::endl;
        for(auto ind : near_node_ind_vec)
        {
          // start_position.x() = frontier_viewpoints[i]->GetPosition().x;
          // start_position.y() = frontier_viewpoints[i]->GetPosition().y;
          // start_position.z() = frontier_viewpoints[i]->GetPosition().z;
          start_position = frontier_viewpoint_position;
          end_position.x() = keypose_graph_->Get_nodes_()[ind].position_.x;
          end_position.y() = keypose_graph_->Get_nodes_()[ind].position_.y;
          end_position.z() = keypose_graph_->Get_nodes_()[ind].position_.z;
          nav_msgs::msg::Path path_in_between = GetViewPointShortestPath_no_warn(
                        start_position, end_position);
          // std::cout<<path_in_between.poses.size()<<std::endl;
          if(path_in_between.poses.size()!=0)
          {
            
            frontier_viewpoints[i]->Set_connected_to_graph(true);
            path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
            // std::cout<<"!!!!!!!!!!!!!!连接地铁："<<path_in_between.poses.size()<<std::endl;
            for (auto& pose : path_in_between.poses)
            {
              pose.pose.orientation.w = -1;
            }
            // std::cout<<"------zz"<<std::endl;
            frontier_viewpoints[i]->Set_station_ind(keypose_graph_->AddPath_for_frontier_viewpoint(path_in_between));
            // std::cout<<"------1zz"<<std::endl;
            break;
          }
        }
        //那种隔岸cantidate 但是就是连不上类型 
        if(!frontier_viewpoints[i]->connectedtograph() && frontier_viewpoints[i]->IsCandidate())//上面没通过 对于那种隔岸 但是 已经被选为cantidate的
        {
          int new_frontier_viewpoint_ind = keypose_graph_->AddNonKeyposeNode(frontier_viewpoints[i]->GetPosition());
          int new_viewpoint_ind = keypose_graph_->AddNonKeyposeNode(Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->GetPosition());
          temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(frontier_viewpoints[i]->GetPosition(), Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->GetPosition());
          keypose_graph_->AddEdge(new_frontier_viewpoint_ind, new_viewpoint_ind, temp_dist);

          for(auto ind : near_node_ind_vec)
          {
            geometry_msgs::msg::Point com_viewpoint_position = Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->GetPosition();
            start_position.x() = com_viewpoint_position.x;
            start_position.y() = com_viewpoint_position.y;
            start_position.z() = com_viewpoint_position.z;
            end_position.x() = keypose_graph_->Get_nodes_()[ind].position_.x;
            end_position.y() = keypose_graph_->Get_nodes_()[ind].position_.y;
            end_position.z() = keypose_graph_->Get_nodes_()[ind].position_.z;
            nav_msgs::msg::Path path_in_between = GetViewPointShortestPath_no_warn(
                          start_position, end_position);
            // std::cout<<path_in_between.poses.size()<<std::endl;
            if(path_in_between.poses.size()!=0)
            {
              
              path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
              // std::cout<<"!!!!!!!!!!!!!!连接地铁："<<path_in_between.poses.size()<<std::endl;
              for (auto& pose : path_in_between.poses)
              {
                pose.pose.orientation.w = -1;
              }
              // std::cout<<"------zz"<<std::endl;
              frontier_viewpoints[i]->Set_station_ind(keypose_graph_->AddPath_for_frontier_viewpoint(path_in_between));
              // std::cout<<"------1zz"<<std::endl;
              temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[0].pose.position, Get_littlebox_from_Hierarchy_ind(frontier_viewpoints[i]->conntedcomviewpointHierind())->GetPosition());

              keypose_graph_->AddEdge(frontier_viewpoints[i]->Get_keypose_station_ind()[0],new_viewpoint_ind,temp_dist);
              frontier_viewpoints[i]->Get_keypose_station_ind().push_back(new_frontier_viewpoint_ind);
              frontier_viewpoints[i]->Get_keypose_station_ind().push_back(new_viewpoint_ind);
              break;
            }
          }

          frontier_viewpoints[i]->Set_connected_to_graph(true);
        }
      }
      timmer4.Stop(vp_.show_time);
      //连接地铁

      timmer5.Start();
      // timmer1.Start();
      bool set_cantidate_flag = false;
      Eigen::Vector3d connected_viewpoint_position;
      std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid>& rolling_occupancy_grid_ = planning_env_->Get_rolling_occupancy_grid_();
      Eigen::Vector3i frontier_viewpoint_sub_in_occupancy 
                    = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(frontier_viewpoint_position);
      double min_dist_vie_fronview=DBL_MAX;
      int min_connect_viewpoint_ind = -1;
      if(rolling_occupancy_grid_->Get_occupancy_array_()->InRange(frontier_viewpoint_sub_in_occupancy))
      {
        for(auto ind : ind_of_connected_viewpoints)
        {
          geometry_msgs::msg::Point temp_point = Get_littlebox_from_Hierarchy_ind(ind)->GetPosition();
          connected_viewpoint_position.x() = temp_point.x;
          connected_viewpoint_position.y() = temp_point.y;
          connected_viewpoint_position.z() = temp_point.z;
          if(rolling_occupancy_grid_->Visible_in_RayTraceHelper_for_viewpoint_connected(frontier_viewpoint_position,connected_viewpoint_position,true,1))//可以出现未知点，但是不能出现障碍点
          {
            
            temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(frontier_viewpoints[i]->GetPosition(), temp_point);
            // if(temp_dist >vp_.middle_box_bigger_side*2)
            // {
            //   continue;
            // }
            set_cantidate_flag=true;
            if(min_dist_vie_fronview > temp_dist)
            {
              min_dist_vie_fronview = temp_dist;
              min_connect_viewpoint_ind = ind;
            }
            
          }
        }
      }
      if(set_cantidate_flag)
      {
        planning_env_->Cantidate_frontier_viewpoint->cloud_->points.push_back(point_pcl);
        frontier_viewpoints[i]->Set_connted_com_viewpoint_Hier_ind(min_connect_viewpoint_ind);
        
        Get_littlebox_from_Hierarchy_ind(min_connect_viewpoint_ind)->SetSelected(true);
        frontier_viewpoints_cantidate.push_back(frontier_viewpoints[i]);
      }
      else{//针对卡在墙里面 或者第一时间不能跟connected viewpoint面对面 的frontier viewpoint点

        if(frontier_viewpoints[i]->Getnear_free_position_in_occupancy().empty())
        {
          frontier_viewpoints[i]->Getnear_free_position_in_occupancy() = rolling_occupancy_grid_->Get_near_free_position_in_occupancy(frontier_initi_viewpoint_position, search_radius_thr);
        }
        if(!frontier_viewpoints[i]->Getnear_free_position_in_occupancy().empty())
        {
          Eigen::Vector3d reset_position = frontier_viewpoints[i]->Getnear_free_position_in_occupancy()[0];
          geometry_msgs::msg::Point reset_position_;
          reset_position_.x = reset_position.x();
          reset_position_.y = reset_position.y();
          reset_position_.z = reset_position.z();
          frontier_viewpoints[i]->SetPosition(reset_position_);
          frontier_viewpoints[i]->Reset();
          frontier_viewpoints[i]->Getnear_free_position_in_occupancy().erase(frontier_viewpoints[i]->Getnear_free_position_in_occupancy().begin());
        }
          
      }
      frontier_viewpoints[i]->SetCandidate(set_cantidate_flag);
      timmer5.Stop(vp_.show_time);
      // timmer1.Stop(true,"ms");
      // 跟任何 connectviewpoint点能对上眼 设置candidate


      //第一眼出现还没被选为cantidate的，选取周围free位置
      //TODO 工作
      
    }
    
    //TODO 要把frontier_viewpoint加入到 维护 viewpoint的 graph_三件套体系（还是四件套？）

    //TODO 确定一下 全局路径规划是怎么被留下来的， 毕竟这里的viewpoint与那个viewpoint不是一个体系的， 他作为cantidate点离开边境的时候会不会留下全局路径规划
    Get_frontier_viewpoint_timer.Stop(vp_.show_time,"ms");
    // std::cout<<"总共的数量："<<frontier_viewpoints.size()<<std::endl;
    // std::cout<<"里面的数量："<<planning_env_->In_field_frontier_viewpoint->cloud_->points.size()<<std::endl;
    // std::cout<<"候选的数量："<<planning_env_->Cantidate_frontier_viewpoint->cloud_->points.size()<<std::endl;
    // std::cout<<"-----"<<std::endl;
    planning_env_->Cantidate_frontier_viewpoint->Publish();
    planning_env_->In_field_frontier_viewpoint->Publish();
    planning_env_->All_frontier_viewpoint->Publish();
    //踢出去（real collision）
    
  }
  const std::vector<std::vector<int>>& Return_uav_viewpoints_neighbor_indices_()
  {
    return uav_viewpoints_neighbor_indices_;
  }
  const std::vector<std::shared_ptr<viewpoint_ns::ViewPoint>>& Return_frontier_viewpoints()
  {
    return frontier_viewpoints;
  }
  double Return_middle_box_bigger_side()
  {
    return vp_.middle_box_bigger_side;
  }
  double Return_big_box_bigger_side()
  {
    return vp_.big_box_bigger_side;
  }
  
private:
  void ComputeConnectedNeighborIndices();
  void Compute_Uav_Viewpoints_NeighborIndices();//对标上面这行函数
  void ComputeInRangeNeighborIndices();
  void GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph, std::vector<std::vector<double>>& dist,
                                  std::vector<geometry_msgs::msg::Point>& positions, bool add_frontier_viewpoint=false);
  void GetCollisionCorrespondence();
  
  bool initialized_;
  ViewPointManagerParameter vp_;
  //EXPL 已经清理
  std::unique_ptr<rolling_grid_ns::RollingGrid> grid_;
  std::vector<viewpoint_ns::ViewPoint> viewpoints_;
  std::vector<std::shared_ptr<viewpoint_ns::ViewPoint>>  frontier_viewpoints;
  std::vector<std::shared_ptr<viewpoint_ns::ViewPoint>>  frontier_viewpoints_cantidate;
  std::vector<std::vector<int>> connected_neighbor_indices_;
  std::vector<std::vector<double>> connected_neighbor_dist_;
  std::vector<std::vector<double>> uav_viewpoint_neighbor_dist_;
  std::vector<std::vector<int>> in_range_neighbor_indices_;
  std::vector<int> updated_viewpoint_indices_;
  std::vector<int> graph_index_map_;
  std::vector<int> graph_index_map_for_frontier_viewpoint_;
  Eigen::Vector3d robot_position_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d origin_last_for_viewpoint_update_fail;
  bool traspass;
  Eigen::Vector3d collision_grid_origin_;
  Eigen::Vector3d local_planning_horizon_size_;
  std::unique_ptr<grid_ns::Grid<std::vector<int>>> collision_grid_;
  std::vector<int> collision_point_count_;
  std::vector<std::vector<int>> candidate_viewpoint_graph_;
  std::vector<std::vector<double>> candidate_viewpoint_dist_;
  std::vector<geometry_msgs::msg::Point> candidate_viewpoint_position_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_viewpoint_candidate_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_viewpoint_in_collision_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_occupy_unknow_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_candidate_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_in_collision_cloud_;

  geometry_msgs::msg::Polygon viewpoint_boundary_;
  std::vector<geometry_msgs::msg::Polygon> nogo_boundary_;
  std::unique_ptr<grid_ns::Grid<std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>>>> big_box;
  std::vector<int> ind_of_free_viewpoints;
  std::vector<std::vector<int>> uav_viewpoints_neighbor_indices_;//对标connected_neighbor_indices_
  std::vector<Eigen::Vector3i> idx_addon_check_collision;
  pcl::CropBox<pcl::PointXYZI> cropBoxFilter_unknown_point;
  double intervel_radius;
  double para_visited_radius;
  double para_visited_radius_z;
  double position_add_high_for_guv;
  int least_visible_unknown_num;
  int least_visible_free_num;
  double in_collision_viewpoint_get_connected_viewpoint_dis;
  double search_radius_thr;
  bool Set_robot_neigbour_connected;

  bool relocation_;
};

}  // namespace uav_viewpoint_manager_ns