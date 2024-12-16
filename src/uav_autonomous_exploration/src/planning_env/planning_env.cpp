/**
 * @file planning_env.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the world representation using point clouds
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 * @modified_by Huazhang Zhu (22210720332@m.fudan.edu.cn)
 * @date 2023-12-21
 */

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>
#include <uav_viewpoint_manager/viewpoint_manager.h>
namespace planning_env_ns
{
void PlanningEnvParameters::ReadParameters(std::shared_ptr<rclcpp::Node>& nh)
{
  invalidate_viewpoint_and_point_in_covered_boxes = misc_utils_ns::getParam<bool>(nh, "invalidate_viewpoint_and_point_in_covered_boxes", false);
  std::cout<<"planenv invalidate_viewpoint_and_point_in_covered_boxes:"<<invalidate_viewpoint_and_point_in_covered_boxes<<std::endl;
  kSurfaceCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 0.2);
  kCollisionCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kCollisionCloudDwzLeafSize", 0.2);
  kKeyposeGraphCollisionCheckRadius =
      misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.4);
  kKeyposeGraphCollisionCheckPointNumThr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 1);

  kKeyposeCloudStackNum = misc_utils_ns::getParam<int>(nh, "kKeyposeCloudStackNum", 5);

  kPointCloudRowNum = misc_utils_ns::getParam<int>(nh, "kPointCloudRowNum", 20);
  kPointCloudColNum = misc_utils_ns::getParam<int>(nh, "kPointCloudColNum", 20);
  kPointCloudLevelNum = misc_utils_ns::getParam<int>(nh, "kPointCloudLevelNum", 10);
  kMaxCellPointNum = misc_utils_ns::getParam<int>(nh, "kMaxCellPointNum", 100000);
  kPointCloudCellSize = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 24.0);
  kPointCloudCellHeight = misc_utils_ns::getParam<double>(nh, "kPointCloudCellHeight", 3.0);
  kPointCloudManagerNeighborCellNum = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
  kCoverCloudZSqueezeRatio = misc_utils_ns::getParam<double>(nh, "kCoverCloudZSqueezeRatio", 2.0);

  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);
  kFrontierClusterTolerance = misc_utils_ns::getParam<double>(nh, "kFrontierClusterTolerance", 1.0);
  kFrontierClusterMinSize = misc_utils_ns::getParam<int>(nh, "kFrontierClusterMinSize", 30);

  kUseCoverageBoundaryOnFrontier = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnFrontier", false);
  kUseCoverageBoundaryOnObjectSurface = misc_utils_ns::getParam<bool>(nh, "kUseCoverageBoundaryOnObjectSurface", false);

  int viewpoint_number = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_x", 40);
  double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
  double local_planning_horizon_half_size = viewpoint_number * viewpoint_resolution / 2;
  double sensor_range = misc_utils_ns::getParam<double>(nh, "kSensorRange", 15);
  
  kExtractFrontierRange.x() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.y() = local_planning_horizon_half_size + sensor_range * 2;
  kExtractFrontierRange.z() = 2;
}

PlanningEnv::PlanningEnv(std::shared_ptr<rclcpp::Node>& nh,std::shared_ptr<rclcpp::Node>& nh_p, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
  , local_pcl_map_for_ego_flag(false)
  , valid_length_of_frontier_point_clouds_vec(0)
  , valid_frontier_point_num_thred(100)
  // , local_pcl_map_for_ego_cloud_kdtree_flag(false)
{
  // subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("local_pcl_map_for_ego", 5, std::bind(&PlanningEnv::Pcl_call_back, this, std::placeholders::_1));
  parameters_.ReadParameters(nh);
  keypose_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }

  vertical_surface_cloud_stack_.resize(parameters_.kKeyposeCloudStackNum);
  for (int i = 0; i < vertical_surface_cloud_stack_.size(); i++)
  {
    vertical_surface_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  keypose_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", world_frame_id);
  stacked_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
  stacked_vertical_surface_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "stacked_vertical_surface_cloud", world_frame_id);

  stacked_vertical_surface_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  local_pcl_map_for_ego_cloud_kdtree_ =
      pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  vertical_surface_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

  diff_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  terrain_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

  planner_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);
  pointcloud_manager_ = std::make_unique<pointcloud_manager_ns::PointCloudManager>(
      parameters_.kPointCloudRowNum, parameters_.kPointCloudColNum, parameters_.kPointCloudLevelNum,
      parameters_.kMaxCellPointNum, parameters_.kPointCloudCellSize, parameters_.kPointCloudCellHeight,
      parameters_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = parameters_.kSurfaceCloudDwzLeafSize;

  rolling_occupancy_grid_ = std::make_unique<rolling_occupancy_grid_ns::RollingOccupancyGrid>(nh);

  squeezed_planner_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id);
  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  uav_uncovered_cloud_ = 
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uav_uncovered_cloud", world_frame_id);
  uncovered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
  uav_uncovered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uav_uncovered_frontier_cloud", world_frame_id);
  uncovered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
  frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
  filtered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
  occupied_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
  free_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
  unknown_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);
  frontier_center_point = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_center_point", world_frame_id);
  rolling_occupancy_grid_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_occupancy_grid_cloud", world_frame_id);

  rolling_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolling_frontier_cloud", world_frame_id);

  rolling_filtered_frontier_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "rolling_filtered_frontier_cloud", world_frame_id);

  rolled_in_occupancy_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_in_occupancy_cloud", world_frame_id);
  rolled_out_occupancy_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "rolled_out_occupancy_cloud", world_frame_id);

  points_from_occupancy_cloud_grid_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "points_from_occupancy_cloud_grid_", world_frame_id);
  KNOWN_WORLD = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "KNOWN_WORLD", world_frame_id);//可视化
  //Local_free_points_、Local_Occupy_points_、Local_normal_points
  Local_free_points_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Local_free_points_", world_frame_id);//可视化
  Local_Occupy_points_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Local_Occupy_points_", world_frame_id);//可视化
  Occupancy_non_free_cloud = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Occupancy_non_free_cloud", world_frame_id);//需要使用
  Occupancy_unknow_cloud = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Occupancy_unknow_cloud", world_frame_id);//需要使用
  Occupancy_unknow_cloud_for_frontier_viewpoint = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Occupancy_unknow_cloud_for_frontier_viewpoint", world_frame_id);//需要使用
  Local_free_points_for_frontier_viewpoint = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Local_free_points_for_frontier_viewpoint", world_frame_id);//需要使用
  Cantidate_frontier_viewpoint = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "Cantidate_frontier_viewpoint", world_frame_id);//可视化
  All_frontier_viewpoint = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "All_frontier_viewpoint", world_frame_id);//可视化
  In_field_frontier_viewpoint = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "In_field_frontier_viewpoint", world_frame_id);//可视化
  Local_normal_points = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZINormal>>(
      nh, "Local_normal_points", world_frame_id);//可视化

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree_rolling_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(parameters_.kSurfaceCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  Eigen::Vector3d rolling_occupancy_grid_resolution = rolling_occupancy_grid_->GetResolution();
  double vertical_frontier_neighbor_search_radius =
      std::max(rolling_occupancy_grid_resolution.x(), rolling_occupancy_grid_resolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, rolling_occupancy_grid_resolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
  std::shared_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> temp;
  for(int i=0;i<10;i++)
  {
    frontier_point_clouds_vec.push_back(temp);
    frontier_point_clouds_vec[i] = 
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cluster_"+std::to_string(i), world_frame_id);
  }
  valid_frontier_point_num_thred = misc_utils_ns::getParam<int>(nh, "valid_frontier_point_num_thred", 100);
}

void PlanningEnv::UpdateCollisionCloud()
{
  collision_cloud_->clear();
  for (int i = 0; i < parameters_.kKeyposeCloudStackNum; i++)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*vertical_surface_cloud_stack_[i], *cloud_tmp);
    *(collision_cloud_) += *cloud_tmp;
  }
  collision_cloud_downsizer_.Downsize(collision_cloud_, parameters_.kCollisionCloudDwzLeafSize,
                                      parameters_.kCollisionCloudDwzLeafSize, parameters_.kCollisionCloudDwzLeafSize);
}

void PlanningEnv::UpdateFrontiers()
{
  if (parameters_.kUseFrontier)
  {
    prev_robot_position_ = robot_position_;
    rolling_occupancy_grid_->GetFrontier(frontier_cloud_->cloud_, robot_position_, parameters_.kExtractFrontierRange);

    if (!frontier_cloud_->cloud_->points.empty())
    {
      if (parameters_.kUseCoverageBoundaryOnFrontier)
      {
        GetCoverageCloudWithinBoundary<pcl::PointXYZI>(frontier_cloud_->cloud_);
      }
      vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
          frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
    }

    // Cluster frontiers
    if (!filtered_frontier_cloud_->cloud_->points.empty())
    {
      kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(parameters_.kFrontierClusterTolerance);
      ec.setMinClusterSize(1);
      ec.setMaxClusterSize(10000);
      ec.setSearchMethod(kdtree_frontier_cloud_);
      ec.setInputCloud(filtered_frontier_cloud_->cloud_);
      ec.extract(cluster_indices);

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      int cluster_count = 0;
      for (int i = 0; i < cluster_indices.size(); i++)
      {
        if (cluster_indices[i].indices.size() < parameters_.kFrontierClusterMinSize)
        {
          continue;
        }
        for (int j = 0; j < cluster_indices[i].indices.size(); j++)
        {
          int point_ind = cluster_indices[i].indices[j];
          filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
          inliers->indices.push_back(point_ind);
        }
        cluster_count++;
      }
      pcl::ExtractIndices<pcl::PointXYZI> extract;
      extract.setInputCloud(filtered_frontier_cloud_->cloud_);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*(filtered_frontier_cloud_->cloud_));
      filtered_frontier_cloud_->Publish();


      misc_utils_ns::Timer Get_Frontier_viewpoints_timer("Get_Frontier_viewpoints");
      Get_Frontier_viewpoints_timer.Start();
      valid_length_of_frontier_point_clouds_vec = frontier_point_clouds_vec.size();
      if(cluster_count<frontier_point_clouds_vec.size())
      {
        valid_length_of_frontier_point_clouds_vec = cluster_count;
        for(int i=0;i<frontier_point_clouds_vec.size();i++)
        {
          frontier_point_clouds_vec[i]->cloud_->points.clear();
        }
      }
      // std::cout<<"111"<<std::endl;
      for(auto point : filtered_frontier_cloud_->cloud_->points)
      {
        if(point.intensity>=frontier_point_clouds_vec.size()||point.intensity<0)
        {
          continue;
        }
        // std::cout<<point.intensity-1<<std::endl;
        frontier_point_clouds_vec[point.intensity]->cloud_->points.push_back(point);
      }
      // std::cout<<"2222"<<std::endl;
      Get_Frontier_viewpoints();
      // Get_Frontier_viewpoints_timer.Stop(true,"ms");
    }
  }
}
void PlanningEnv::Get_Frontier_viewpoints()
{
  //发布出来
  Eigen::Vector4f centroid;
  frontier_center_point->cloud_->points.clear();
  pcl::PointXYZI pt;
  for(int i=0;i<frontier_point_clouds_vec.size();i++)
  {
    if(frontier_point_clouds_vec[i]->cloud_->points.size()>valid_frontier_point_num_thred)
    {
      frontier_point_clouds_vec[i]->cloud_->points.clear();
      frontier_point_clouds_vec[i]->Publish();
      continue;
    }
    frontier_point_clouds_vec[i]->Publish();
    if(i<valid_length_of_frontier_point_clouds_vec)
    {
      centroid.setZero();
      for (size_t h = 0; h < frontier_point_clouds_vec[i]->cloud_->points.size(); ++h)
      {
        centroid += frontier_point_clouds_vec[i]->cloud_->points[h].getVector4fMap();
      }
      centroid /= frontier_point_clouds_vec[i]->cloud_->points.size();
      pt.x = centroid.x();
      pt.y = centroid.y();
      pt.z = centroid.z();
      frontier_center_point->cloud_->points.push_back(pt);
    }
  }
  frontier_center_point->Publish();

  //
}
void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    // ROS_WARN("Terrain cloud empty");
    RCLCPP_WARN(rclcpp::get_logger("my_logger"),"Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    // ROS_WARN("PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    RCLCPP_WARN(rclcpp::get_logger("my_logger"), "PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  pcl::PointXYZI check_point_I;
  check_point_I.x = x;
  check_point_I.y = y;
  check_point_I.z = z;
  check_point_I.intensity = 1;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  // std::cout<<"check_point_I"<<check_point_I.x<<" "<<check_point_I.y<<" "<<check_point_I.z<<std::endl;
  if(pcl_isnan(check_point_I.x)||pcl_isnan(check_point_I.y)||pcl_isnan(check_point_I.z))
  {
    std::cout<<"check_point_I nan"<<std::endl;
    return false;
  }
  if(local_pcl_map_for_ego_flag)
  {
    // std::cout<<"-----radiusSearch-----"<<std::endl;
    local_pcl_map_for_ego_cloud_kdtree_->radiusSearch(check_point_I, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
    // std::cout<<"----------"<<std::endl;
  }
  else{
    stacked_vertical_surface_cloud_kdtree_->radiusSearch(check_point, parameters_.kKeyposeGraphCollisionCheckRadius,
                                                       neighbor_indices, neighbor_sqdist);
  }
  
  
  
  if (neighbor_indices.size() > parameters_.kKeyposeGraphCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}
// void PlanningEnv::Pcl_call_back(const sensor_msgs::msg::PointCloud2::SharedPtr local_pcl_map_for_ego_msg)
// {
//   pcl::fromROSMsg(*local_pcl_map_for_ego_msg, *local_pcl_map_for_ego);
//   pcl::copyPointCloud<pcl::PointXYZI, PlannerCloudPointType>(*local_pcl_map_for_ego, *(local_pcl_map_for_ego_inputkd_cloud->cloud_));
//   if(!local_pcl_map_for_ego_inputkd_cloud->cloud_->points.empty())
//   {
//     local_pcl_map_for_ego_cloud_kdtree_->setInputCloud(local_pcl_map_for_ego_inputkd_cloud->cloud_);
//     local_pcl_map_for_ego_cloud_kdtree_flag=true;
//   }
// }
void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }
  geometry_msgs::msg::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
    }
  }
}
void PlanningEnv::UpdateCoveredArea_uav(const lidar_model_ns::LiDARModel& robot_viewpoint,
                         const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }
  geometry_msgs::msg::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = uav_viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = uav_viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = uav_viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (parameters_.kSurfaceCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * parameters_.kSurfaceCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.b > 0)
    {
      planner_cloud_->cloud_->points[i].b = 255;
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].b = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    // for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_connected_viewpoints)
    // {
    //   if (uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->Visited())
    //   {
    //     if (uav_viewpoint_manager->VisibleByViewPoint_uav<PlannerCloudPointType>(point, viewpoint_ind))
    //     {
    //       planner_cloud_->cloud_->points[i].b = 255;
    //       covered_point_indices.push_back(i);
    //       break;
    //     }
    //   }
    // }
    if(parameters_.invalidate_viewpoint_and_point_in_covered_boxes)
    {
      for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_connected_viewpoints_in_noncovered_area)
      {
        if(!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->Visited())
        {
          continue;
        }
        if (uav_viewpoint_manager->VisibleByViewPoint_uav<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].b = 255;
          covered_point_indices.push_back(i);
          break;
        }
        
      }
    }
    else{
      for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_visited_viewpoints)
      {
      
        if (uav_viewpoint_manager->VisibleByViewPoint_uav<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].b = 255;
          covered_point_indices.push_back(i);
          break;
        }
        
      }
    }
    

  }

  // Dilate the covered area
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / parameters_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].b = 255;
      }
    }
  }

  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.b > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints_uav(cloud_idx, cloud_point_idx);
    }
  }
}

void PlanningEnv::GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }

  // Get uncovered points
  uncovered_cloud_->cloud_->clear();
  uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_point_num++;
    }
  }

  // Check uncovered frontiers
  if (parameters_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true;
          }
        }
      }
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_point_num++;
      }
    }
  }
}
void PlanningEnv::GetUncoveredArea_uav(const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_connected_viewpoints)
  {
    // viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
    uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->ResetCoveredPointList();
    uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->ResetCoveredFrontierPointList();

  }

  // Get uncovered points
  uav_uncovered_cloud_->cloud_->clear();
  uav_uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.b > 0)
    {
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_connected_viewpoints)
    {
      if (!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->Visited()&&!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->In_covered_box())
      {
        if (uav_viewpoint_manager->VisibleByViewPoint_uav<PlannerCloudPointType>(point, viewpoint_ind))
        {
          uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->AddCoveredPoint(uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uav_uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_point_num++;
    }
  }//闪现1
  // std::cout<<uncovered_point_num<<"   "<<std::endl;

  // Check uncovered frontiers
  if (parameters_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      for (const auto& viewpoint_ind : uav_viewpoint_manager->ind_of_connected_viewpoints)
      {
        if (!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->Visited()&&!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->In_covered_box())
        {
          if (uav_viewpoint_manager->VisibleByViewPoint_uav<pcl::PointXYZI>(point, viewpoint_ind))
          {
            uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->AddCoveredFrontierPoint(uncovered_frontier_point_num);
            observed = true;
          }
        }
      }
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uav_uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_point_num++;
      }
    }
  }
}
void PlanningEnv::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  pointcloud_manager_->GetVisualizationPointCloud(vis_cloud);
}

void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
  uav_uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
  uav_uncovered_frontier_cloud_->Publish();
}

}  // namespace planning_env_ns