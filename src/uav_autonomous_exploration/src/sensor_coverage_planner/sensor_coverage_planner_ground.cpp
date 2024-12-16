/**
 * @file sensor_coverage_planner_ground.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 * 
 * 
 * @modified_by Huazhang Zhu (22210720332@m.fudan.edu.cn)
 * @date 2023-11-13
 */

#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"
#include "graph/graph.h"

namespace uav_exploration_ros_ns
{
bool PlannerParameters::ReadParameters(std::shared_ptr<rclcpp::Node>& nh)
{
  sub_start_exploration_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_start_exploration_topic_", "/exploration_start");
  sub_state_estimation_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_state_estimation_topic_", "/uav_state_estimation");
  sub_uav_3d_lidar_msg_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_uav_3d_lidar_msg_topic_", "/uav_3d_lidar_msg");
  sub_terrain_map_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_topic_", "/terrain_map");
  sub_terrain_map_ext_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_terrain_map_ext_topic_", "/terrain_map_ext");
  sub_coverage_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_coverage_boundary_topic_", "/coverage_boundary");
  sub_viewpoint_boundary_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_viewpoint_boundary_topic_", "/navigation_boundary");
  sub_nogo_boundary_topic_ = misc_utils_ns::getParam<std::string>(nh, "sub_nogo_boundary_topic_", "/nogo_boundary");
  pub_exploration_finish_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_exploration_finish_topic_", "exploration_finish");
  pub_runtime_breakdown_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_runtime_breakdown_topic_", "runtime_breakdown");
  pub_runtime_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_runtime_topic_", "/runtime");
  pub_waypoint_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_waypoint_topic_", "/way_point");
  pub_momentum_activation_count_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_momentum_activation_count_topic_", "momentum_activation_count");

  // Bool
  kAutoStart = misc_utils_ns::getParam<bool>(nh, "kAutoStart", false);
  kRushHome = misc_utils_ns::getParam<bool>(nh, "kRushHome", false);
  kUseTerrainHeight = misc_utils_ns::getParam<bool>(nh, "kUseTerrainHeight", true);
  kCheckTerrainCollision = misc_utils_ns::getParam<bool>(nh, "kCheckTerrainCollision", true);
  kExtendWayPoint = misc_utils_ns::getParam<bool>(nh, "kExtendWayPoint", true);
  kUseLineOfSightLookAheadPoint = misc_utils_ns::getParam<bool>(nh, "kUseLineOfSightLookAheadPoint", true);
  kNoExplorationReturnHome = misc_utils_ns::getParam<bool>(nh, "kNoExplorationReturnHome", true);
  kUseMomentum = misc_utils_ns::getParam<bool>(nh, "kUseMomentum", false);
  kextendz = misc_utils_ns::getParam<bool>(nh, "kextendz", false);
  use_time_interval = misc_utils_ns::getParam<bool>(nh, "use_time_interval", true);
  way_point_in_free = misc_utils_ns::getParam<bool>(nh, "way_point_in_free", true);
  // Double
  kKeyposeCloudDwzFilterLeafSize = misc_utils_ns::getParam<double>(nh, "kKeyposeCloudDwzFilterLeafSize", 0.2);
  kRushHomeDist = misc_utils_ns::getParam<double>(nh, "kRushHomeDist", 10.0);
  kAtHomeDistThreshold = misc_utils_ns::getParam<double>(nh, "kAtHomeDistThreshold", 0.5);
  kTerrainCollisionThreshold = misc_utils_ns::getParam<double>(nh, "kTerrainCollisionThreshold", 0.5);
  kLookAheadDistance = misc_utils_ns::getParam<double>(nh, "kLookAheadDistance", 5.0);
  kExtendWayPointDistanceBig = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceBig", 8.0);
  kExtendWayPointDistanceBig_store = kExtendWayPointDistanceBig;
  kExtendWayPointDistanceSmall = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceSmall", 3.0);

  // Int
  kDirectionChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionChangeCounterThr", 4);
  kDirectionNoChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionNoChangeCounterThr", 5);

  return true;
}

void PlannerData::Initialize(std::shared_ptr<rclcpp::Node>& nh,std::shared_ptr<rclcpp::Node>& nh_p)
{
  keypose_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", kWorldFrameID);
  uav_3d_lidar_msg_stack_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(nh, "uav_3d_lidar_msg_stack", kWorldFrameID);
  registered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "registered_cloud", kWorldFrameID);
  large_terrain_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud_large", kWorldFrameID);
  terrain_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_collision_cloud", kWorldFrameID);
  terrain_ext_collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_ext_collision_cloud", kWorldFrameID);
  viewpoint_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "viewpoint_vis_cloud", kWorldFrameID);
  viewpoint_visited_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "viewpoint_visited_cloud", kWorldFrameID);
  grid_world_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "grid_world_vis_cloud", kWorldFrameID);
  exploration_path_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "bspline_path_cloud", kWorldFrameID);

  selected_viewpoint_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "selected_viewpoint_vis_cloud", kWorldFrameID);
  exploring_cell_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "exploring_cell_vis_cloud", kWorldFrameID);
  collision_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "collision_cloud", kWorldFrameID);
  lookahead_point_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "lookahead_point_cloud", kWorldFrameID);
  keypose_graph_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "keypose_graph_cloud", kWorldFrameID);
  uav_keypose_graph_vis_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uav_keypose_graph_cloud", kWorldFrameID);
  viewpoint_in_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "viewpoint_in_collision_cloud_", kWorldFrameID);
  point_cloud_manager_neighbor_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "pointcloud_manager_cloud", kWorldFrameID);
  reordered_global_subspace_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "reordered_global_subspace_cloud", kWorldFrameID);
  
  planning_env_ = std::make_unique<planning_env_ns::PlanningEnv>(nh, nh_p);

  uav_viewpoint_manager_ =  std::make_shared<uav_viewpoint_manager_ns::ViewPointManager>(nh);

  local_coverage_planner_ = std::make_unique<local_coverage_planner_ns::LocalCoveragePlanner>(nh);

  local_coverage_planner_->SetUavViewPointManager(uav_viewpoint_manager_);
  keypose_graph_ = std::make_unique<keypose_graph_ns::KeyposeGraph>(nh);
  // uav_keypose_graph_ = std::make_unique<uav_keypose_graph_ns::KeyposeGraph>(nh);
  grid_world_ = std::make_unique<grid_world_ns::GridWorld>(nh);
    
  grid_world_->SetUseKeyposeGraph(true);
  visualizer_ = std::make_unique<tare_visualizer_ns::TAREVisualizer>(nh, nh_p);

  initial_position_.x() = 0.0;
  initial_position_.y() = 0.0;
  initial_position_.z() = 0.0;

  cur_keypose_node_ind_ = 0;

  keypose_graph_node_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_node_marker", kWorldFrameID);
  keypose_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  uav_keypose_graph_node_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "uav_keypose_graph_node_marker", kWorldFrameID);
  uav_keypose_graph_node_marker_->SetType(visualization_msgs::msg::Marker::POINTS);
  uav_keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
  uav_keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);
  keypose_graph_edge_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_edge_marker", kWorldFrameID);
  keypose_graph_edge_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);
  uav_keypose_graph_edge_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "uav_keypose_graph_edge_marker", kWorldFrameID);
  uav_keypose_graph_edge_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  uav_keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
  uav_keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);
  //connected_viewpoint_marker_test
  connected_viewpoint_marker_test = std::make_unique<misc_utils_ns::Marker>(nh, "connected_viewpoint_marker_test", kWorldFrameID);//可视化
  connected_viewpoint_marker_test->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  connected_viewpoint_marker_test->SetScale(0.05, 0.0, 0.0);
  connected_viewpoint_marker_test->SetColorRGBA(1.0, 1.0, 0.0, 0.9);


  nogo_boundary_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "nogo_boundary_marker", kWorldFrameID);
  nogo_boundary_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
  nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  grid_world_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "grid_world_marker", kWorldFrameID);
  grid_world_marker_->SetType(visualization_msgs::msg::Marker::CUBE_LIST);
  grid_world_marker_->SetScale(1.0, 1.0, 1.0);
  grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

  robot_yaw_ = 0.0;
  lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
  moving_forward_ = true;


  Eigen::Vector3d uav_viewpoint_resolution = uav_viewpoint_manager_->GetResolution();

  double uav_add_non_keypose_node_min_dist = std::min(std::min(uav_viewpoint_resolution.x(), uav_viewpoint_resolution.y()), uav_viewpoint_resolution.z()) / 2;

  keypose_graph_->SetAddNonKeyposeNodeMinDist() = uav_add_non_keypose_node_min_dist;

  // uav_keypose_graph_->SetAddNonKeyposeNodeMinDist() = uav_add_non_keypose_node_min_dist;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;

  last_robot_position_ = robot_position_;
}

uav_exploration_ros::uav_exploration_ros(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle)
  : keypose_cloud_update_(false)
  , initialized_(false)
  , lookahead_point_update_(false)
  , relocation_(false)
  , relocation_count_(0)
  , start_exploration_(false)
  , exploration_finished_(false)
  , near_home_(false)
  , at_home_(false)
  , stopped_(false)
  , test_point_update_(false)
  , viewpoint_ind_update_(false)
  , step_(false)
  , use_momentum_(false)
  , lookahead_point_in_line_of_sight_(true)
  , registered_cloud_count_(0)
  , ament_occupancy_count_(0)
  , keypose_count_(0)
  , direction_change_count_(0)
  , direction_no_change_count_(0)
  , momentum_activation_count_(0)
  , pub_flag(true)
  , show_time(false)
  , cout_no_pub(0)
  , publish_done(true)
  , env_update_done(false)
  , pub_mode_(0)
  , use_frontier_viewpoint(false)
  , invalidate_viewpoint_and_point_in_covered_boxes(false)
  , finish_triger(false)
  , smaller_viewpoint_wait_time_relocation(2)
  
{
  std::cout<<"局部框size请设置较小值，否则在frontier_viewpoint模块kdtree_occupy_unknow_会占用较大的资源"<<std::endl;
  last_waypoint.pose.position.x = -0.234988;
  last_waypoint.pose.position.y = -0.234111;
  last_waypoint.pose.position.z = -20.234111;
  last_pub_time = _clock.now();
  initialize(node_handle, private_node_handle);
  //M?上面均无问题
  PrintExplorationStatus("Exploration Started", false);
}

void uav_exploration_ros::Parameter_Declare(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle)
{

//   //M? 这里请注意，ROS2只有私有变量，我都默认参数在ros1是读取私有参数的了，在ros2中用node_handle直接读取 ；后面如果有特殊情况用要回头看这里
//Sensor Coverage Planner Ground
  node_handle->declare_parameter<std::string>("sub_start_exploration_topic_","/start_exploration");
  node_handle->declare_parameter<std::string>("sub_terrain_map_topic_","/terrain_map");
  node_handle->declare_parameter<std::string>("sub_terrain_map_ext_topic_","/terrain_map_ext");
  node_handle->declare_parameter<std::string>("sub_state_estimation_topic_","/uav_state_estimation");
  node_handle->declare_parameter<std::string>("sub_uav_3d_lidar_msg_topic_","/uav_3d_lidar_msg");
  node_handle->declare_parameter<std::string>("sub_coverage_boundary_topic_","/sensor_coverage_planner/coverage_boundary");
  node_handle->declare_parameter<std::string>("sub_viewpoint_boundary_topic_","/navigation_boundary");
  node_handle->declare_parameter<std::string>("sub_nogo_boundary_topic_","/sensor_coverage_planner/nogo_boundary");
  node_handle->declare_parameter<std::string>("pub_exploration_finish_topic_","exploration_finish");
  node_handle->declare_parameter<std::string>("pub_runtime_breakdown_topic_","runtime_breakdown");
  node_handle->declare_parameter<std::string>("pub_runtime_topic_","/runtime");
  node_handle->declare_parameter<std::string>("pub_waypoint_topic_","/way_point");
  node_handle->declare_parameter<std::string>("pub_momentum_activation_count_topic_","momentum_activation_count");
  
  //bool
  node_handle->declare_parameter<bool>("kAutoStart",true);
  node_handle->declare_parameter<bool>("kRushHome",true);
  node_handle->declare_parameter<bool>("kUseTerrainHeight",true);
  node_handle->declare_parameter<bool>("kCheckTerrainCollision",true);
  node_handle->declare_parameter<bool>("kExtendWayPoint",true);
  node_handle->declare_parameter<bool>("kUseLineOfSightLookAheadPoint",true);
  node_handle->declare_parameter<bool>("kNoExplorationReturnHome",true);
  node_handle->declare_parameter<bool>("kUseMomentum",true);
  node_handle->declare_parameter<bool>("kextendz",false);
  node_handle->declare_parameter<bool>("use_time_interval",true);
  node_handle->declare_parameter<bool>("way_point_in_free",true);
  
  
  //double
  node_handle->declare_parameter<double>("kExtendWayPointDistanceBig",8.0);
  node_handle->declare_parameter<double>("kExtendWayPointDistanceSmall",4.5);
  node_handle->declare_parameter<double>("kKeyposeCloudDwzFilterLeafSize",0.2);
  node_handle->declare_parameter<double>("kRushHomeDist",10);
  node_handle->declare_parameter<double>("kAtHomeDistThreshold",0.5);
  node_handle->declare_parameter<double>("kTerrainCollisionThreshold",0.5);
  node_handle->declare_parameter<double>("kLookAheadDistance",8);

  //int
  node_handle->declare_parameter<int>("kDirectionChangeCounterThr",6);
  node_handle->declare_parameter<int>("kDirectionNoChangeCounterThr",5);
  node_handle->declare_parameter<int>("pub_mode_",0);
  node_handle->declare_parameter<int>("smaller_viewpoint_wait_time_relocation",2);
  
  // PlanningEnv
  node_handle->declare_parameter<bool>("kUseFrontier",true);
  node_handle->declare_parameter<bool>("kUseCoverageBoundaryOnFrontier",false);
  node_handle->declare_parameter<bool>("kUseCoverageBoundaryOnObjectSurface",false);
  node_handle->declare_parameter<double>("kFrontierClusterTolerance",1.0);
  node_handle->declare_parameter<int>("kFrontierClusterMinSize",30);
  node_handle->declare_parameter<int>("valid_frontier_point_num_thred",100);
  //Rolling occupancy grid
  node_handle->declare_parameter<double>("rolling_occupancy_grid/resolution_x",0.3);
  node_handle->declare_parameter<double>("rolling_occupancy_grid/resolution_y",0.3);
  node_handle->declare_parameter<double>("rolling_occupancy_grid/resolution_z",0.3);

  node_handle->declare_parameter<double>("kSurfaceCloudDwzLeafSize",0.5);
  node_handle->declare_parameter<double>("kCollisionCloudDwzLeafSize",0.2);  
  node_handle->declare_parameter<int>("kKeyposeCloudStackNum",5);
  node_handle->declare_parameter<int>("kPointCloudRowNum",50);
  node_handle->declare_parameter<int>("kPointCloudColNum",50);
  node_handle->declare_parameter<int>("kPointCloudLevelNum",30);
  node_handle->declare_parameter<int>("kMaxCellPointNum",100000);
  node_handle->declare_parameter<double>("kPointCloudCellSize",18);  
  node_handle->declare_parameter<double>("kPointCloudCellHeight",1.8);  
  node_handle->declare_parameter<int>("kPointCloudManagerNeighborCellNum",5);
  node_handle->declare_parameter<double>("kCoverCloudZSqueezeRatio",2.0);  

  //KeyposeGraph
  node_handle->declare_parameter<double>("keypose_graph/kAddNodeMinDist",1.0);  
  node_handle->declare_parameter<double>("keypose_graph/kAddNonKeyposeNodeMinDist",0.5);  
  node_handle->declare_parameter<double>("keypose_graph/kAddEdgeConnectDistThr",3.0);  
  node_handle->declare_parameter<int>("keypose_graph/add_edge_Thr",5);  
  node_handle->declare_parameter<double>("keypose_graph/kAddEdgeToLastKeyposeDistThr",3.0);  
  node_handle->declare_parameter<double>("keypose_graph/kAddEdgeVerticalThreshold",1.0);  
  node_handle->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckRadius",0.4);  
  node_handle->declare_parameter<double>("keypose_graph/kAddEdgeCollisionCheckResolution",0.4);  
  node_handle->declare_parameter<int>("keypose_graph/kAddEdgeCollisionCheckPointNumThr",1);
  node_handle->declare_parameter<int>("keypose_graph/ckeck_loger_than_length",4);
  node_handle->declare_parameter<int>("keypose_graph/eliminate_edge_non_free_thr",10);
  
  //ViewPointManager
  node_handle->declare_parameter<bool>("viewpoint_manager/first_frame_grace",false);
  node_handle->declare_parameter<bool>("viewpoint_manager/disable_outsider_viewpoints",false);
  node_handle->declare_parameter<bool>("viewpoint_manager/smaller_visited_radius",true);
  node_handle->declare_parameter<int>("check_collision_margin",5);
  node_handle->declare_parameter<double>("intervel_radius",1.9);
  node_handle->declare_parameter<double>("para_visited_radius",2.4);
  node_handle->declare_parameter<double>("para_visited_radius_z",2.4);
  node_handle->declare_parameter<double>("position_add_high_for_guv",0.0);
  node_handle->declare_parameter<int>("least_visible_unknown_num",29);
  node_handle->declare_parameter<int>("least_visible_free_num",29);
  node_handle->declare_parameter<bool>("Set_robot_neigbour_connected",false);
  node_handle->declare_parameter<double>("in_collision_viewpoint_get_connected_viewpoint_dis",1.1);
  node_handle->declare_parameter<double>("search_radius_thr",0.9);
  
  
  node_handle->declare_parameter<double>("visitd_radius",3);
  node_handle->declare_parameter<double>("visitd_radius_z",1.2);
  node_handle->declare_parameter<double>("viewpoint_manager/number_x",40);  
  node_handle->declare_parameter<double>("viewpoint_manager/number_y",40);  
  node_handle->declare_parameter<double>("viewpoint_manager/number_z",1);  
  node_handle->declare_parameter<double>("viewpoint_manager/resolution_x",1.2);
  node_handle->declare_parameter<double>("viewpoint_manager/resolution_y",1.2);
  node_handle->declare_parameter<double>("viewpoint_manager/resolution_z",0.0);
  node_handle->declare_parameter<int>("viewpoint_manager/big_box_size_x",5);
  node_handle->declare_parameter<int>("viewpoint_manager/big_box_size_y",5);
  node_handle->declare_parameter<int>("viewpoint_manager/big_box_size_z",3);
  node_handle->declare_parameter<int>("viewpoint_manager/min_box_size_x",8);
  node_handle->declare_parameter<int>("viewpoint_manager/min_box_size_y",8);
  node_handle->declare_parameter<int>("viewpoint_manager/min_box_size_z",3);
  node_handle->declare_parameter<int>("viewpoint_manager/incollision_barrier_level",0);

  node_handle->declare_parameter<int>("kGreedyViewPointSampleRange",3);
  node_handle->declare_parameter<int>("kLocalPathOptimizationItrMax",10);
  node_handle->declare_parameter<double>("kViewPointCollisionMargin",0.7); 
  node_handle->declare_parameter<double>("kViewPointCollisionMarginZPlus",0.5); 
  node_handle->declare_parameter<double>("kViewPointCollisionMarginZMinus",0.5); 
  node_handle->declare_parameter<double>("kCollisionGridZScale",1.0); 
  node_handle->declare_parameter<double>("kCollisionGridResolutionX",0.2); 
  node_handle->declare_parameter<double>("kCollisionGridResolutionY",0.2); 
  node_handle->declare_parameter<double>("kCollisionGridResolutionZ",0.0); 
  // node_handle->declare_parameter<double>("",); 
  node_handle->declare_parameter<int>("kCollisionPointThr",1);
  node_handle->declare_parameter<bool>("kLineOfSightStopAtNearestObstacle",true);
  node_handle->declare_parameter<double>("kViewPointHeightFromTerrain",0.75);  
  node_handle->declare_parameter<double>("kViewPointHeightFromTerrainChangeThreshold",0.6);  
  node_handle->declare_parameter<double>("kSensorRange",12.0);  
  node_handle->declare_parameter<double>("kNeighborRange",3.0);  
  node_handle->declare_parameter<double>("kCoverageOcclusionThr",0.3);  
  node_handle->declare_parameter<double>("kCoverageDilationRadius",1.0);  
  node_handle->declare_parameter<int>("kCollisionFrameCountMax",3);
  node_handle->declare_parameter<bool>("kCheckDynamicObstacleCollision",true);

  //Grid World
  node_handle->declare_parameter<int>("kGridWorldXNum",121);
  node_handle->declare_parameter<int>("kGridWorldYNum",121);
  node_handle->declare_parameter<int>("kGridWorldZNum",121);

  node_handle->declare_parameter<int>("kGridWorldNearbyGridNum",5);
  node_handle->declare_parameter<int>("kMinAddPointNumSmall",40);
  node_handle->declare_parameter<int>("kMinAddPointNumBig",60);
  node_handle->declare_parameter<int>("kMinAddFrontierPointNum",15);
  node_handle->declare_parameter<int>("kCellExploringToCoveredThr",1);
  node_handle->declare_parameter<int>("kCellCoveredToExploringThr",10);
  node_handle->declare_parameter<int>("kCellExploringToAlmostCoveredThr",10);
  node_handle->declare_parameter<int>("kCellAlmostCoveredToExploringThr",20);
  node_handle->declare_parameter<int>("kCellUnknownToExploringThr",1);
  node_handle->declare_parameter<bool>("Estime_any_Frontier_viewpoint_asexploring",false);
  

  //Visualization
  node_handle->declare_parameter<bool>("kExploringSubspaceMarkerColorGradientAlpha",true);
  node_handle->declare_parameter<double>("kExploringSubspaceMarkerColorMaxAlpha",0.8);  
  node_handle->declare_parameter<double>("kExploringSubspaceMarkerColorR",0.0);  
  node_handle->declare_parameter<double>("kExploringSubspaceMarkerColorG",1.0);  
  node_handle->declare_parameter<double>("kExploringSubspaceMarkerColorB",0.0);  
  node_handle->declare_parameter<double>("kExploringSubspaceMarkerColorA",1.0);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonMarkerColorR",0.0);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonMarkerColorG",1.0);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonMarkerColorB",0.0);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonMarkerColorA",1.0);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonMarkerWidth",0.3);  
  node_handle->declare_parameter<double>("kLocalPlanningHorizonHeight",3.0);  

  node_handle->declare_parameter<bool>("show_time",false);  
  node_handle->get_parameter("show_time",show_time);
  node_handle->get_parameter("pub_mode_",pub_mode_);
  node_handle->get_parameter("smaller_viewpoint_wait_time_relocation",smaller_viewpoint_wait_time_relocation);
  std::cout<<"smaller_viewpoint_wait_time_relocation:"<<smaller_viewpoint_wait_time_relocation<<std::endl;
  pub_mode_store = pub_mode_;
  std::cout<<"pub_mode_"<<pub_mode_<<std::endl;
  node_handle->declare_parameter<double>("excute_triger_time",200);  
  node_handle->get_parameter("excute_triger_time",excute_triger_time);
  node_handle->declare_parameter<double>("waypointpub_triger_time",300);  
  node_handle->get_parameter("waypointpub_triger_time",waypointpub_triger_time);
  node_handle->declare_parameter<bool>("low_speed_run",false);  
  node_handle->get_parameter("low_speed_run",low_speed_run);
  node_handle->declare_parameter<bool>("use_frontier_viewpoint",false);  
  node_handle->get_parameter("use_frontier_viewpoint",use_frontier_viewpoint);
  node_handle->declare_parameter<int>("least_explore_time_s",179);  
  node_handle->get_parameter("least_explore_time_s",least_explore_time_s);

  node_handle->declare_parameter<bool>("invalidate_viewpoint_and_point_in_covered_boxes",false);  
  node_handle->get_parameter("invalidate_viewpoint_and_point_in_covered_boxes",invalidate_viewpoint_and_point_in_covered_boxes);

  std::cout<<"use_frontier_viewpoint:"<<use_frontier_viewpoint<<std::endl;
  std::cout<<"waypointpub_triger_time:"<<waypointpub_triger_time<<std::endl;
  std::cout<<"excute_triger_time:"<<excute_triger_time<<std::endl;
  std::cout<<"least_explore_time_s:"<<least_explore_time_s<<std::endl;
  std::cout<<"invalidate_viewpoint_and_point_in_covered_boxes:"<<invalidate_viewpoint_and_point_in_covered_boxes<<std::endl;
}


bool uav_exploration_ros::initialize(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle)
{
  Parameter_Declare(node_handle, private_node_handle);
  if (!pp_.ReadParameters(node_handle))
  {
    RCLCPP_ERROR(node_handle->get_logger(), "Read parameters failed");
    return false;
  }

  pd_.Initialize(node_handle, private_node_handle);
  pd_.uav_viewpoint_manager_->use_frontier_viewpoint = use_frontier_viewpoint;
  pd_.local_coverage_planner_->use_frontier_viewpoint = pd_.uav_viewpoint_manager_->use_frontier_viewpoint;
  pd_.planning_env_->Setpointcloud_manager_invalidate_viewpoint_and_point_in_covered_boxes(invalidate_viewpoint_and_point_in_covered_boxes);
  pd_.keypose_graph_->SetAllowVerticalEdge(true);
  // pd_.uav_keypose_graph_->SetAllowVerticalEdge(true);
  lidar_model_ns::LiDARModel::setCloudDWZResol(pd_.planning_env_->GetPlannerCloudResolution());

  // execution_timer_ = node_handle.createTimer(ros::Duration(1.0), &uav_exploration_ros::execute, this);
  // execution_timer_ = node_handle->create_timer(std::chrono::seconds(1), std::bind(&uav_exploration_ros::execute, this));
  // execution_timer_ = node_handle->create_wall_timer(std::chrono::seconds(1), execute);
  
  execution_timer_ = node_handle->create_wall_timer(std::chrono::milliseconds(static_cast<long long>(excute_triger_time)), std::bind(&uav_exploration_ros::execute, this));
  
  execution_timer_waypoint_pub = node_handle->create_wall_timer(std::chrono::milliseconds(static_cast<long long>(waypointpub_triger_time)), std::bind(&uav_exploration_ros::waypointpub, this));

  Pub_switch = 
      node_handle->create_subscription<std_msgs::msg::Bool>("/Pub_switch", 5, std::bind(&uav_exploration_ros::Pub_Switch_, this, std::placeholders::_1));             
             
  // exploration_start_sub_ =
  //     nh.subscribe(pp_.sub_start_exploration_topic_, 5, &uav_exploration_ros::ExplorationStartCallback, this);
  exploration_start_sub_ =
      node_handle->create_subscription<std_msgs::msg::Bool>(pp_.sub_start_exploration_topic_, 5, std::bind(&uav_exploration_ros::ExplorationStartCallback, this, std::placeholders::_1));  
  // uav_3d_lidar_msg_sub_ =
  //     nh.subscribe(pp_.sub_uav_3d_lidar_msg_topic_, 5, &uav_exploration_ros::RegisteredScanCallback, this);
  uav_3d_lidar_msg_sub_ = 
      node_handle->create_subscription<sensor_msgs::msg::PointCloud2>(pp_.sub_uav_3d_lidar_msg_topic_, 5, std::bind(&uav_exploration_ros::RegisteredScanCallback, this, std::placeholders::_1));



  // terrain_map_sub_ = nh.subscribe(pp_.sub_terrain_map_topic_, 5, &uav_exploration_ros::TerrainMapCallback, this);
  terrain_map_sub_ = node_handle->create_subscription<sensor_msgs::msg::PointCloud2>(pp_.sub_terrain_map_topic_, 5, std::bind(&uav_exploration_ros::TerrainMapCallback, this, std::placeholders::_1));

  terrain_map_ext_sub_ =
      // nh.subscribe(pp_.sub_terrain_map_ext_topic_, 5, &uav_exploration_ros::TerrainMapExtCallback, this);
      node_handle->create_subscription<sensor_msgs::msg::PointCloud2>(pp_.sub_terrain_map_ext_topic_, 5, std::bind(&uav_exploration_ros::TerrainMapExtCallback, this, std::placeholders::_1));

  state_estimation_sub_ =
      // nh.subscribe(pp_.sub_state_estimation_topic_, 5, &uav_exploration_ros::StateEstimationCallback, this);
      node_handle->create_subscription<nav_msgs::msg::Odometry>(pp_.sub_state_estimation_topic_, 5, std::bind(&uav_exploration_ros::StateEstimationCallback, this, std::placeholders::_1));

  coverage_boundary_sub_ =
      // nh.subscribe(pp_.sub_coverage_boundary_topic_, 1, &uav_exploration_ros::CoverageBoundaryCallback, this);
      node_handle->create_subscription<geometry_msgs::msg::PolygonStamped>(pp_.sub_coverage_boundary_topic_, 5, std::bind(&uav_exploration_ros::CoverageBoundaryCallback, this, std::placeholders::_1));

  viewpoint_boundary_sub_ =
      // nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1, &uav_exploration_ros::ViewPointBoundaryCallback, this);
      node_handle->create_subscription<geometry_msgs::msg::PolygonStamped>(pp_.sub_viewpoint_boundary_topic_, 5, std::bind(&uav_exploration_ros::ViewPointBoundaryCallback, this, std::placeholders::_1));

  nogo_boundary_sub_ =
      // nh.subscribe(pp_.sub_nogo_boundary_topic_, 1, &uav_exploration_ros::NogoBoundaryCallback, this);
      node_handle->create_subscription<geometry_msgs::msg::PolygonStamped>(pp_.sub_nogo_boundary_topic_, 5, std::bind(&uav_exploration_ros::NogoBoundaryCallback, this, std::placeholders::_1));


  global_path_full_publisher_ = node_handle->create_publisher<nav_msgs::msg::Path>("global_path_full", 1);
  // = nh.advertise<nav_msgs::msg::Path>("global_path_full", 1);

  global_path_publisher_ = node_handle->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  // = nh.advertise<nav_msgs::msg::Path>("global_path", 1);
  // old_global_path_publisher_  = node_handle->create_publisher<nav_msgs::msg::Path>("old_global_path", 1);
  // = nh.advertise<nav_msgs::msg::Path>("old_global_path", 1);
  // to_nearest_global_subspace_path_publisher_ 
  // = nh.advertise<nav_msgs::msg::Path>("to_nearest_global_subspace_path", 1);
  local_tsp_path_publisher_ = node_handle->create_publisher<nav_msgs::msg::Path>("local_path", 1);
  // = nh.advertise<nav_msgs::msg::Path>("local_path", 1);
  // exploration_path_publisher_ = nh.advertise<nav_msgs::msg::Path>("exploration_path", 1);
  waypoint_pub_ = node_handle->create_publisher<geometry_msgs::msg::PointStamped>(pp_.pub_waypoint_topic_, 2);
  // = nh.advertise<geometry_msgs::msg::PointStamped>(pp_.pub_waypoint_topic_, 2);
  exploration_finish_pub_ = node_handle->create_publisher<std_msgs::msg::Bool>(pp_.pub_exploration_finish_topic_, 2);
  // = nh.advertise<std_msgs::msg::Bool>(pp_.pub_exploration_finish_topic_, 2);
  runtime_breakdown_pub_ = node_handle->create_publisher<std_msgs::msg::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
  // = nh.advertise<std_msgs::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
  runtime_pub_ = node_handle->create_publisher<std_msgs::msg::Float32>(pp_.pub_runtime_topic_, 2);
  // = nh.advertise<std_msgs::Float32>(pp_.pub_runtime_topic_, 2);
  mission_over_pub_ = node_handle->create_publisher<std_msgs::msg::Float32>("/mission_over", 2);

  momentum_activation_count_pub_ = node_handle->create_publisher<std_msgs::msg::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  // = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
  // Debug
  pointcloud_manager_neighbor_cells_origin_pub_ = node_handle->create_publisher<geometry_msgs::msg::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);
      // nh.advertise<geometry_msgs::msg::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);

  return true;
}
void uav_exploration_ros::Pub_Switch_(const std_msgs::msg::Bool::SharedPtr   start_msg)
{
  pub_flag=start_msg->data;
  std::cout  <<"Pub_Switch_ "<<pub_flag<<std::endl;
}


void uav_exploration_ros::ExplorationStartCallback(const std_msgs::msg::Bool::SharedPtr   start_msg)
{
  if (start_msg->data)
  {
    start_exploration_ = true;
  }
}

void uav_exploration_ros::StateEstimationCallback(const nav_msgs::msg::Odometry::SharedPtr state_estimation_msg)
{
  // if(!start_signal_bool)
  // {
  //   return;
  // }
  pd_.robot_position_ = state_estimation_msg->pose.pose.position;
  // Todo: use a boolean
  if (std::abs(pd_.initial_position_.x()) < 0.01 && std::abs(pd_.initial_position_.y()) < 0.01 &&
      std::abs(pd_.initial_position_.z()) < 0.01)
  {
    pd_.initial_position_.x() = pd_.robot_position_.x;
    pd_.initial_position_.y() = pd_.robot_position_.y;
    pd_.initial_position_.z() = pd_.robot_position_.z;
  }
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);

  pd_.robot_yaw_ = yaw;

  if (state_estimation_msg->twist.twist.linear.x > 0.4)
  {
    pd_.moving_forward_ = true;
  }
  else if (state_estimation_msg->twist.twist.linear.x < -0.4)
  {
    pd_.moving_forward_ = false;
  }
  initialized_ = true;
}

void uav_exploration_ros::RegisteredScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr uav_3d_lidar_msg_msg)
{
  // if(!start_signal_bool)
  // {
  //   return;
  // }
  if (!initialized_)
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr uav_3d_lidar_msg_tmp(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*uav_3d_lidar_msg_msg, *uav_3d_lidar_msg_tmp);
  if (uav_3d_lidar_msg_tmp->points.empty())
  {
    return;
  }
  *(pd_.uav_3d_lidar_msg_stack_->cloud_) += *(uav_3d_lidar_msg_tmp);
  pointcloud_downsizer_.Downsize(uav_3d_lidar_msg_tmp, pp_.kKeyposeCloudDwzFilterLeafSize,
                                 pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);
  pd_.registered_cloud_->cloud_->clear();
  pcl::copyPointCloud(*uav_3d_lidar_msg_tmp, *(pd_.registered_cloud_->cloud_));

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(pd_.registered_cloud_->cloud_);

  registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
  UpdateVisitedPositions();
  if (registered_cloud_count_ == 0)
  {
    // initialized_ = true;
    pd_.planning_env_->seperate_point_from_occupancy_array_();
    pd_.keypose_.pose.pose.position = pd_.robot_position_;
    pd_.keypose_.pose.covariance[0] = keypose_count_++;
    //TODO uav_keypose_graph_ ： 后期如果使用uav_keypose_graph_这里是要把下面的也替换的
    misc_utils_ns::Timer AddKeyposeNode_timer("AddKeyposeNode");
    AddKeyposeNode_timer.Start();
    // pd_.keypose_graph_->Check_near_station_connect(pd_.robot_position_,pd_.keypose_, *(pd_.planning_env_),pd_.planning_env_->Get_rolling_occupancy_grid_(),pd_.uav_viewpoint_manager_);
    pd_.cur_keypose_node_ind_ = pd_.keypose_graph_->AddKeyposeNode(pd_.keypose_, *(pd_.planning_env_),pd_.planning_env_->Get_rolling_occupancy_grid_(),pd_.uav_viewpoint_manager_, pd_.visited_positions_last_40_time);
    // pd_.cur_uav_keypose_node_ind_ = pd_.uav_keypose_graph_->AddKeyposeNode(pd_.keypose_, *(pd_.planning_env_));
    AddKeyposeNode_timer.Stop(false);
    global_relocation_graph_maintainance_runtime_ = AddKeyposeNode_timer.GetDuration();

    
    pointcloud_downsizer_.Downsize(pd_.uav_3d_lidar_msg_stack_->cloud_, pp_.kKeyposeCloudDwzFilterLeafSize,
                                   pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);

    pd_.keypose_cloud_->cloud_->clear();
    pcl::copyPointCloud(*(pd_.uav_3d_lidar_msg_stack_->cloud_), *(pd_.keypose_cloud_->cloud_));
    // pd_.keypose_cloud_->Publish();
    pd_.uav_3d_lidar_msg_stack_->cloud_->clear();
    keypose_cloud_update_ = true;
    //这里修一次occupancy，补齐漏洞
    // ament_occupancy_count_ = (ament_occupancy_count_ + 1) % 5;
    // if (ament_occupancy_count_ == 0)
    // {
      // pd_.planning_env_->Get_rolling_occupancy_grid_()->Amend_occupancy_array_();
    // }
    
    
  }
}

void uav_exploration_ros::TerrainMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_map_msg)
{
  // if(!start_signal_bool)
  // {
  //   return;
  // }
  if (pp_.kCheckTerrainCollision)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
    pd_.terrain_collision_cloud_->cloud_->clear();
    for (auto& point : terrain_map_tmp->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void uav_exploration_ros::TerrainMapExtCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_map_ext_msg)
{
  // if(!start_signal_bool)
  // {
  //   return;
  // }
  if (pp_.kUseTerrainHeight)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));
  }
  if (pp_.kCheckTerrainCollision)
  {
    pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));
    pd_.terrain_ext_collision_cloud_->cloud_->clear();
    for (auto& point : pd_.large_terrain_cloud_->cloud_->points)
    {
      if (point.intensity > pp_.kTerrainCollisionThreshold)
      {
        pd_.terrain_ext_collision_cloud_->cloud_->points.push_back(point);
      }
    }
  }
}

void uav_exploration_ros::CoverageBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg)
{
  pd_.planning_env_->UpdateCoverageBoundary((*polygon_msg).polygon);
}

void uav_exploration_ros::ViewPointBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg)
{

  pd_.uav_viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
}

void uav_exploration_ros::NogoBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg)
{
  if (polygon_msg->polygon.points.empty())
  {
    return;
  }
  double polygon_id = polygon_msg->polygon.points[0].z;
  int polygon_point_size = polygon_msg->polygon.points.size();
  std::vector<geometry_msgs::msg::Polygon> nogo_boundary;
  geometry_msgs::msg::Polygon polygon;
  for (int i = 0; i < polygon_point_size; i++)
  {
    if (polygon_msg->polygon.points[i].z == polygon_id)
    {
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
    else
    {
      nogo_boundary.push_back(polygon);
      polygon.points.clear();
      polygon_id = polygon_msg->polygon.points[i].z;
      polygon.points.push_back(polygon_msg->polygon.points[i]);
    }
  }
  nogo_boundary.push_back(polygon);

  pd_.uav_viewpoint_manager_->UpdateNogoBoundary(nogo_boundary);
  geometry_msgs::msg::Point point;
  for (int i = 0; i < nogo_boundary.size(); i++)
  {
    for (int j = 0; j < nogo_boundary[i].points.size() - 1; j++)
    {
      point.x = nogo_boundary[i].points[j].x;
      point.y = nogo_boundary[i].points[j].y;
      point.z = nogo_boundary[i].points[j].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
      point.x = nogo_boundary[i].points[j + 1].x;
      point.y = nogo_boundary[i].points[j + 1].y;
      point.z = nogo_boundary[i].points[j + 1].z;
      pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    }
    point.x = nogo_boundary[i].points.back().x;
    point.y = nogo_boundary[i].points.back().y;
    point.z = nogo_boundary[i].points.back().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
    point.x = nogo_boundary[i].points.front().x;
    point.y = nogo_boundary[i].points.front().y;
    point.z = nogo_boundary[i].points.front().z;
    pd_.nogo_boundary_marker_->marker_.points.push_back(point);
  }
  pd_.nogo_boundary_marker_->Publish();
}

void uav_exploration_ros::SendInitialWaypoint()
{
  // send waypoint ahead
  double lx = 12.0;
  double ly = 0.0;
  double dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
  double dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;

  geometry_msgs::msg::PoseStamped waypoint;
  waypoint.header.frame_id = "map";
  rclcpp::Clock clock;
  waypoint.header.stamp = clock.now();
  waypoint.pose.position.x = pd_.robot_position_.x + dx;
  waypoint.pose.position.y = pd_.robot_position_.y + dy;
  waypoint.pose.position.z = pd_.robot_position_.z;

  geometry_msgs::msg::PointStamped waypoint1;
  waypoint1.header=waypoint.header;
  waypoint1.point = waypoint.pose.position;
  waypoint_pub_->publish(waypoint1);
  // waypoint_pub_->publish(waypoint);
}

void uav_exploration_ros::UpdateKeyposeGraph()
{
  misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
  update_keypose_graph_timer.Start();

  pd_.keypose_graph_->GetMarker(pd_.keypose_graph_node_marker_->marker_, pd_.keypose_graph_edge_marker_->marker_);
  // pd_.keypose_graph_node_marker_->Publish();
  pd_.keypose_graph_edge_marker_->Publish();
  pd_.keypose_graph_vis_cloud_->cloud_->clear();
  pd_.keypose_graph_->CheckLocalCollision(pd_.robot_position_, pd_.uav_viewpoint_manager_,pd_.planning_env_);
  pd_.keypose_graph_->CheckConnectivity(pd_.robot_position_);
  pd_.keypose_graph_->GetVisualizationCloud(pd_.keypose_graph_vis_cloud_->cloud_);
  pd_.keypose_graph_vis_cloud_->Publish();

//uav_keypose_graph_
  update_keypose_graph_timer.Stop(show_time);
  global_relocation_graph_maintainance_runtime_ += update_keypose_graph_timer.GetDuration();
  // std::cout<<"UpdateKeyposeGraph："<<update_keypose_graph_timer.GetDuration("ms")/1000<<std::endl;
}



void uav_exploration_ros::Diplomatic_acition_Viewpoint_and_Occupancy_array_(std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager_, const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_)
{
  //TODO 这里的occupy_array_   怎么把这里的指针弄成不能修改occupy_array_这个对象
  uav_viewpoint_manager_->visualize_all_viewpoints_();//可视化
  // 这里要确保下面的初始化了
  if(rolling_occupancy_grid_->return_initialized_())
  {
    // std::cout<<"这里"<<std::endl;
    uav_viewpoint_manager_->Abstract_free_viewpoints(rolling_occupancy_grid_);
    uav_viewpoint_manager_->visualize_free_viewpoints_();//可视化
    uav_viewpoint_manager_->Abstract_connected_viewpoints(rolling_occupancy_grid_);
    uav_viewpoint_manager_->visualize_connected_viewpoints_();
    uav_viewpoint_manager_->visualize_in_collision_viewpoints_();//可视化


    pd_.connected_viewpoint_marker_test->marker_.points = uav_viewpoint_manager_->Return_connected_viewpoint_marker_().points;//可视化
    pd_.connected_viewpoint_marker_test->Publish();//可视化
  }
  
  // uav_viewpoint_manager_->test_();//可视化
}
int uav_exploration_ros::Update_uav_ViewPoints_with_rolling_occupancy_grid_()
{
  misc_utils_ns::Timer local_graph_maintainance_timer("local graph maintainance");
  
  misc_utils_ns::Timer viewpoint_manager_update_timer("update common viewpoint");
  viewpoint_manager_update_timer.Start();
  // if (pp_.kUseTerrainHeight)
  // {
  //   pd_.viewpoint_manager_->SetViewPointHeightWithTerrain(pd_.large_terrain_cloud_->cloud_);
  // }
  // if (pp_.kCheckTerrainCollision)
  // {
  //   *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_collision_cloud_->cloud_);
  //   *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_ext_collision_cloud_->cloud_);
  // }
  // pd_.viewpoint_manager_->CheckViewPointCollision(pd_.collision_cloud_->cloud_);
  
  Diplomatic_acition_Viewpoint_and_Occupancy_array_(pd_.uav_viewpoint_manager_,pd_.planning_env_->Get_rolling_occupancy_grid_());
  //提取了空闲空间中的viewpoint
  //从机器人所在位置开始 确保viewpoint之间相互连接
  


  // pd_.robot_position_

  //先进行障碍判断

  //TODO 从这里开始进行viewpoint的一系列判断操作

  // pd_.uav_viewpoint_manager_->CheckViewPointLineOfSight();
  // int viewpoint_candidate_count = pd_.viewpoint_manager_->GetViewPointCandidate();
  UpdateVisitedPositions();
  pd_.uav_viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_,pd_.grid_world_);
  pd_.uav_viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_);
  viewpoint_manager_update_timer.Stop(show_time);
  local_viewpoint_update_runtime_ = viewpoint_manager_update_timer.GetDuration();

  local_graph_maintainance_timer.Start();
  pd_.uav_viewpoint_manager_->UpdateViewPointGraph(false);//维护viewpoint dist graph position
  local_graph_maintainance_timer.Stop(false);
  local_graph_maintainance_runtime_ = local_graph_maintainance_timer.GetDuration();

  


  if(use_frontier_viewpoint)
  {
    misc_utils_ns::Timer frontier_viewpoint_update_timer("update frontier viewpoint");
    frontier_viewpoint_update_timer.Start();
    pd_.uav_viewpoint_manager_->Get_frontier_viewpoint<PlannerCloudPointType>(pd_.planning_env_,pd_.visited_positions_,pd_.keypose_graph_);
    frontier_viewpoint_update_timer.Stop(show_time);
    local_frontier_viewpoint_update_and_sampling_runtime_ = frontier_viewpoint_update_timer.GetDuration();
    local_graph_maintainance_timer.Start();
    pd_.uav_viewpoint_manager_->UpdateViewPointGraph(true);//true是在原本 普通 viewpoint的基础上 加上frontier viewpoint到graph_上
    local_graph_maintainance_timer.Stop(false);
    local_graph_maintainance_runtime_ += local_graph_maintainance_timer.GetDuration();
  }

  if(show_time)
  std::cout<<"local graph maintainance(ms): "<<local_graph_maintainance_runtime_<<std::endl;

  return pd_.uav_viewpoint_manager_->ind_of_connected_viewpoints.size(); 
}


void uav_exploration_ros::UpdateViewPointCoverage()
{

  // Update viewpoint coverage

  pd_.uav_viewpoint_manager_->Updateuav_ViewPointCoverage<PlannerCloudPointType>(pd_.planning_env_->GetDiffCloud(),invalidate_viewpoint_and_point_in_covered_boxes);
  pd_.uav_viewpoint_manager_->UpdateRolledOveruav_ViewPointCoverage<PlannerCloudPointType>(
      pd_.planning_env_->GetStackedCloud(),invalidate_viewpoint_and_point_in_covered_boxes);
  // Update robot coverage
  pd_.robot_viewpoint_.ResetCoverage();
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position = pd_.robot_position_;
  pd_.robot_viewpoint_.setPose(robot_pose);
  UpdateRobotViewPointCoverage();
  //TODO 这里是robot_viewpoint_的操作我没做，还没搞清楚它的一个作用，它这里是用了5次墙面点做壁垒，  即使后面换成加上天花板点云不知道是不是也有必要这里再变

}

void uav_exploration_ros::UpdateRobotViewPointCoverage()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pd_.planning_env_->GetCollisionCloud();
  for (const auto& point : cloud->points)
  {
    if (pd_.uav_viewpoint_manager_->InFOVAndRange(
            Eigen::Vector3d(point.x, point.y, point.z),
            Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z)))
    {
      pd_.robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
    }
  }
}

void uav_exploration_ros::UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num,int& uav_uncovered_point_num, int& uav_uncovered_frontier_point_num)
{
  // Update covered area

  pd_.planning_env_->UpdateCoveredArea_uav(pd_.robot_viewpoint_, pd_.uav_viewpoint_manager_);

  
  //TODO 这里后面需要换成
  pd_.planning_env_->GetUncoveredArea_uav(pd_.uav_viewpoint_manager_, uav_uncovered_point_num, uav_uncovered_frontier_point_num);
  
  pd_.planning_env_->PublishUncoveredCloud();
  pd_.planning_env_->PublishUncoveredFrontierCloud();
}

void uav_exploration_ros::UpdateVisitedPositions()
{
  Eigen::Vector3d robot_current_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  bool existing = false;
  for (int i = 0; i < pd_.visited_positions_.size(); i++)
  {
    
    if ((robot_current_position - pd_.visited_positions_[i]).norm() < 1)
    {
      existing = true;
      break;
    }
  }
  if (!existing)
  {
    pd_.visited_positions_.push_back(robot_current_position);
  }
  pd_.visited_positions_last_40_time.push_back(robot_current_position);
  if(pd_.visited_positions_last_40_time.size()>40)
  {
    pd_.visited_positions_last_40_time.erase(pd_.visited_positions_last_40_time.begin());
  }
}

void uav_exploration_ros::UpdateGlobalRepresentation()
{
  misc_utils_ns::Timer update_representation_timer("update representation");
  update_representation_timer.Start();
  pd_.local_coverage_planner_->SetRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));

  bool viewpoint_rollover_test = pd_.uav_viewpoint_manager_->UpdateRobotPosition(
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z));
  // pd_.uav_viewpoint_manager_->visualize_all_viewpoints_();//可视化
  if (!pd_.grid_world_->Initialized() || viewpoint_rollover_test)
  {
    pd_.grid_world_->UpdateNeighborCells(pd_.robot_position_);
  }

  pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
  pd_.planning_env_->GetVisualizationPointCloud(pd_.point_cloud_manager_neighbor_cloud_->cloud_);
  pd_.point_cloud_manager_neighbor_cloud_->Publish();

  // DEBUG
  Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
      pd_.planning_env_->GetPointCloudManagerNeighborCellsOrigin();
  geometry_msgs::msg::PointStamped pointcloud_manager_neighbor_cells_origin_point;
  pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
  rclcpp::Clock clock;
  pointcloud_manager_neighbor_cells_origin_point.header.stamp = clock.now();
  pointcloud_manager_neighbor_cells_origin_point.point.x = pointcloud_manager_neighbor_cells_origin.x();
  pointcloud_manager_neighbor_cells_origin_point.point.y = pointcloud_manager_neighbor_cells_origin.y();
  pointcloud_manager_neighbor_cells_origin_point.point.z = pointcloud_manager_neighbor_cells_origin.z();
  pointcloud_manager_neighbor_cells_origin_pub_->publish(pointcloud_manager_neighbor_cells_origin_point);

  if (exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    pd_.planning_env_->SetUseFrontier(false);
  }
  pd_.planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(pd_.keypose_cloud_->cloud_);

  int closest_node_ind = pd_.keypose_graph_->GetClosestNodeInd(pd_.robot_position_);
  // int uav_closest_node_ind = pd_.uav_keypose_graph_->GetClosestNodeInd(pd_.robot_position_);
  geometry_msgs::msg::Point closest_node_position = pd_.keypose_graph_->GetClosestNodePosition(pd_.robot_position_);
  // geometry_msgs::msg::Point uav_closest_node_position = pd_.uav_keypose_graph_->GetClosestNodePosition(pd_.robot_position_);
  //TODO uav_keypose_graph_ ： 这里涉及到了应用，后期如果使用uav_keypose_graph_这里是要把下面的也替换的
  pd_.grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind);
  // pd_.grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position);

  pd_.grid_world_->UpdateRobotPosition(pd_.robot_position_);
  if (!pd_.grid_world_->HomeSet())
  {
    pd_.grid_world_->SetHomePosition(pd_.initial_position_);
  }
  update_representation_timer.Stop(show_time);
  update_representation_runtime_ = update_representation_timer.GetDuration();

}

void uav_exploration_ros::GlobalPlanning(std::vector<int>& global_cell_covered_marker_vec, std::vector<int>& global_cell_tsp_order,
                                             exploration_path_ns::ExplorationPath& global_path)
{
  misc_utils_ns::Timer global_tsp_timer("Global planning");
  global_tsp_timer.Start();
  
  pd_.grid_world_->UpdateCellStatus(pd_.planning_env_, pd_.keypose_graph_, pd_.uav_viewpoint_manager_, global_cell_covered_marker_vec);
  
  pd_.grid_world_->UpdateCellKeyposeGraphNodes(pd_.keypose_graph_);
  
  pd_.grid_world_->AddPathsInBetweenCells(pd_.uav_viewpoint_manager_, pd_.keypose_graph_);
  
  //TODO uav_keypose_graph_ ： 这里涉及到了uav_keypose_graph_的应用，后面在确定使用uav_keypose_graph_记得补上
  // pd_.grid_world_->uav_UpdateCellKeyposeGraphNodes(pd_.uav_keypose_graph_);
  // pd_.grid_world_->uav_AddPathsInBetweenCells(pd_.viewpoint_manager_, pd_.uav_keypose_graph_);

  // pd_.viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_);
  pd_.uav_viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_);
  //TODO uav_keypose_graph_ ： 这里涉及到了uav_keypose_graph_的应用，后面在确定使用uav_keypose_graph_记得补上
  global_path = pd_.grid_world_->SolveGlobalTSP(pd_.uav_viewpoint_manager_,global_cell_covered_marker_vec, global_cell_tsp_order, pd_.keypose_graph_);

  global_tsp_timer.Stop(show_time,"ms");
  global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
  
}

void uav_exploration_ros::PublishGlobalPlanningVisualization(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  nav_msgs::msg::Path global_path_full = global_path.GetPath();
  global_path_full.header.frame_id = "map";
  // rclcpp::Clock clock;
  global_path_full.header.stamp = clock1.now();
  global_path_full_publisher_->publish(global_path_full);
  // Get the part that connects with the local path

  int start_index = 0;
  for (int i = 0; i < global_path.nodes_.size(); i++)
  {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !pd_.uav_viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    start_index = i;
  }

  int end_index = global_path.nodes_.size() - 1;
  for (int i = global_path.nodes_.size() - 1; i >= 0; i--)
  {
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
        global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
        !pd_.uav_viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
    {
      break;
    }
    end_index = i;
  }
  
 
  nav_msgs::msg::Path global_path_trim;
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::msg::PoseStamped first_pose;
    first_pose.pose.position.x = local_path.nodes_.front().position_.x();
    first_pose.pose.position.y = local_path.nodes_.front().position_.y();
    first_pose.pose.position.z = local_path.nodes_.front().position_.z();
    global_path_trim.poses.push_back(first_pose);
  }

  for (int i = start_index; i <= end_index; i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = global_path.nodes_[i].position_.x();
    pose.pose.position.y = global_path.nodes_[i].position_.y();
    pose.pose.position.z = global_path.nodes_[i].position_.z();
    global_path_trim.poses.push_back(pose);
  }
  if (local_path.nodes_.size() >= 2)
  {
    geometry_msgs::msg::PoseStamped last_pose;
    last_pose.pose.position.x = local_path.nodes_.back().position_.x();
    last_pose.pose.position.y = local_path.nodes_.back().position_.y();
    last_pose.pose.position.z = local_path.nodes_.back().position_.z();
    global_path_trim.poses.push_back(last_pose);
  }
  global_path_trim.header.frame_id = "map";
  // rclcpp::Clock clock;
  global_path_trim.header.stamp = clock1.now();
  global_path_publisher_->publish(global_path_trim);

  pd_.grid_world_->GetVisualizationCloud(pd_.grid_world_vis_cloud_->cloud_);
  pd_.grid_world_vis_cloud_->Publish();
  pd_.grid_world_->GetMarker(pd_.grid_world_marker_->marker_);
  pd_.grid_world_marker_->Publish();
  nav_msgs::msg::Path full_path = pd_.exploration_path_.GetPath();
  full_path.header.frame_id = "map";
  full_path.header.stamp = clock1.now();
  // exploration_path_publisher_.publish(full_path);
  pd_.exploration_path_.GetVisualizationCloud(pd_.exploration_path_cloud_->cloud_);
  pd_.exploration_path_cloud_->Publish();
  // pd_.planning_env_->PublishStackedCloud();
}

void uav_exploration_ros::LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                                            const exploration_path_ns::ExplorationPath& global_path,
                                            exploration_path_ns::ExplorationPath& local_path)
{
  misc_utils_ns::Timer local_tsp_timer("Local planning");
  local_tsp_timer.Start();
  if (lookahead_point_update_)
  {
    pd_.local_coverage_planner_->SetLookAheadPoint(pd_.lookahead_point_);
  }
  
  local_path = pd_.local_coverage_planner_->SolveLocalCoverageProblem(global_path, uncovered_point_num,
                                                                      uncovered_frontier_point_num);
  local_tsp_timer.Stop(show_time,"ms");
}


void uav_exploration_ros::PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path)
{
  
  pd_.lookahead_point_cloud_->Publish();
  nav_msgs::msg::Path local_tsp_path = local_path.GetPath();
  local_tsp_path.header.frame_id = "map";
    // rclcpp::Clock clock;

  local_tsp_path.header.stamp = clock1.now();
  local_tsp_path_publisher_->publish(local_tsp_path);
  pd_.local_coverage_planner_->GetSelectedViewPointVisCloud(pd_.selected_viewpoint_vis_cloud_->cloud_);
  pd_.selected_viewpoint_vis_cloud_->Publish();

  // Visualize local planning horizon box
}

exploration_path_ns::ExplorationPath uav_exploration_ros::ConcatenateGlobalLocalPath(
    const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path)
{
  exploration_path_ns::ExplorationPath full_path;
  if (exploration_finished_ && near_home_ && pp_.kRushHome)
  {
    exploration_path_ns::Node node;
    node.position_.x() = pd_.robot_position_.x;
    node.position_.y() = pd_.robot_position_.y;
    node.position_.z() = pd_.robot_position_.z;
    node.type_ = exploration_path_ns::NodeType::ROBOT;
    full_path.nodes_.push_back(node);
    node.position_ = pd_.initial_position_;
    node.type_ = exploration_path_ns::NodeType::HOME;
    full_path.nodes_.push_back(node);
    return full_path;
  }

  double global_path_length = global_path.GetLength();
  double local_path_length = local_path.GetLength();
  if (global_path_length < 3 && local_path_length < 5)
  {
    return full_path;
  }
  else
  {
    full_path = local_path;
    if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
        local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START)
    {
      full_path.Reverse();
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
    }
    else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
             local_path.nodes_.back() == local_path.nodes_.front())
    {
      full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
    }
  }

  return full_path;
}

bool uav_exploration_ros::GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                                                const exploration_path_ns::ExplorationPath& global_path,
                                                Eigen::Vector3d& lookahead_point)
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);

  // Determine which direction to follow on the global path
  double dist_from_start = 0.0;
  for (int i = 1; i < global_path.nodes_.size(); i++)
  {
    dist_from_start += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      break;
    }
  }

  double dist_from_end = 0.0;
  for (int i = global_path.nodes_.size() - 2; i > 0; i--)
  {
    dist_from_end += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
    if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
    {
      break;
    }
  }

  bool local_path_too_short = true;
  for (int i = 0; i < local_path.nodes_.size(); i++)
  {
    double dist_to_robot = (robot_position - local_path.nodes_[i].position_).norm();
    if (dist_to_robot > pp_.kLookAheadDistance / 5)
    {
      local_path_too_short = false;
      break;
    }
  }
  if (local_path.GetNodeNum() < 1 || local_path_too_short)
  {
    if (dist_from_start < dist_from_end)
    {
      double dist_from_robot = 0.0;
      for (int i = 1; i < global_path.nodes_.size(); i++)
      {
        dist_from_robot += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > pp_.kLookAheadDistance / 2)
        {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    else
    {
      double dist_from_robot = 0.0;
      for (int i = global_path.nodes_.size() - 2; i > 0; i--)
      {
        dist_from_robot += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
        if (dist_from_robot > pp_.kLookAheadDistance / 2)
        {
          lookahead_point = global_path.nodes_[i].position_;
          break;
        }
      }
    }
    return false;
  }

  bool has_lookahead = false;
  bool dir = true;
  int robot_i = 0;
  int lookahead_i = 0;
  for (int i = 0; i < local_path.nodes_.size(); i++)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT)
    {
      robot_i = i;
    }
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT)
    {
      has_lookahead = true;
      lookahead_i = i;
    }
  }

  int forward_viewpoint_count = 0;
  int backward_viewpoint_count = 0;

  bool local_loop = false;
  if (local_path.nodes_.front() == local_path.nodes_.back() &&
      local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT)
  {
    local_loop = true;
  }

  if (local_loop)
  {
    robot_i = 0;
  }
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      forward_viewpoint_count++;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  for (int i = robot_i - 1; i >= 0; i--)
  {
    if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
    {
      backward_viewpoint_count++;
    }
  }

  Eigen::Vector3d forward_lookahead_point = robot_position;
  Eigen::Vector3d backward_lookahead_point = robot_position;

  bool has_forward = false;
  bool has_backward = false;

  if (local_loop)
  {
    robot_i = 0;
  }
  bool forward_lookahead_point_in_los = true;
  bool backward_lookahead_point_in_los = true;
  double length_from_robot = 0.0;
  for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i < local_path.GetNodeNum() - 1)
    {
      //TODO 计算是否在视野里消耗太大，用connected代替 这个值（后面用RayTraceHelper解决）
      in_line_of_sight = pd_.uav_viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i + 1].position_,pd_.planning_env_->Get_rolling_occupancy_grid_());
    }
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
         i == local_path.GetNodeNum() - 1))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        forward_lookahead_point_in_los = false;
      }
      forward_lookahead_point = local_path.nodes_[i].position_;
      has_forward = true;
      break;
    }
  }
  if (local_loop)
  {
    robot_i = local_path.nodes_.size() - 1;
  }
  length_from_robot = 0.0;
  for (int i = robot_i - 1; i >= 0; i--)
  {
    length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();
    double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
    bool in_line_of_sight = true;
    if (i > 0)
    {
      //TODO 计算是否在视野里消耗太大，用connected代替 这个值
      in_line_of_sight = pd_.uav_viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_,pd_.planning_env_->Get_rolling_occupancy_grid_());
    }
    if ((length_from_robot > pp_.kLookAheadDistance || (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
         local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))

    {
      if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
      {
        backward_lookahead_point_in_los = false;
      }
      backward_lookahead_point = local_path.nodes_[i].position_;
      has_backward = true;
      break;
    }
  }

  if (forward_viewpoint_count > 0 && !has_forward)
  {
    std::cout << "forward viewpoint count > 0 but does not have forward lookahead point" << std::endl;
    exit(1);
  }
  if (backward_viewpoint_count > 0 && !has_backward)
  {
    std::cout << "backward viewpoint count > 0 but does not have backward lookahead point" << std::endl;
    exit(1);
  }

  double dx = pd_.lookahead_point_direction_.x();
  double dy = pd_.lookahead_point_direction_.y();

  // double lx = 1.0;
  // double ly = 0.0;
  // double dx = 1.0;
  // double dy = 0.0;
  // if (pd_.moving_forward_)
  // {
  //   lx = 1.0;
  // }
  // else
  // {
  //   lx = -1.0;
  // }

  // dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
  // dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;

  // double dx = pd_.moving_direction_.x();
  // double dy = pd_.moving_direction_.y();

  double forward_angle_score = -2;
  double backward_angle_score = -2;
  double lookahead_angle_score = -2;

  double dist_robot_to_lookahead = 0.0;
  if (has_forward)
  {
    Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
    forward_diff.z() = 0.0;
    forward_diff = forward_diff.normalized();
    forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y();
  }
  if (has_backward)
  {
    Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
    backward_diff.z() = 0.0;
    backward_diff = backward_diff.normalized();
    backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y();
  }
  if (has_lookahead)
  {
    Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
    dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
    Eigen::Vector3d diff = prev_lookahead_point - robot_position;
    diff.z() = 0.0;
    diff = diff.normalized();
    lookahead_angle_score = dx * diff.x() + dy * diff.y();
  }

  pd_.lookahead_point_cloud_->cloud_->clear();

  if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0)
  {
    relocation_ = true;
    relocation_count_++;
    pub_mode_ = 1;
    if(relocation_count_>smaller_viewpoint_wait_time_relocation)//这里是不要一下子缩紧太快，这样对于那些relocation到的位置，第一时间没有选取到selected，就麻烦了
    {
      pd_.uav_viewpoint_manager_->Get_disable_outsider_viewpoints() = true;//relocation的时候Viewpint缩紧的意义在于让机器人 在relocation和局部路径规划中反复横跳（针对隔墙的relocation和局部探索那种）
      pd_.uav_viewpoint_manager_->Get_relocation_() = true;//这两这是绑定在一起的，因为当Viewpont缩紧的时候，找全局规划边界点也应该减小
    }
  }
  else
  {
    pub_mode_ = pub_mode_store;
    relocation_ = false;
    relocation_count_ = 0;
    pd_.uav_viewpoint_manager_->Get_disable_outsider_viewpoints() = false;
    pd_.uav_viewpoint_manager_->Get_relocation_() = false;
  }
  if (relocation_)
  {
    if (use_momentum_ && pp_.kUseMomentum)
    {
      if (forward_angle_score > backward_angle_score)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
      }
    }
    else
    {
      // follow the shorter distance one
      if (dist_from_start < dist_from_end && local_path.nodes_.front().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = backward_lookahead_point;
      }
      else if (dist_from_end < dist_from_start &&
               local_path.nodes_.back().type_ != exploration_path_ns::NodeType::ROBOT)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point =
            forward_angle_score > backward_angle_score ? forward_lookahead_point : backward_lookahead_point;
      }
    }
  }
  else if (has_lookahead && lookahead_angle_score > 0 && dist_robot_to_lookahead > pp_.kLookAheadDistance / 2 &&
           pd_.uav_viewpoint_manager_->InLocalPlanningHorizon(local_path.nodes_[lookahead_i].position_))

  {
    lookahead_point = local_path.nodes_[lookahead_i].position_;
  }
  else
  {
    if (forward_angle_score > backward_angle_score)
    {
      if (forward_viewpoint_count > 0)
      {
        lookahead_point = forward_lookahead_point;
      }
      else
      {
        lookahead_point = backward_lookahead_point;
      }
    }
    else
    {
      if (backward_viewpoint_count > 0)
      {
        lookahead_point = backward_lookahead_point;
      }
      else
      {
        lookahead_point = forward_lookahead_point;
      }
    }
  }

  if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
      (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los))
  {
    lookahead_point_in_line_of_sight_ = false;
  }
  else
  {
    lookahead_point_in_line_of_sight_ = true;
  }

  pd_.lookahead_point_direction_ = lookahead_point - robot_position;
  pd_.lookahead_point_direction_.z() = 0.0;
  pd_.lookahead_point_direction_.normalize();

  pcl::PointXYZI point;
  point.x = lookahead_point.x();
  point.y = lookahead_point.y();
  point.z = lookahead_point.z();
  point.intensity = 1.0;
  pd_.lookahead_point_cloud_->cloud_->points.push_back(point);

  if (has_lookahead)
  {
    point.x = local_path.nodes_[lookahead_i].position_.x();
    point.y = local_path.nodes_[lookahead_i].position_.y();
    point.z = local_path.nodes_[lookahead_i].position_.z();
    point.intensity = 0;
    pd_.lookahead_point_cloud_->cloud_->points.push_back(point);
  }
  return true;
}

void uav_exploration_ros::PublishWaypoint()
{
  if(pp_.use_time_interval)
  {
    if((_clock.now()-last_pub_time).seconds()<1.0)//防止更新过快，无人机撞墙
    {
      publish_done = true;
      return;
    }
    else{last_pub_time = _clock.now();}
  }
    
  
  if(!pub_flag)
  {
    publish_done = true;
    return;
  }
  geometry_msgs::msg::PoseStamped waypoint;
  if (exploration_finished_ && near_home_ && pp_.kRushHome)
  {
    waypoint.pose.position.x = pd_.initial_position_.x();
    waypoint.pose.position.y = pd_.initial_position_.y();
    waypoint.pose.position.z = pd_.initial_position_.z();
  }
  else
  {
    double dx = pd_.lookahead_point_.x() - pd_.robot_position_.x;
    double dy = pd_.lookahead_point_.y() - pd_.robot_position_.y;
    double dz = pd_.lookahead_point_.z() - pd_.robot_position_.z;
    double extend_way_point_x,extend_way_point_y,extend_way_point_z;
    double r = sqrt(dx * dx + dy * dy + dz * dz);
    double extend_dist =
        lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;
    if (r < extend_dist && pp_.kExtendWayPoint)
    {
      dx = dx / r * extend_dist;
      dy = dy / r * extend_dist;
      dz = dz / r * extend_dist;
    }

    waypoint.pose.position.x = dx + pd_.robot_position_.x;
    waypoint.pose.position.y = dy + pd_.robot_position_.y;
    if(pp_.kextendz)
    {
      waypoint.pose.position.z = dz + pd_.robot_position_.z;
    }
    else{
      waypoint.pose.position.z = pd_.lookahead_point_.z();
    }


    while (pp_.way_point_in_free)
    {
      double dx = pd_.lookahead_point_.x() - pd_.robot_position_.x;
      double dy = pd_.lookahead_point_.y() - pd_.robot_position_.y;
      double dz = pd_.lookahead_point_.z() - pd_.robot_position_.z;
      double extend_way_point_x,extend_way_point_y,extend_way_point_z;
      double r = sqrt(dx * dx + dy * dy + dz * dz);
      // double extend_dist =
      //     lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;
      if (r < extend_dist && pp_.kExtendWayPoint)
      {
        dx = dx / r * extend_dist;
        dy = dy / r * extend_dist;
        dz = dz / r * extend_dist;
      }

      waypoint.pose.position.x = dx + pd_.robot_position_.x;
      waypoint.pose.position.y = dy + pd_.robot_position_.y;
      if(pp_.kextendz)
      {
        waypoint.pose.position.z = dz + pd_.robot_position_.z;
      }
      else{
        waypoint.pose.position.z = pd_.lookahead_point_.z();
      }
      Eigen::Vector3d tem_3d;
      tem_3d.x() = waypoint.pose.position.x;
      tem_3d.y() = waypoint.pose.position.y;
      tem_3d.z() = waypoint.pose.position.z;
      Eigen::Vector3d tem_3d_robot_position;
      tem_3d_robot_position.x() = pd_.robot_position_.x;
      tem_3d_robot_position.y() = pd_.robot_position_.y;
      tem_3d_robot_position.z() = pd_.robot_position_.z;
      Eigen::Vector3i sub = pd_.uav_viewpoint_manager_->Hierarchy_Pos_2_Hierarchy_sub(tem_3d);

      if(pd_.uav_viewpoint_manager_->InRange_Hier_sub(sub))
      {
        // if(pd_.uav_viewpoint_manager_->Get_littlebox_from_Hierarchy_ind(pd_.uav_viewpoint_manager_->Hierarchy_sub_2_Hierarchy_ind(sub))->Connected())
        if(pd_.planning_env_->Get_rolling_occupancy_grid_()->Visible_in_RayTraceHelper_for_viewpoint_connected(tem_3d,tem_3d_robot_position,false,0))
        {
          break;
        }
      }
      extend_dist = extend_dist-0.8;
      if(extend_dist<r)
      {
        waypoint.pose.position.x = pd_.lookahead_point_.x();
        waypoint.pose.position.y = pd_.lookahead_point_.y();
        waypoint.pose.position.z = pd_.lookahead_point_.z();
        break;
      }
    }
    
      

  }
  if(low_speed_run)
  {
    geometry_msgs::msg::PointStamped waypoint1;
    waypoint1.header=waypoint.header;
    waypoint1.point = waypoint.pose.position;
    misc_utils_ns::Publish<geometry_msgs::msg::PointStamped>(waypoint_pub_, waypoint1, kWorldFrameID);
    // misc_utils_ns::Publish<geometry_msgs::msg::PoseStamped>(waypoint_pub_, waypoint, kWorldFrameID);
    publish_done = true;
  }
  // if(last_waypoint.pose.position.x == waypoint.pose.position.x && last_waypoint.pose.position.y == waypoint.pose.position.y && last_waypoint.pose.position.z == waypoint.pose.position.z)
  // {
  //   cout_no_pub++;
    
  //   if(cout_no_pub>5)
  //   {

  //     publish_done = true;
  //     std::this_thread::sleep_for(std::chrono::milliseconds(300));
  //     // std::cout<<"publish_done"<<publish_done<<std::endl;
  //     // if(the_onebefore_last_waypoint.size()>0)
  //     // {
  //     //   std::cout<<"连续10帧没发布，后退一下"<<std::endl;
  //     //   cout_no_pub=0;
  //     //   geometry_msgs::msg::PointStamped waypoint1;
  //     //   waypoint1.header=the_onebefore_last_waypoint[the_onebefore_last_waypoint.size()-1].header;
  //     //   waypoint1.point = the_onebefore_last_waypoint[the_onebefore_last_waypoint.size()-1].pose.position;
  //     geometry_msgs::msg::PointStamped waypoint1;
  //     waypoint1.header=waypoint.header;
  //     waypoint1.point = waypoint.pose.position;
  //     misc_utils_ns::Publish<geometry_msgs::msg::PointStamped>(waypoint_pub_, waypoint1, kWorldFrameID);


  //     //   // misc_utils_ns::Publish<geometry_msgs::msg::PoseStamped>(waypoint_pub_, the_onebefore_last_waypoint[the_onebefore_last_waypoint.size()-1], kWorldFrameID);
  //     //   the_onebefore_last_waypoint.erase(the_onebefore_last_waypoint.begin()+the_onebefore_last_waypoint.size()-1);
  //     // }
  //   }
  //   return;
  // }
  // cout_no_pub=0;

  geometry_msgs::msg::PointStamped waypoint1;
  waypoint1.header=waypoint.header;
  waypoint1.point = waypoint.pose.position;

  misc_utils_ns::Publish<geometry_msgs::msg::PointStamped>(waypoint_pub_, waypoint1, kWorldFrameID);
  // misc_utils_ns::Publish<geometry_msgs::msg::PoseStamped>(waypoint_pub_, waypoint, kWorldFrameID);
  publish_done = true;
  if(pub_mode_==1)
  {
    env_update_done=false;
  }
  if(the_onebefore_last_waypoint.size()>=5)
  {
    the_onebefore_last_waypoint.erase(the_onebefore_last_waypoint.begin());
  }
  the_onebefore_last_waypoint.push_back(waypoint);
  last_waypoint=waypoint;
}

void uav_exploration_ros::PublishRuntime()
{
  local_viewpoint_sampling_runtime_ = pd_.local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
  local_path_finding_runtime_ =
      (pd_.local_coverage_planner_->GetFindPathRuntime() + pd_.local_coverage_planner_->GetTSPRuntime()) / 1000;

  std_msgs::msg::Int32MultiArray runtime_breakdown_msg;
  runtime_breakdown_msg.data.clear();
  runtime_breakdown_msg.data.push_back(update_representation_runtime_);
  runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
  runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
  runtime_breakdown_msg.data.push_back(global_planning_runtime_);
  runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
  runtime_breakdown_msg.data.push_back(overall_runtime_);
  runtime_breakdown_pub_->publish(runtime_breakdown_msg);

  float runtime = 0;
  if (!exploration_finished_ && pp_.kNoExplorationReturnHome)
  {
    for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++)
    {
      runtime += runtime_breakdown_msg.data[i];
    }
  }

  std_msgs::msg::Float32 runtime_msg;
  // runtime_msg.data = runtime / 1000.0;
  runtime_msg.data = overall_runtime_ / 1000.0;
  runtime_pub_->publish(runtime_msg);
  // std::cout<<"overall_runtime_(s): "<<runtime_msg.data<<std::endl;
  overall_runtime_ = 0;

  
}

double uav_exploration_ros::GetRobotToHomeDistance()
{
  Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
  return (robot_position - pd_.initial_position_).norm();
}

void uav_exploration_ros::PublishExplorationState()
{
  std_msgs::msg::Bool exploration_finished_msg;
  exploration_finished_msg.data = exploration_finished_;
  exploration_finish_pub_->publish(exploration_finished_msg);
}

void uav_exploration_ros::PrintExplorationStatus(std::string status, bool clear_last_line)
{
  if (clear_last_line)
  {
    printf(cursup);
    printf(cursclean);
    printf(cursup);
    printf(cursclean);
  }
  std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
}

void uav_exploration_ros::CountDirectionChange()
{
  Eigen::Vector3d current_moving_direction_ =
      Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z) -
      Eigen::Vector3d(pd_.last_robot_position_.x, pd_.last_robot_position_.y, pd_.last_robot_position_.z);

  if (current_moving_direction_.norm() > 0.5)
  {
    if (pd_.moving_direction_.dot(current_moving_direction_) < 0)
    {
      direction_change_count_++;
      direction_no_change_count_ = 0;
      if (direction_change_count_ > pp_.kDirectionChangeCounterThr)
      {
        if (!use_momentum_)
        {
          momentum_activation_count_++;
        }
        use_momentum_ = true;
      }
    }
    else
    {
      direction_no_change_count_++;
      if (direction_no_change_count_ > pp_.kDirectionNoChangeCounterThr)
      {
        direction_change_count_ = 0;
        use_momentum_ = false;
      }
    }
    pd_.moving_direction_ = current_moving_direction_;
  }
  pd_.last_robot_position_ = pd_.robot_position_;

  std_msgs::msg::Int32 momentum_activation_count_msg;
  momentum_activation_count_msg.data = momentum_activation_count_;
  momentum_activation_count_pub_->publish(momentum_activation_count_msg);
}
void uav_exploration_ros::waypointpub()
{
  misc_utils_ns::Timer other_timer("other                      ");
  Timer overall_processing_timer2("overall processing part2 ");
  overall_processing_timer2.Start();
  if(!env_update_done)
  {
    return;
  }
  misc_utils_ns::Timer viewpoint_pub("viewpoint_pub                      ");
  viewpoint_pub.Start();
  if(pub_mode_==0)
  {
    env_update_done=false;
    publish_done = true;
  }
  if(pub_mode_==2)
  {
    Eigen::Vector3d robot_current_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
    if((robot_current_position-pd_.visited_positions_[pd_.visited_positions_.size()-1]).norm()>2)
    {
      UpdateVisitedPositions();
    }
    pd_.uav_viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_,pd_.grid_world_);
    pd_.uav_viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_);
  }
  if(show_time)
  std::cout<<"--------------------"<<std::endl;
  
  // Global TSP
    std::vector<int> global_cell_tsp_order;
    std::vector<int> global_cell_covered_marker_vec;
    exploration_path_ns::ExplorationPath global_path;
    // GlobalPlanning_timer.Start();
    GlobalPlanning(global_cell_covered_marker_vec, global_cell_tsp_order, global_path);
    // std::cout<<"global_cell_tsp_order"<<global_cell_tsp_order.size()<<std::endl;
    // GlobalPlanning_timer.Stop(false);
    // Local TSP
    exploration_path_ns::ExplorationPath local_path;
    LocalPlanning(uav_uncovered_point_num_, uav_uncovered_frontier_point_num_, global_path, local_path);
    other_timer.Start();

    near_home_ = GetRobotToHomeDistance() < pp_.kRushHomeDist;
    at_home_ = GetRobotToHomeDistance() < pp_.kAtHomeDistThreshold;
            // rclcpp::Clock clock;
    
    if (pd_.grid_world_->IsReturningHome() && pd_.local_coverage_planner_->IsLocalCoverageComplete() &&
        (clock1.now() - start_time_).seconds() > least_explore_time_s)
    {
      if(!finish_triger)
      {
        finish_triger = true;
        finished_triger_time_ = clock1.now();
      }
      else if((clock1.now()-finished_triger_time_).seconds()>5)
      {
        if (!exploration_finished_)
        {
          PrintExplorationStatus("Exploration completed, returning home", false);
          std_msgs::msg::Float32 runtime_msg;
          mission_over_pub_->publish(runtime_msg);
          
        }
        exploration_finished_ = true;
      }
      
    }
    else{
      finish_triger = false;
    }

    if (exploration_finished_ && at_home_ && !stopped_)
    {
      PrintExplorationStatus("Return home completed", false);
      stopped_ = true;
    }

    pd_.exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);

    PublishExplorationState();

    lookahead_point_update_ = GetLookAheadPoint(pd_.exploration_path_, global_path, pd_.lookahead_point_);
    
    PublishWaypoint();
    other_timer.Stop(show_time,"ms");


    pd_.visualizer_->GetGlobalSubspaceMarker(pd_.grid_world_, global_cell_tsp_order);
    pd_.visualizer_->GetcoveredMarker(pd_.grid_world_, global_cell_covered_marker_vec);
    Eigen::Vector3d viewpoint_origin = pd_.uav_viewpoint_manager_->GetOrigin();
    pd_.visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(), pd_.robot_position_.z);
    pd_.visualizer_->PublishMarkers();

    
    PublishLocalPlanningVisualization(local_path);
    PublishGlobalPlanningVisualization(global_path, local_path);
    
    viewpoint_pub.Stop(show_time,"ms");
  overall_processing_timer2.Stop(show_time);
  overall_runtime_ += overall_processing_timer2.GetDuration();
  PublishRuntime();
}

void uav_exploration_ros::execute()
{
  // if(!start_signal_bool)
  // {
  //   // std::cout<<"start11"<<std::endl;
  //   return;
  // }
  
  if(!publish_done)
  {
    return;
  }
  if (!pp_.kAutoStart && !start_exploration_)
  {
    RCLCPP_INFO(rclcpp::get_logger("my_logger"), "Your message here");
    return;
  }
  
  Timer overall_processing_timer("overall processing part1 ");
  update_representation_runtime_ = 0;
  local_viewpoint_sampling_runtime_ = 0;
  local_path_finding_runtime_ = 0;
  global_planning_runtime_ = 0;
  trajectory_optimization_runtime_ = 0;
  overall_runtime_ = 0;
  tick_green_runtime_ = 0;
  if (!initialized_)
  {
    SendInitialWaypoint();
        // rclcpp::Clock clock;
    start_time_ = clock1.now();
    global_direction_switch_time_ = clock1.now();
    return;
  }

  overall_processing_timer.Start();
  
  if (keypose_cloud_update_)
  {
    if(show_time)
    std::cout<<"***************"<<std::endl;
    keypose_cloud_update_ = false;
    misc_utils_ns::Timer UpdateGlobalRepresentation_timer("CountDirectionChange");
    UpdateGlobalRepresentation_timer.Start();
    CountDirectionChange();
    UpdateGlobalRepresentation_timer.Stop(show_time);
    
    

    
    
    
    // misc_utils_ns::Timer GlobalPlanning_timer("GlobalPlanning");
    // Update grid world
    UpdateGlobalRepresentation();
    // int viewpoint_candidate_count = UpdateViewPoints();//被对标 2d viewpoint参数更新
    int viewpoint_candidate_count_test = Update_uav_ViewPoints_with_rolling_occupancy_grid_();
    // if (viewpoint_candidate_count == 0)
    // {
    //   // ROS_WARN("Cannot get candidate viewpoints, skipping this round");
    //   RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Cannot get candidate viewpoints, skipping this round");
    //   return;
    // }
    if (viewpoint_candidate_count_test == 0)
    {
      // ROS_WARN("Cannot get candidate viewpoints, skipping this round");
      RCLCPP_WARN(rclcpp::get_logger("my_logger"), "Cannot get candidate viewpoints, skipping this round");
      return;
    }

    UpdateKeyposeGraph();//里面是干relocation地图的（机器人keypose部分）

    int uncovered_point_num = 0;
    int uncovered_frontier_point_num = 0;
    //TODO 这里也是对标的，如果选择新的工作方式则上面的被遗弃，并且下面函数要改变
    uav_uncovered_point_num_ = 0;
    uav_uncovered_frontier_point_num_ = 0;
    //这里的作用在于进行探测点的已探知划分工作，也就是标绿，
    //第一个函数用于确定每个viewpoint点能覆盖到的范围， 
    //第二个函数用于基于已拜访的点进行标绿 并且计算出未拜访候选viewpoint点能够框到的未标绿点 （多个viewpoint可能框到同一个点）
    //计算出每个未标绿点框到的这些点直接用于 选择viewpoint做局部路径规划的依据
    
    if (!exploration_finished_ || !pp_.kNoExplorationReturnHome)
    {
      if(invalidate_viewpoint_and_point_in_covered_boxes)
      {
        pd_.uav_viewpoint_manager_->Get_connected_Hier_ind_vec_in_exploring_area(pd_.grid_world_);
        pd_.uav_viewpoint_manager_->visualize_connected_viewpoints_in_exploringarea();//可视化
        //TODO 记得把 概括到的非绿点 向量 清空
      }
      misc_utils_ns::Timer update_coverage_area_timer("update covered area");
      update_coverage_area_timer.Start();
      UpdateViewPointCoverage();
      UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num, uav_uncovered_point_num_, uav_uncovered_frontier_point_num_);
      update_coverage_area_timer.Stop(show_time);
      tick_green_runtime_ = update_coverage_area_timer.GetDuration();
    }
    else
    {
      pd_.uav_viewpoint_manager_->ResetViewPointCoverage();
    }

    env_update_done = true;
    publish_done = false;
    overall_processing_timer.Stop(show_time);
    overall_runtime_ += overall_processing_timer.GetDuration();
    if(show_time)
    std::cout<<"-------"<<std::endl;
  }
}
}  // namespace uav_exploration_ros_ns
