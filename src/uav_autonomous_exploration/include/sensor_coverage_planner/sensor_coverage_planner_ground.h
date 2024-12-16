/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
// #include <nav_msgs/msg/odometry.hpp>

// #include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int32.h>
// #include <std_msgs/Int32MultiArray.h>
// #include <std_msgs/Float32.h>
// #include "geometry_msgs/msg/polygon_stamped.hpp"
// #include <geometry_msgs/Pose.h>
//ROS2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include <utils/pointcloud_utils.h>
#include <utils/misc_utils.h>
// Components
#include "keypose_graph/keypose_graph.h"
#include "uav_keypose_graph/keypose_graph.h"
#include "planning_env/planning_env.h"
#include "viewpoint_manager/viewpoint_manager.h"
#include "uav_viewpoint_manager/viewpoint_manager.h"
#include "grid_world/grid_world.h"
#include "exploration_path/exploration_path.h"
#include "local_coverage_planner/local_coverage_planner.h"
#include "tare_visualizer/tare_visualizer.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace uav_exploration_ros_ns
{
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
typedef misc_utils_ns::Timer Timer;

struct PlannerParameters
{
  // String
  std::string sub_start_exploration_topic_;
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_uav_3d_lidar_msg_topic_;
  std::string sub_terrain_map_topic_;
  std::string sub_terrain_map_ext_topic_;
  std::string sub_coverage_boundary_topic_;
  std::string sub_viewpoint_boundary_topic_;
  std::string sub_nogo_boundary_topic_;

  std::string pub_exploration_finish_topic_;
  std::string pub_runtime_breakdown_topic_;
  std::string pub_runtime_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_momentum_activation_count_topic_;

  // Bool
  bool kAutoStart;
  bool kRushHome;
  bool kUseTerrainHeight;
  bool kCheckTerrainCollision;
  bool kExtendWayPoint;
  bool kUseLineOfSightLookAheadPoint;
  bool kNoExplorationReturnHome;
  bool kUseMomentum;
  bool kextendz;
  bool use_time_interval;
  bool way_point_in_free;
  // Double
  double kKeyposeCloudDwzFilterLeafSize;
  double kRushHomeDist;
  double kAtHomeDistThreshold;
  double kTerrainCollisionThreshold;
  double kLookAheadDistance;
  double kExtendWayPointDistanceBig;
  double kExtendWayPointDistanceBig_store;
  double kExtendWayPointDistanceSmall;

  // Int
  int kDirectionChangeCounterThr;
  int kDirectionNoChangeCounterThr;

  bool ReadParameters(std::shared_ptr<rclcpp::Node>& nh);
};

struct PlannerData
{
  // PCL clouds TODO: keypose cloud does not need to be PlannerCloudPointType
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> keypose_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> uav_3d_lidar_msg_stack_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> registered_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> large_terrain_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_ext_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_visited_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> grid_world_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> selected_viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploring_cell_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploration_path_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> lookahead_point_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> keypose_graph_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> uav_keypose_graph_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_in_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> point_cloud_manager_neighbor_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> reordered_global_subspace_cloud_;
  
  nav_msgs::msg::Odometry keypose_;
  geometry_msgs::msg::Point robot_position_;
  geometry_msgs::msg::Point last_robot_position_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  exploration_path_ns::ExplorationPath exploration_path_;
  Eigen::Vector3d lookahead_point_;
  Eigen::Vector3d lookahead_point_direction_;
  Eigen::Vector3d moving_direction_;
  double robot_yaw_;
  bool moving_forward_;
  std::vector<Eigen::Vector3d> visited_positions_;
  std::vector<Eigen::Vector3d> visited_positions_last_40_time;
  int cur_keypose_node_ind_;
  int cur_uav_keypose_node_ind_;
  Eigen::Vector3d initial_position_;

  std::unique_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  // std::unique_ptr<uav_keypose_graph_ns::KeyposeGraph> uav_keypose_graph_;
  std::unique_ptr<planning_env_ns::PlanningEnv> planning_env_;

  std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager> uav_viewpoint_manager_;
  
  std::unique_ptr<local_coverage_planner_ns::LocalCoveragePlanner> local_coverage_planner_;
  std::unique_ptr<grid_world_ns::GridWorld> grid_world_;
  std::unique_ptr<tare_visualizer_ns::TAREVisualizer> visualizer_;

  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::unique_ptr<misc_utils_ns::Marker> uav_keypose_graph_node_marker_;
  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  std::unique_ptr<misc_utils_ns::Marker> uav_keypose_graph_edge_marker_;
  std::unique_ptr<misc_utils_ns::Marker> connected_viewpoint_marker_test;//可视化
  std::unique_ptr<misc_utils_ns::Marker> nogo_boundary_marker_;
  std::unique_ptr<misc_utils_ns::Marker> grid_world_marker_;

  void Initialize(std::shared_ptr<rclcpp::Node>& nh,std::shared_ptr<rclcpp::Node>& nh_p);
};

class uav_exploration_ros
{
public:
  explicit uav_exploration_ros(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle);
  bool initialize(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle);
  void execute();
  void waypointpub();
  void Parameter_Declare(std::shared_ptr<rclcpp::Node>& node_handle,std::shared_ptr<rclcpp::Node>& private_node_handle);
  ~uav_exploration_ros() = default;
  

private:
  bool invalidate_viewpoint_and_point_in_covered_boxes;
  bool show_time;
  geometry_msgs::msg::PoseStamped last_waypoint;
  rclcpp::Clock _clock;
  rclcpp::Time last_pub_time;
  int cout_no_pub;
  std::vector<geometry_msgs::msg::PoseStamped> the_onebefore_last_waypoint;
  bool pub_flag;
  
  bool keypose_cloud_update_;
  bool initialized_;
  bool lookahead_point_update_;
  bool relocation_;
  int relocation_count_;
  bool start_exploration_;
  bool exploration_finished_;
  bool near_home_;
  bool at_home_;
  bool stopped_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  bool use_momentum_;
  bool lookahead_point_in_line_of_sight_;
  bool use_frontier_viewpoint;
  int least_explore_time_s;
  PlannerParameters pp_;
  PlannerData pd_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;

  int update_representation_runtime_;
  int local_viewpoint_update_runtime_;
  int local_frontier_viewpoint_update_and_sampling_runtime_;
  int local_viewpoint_sampling_runtime_;
  int global_relocation_graph_maintainance_runtime_;
  int local_graph_maintainance_runtime_;
  int local_path_finding_runtime_;
  int tick_green_runtime_;
  int global_planning_runtime_;
  int trajectory_optimization_runtime_;
  int overall_runtime_;
  int registered_cloud_count_;
  int ament_occupancy_count_;
  int keypose_count_;
  int direction_change_count_;
  int direction_no_change_count_;
  int momentum_activation_count_;
  int uav_uncovered_point_num_ ;
  int uav_uncovered_frontier_point_num_ ;
  bool env_update_done;
  bool publish_done;
  int pub_mode_;
  int smaller_viewpoint_wait_time_relocation;
  int pub_mode_store;
  double excute_triger_time;
  double waypointpub_triger_time;
  bool low_speed_run;
  bool finish_triger;
  rclcpp::Clock clock1;
  rclcpp::Time start_time_ ;
  rclcpp::Time finished_triger_time_ ;
  rclcpp::Time global_direction_switch_time_ ;
  // ros::Time start_time_;
  // ros::Time global_direction_switch_time_;

  // ros::Timer execution_timer_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  rclcpp::TimerBase::SharedPtr execution_timer_waypoint_pub;
  // ROS subscribers

  // ros::Subscriber exploration_start_sub_;
  // ros::Subscriber state_estimation_sub_;
  // ros::Subscriber uav_3d_lidar_msg_sub_;
  // ros::Subscriber terrain_map_sub_;
  // ros::Subscriber terrain_map_ext_sub_;
  // ros::Subscriber coverage_boundary_sub_;
  // ros::Subscriber viewpoint_boundary_sub_;
  // ros::Subscriber nogo_boundary_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Pub_switch;
  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exploration_start_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr uav_3d_lidar_msg_sub_;
  // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uav_3d_lidar_msg_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_ext_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_estimation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr coverage_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr viewpoint_boundary_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr nogo_boundary_sub_;

  // ROS publishers
  // ros::Publisher global_path_full_publisher_;
  // ros::Publisher global_path_publisher_;
  // ros::Publisher old_global_path_publisher_;
  // ros::Publisher to_nearest_global_subspace_path_publisher_;
  // ros::Publisher local_tsp_path_publisher_;
  // ros::Publisher exploration_path_publisher_;
  // ros::Publisher waypoint_pub_;
  // ros::Publisher exploration_finish_pub_;
  // ros::Publisher runtime_breakdown_pub_;
  // ros::Publisher runtime_pub_;
  // ros::Publisher momentum_activation_count_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_full_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_tsp_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr exploration_finish_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr runtime_breakdown_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr runtime_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mission_over_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr momentum_activation_count_pub_;
  // Debug
  // ros::Publisher pointcloud_manager_neighbor_cells_origin_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pointcloud_manager_neighbor_cells_origin_pub_;

  // Callback functions
  void Pub_Switch_(const std_msgs::msg::Bool::SharedPtr   start_msg);
  
  void ExplorationStartCallback(const std_msgs::msg::Bool::SharedPtr start_msg);
  void StateEstimationCallback(const nav_msgs::msg::Odometry::SharedPtr state_estimation_msg);
  void RegisteredScanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr uav_3d_lidar_msg_msg);
  void TerrainMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_map_msg);
  void TerrainMapExtCallback(const sensor_msgs::msg::PointCloud2::SharedPtr terrain_map_ext_msg);
  void CoverageBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg);
  void ViewPointBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg);
  void NogoBoundaryCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr polygon_msg);

  void SendInitialWaypoint();
  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  int Update_uav_ViewPoints_with_rolling_occupancy_grid_();
  void Diplomatic_acition_Viewpoint_and_Occupancy_array_(std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager_, const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_);
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num,int& uav_uncovered_point_num, int& uav_uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  void GlobalPlanning(std::vector<int>& global_cell_covered_marker_vec, std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path);
  void PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath& global_path,
                                          const exploration_path_ns::ExplorationPath& local_path);
  void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                     const exploration_path_ns::ExplorationPath& global_path,
                     exploration_path_ns::ExplorationPath& local_path);
  
  void PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path);
  exploration_path_ns::ExplorationPath ConcatenateGlobalLocalPath(
      const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path);

  void PublishRuntime();
  double GetRobotToHomeDistance();
  void PublishExplorationState();
  void PublishWaypoint();
  bool GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                         const exploration_path_ns::ExplorationPath& global_path, Eigen::Vector3d& lookahead_point);

  void PrintExplorationStatus(std::string status, bool clear_last_line = true);
  void CountDirectionChange();
};

}  // namespace uav_exploration_ros_ns
