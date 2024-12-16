//
// Created by caochao on 12/31/19.
//

#ifndef SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
#define SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H

#include <functional>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <planning_env/planning_env.h>
#include <utils/misc_utils.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace keypose_graph_ns
{
struct KeyposeNode;
class KeyposeGraph;
const double INF = 9999.0;
typedef std::pair<int, int> iPair;
}  // namespace keypose_graph_ns

struct keypose_graph_ns::KeyposeNode
{
  geometry_msgs::msg::Point position_;
  geometry_msgs::msg::Point offset_to_keypose_;
  int keypose_id_;
  int node_ind_;
  int cell_ind_;
  bool is_keypose_;
  bool is_connected_;

public:
  explicit KeyposeNode(double x = 0, double y = 0, double z = 0, int node_ind = 0, int keypose_id = 0,
                       bool is_keypose = true);
  explicit KeyposeNode(const geometry_msgs::msg::Point& point, int node_ind = 0, int keypose_id = 0, bool is_keypose = true);
  ~KeyposeNode() = default;
  bool IsKeypose() const
  {
    return is_keypose_;
  }
  bool IsConnected() const
  {
    return is_connected_;
  }
  void SetOffsetToKeypose(const geometry_msgs::msg::Point& offset_to_keypose)
  {
    offset_to_keypose_ = offset_to_keypose;
  }
  void SetCurrentKeyposePosition(const geometry_msgs::msg::Point& current_keypose_position)
  {
    offset_to_keypose_.x = position_.x - current_keypose_position.x;
    offset_to_keypose_.y = position_.y - current_keypose_position.y;
    offset_to_keypose_.z = position_.z - current_keypose_position.z;
  }
};

class keypose_graph_ns::KeyposeGraph
{
private:
  bool allow_vertical_edge_;
  int current_keypose_id_;
  geometry_msgs::msg::Point current_keypose_position_;
  std::vector<std::vector<bool>> viewpoints_check_;
  std::vector<std::vector<int>> graph_;
  std::vector<std::vector<double>> dist_;
  std::vector<Eigen::Vector3i> wait_for_check; 
  int ckeck_loger_than_length;
  int eliminate_edge_non_free_thr;
  std::vector<bool> in_local_planning_horizon_;
  std::vector<KeyposeNode> nodes_;
  std::vector<geometry_msgs::msg::Point> node_positions_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_connected_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr connected_nodes_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_nodes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr nodes_cloud_;
  
  std::vector<int> connected_node_indices_;
  double kAddNodeMinDist;
  double kAddNonKeyposeNodeMinDist;
  double kAddEdgeConnectDistThr;
  int ADD_edge_Thr;
  double kAddEdgeToLastKeyposeDistThr;
  double kAddEdgeVerticalThreshold;
  double kAddEdgeCollisionCheckResolution;
  double kAddEdgeCollisionCheckRadius;
  int kAddEdgeCollisionCheckPointNumThr;
  int count_time;
  
  static bool ComparePair(const std::pair<int, int>& a, const std::pair<int, int>& b)
  {
    return (a.first == b.first && a.second == b.second) || (a.first == b.second && a.second == b.first);
  }
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> station_illegal_connecte;//可视化
public:
  KeyposeGraph(std::shared_ptr<rclcpp::Node>& nh);
  ~KeyposeGraph() = default;
  void ReadParameters(std::shared_ptr<rclcpp::Node>& nh);
  void AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose ,std::string string_);
  void AddNodeAndEdge(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose,
                      int connected_node_ind, double connected_node_dist);
  void AddEdge(int from_node_ind, int to_node_ind, double dist);
  bool HasNode(const Eigen::Vector3d& position);
  bool Find_edge_connected(std::vector<int> A, std::vector<int> B);
  bool InBound(int index)
  {
    return index >= 0 && index < graph_.size();
  }
  int GetNodeNum()
  {
    return nodes_.size();
  }
  int GetConnectedNodeNum();
  void GetMarker(visualization_msgs::msg::Marker& node_marker, visualization_msgs::msg::Marker& edge_marker);
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
  
  std::vector<int> GetConnectedGraphNodeIndices()
  {
    return connected_node_indices_;
  }
  void GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices, std::vector<bool> constraints);
  void CheckLocalCollision(const geometry_msgs::msg::Point& robot_position,
                           const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager,
                           const std::unique_ptr<planning_env_ns::PlanningEnv> & planning_env_);
  void erase_node_edge_for_frontier_viewpoint(const std::vector<int> & viewpoint_stations_ind);
  void erase_edge_between_nodes_(int node_1_ind, int node_2_ind);
  void UpdateNodes();
  void CheckConnectivity(const geometry_msgs::msg::Point& robot_position);
  int AddKeyposeNode(const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_,const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager, std::vector<Eigen::Vector3d>& visited_positions_);
  void Check_near_station_connect(const geometry_msgs::msg::Point& robot_position,const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_,const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager);
  std::vector<int> Get_near_pose(int num, geometry_msgs::msg::Point point);
  bool HasEdgeBetween(int node_ind1, int node_ind2);
  bool IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position);
  int AddNonKeyposeNode(const geometry_msgs::msg::Point& new_node_position);
  void AddPath(const nav_msgs::msg::Path& path);
  std::vector<int> AddPath_for_frontier_viewpoint(const nav_msgs::msg::Path& path);
  void SetAllowVerticalEdge(bool allow_vertical_edge)
  {
    allow_vertical_edge_ = allow_vertical_edge;
  }
  bool IsPositionReachable(const geometry_msgs::msg::Point& point, double dist_threshold);
  bool IsPositionReachable(const geometry_msgs::msg::Point& point);
  int GetClosestNodeInd(const geometry_msgs::msg::Point& point);
  void GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  void GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist);
  int GetClosestKeyposeID(const geometry_msgs::msg::Point& point);
  geometry_msgs::msg::Point GetClosestNodePosition(const geometry_msgs::msg::Point& point);
  bool GetShortestPathWithMaxLength(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                                    double max_path_length, bool get_path, nav_msgs::msg::Path& path);
  double GetShortestPath(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                         bool get_path, nav_msgs::msg::Path& path, bool use_connected_nodes = false);

  double& SetAddNodeMinDist()
  {
    return kAddNodeMinDist;
  }
  double& SetAddNonKeyposeNodeMinDist()
  {
    return kAddNonKeyposeNodeMinDist;
  }
  double& SetAddEdgeCollisionCheckResolution()
  {
    return kAddEdgeCollisionCheckResolution;
  }
  double& SetAddEdgeCollisionCheckRadius()
  {
    return kAddEdgeCollisionCheckRadius;
  }
  int& SetAddEdgeCollisionCheckPointNumThr()
  {
    return kAddEdgeCollisionCheckPointNumThr;
  }
  double& SetAddEdgeConnectDistThr()
  {
    return kAddEdgeConnectDistThr;
  }
  double& SetAddEdgeToLastKeyposeDistThr()
  {
    return kAddEdgeToLastKeyposeDistThr;
  }
  double& SetAddEdgeVerticalThreshold()
  {
    return kAddEdgeVerticalThreshold;
  }
  const std::vector<KeyposeNode>& Get_nodes_() const
  {
    return nodes_;
  }
  geometry_msgs::msg::Point GetFirstKeyposePosition();
  geometry_msgs::msg::Point GetKeyposePosition(int keypose_id);
  void GetKeyposePositions(std::vector<Eigen::Vector3d>& positions);
  geometry_msgs::msg::Point GetNodePosition(int node_ind);
};

#endif  // SENSOR_COVERAGE_PLANNER_KEYPOSE_GRAPH_H
