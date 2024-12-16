//
// Created by caochao on 12/31/19.
//
// modified_by Huazhang Zhu (22210720332@m.fudan.edu.cn)
// date 2023-09-22
//
#include "../../include/keypose_graph/keypose_graph.h"
#include <uav_viewpoint_manager/viewpoint_manager.h>

namespace keypose_graph_ns
{
KeyposeNode::KeyposeNode(double x, double y, double z, int node_ind, int keypose_id, bool is_keypose)
  : cell_ind_(0), node_ind_(node_ind), keypose_id_(keypose_id), is_keypose_(is_keypose), is_connected_(true)
{
  position_.x = x;
  position_.y = y;
  position_.z = z;

  offset_to_keypose_.x = 0.0;
  offset_to_keypose_.y = 0.0;
  offset_to_keypose_.z = 0.0;
}

KeyposeNode::KeyposeNode(const geometry_msgs::msg::Point& point, int node_ind, int keypose_id, bool is_keypose)
  : KeyposeNode(point.x, point.y, point.z, node_ind, keypose_id, is_keypose)
{
}

KeyposeGraph::KeyposeGraph(std::shared_ptr<rclcpp::Node>& nh)
  : allow_vertical_edge_(true)
  , current_keypose_id_(0)
  , kAddNodeMinDist(1.0)
  , kAddEdgeCollisionCheckResolution(0.4)
  , kAddEdgeCollisionCheckRadius(0.3)
  , kAddEdgeConnectDistThr(3.0)
  , kAddEdgeToLastKeyposeDistThr(3.0)
  , kAddEdgeVerticalThreshold(1.0)
  , kAddEdgeCollisionCheckPointNumThr(1)
  , count_time(20)
  
{
  ReadParameters(nh);
  kdtree_connected_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  connected_nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_nodes_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  nodes_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  station_illegal_connecte = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "station_illegal_connecte", "map");//可视化
  
}

void KeyposeGraph::ReadParameters(std::shared_ptr<rclcpp::Node>& nh)
{
  kAddNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNodeMinDist", 0.5);
  kAddNonKeyposeNodeMinDist = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddNonKeyposeNodeMinDist", 0.5);
  kAddEdgeConnectDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeConnectDistThr", 0.5);
  ADD_edge_Thr = misc_utils_ns::getParam<int>(nh, "keypose_graph/add_edge_Thr", 5);
  kAddEdgeToLastKeyposeDistThr = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeToLastKeyposeDistThr", 0.5);
  kAddEdgeVerticalThreshold = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeVerticalThreshold", 0.5);
  kAddEdgeCollisionCheckResolution =
      misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckResolution", 0.5);
  kAddEdgeCollisionCheckRadius = misc_utils_ns::getParam<double>(nh, "keypose_graph/kAddEdgeCollisionCheckRadius", 0.5);
  kAddEdgeCollisionCheckPointNumThr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/kAddEdgeCollisionCheckPointNumThr", 0.5);
  ckeck_loger_than_length =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/ckeck_loger_than_length", 4);
  eliminate_edge_non_free_thr =
      misc_utils_ns::getParam<int>(nh, "keypose_graph/eliminate_edge_non_free_thr", 10);
  std::cout<<"ckeck_loger_than_length: "<<ckeck_loger_than_length<<std::endl;
  std::cout<<"eliminate_edge_non_free_thr: "<<eliminate_edge_non_free_thr<<std::endl;
}


void KeyposeGraph::AddNode(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose ,std::string string_)
{

  KeyposeNode new_node(position, node_ind, keypose_id, is_keypose);
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);
}
void KeyposeGraph::AddNodeAndEdge(const geometry_msgs::msg::Point& position, int node_ind, int keypose_id, bool is_keypose,
                                  int connected_node_ind, double connected_node_dist)
{
  AddNode(position, node_ind, keypose_id, is_keypose ,"AddNodeAndEdge");
  AddEdge(connected_node_ind, node_ind, connected_node_dist);
}

void KeyposeGraph::AddEdge(int from_node_ind, int to_node_ind, double dist)
{
  MY_ASSERT(from_node_ind >= 0 && from_node_ind < graph_.size() && from_node_ind < dist_.size());
  MY_ASSERT(to_node_ind >= 0 && to_node_ind < graph_.size() && to_node_ind < dist_.size());

  graph_[from_node_ind].push_back(to_node_ind);
  graph_[to_node_ind].push_back(from_node_ind);

  dist_[from_node_ind].push_back(dist);
  dist_[to_node_ind].push_back(dist);
  if(dist>ckeck_loger_than_length)//专门清理keypose加入时候超长跨越未知区域
  {
    Eigen::Vector3i node_pair;
    node_pair.x() = from_node_ind;
    node_pair.y() = to_node_ind;
    node_pair.z() = dist;
    wait_for_check.push_back(node_pair);
  }
}
bool KeyposeGraph::Find_edge_connected(std::vector<int> A, std::vector<int> B)
{
  bool connected = false;
  for(auto A_node_ind : A)
  {
    for(auto A_node_neibor_ind : graph_[A_node_ind])
    {
      for(auto B_node_ind : B)
      {
        if(A_node_neibor_ind == B_node_ind)
        {
          connected = true;
          break;
        }
      }
      if(connected)
      {
        break;
      }
    }
    if(connected)
    {
      break;
    }
  }
  return connected;
}
bool KeyposeGraph::HasNode(const Eigen::Vector3d& position)
{
  int closest_node_ind = -1;
  double min_dist = DBL_MAX;
  geometry_msgs::msg::Point geo_position;
  geo_position.x = position.x();
  geo_position.y = position.y();
  geo_position.z = position.z();
  GetClosestNodeIndAndDistance(geo_position, closest_node_ind, min_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist = misc_utils_ns::PointXYDist<geometry_msgs::msg::Point>(geo_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(geo_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return true;
    }
  }
  return false;
}

bool KeyposeGraph::HasEdgeBetween(int node_ind1, int node_ind2)
{
  if (node_ind1 >= 0 && node_ind1 < nodes_.size() && node_ind2 >= 0 && node_ind2 < nodes_.size())
  {
    if (std::find(graph_[node_ind1].begin(), graph_[node_ind1].end(), node_ind2) != graph_[node_ind1].end() ||
        std::find(graph_[node_ind2].begin(), graph_[node_ind2].end(), node_ind1) != graph_[node_ind2].end())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

bool KeyposeGraph::IsConnected(const Eigen::Vector3d& from_position, const Eigen::Vector3d& to_position)
{
  geometry_msgs::msg::Point from_node_position;
  from_node_position.x = from_position.x();
  from_node_position.y = from_position.y();
  from_node_position.z = from_position.z();
  int closest_from_node_ind = -1;
  double closest_from_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(from_node_position, closest_from_node_ind, closest_from_node_dist);

  geometry_msgs::msg::Point to_node_position;
  to_node_position.x = to_position.x();
  to_node_position.y = to_position.y();
  to_node_position.z = to_position.z();
  int closest_to_node_ind = -1;
  double closest_to_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(to_node_position, closest_to_node_ind, closest_to_node_dist);

  if (closest_from_node_ind != -1 && closest_from_node_ind == closest_to_node_ind)
  {
    return true;
  }
  else if (HasEdgeBetween(closest_from_node_ind, closest_to_node_ind))
  {
    return true;
  }
  else
  {
    return false;
  }
}

int KeyposeGraph::AddNonKeyposeNode(const geometry_msgs::msg::Point& new_node_position)
{
  int new_node_index = -1;
  int closest_node_ind = -1;
  double closest_node_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(new_node_position, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    double xy_dist =
        misc_utils_ns::PointXYDist<geometry_msgs::msg::Point>(new_node_position, nodes_[closest_node_ind].position_);
    double z_dist = std::abs(new_node_position.z - nodes_[closest_node_ind].position_.z);
    if (xy_dist < kAddNonKeyposeNodeMinDist && z_dist < 1.0)
    {
      return closest_node_ind;
    }
  }
  new_node_index = nodes_.size();
  KeyposeNode new_node(new_node_position, new_node_index, current_keypose_id_, false);
  new_node.SetCurrentKeyposePosition(current_keypose_position_);
  nodes_.push_back(new_node);
  std::vector<int> neighbors;
  graph_.push_back(neighbors);
  std::vector<double> neighbor_dist;
  dist_.push_back(neighbor_dist);

  return new_node_index;
}

void KeyposeGraph::AddPath(const nav_msgs::msg::Path& path)
{
  if (path.poses.size() < 2)
  {
    return;
  }
  int prev_node_index = -1;
  for (int i = 0; i < path.poses.size(); i++)
  {
    int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
    if (i != 0)
    {
      // Add edge to previous node
      if (prev_node_index >= 0 && prev_node_index < nodes_.size())
      {
        // Check duplication
        if (!HasEdgeBetween(prev_node_index, cur_node_index))
        {
          geometry_msgs::msg::Point prev_node_position = nodes_[prev_node_index].position_;
          double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              prev_node_position, path.poses[i].pose.position);
          graph_[prev_node_index].push_back(cur_node_index);
          graph_[cur_node_index].push_back(prev_node_index);

          dist_[prev_node_index].push_back(dist_to_prev);
          dist_[cur_node_index].push_back(dist_to_prev);

          if(dist_to_prev>ckeck_loger_than_length)//专门清理keypose加入时候超长跨越未知区域
          {
            Eigen::Vector3i node_pair;
            node_pair.x() = prev_node_index;
            node_pair.y() = cur_node_index;
            node_pair.z() = dist_to_prev;
            wait_for_check.push_back(node_pair);
          }
        }
      }
      else
      {
        // ROS_ERROR_STREAM("KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
        //                                                            << nodes_.size() - 1 << "]");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
                                                                   << nodes_.size() - 1 << "]");
        return;
      }
    }
    prev_node_index = cur_node_index;
  }
  UpdateNodes();
}


std::vector<int> KeyposeGraph::AddPath_for_frontier_viewpoint(const nav_msgs::msg::Path& path)
{
  std::vector<int> path_node_ind;
  if (path.poses.size() < 2)
  {
    return path_node_ind;
  }
  int prev_node_index = -1;
  for (int i = 0; i < path.poses.size(); i++)
  {
    int cur_node_index = AddNonKeyposeNode(path.poses[i].pose.position);
    path_node_ind.push_back(cur_node_index);
    if (i != 0)
    {
      // Add edge to previous node
      if (prev_node_index >= 0 && prev_node_index < nodes_.size())
      {
        // Check duplication
        if (!HasEdgeBetween(prev_node_index, cur_node_index))
        {
          geometry_msgs::msg::Point prev_node_position = nodes_[prev_node_index].position_;
          double dist_to_prev = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              prev_node_position, path.poses[i].pose.position);
          graph_[prev_node_index].push_back(cur_node_index);
          graph_[cur_node_index].push_back(prev_node_index);

          dist_[prev_node_index].push_back(dist_to_prev);
          dist_[cur_node_index].push_back(dist_to_prev);
          
          if(dist_to_prev>ckeck_loger_than_length)//专门清理keypose加入时候超长跨越未知区域
          {
            Eigen::Vector3i node_pair;
            node_pair.x() = prev_node_index;
            node_pair.y() = cur_node_index;
            node_pair.z() = dist_to_prev;
            wait_for_check.push_back(node_pair);
          }
        }
      }
      else
      {
        // ROS_ERROR_STREAM("KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
        //                                                            << nodes_.size() - 1 << "]");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::AddPath: prev_node_index " << prev_node_index << " out of bound [0, "
                                                                   << nodes_.size() - 1 << "]");
        return path_node_ind;
      }
    }
    prev_node_index = cur_node_index;
  }
  UpdateNodes();
  return path_node_ind;
}


void KeyposeGraph::GetMarker(visualization_msgs::msg::Marker& node_marker, visualization_msgs::msg::Marker& edge_marker)
{
  node_marker.points.clear();
  edge_marker.points.clear();

  for (const auto& node : nodes_)
  {
    node_marker.points.push_back(node.position_);
  }

  std::vector<std::pair<int, int>> added_edge;
  for (int i = 0; i < graph_.size(); i++)
  {
    int start_ind = i;
    for (int j = 0; j < graph_[i].size(); j++)
    {
      int end_ind = graph_[i][j];
      if (std::find(added_edge.begin(), added_edge.end(), std::make_pair(start_ind, end_ind)) == added_edge.end())
      {
        geometry_msgs::msg::Point start_node_position = nodes_[start_ind].position_;
        geometry_msgs::msg::Point end_node_position = nodes_[end_ind].position_;
        edge_marker.points.push_back(start_node_position);
        edge_marker.points.push_back(end_node_position);
        added_edge.emplace_back(start_ind, end_ind);
      }
    }
  }
}

int KeyposeGraph::GetConnectedNodeNum()
{
  int connected_node_num = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_connected_)
    {
      connected_node_num++;
    }
  }
  return connected_node_num;
}

void KeyposeGraph::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  cloud->clear();
  for (const auto& node : nodes_)
  {
    pcl::PointXYZI point;
    point.x = node.position_.x;
    point.y = node.position_.y;
    point.z = node.position_.z;
    if (node.is_connected_)
    {
      point.intensity = 10;
    }
    else
    {
      point.intensity = -1;
    }
    cloud->points.push_back(point);
  }
}

void KeyposeGraph::GetConnectedNodeIndices(int query_ind, std::vector<int>& connected_node_indices,
                                           std::vector<bool> constraints)
{
  if (nodes_.size() != constraints.size())
  {
    // ROS_ERROR("KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::GetConnectedNodeIndices: constraints size not equal to node size");

    return;
  }
  if (query_ind < 0 || query_ind >= nodes_.size())
  {
    // ROS_ERROR_STREAM("KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
    //                                                                       << nodes_.size() << "]");
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::GetConnectedNodeIndices: query_ind: " << query_ind << " out of range: [0, "
                                                                          << nodes_.size() << "]");
                                                                      
    return;
  }
  connected_node_indices.clear();
  std::vector<bool> visited(nodes_.size(), false);
  std::stack<int> dfs_stack;
  dfs_stack.push(query_ind);
  while (!dfs_stack.empty())
  {
    int current_ind = dfs_stack.top();
    connected_node_indices.push_back(current_ind);
    dfs_stack.pop();
    if (!visited[current_ind])
    {
      visited[current_ind] = true;
    }
    for (int i = 0; i < graph_[current_ind].size(); i++)
    {
      int neighbor_ind = graph_[current_ind][i];
      if (!visited[neighbor_ind] && constraints[neighbor_ind])
      {
        dfs_stack.push(neighbor_ind);
      }
    }
  }
}

void KeyposeGraph::CheckLocalCollision(const geometry_msgs::msg::Point& robot_position,
                                       const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager,
                                       const std::unique_ptr<planning_env_ns::PlanningEnv> & planning_env_)
{
  // Get local planning horizon xy size
  int in_local_planning_horizon_count = 0;
  int collision_node_count = 0;
  int collision_edge_count = 0;
  int in_viewpoint_range_count = 0;
  Eigen::Vector3d viewpoint_resolution = uav_viewpoint_manager->GetResolution();
  double max_z_diff = std::max(viewpoint_resolution.x(), viewpoint_resolution.y()) * 2;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      continue;
    }

    Eigen::Vector3d node_position =
        Eigen::Vector3d(nodes_[i].position_.x, nodes_[i].position_.y, nodes_[i].position_.z);
    Eigen::Vector3i Hier_sub = uav_viewpoint_manager->Hierarchy_Pos_2_Hierarchy_sub(node_position);
    int viewpoint_Hier_ind = uav_viewpoint_manager->Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
    bool node_in_collision = false;
    if ( uav_viewpoint_manager->InRange_Hier_sub_for_keypose(Hier_sub))
    {
      in_local_planning_horizon_count++;
      in_viewpoint_range_count++;
      if (uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_real_collision_())
      {
        node_in_collision = true;
        collision_node_count++;
        // Delete all the associated edges
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          for (int k = 0; k < graph_[neighbor_ind].size(); k++)
          {
            if (graph_[neighbor_ind][k] == i)
            {
              graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
              dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
              k--;
              // if(graph_[neighbor_ind].empty())
              // {
              //   std::cout<<"地铁站空了，清理！"<<std::endl;
              //   graph_.erase(graph_.begin()+neighbor_ind);
              //   dist_.erase(dist_.begin()+neighbor_ind);
              //   std::cout<<nodes_.size()<<std::endl;
              //   nodes_.erase(nodes_.begin()+neighbor_ind);
              //   std::cout<<nodes_.size()<<std::endl;
              // }
            }
          }
        }
        graph_[i].clear();
        dist_[i].clear();
        // graph_.erase(graph_.begin()+i);
        // dist_.erase(dist_.begin()+i);
        // nodes_.erase(nodes_.begin()+i);
        // std::cout<<"干活了"<<std::endl;
      }
      else
      {
        Eigen::Vector3d viewpoint_resolution = uav_viewpoint_manager->GetResolution();
        double collision_check_resolution = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
        // Check edge collision
        for (int j = 0; j < graph_[i].size(); j++)
        {
          int neighbor_ind = graph_[i][j];
          Eigen::Vector3d start_position = node_position;
          Eigen::Vector3d end_position = Eigen::Vector3d(
              nodes_[neighbor_ind].position_.x, nodes_[neighbor_ind].position_.y, nodes_[neighbor_ind].position_.z);
          std::vector<Eigen::Vector3d> interp_points;
          misc_utils_ns::LinInterpPoints(start_position, end_position, collision_check_resolution, interp_points);
          for (const auto& collision_check_position : interp_points)
          {
            Eigen::Vector3i Hier_sub = uav_viewpoint_manager->Hierarchy_Pos_2_Hierarchy_sub(collision_check_position);
            int viewpoint_Hier_ind = uav_viewpoint_manager->Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
            if (uav_viewpoint_manager->InRange_Hier_sub_for_keypose(Hier_sub))
            {
              if (uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_real_collision_())
              {
                geometry_msgs::msg::Point viewpoint_position = uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->GetPosition();
                // Delete neighbors' edges
                for (int k = 0; k < graph_[neighbor_ind].size(); k++)
                {
                  if (graph_[neighbor_ind][k] == i)
                  {
                    collision_edge_count++;
                    graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
                    dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
                    k--;
                  }
                }
                // Delete the node's edge
                graph_[i].erase(graph_[i].begin() + j);
                dist_[i].erase(dist_[i].begin() + j);
                j--;
                break;
              }
            }
          }
        }
      }
    }
  }

  //先判是否有其中一个点在局部中，然后从rayhelper判断  从这个点 到 边界  注意是occupy的边界 是否空闲带你
  // std::cout<<"wait_for_check.size() ："<<wait_for_check.size()<<std::endl;
  
  int temp_count = 0;
  for(auto pair : wait_for_check)
  {
      //一段
                //另一端
    Eigen::Vector3d point_1,point_2;
    point_1.x() = nodes_[pair.x()].position_.x;
    point_1.y() = nodes_[pair.x()].position_.y;
    point_1.z() = nodes_[pair.x()].position_.z;

    point_2.x() = nodes_[pair.y()].position_.x;
    point_2.y() = nodes_[pair.y()].position_.y;
    point_2.z() = nodes_[pair.y()].position_.z;

    int array_ind_for_occupancy;
    Eigen::Vector3i point_1_sub_in_occupancy = planning_env_->Get_rolling_occupancy_grid_()->Get_occupancy_array_()->Pos2Sub(point_1);
    Eigen::Vector3i point_2_sub_in_occupancy = planning_env_->Get_rolling_occupancy_grid_()->Get_occupancy_array_()->Pos2Sub(point_2);
    bool point_1_in_range = planning_env_->Get_rolling_occupancy_grid_()->Get_occupancy_array_()->InRange(point_1_sub_in_occupancy);
    bool point_2_in_range = planning_env_->Get_rolling_occupancy_grid_()->Get_occupancy_array_()->InRange(point_2_sub_in_occupancy);
    int non_free_point_count=0;
    if(point_1_in_range || point_2_in_range)
    {
      pcl::PointXYZI point;
      if(point_1_in_range)
      {
        non_free_point_count = planning_env_->Get_rolling_occupancy_grid_()->Get_collision_unkown_in_path(point_1,point_2);
      }
      else{
        non_free_point_count = planning_env_->Get_rolling_occupancy_grid_()->Get_collision_unkown_in_path(point_2,point_1);
      }
      // std::cout<<"dist:"<<pair.z() <<"   non_free_point_count : "<<non_free_point_count<<std::endl;
      if(non_free_point_count >= eliminate_edge_non_free_thr)
      {
        point.x = point_1.x();
        point.y = point_1.y();
        point.z = point_1.z();
        station_illegal_connecte->cloud_->points.push_back(point);
        point.x = point_2.x();
        point.y = point_2.y();
        point.z = point_2.z();
        station_illegal_connecte->cloud_->points.push_back(point);
        erase_edge_between_nodes_(pair.x(),pair.y());
      }
      wait_for_check.erase(wait_for_check.begin()+temp_count);
    }

    
    temp_count++;

  }
  if(station_illegal_connecte->cloud_->points.size()>0)
  {
    station_illegal_connecte->Publish();//可视化  
  }
  
}



void KeyposeGraph::erase_node_edge_for_frontier_viewpoint(const std::vector<int> & viewpoint_stations_ind)
{
  for(auto ind : viewpoint_stations_ind)
  {
    for (int j = 0; j < graph_[ind].size(); j++)
    {
      int neighbor_ind = graph_[ind][j];
      for (int k = 0; k < graph_[neighbor_ind].size(); k++)
      {
        if (graph_[neighbor_ind][k] == ind)
        {
          graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
          dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
          k--;

        }
      }
    }
    graph_[ind].clear();
    dist_[ind].clear();
    auto pos_ = nodes_[ind].position_;
    pcl::PointXYZI point;
    point.x = pos_.x;
    point.y = pos_.y;
    point.z = pos_.z;
    station_illegal_connecte->cloud_->points.push_back(point);
  }
}
void KeyposeGraph::erase_edge_between_nodes_(int node_1_ind, int node_2_ind)
{
      for (int k = 0; k < graph_[node_1_ind].size(); k++)
      {
        if (graph_[node_1_ind][k] == node_2_ind)
        {
          graph_[node_1_ind].erase(graph_[node_1_ind].begin() + k);
          dist_[node_1_ind].erase(dist_[node_1_ind].begin() + k);
          break;
        }
      }
      for (int k = 0; k < graph_[node_2_ind].size(); k++)
      {
        if (graph_[node_2_ind][k] == node_1_ind)
        {
          graph_[node_2_ind].erase(graph_[node_2_ind].begin() + k);
          dist_[node_2_ind].erase(dist_[node_2_ind].begin() + k);
          break;
        }
      }
}

void KeyposeGraph::UpdateNodes()
{
  nodes_cloud_->clear();
  for (int i = 0; i < nodes_.size(); i++)
  {
    pcl::PointXYZI point;
    point.x = nodes_[i].position_.x;
    point.y = nodes_[i].position_.y;
    point.z = nodes_[i].position_.z;
    point.intensity = i;
    nodes_cloud_->points.push_back(point);
  }
  if (!nodes_cloud_->points.empty())
  {
    kdtree_nodes_->setInputCloud(nodes_cloud_);
  }
}

void KeyposeGraph::CheckConnectivity(const geometry_msgs::msg::Point& robot_position)
{
  if (nodes_.empty())
  {
    return;
  }
  UpdateNodes();

  // The first keypose node is always connected, set all the others to be disconnected
  int first_keypose_node_ind = -1;
  bool found_connected = false;

  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      first_keypose_node_ind = i;
      break;
    }
  }

  // Check the connectivity starting from the robot
  for (int i = 0; i < nodes_.size(); i++)
  {
    nodes_[i].is_connected_ = false;
  }
  if (first_keypose_node_ind >= 0 && first_keypose_node_ind < nodes_.size())
  {
    nodes_[first_keypose_node_ind].is_connected_ = true;
    connected_node_indices_.clear();
    std::vector<bool> constraint(nodes_.size(), true);
    GetConnectedNodeIndices(first_keypose_node_ind, connected_node_indices_, constraint);
  }
  else
  {
    int robot_node_ind = -1;
    double robot_node_dist = DBL_MAX;
    GetClosestNodeIndAndDistance(robot_position, robot_node_ind, robot_node_dist);
    if (robot_node_ind >= 0 && robot_node_ind < nodes_.size())
    {
      nodes_[robot_node_ind].is_connected_ = true;
      connected_node_indices_.clear();
      std::vector<bool> constraint(nodes_.size(), true);
      GetConnectedNodeIndices(robot_node_ind, connected_node_indices_, constraint);
    }
    else
    {
      // ROS_ERROR_STREAM("KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::CheckConnectivity: Cannot get closest robot node ind " << robot_node_ind);

    }
  }

  connected_nodes_cloud_->clear();
  for (int i = 0; i < connected_node_indices_.size(); i++)
  {
    int node_ind = connected_node_indices_[i];
    nodes_[node_ind].is_connected_ = true;
    pcl::PointXYZI point;
    point.x = nodes_[node_ind].position_.x;
    point.y = nodes_[node_ind].position_.y;
    point.z = nodes_[node_ind].position_.z;
    point.intensity = node_ind;
    connected_nodes_cloud_->points.push_back(point);
  }
  if (!connected_nodes_cloud_->points.empty())
  {
    kdtree_connected_nodes_->setInputCloud(connected_nodes_cloud_);
  }
}
// void KeyposeGraph::Check_near_station_connect(const geometry_msgs::msg::Point& robot_position, const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_,const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager)
// {
//   //因为抹除关系与occupy传过来的点云有关系，注意kd树的概括范围，不应大于occupy点云范围
//   if(nodes_cloud_->points.size()<20)
//   {
//     return;
//   }
//   UpdateNodes();
//   pcl::PointXYZI search_point;
//   search_point.x = robot_position.x;
//   search_point.y = robot_position.y;
//   search_point.z = robot_position.z;
//   std::vector<int> neighbor_indices;
//   std::vector<float> neighbor_sqdist;
//   Eigen::Vector3d start_position,end_position;
//   kdtree_nodes_->radiusSearch(search_point, 10, neighbor_indices, neighbor_sqdist);
//   int node_ind;
//   for(auto ind : neighbor_indices)
//   {
//     node_ind = static_cast<int>(nodes_cloud_->points[ind].intensity);
//     //先判断是否阻断
//     bool disconnected_flag = false;
//     int neighbor_ind;
//     for (int j = 0; j < graph_[node_ind].size(); j++)//每个点；点和它的邻居；
//     {
//       disconnected_flag = false;
//       neighbor_ind = graph_[node_ind][j];

//       double neighbor_node_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[node_ind].position_,
//                                                                                             nodes_[neighbor_ind].position_);
//       double diff_x = nodes_[node_ind].position_.x - nodes_[neighbor_ind].position_.x;
//       double diff_y = nodes_[node_ind].position_.y - nodes_[neighbor_ind].position_.y;
//       double diff_z = nodes_[node_ind].position_.z - nodes_[neighbor_ind].position_.z;
//       int check_point_num = static_cast<int>(neighbor_node_dist / kAddEdgeCollisionCheckResolution);
//       // bool in_collision = false;
//       for (int h = 0; h <= check_point_num+1; h++)
//       {
//         // std::cout << "checking the " << i << " collision point" << std::endl;
//         double check_point_x =
//             nodes_[neighbor_ind].position_.x + kAddEdgeCollisionCheckResolution * h * diff_x / neighbor_node_dist;
//         double check_point_y =
//             nodes_[neighbor_ind].position_.y + kAddEdgeCollisionCheckResolution * h * diff_y / neighbor_node_dist;
//         double check_point_z =
//             nodes_[neighbor_ind].position_.z + kAddEdgeCollisionCheckResolution * h * diff_z / neighbor_node_dist;
//         // std::cout<<"111"<<std::endl;
//         if (planning_env.InCollision(check_point_x, check_point_y, check_point_z))
//         {
//           disconnected_flag = true;
//           break;
//         }
//       }
//       //抹除关系
//       if(disconnected_flag)
//       {
//         std::cout<<"抹除关系"<<std::endl;
//         for (int k = 0; k < graph_[neighbor_ind].size(); k++)
//         {
//           if (graph_[neighbor_ind][k] == node_ind)
//           {
//             graph_[neighbor_ind].erase(graph_[neighbor_ind].begin() + k);
//             dist_[neighbor_ind].erase(dist_[neighbor_ind].begin() + k);
//             k--;
//           }
//         }
//         graph_[node_ind].erase(graph_[node_ind].begin() + j);
//         dist_[node_ind].erase(dist_[node_ind].begin() + j);
//       }
//       //给他两一个机会 用viewpoint帮他们弄回来
//       start_position.x() = nodes_[node_ind].position_.x;
//       start_position.y() = nodes_[node_ind].position_.y;
//       start_position.z() = nodes_[node_ind].position_.z;
//       end_position.x() = nodes_[neighbor_ind].position_.x;
//       end_position.y() = nodes_[neighbor_ind].position_.y;
//       end_position.z() = nodes_[neighbor_ind].position_.z;
//       nav_msgs::msg::Path path_in_between = uav_viewpoint_manager->GetViewPointShortestPath_no_warn(
//                     start_position, end_position);
//       if(path_in_between.poses.size()==0)
//       {
//         // std::cout<<"没法连接"<<std::endl;
//         continue;
//       }
//       else{//找到了路径，弄他
//         //去头去尾
//         path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
//         // path_in_between.poses.erase(path_in_between.poses.begin()+path_in_between.poses.size()-1);
//         // path_in_between.poses.erase(path_in_between.poses.begin());
//         //第一个和lastkey相连
//         double dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[0].pose.position,
//                                                                                       nodes_[node_ind].position_);
//         AddNodeAndEdge(path_in_between.poses[0].pose.position, nodes_.size(), current_keypose_id_, false, node_ind, dist_temp);
//         for(int h=1; h<path_in_between.poses.size();h++)//前后相互联系 add 和edge
//         {
//           dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[h].pose.position,
//                                                                                       path_in_between.poses[h-1].pose.position);
//           AddNodeAndEdge(path_in_between.poses[h].pose.position, nodes_.size(), current_keypose_id_, false, nodes_.size()-1, dist_temp);
//         }
//         dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[path_in_between.poses.size()-1].pose.position,
//                                                                                       nodes_[neighbor_ind].position_);
//         AddEdge(nodes_.size()-1,neighbor_ind,dist_temp);
//         // AddNodeAndEdge(nodes_[neighbor_ind].position_, nodes_.size(), current_keypose_id_, true, nodes_.size()-1, dist_temp);
//         // addpath_keypose_flag = true;
//         // break;
//         std::cout<<"viewpoint连会来"<<std::endl;
//       }
//     }
//   }
    
// }

std::vector<int> KeyposeGraph::Get_near_pose(int num, geometry_msgs::msg::Point point)
{
  double min_dist = DBL_MAX;
  int min_dist_ind = -1;
  std::vector<int> near_node;
  double dist;


  for (int i = 0; i < nodes_.size(); i++)
  {
    dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_,
                                                                                            point);
    if (dist < min_dist )
    {
      min_dist = dist;
      min_dist_ind = i;
      if(near_node.size() >= num)
      {
        near_node.erase(near_node.begin());
        
      }
      near_node.push_back(i);
    }
    
  }
  return near_node;
}

int KeyposeGraph::AddKeyposeNode(const nav_msgs::msg::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_,const std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager>& uav_viewpoint_manager, std::vector<Eigen::Vector3d>& visited_positions_last_40_time)
{
  current_keypose_position_ = keypose.pose.pose.position;

  ///
  // Eigen::Vector3d Hier_pos;
  // Hier_pos.x() = current_keypose_position_.x;
  // Hier_pos.y() = current_keypose_position_.y;
  // Hier_pos.z() = current_keypose_position_.z;
  // Eigen::Vector3i Hier_sub = uav_viewpoint_manager->Hierarchy_Pos_2_Hierarchy_sub(Hier_pos);
  // if(!uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(uav_viewpoint_manager->Hierarchy_sub_2_Hierarchy_ind(Hier_sub))->Connected() && uav_viewpoint_manager->update_Robot_Hier_ind_)
  // {    
  //   uav_viewpoint_manager->update_Robot_Hier_ind_=false;
  //   std::cout<<"成功救一帧地铁--------------------------------三个色色鬼色共克时艰额更好苏恶化"<<std::endl;
  //   current_keypose_position_ = uav_viewpoint_manager->Get_littlebox_from_Hierarchy_ind(uav_viewpoint_manager->Robot_Hier_ind_)->GetPosition();
    
  // }
  ///

  current_keypose_id_ = static_cast<int>(keypose.pose.covariance[0]);
  int new_node_ind = nodes_.size();
  int newest_keypose_ind = new_node_ind;
  int keypose_node_count = 0;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (nodes_[i].is_keypose_)
    {
      keypose_node_count++;
    }
  }
  if (nodes_.empty() || keypose_node_count == 0)
  {
    AddNode(current_keypose_position_, new_node_ind, current_keypose_id_, true ,"nodes_.empty() || keypose_node_count == 0");
    return new_node_ind;
  }
  else
  {
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    double last_keypose_dist = DBL_MAX;
    // double last_com_point_dist = DBL_MAX;
    int last_keypose_ind = -1;
    int max_keypose_id = 0;
    bool last_viewpoint_select=false;
    bool near_viewpoint_select=false;
    std::vector<int> in_range_node_indices;
    std::vector<double> in_range_node_dist;

    std::vector<double> min_dist_vec;
    std::vector<int> min_dist_ind_vec;
    std::vector<double> min_dist_vec_long;//这两个是用于放错机制的，让机器人左右横跳的时候连接到附近的
    std::vector<int> min_dist_ind_vec_long;

    std::vector<double> last_keypose_dist_vec;
    std::vector<int> last_keypose_ind_vec;
    std::vector<int> com_point_in_5_m;
    int neigbour_keypose_count=0;

    for (int i = 0; i < nodes_.size(); i++)
    {
      if (!allow_vertical_edge_)
      {
        if (std::abs(nodes_[i].position_.z - current_keypose_position_.z) > kAddEdgeVerticalThreshold)
        {
          continue;
        }
      }
      double dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_,
                                                                                            current_keypose_position_);
      if (dist < min_dist && nodes_[i].is_keypose_)
      {
        min_dist = dist;
        min_dist_ind = i;
        if(min_dist_vec.size()>=10)
        {
          min_dist_vec.erase(min_dist_vec.begin());
          min_dist_ind_vec.erase(min_dist_ind_vec.begin());
        }
        min_dist_vec.push_back(min_dist);
        min_dist_ind_vec.push_back(min_dist_ind);
      }
      int keypose_id = nodes_[i].keypose_id_;
      if (keypose_id >= max_keypose_id && nodes_[i].is_keypose_)
      {
        last_keypose_dist = dist;
        last_keypose_ind = i;
        max_keypose_id = keypose_id;
        if(last_keypose_dist_vec.size()>=10)
        {
          last_keypose_dist_vec.erase(last_keypose_dist_vec.begin());
          last_keypose_ind_vec.erase(last_keypose_ind_vec.begin());
        }
        last_keypose_dist_vec.push_back(last_keypose_dist);
        last_keypose_ind_vec.push_back(last_keypose_ind);
      }
      if (dist < kAddEdgeConnectDistThr)
      {
        if(in_range_node_indices.size()>ADD_edge_Thr)
        {
          in_range_node_indices.erase(in_range_node_indices.begin());
          in_range_node_dist.erase(in_range_node_dist.begin());
        }
        in_range_node_indices.push_back(i);
        in_range_node_dist.push_back(dist);
      }
      if (dist < 5 && nodes_[i].is_keypose_)
      {
        neigbour_keypose_count++;
      }
      if(dist < 15&& dist > 10 && nodes_[i].is_keypose_)
      {
        min_dist_vec_long.push_back(dist);
        min_dist_ind_vec_long.push_back(i);
      }
      if(dist < 5)
      {
        com_point_in_5_m.push_back(i);
      }
    }
    bool add_flag=true;
    int count=0;
    Eigen::Vector3i node_pair;
    for(int i=0; i < com_point_in_5_m.size(); i++)
    {
      for(auto ind_: graph_[com_point_in_5_m[i]])
      {
        add_flag=true;
        for(int h = i+1; h<com_point_in_5_m.size(); h++)
        {
          if(ind_ == com_point_in_5_m[h])
          {
            // com_point_in_5_m.erase(com_point_in_5_m.begin()+h);
            add_flag = false;
            break;
          }
        }
        if(add_flag)
        {
          
          node_pair.x() = ind_;
          node_pair.y() = com_point_in_5_m[i];
          node_pair.z() = 0;
          wait_for_check.push_back(node_pair);
          count++;
        }
        
      }
    }
    // std::cout<<"add count: "<<count<<std::endl;

    
    // If the closest keypose node is some distance away
    if (min_dist_ind >= 0 && min_dist_ind < nodes_.size())
    {
      //放错机制-------------------
      Eigen::Vector3d start_position,end_position;
      Eigen::Vector3i start_sub_in_occupancy,end_sub_in_occupancy;
      std::vector<Eigen::Vector3i> ray_cast_cells;
      bool success_flag=false;
      int array_ind_for_occupancy;
      
      int adjacent_count_ = 0;
      bool ignore_flag = false;
      Eigen::Vector3d last_pos = visited_positions_last_40_time[visited_positions_last_40_time.size()-1];
      for(int i =0; i < visited_positions_last_40_time.size(); i++)
      {
        if((visited_positions_last_40_time[i]-last_pos).norm()<5)
        {
          adjacent_count_++;
        }
      }
      // std::cout<<"adjacent_count_"<<adjacent_count_<<std::endl;;
      if(adjacent_count_>15)
      {
        // std::cout<<"虽然近但是加进去 adjacent_count_: "<<adjacent_count_<<std::endl;
        ignore_flag = true;
      }
      ignore_flag = true;

      if (min_dist <= kAddNodeMinDist && !ignore_flag)
      {
        return min_dist_ind;
      }
      

      bool addpath_keypose_flag=false;
      std::vector<nav_msgs::msg::Path> path_vec_last;
      std::vector<nav_msgs::msg::Path> path_vec_min;
      std::vector<int> posible_last_ind;
      std::vector<int> posible_min_ind;
      // if(near_viewpoint_select && last_viewpoint_select && keypose_node_count>5 )//
      if( keypose_node_count>5 )
      {
        
        for(int i=last_keypose_dist_vec.size()-1;i>=0;i--)//遍历last的向量
        {
          start_position.x() = nodes_[last_keypose_ind_vec[i]].position_.x;
          start_position.y() = nodes_[last_keypose_ind_vec[i]].position_.y;
          start_position.z() = nodes_[last_keypose_ind_vec[i]].position_.z;
          end_position.x() = current_keypose_position_.x;
          end_position.y() = current_keypose_position_.y;
          end_position.z() = current_keypose_position_.z;
          nav_msgs::msg::Path path_in_between = uav_viewpoint_manager->GetViewPointShortestPath_no_warn(
                        start_position, end_position);
          if(path_in_between.poses.size()==0)
          {
            // std::cout<<"没法连接"<<std::endl;
            continue;
          }
          else{//找到了路径，弄他
            path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
            addpath_keypose_flag = true;
            path_vec_last.push_back(path_in_between);
            posible_last_ind.push_back(last_keypose_ind_vec[i]);
            // break;
          }
        }
        for(int i=min_dist_ind_vec.size()-1;i>=0;i--)//遍历last的向量
        {
          start_position.x() = nodes_[min_dist_ind_vec[i]].position_.x;
          start_position.y() = nodes_[min_dist_ind_vec[i]].position_.y;
          start_position.z() = nodes_[min_dist_ind_vec[i]].position_.z;
          end_position.x() = current_keypose_position_.x;
          end_position.y() = current_keypose_position_.y;
          end_position.z() = current_keypose_position_.z;
          nav_msgs::msg::Path path_in_between = uav_viewpoint_manager->GetViewPointShortestPath_no_warn(
                        start_position, end_position);
          if(path_in_between.poses.size()==0)
          {
            // std::cout<<"没法连接"<<std::endl;
            continue;
          }
          else{//找到了路径，弄他
            //去头去尾
            path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
            addpath_keypose_flag = true;
            path_vec_min.push_back(path_in_between);
            posible_min_ind.push_back(min_dist_ind_vec[i]);
            // break;
          }
        }
        
        if(addpath_keypose_flag)
        {
          double length_of_last_shortest=DBL_MAX;
          double length_of_min_shortest=DBL_MAX;
          nav_msgs::msg::Path path_in_between_last;
          nav_msgs::msg::Path path_in_between_min;
          int last_ind_;
          int min_ind_;
          //last
          for(int for_short_i=0;for_short_i<path_vec_last.size();for_short_i++)
          {
            double length=0;
            for(int for_short_h=0;for_short_h<path_vec_last[for_short_i].poses.size()-1;for_short_h++)
            {
              length = length +  misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_vec_last[for_short_i].poses[for_short_h].pose.position,
                                                                                          path_vec_last[for_short_i].poses[for_short_h+1].pose.position);
            }
            if(length<length_of_last_shortest)
            {
              length_of_last_shortest = length;
              path_in_between_last = path_vec_last[for_short_i];
              last_ind_ = posible_last_ind[for_short_i];
            }
          }
          //min
          for(int for_short_i=0;for_short_i<path_vec_min.size();for_short_i++)
          {
            double length=0;
            for(int for_short_h=0;for_short_h<path_vec_min[for_short_i].poses.size()-1;for_short_h++)
            {
              length = length +  misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_vec_min[for_short_i].poses[for_short_h].pose.position,
                                                                                          path_vec_min[for_short_i].poses[for_short_h+1].pose.position);
            }
            if(length<length_of_min_shortest)
            {
              length_of_min_shortest = length;
              path_in_between_min = path_vec_min[for_short_i];
              min_ind_ = posible_min_ind[for_short_i];
            }
          }
          int final_added_ind = last_keypose_ind;
          nav_msgs::msg::Path final_added_path;
          if(length_of_last_shortest<length_of_min_shortest)
          {
            final_added_ind = last_ind_;
            final_added_path = path_in_between_last;
          }
          else{
            final_added_ind = min_ind_;
            final_added_path = path_in_between_min;
          }
          
          start_position.x() = current_keypose_position_.x;
          start_position.y() = current_keypose_position_.y;
          start_position.z() = current_keypose_position_.z;
          end_position.x() = nodes_[final_added_ind].position_.x;
          end_position.y() = nodes_[final_added_ind].position_.y;
          end_position.z() = nodes_[final_added_ind].position_.z;
          start_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(start_position);
          end_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(end_position);
          // std::cout<<start_sub_in_occupancy.x()<<","<<start_sub_in_occupancy.y()<<","<<start_sub_in_occupancy.z()<<std::endl;
          // std::cout<<end_sub_in_occupancy.x()<<","<<end_sub_in_occupancy.y()<<","<<end_sub_in_occupancy.z()<<std::endl;
          // Eigen::Vector3i A = end_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->GetSize();
          // std::cout<<A.x()<<","<<A.y()<<","<<A.z()<<std::endl;
          if( rolling_occupancy_grid_->Get_occupancy_array_()->InRange(start_sub_in_occupancy)&&rolling_occupancy_grid_->Get_occupancy_array_()->InRange(end_sub_in_occupancy))
          {
            // std::cout<<"111"<<std::endl;
            rolling_occupancy_grid_->RayTraceHelper(end_sub_in_occupancy, start_sub_in_occupancy, ray_cast_cells);
            for(int h=0;h<ray_cast_cells.size();h++)//对于每个cells确定它是否是占据的
            {
              if(!rolling_occupancy_grid_->Get_rolling_grid_()->InRange(ray_cast_cells[h]))
              {

                success_flag=false;//这个没连接上，换下一个
                break;
              }
              
              array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(ray_cast_cells[h]);
              if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCell(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::OCCUPIED)
              {
                success_flag=false;//这个没连接上，换下一个
                break;
              }
              success_flag = true;
            }
          }
            double finaldist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(current_keypose_position_,
                                                                                              nodes_[final_added_ind].position_);
            AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, final_added_ind,
                      finaldist);
          // if(success_flag)//能直接相连就直接向量
          // {
          //   double finaldist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(current_keypose_position_,
          //                                                                                     nodes_[final_added_ind].position_);
          //   AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, final_added_ind,
          //             finaldist);
          //   // std::cout<<"直连"<<std::endl;
          // }
          // else{//否则就一个个连
          //   final_added_path.poses.erase(final_added_path.poses.begin()+final_added_path.poses.size()-1);
          //   final_added_path.poses.erase(final_added_path.poses.begin());
          //   if(final_added_path.poses.size()>0)
          //   {
          //     double dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(final_added_path.poses[0].pose.position,
          //                                                                                   nodes_[final_added_ind].position_);
          //     AddNodeAndEdge(final_added_path.poses[0].pose.position, nodes_.size(), current_keypose_id_, true, final_added_ind, dist_temp);
          //     if(final_added_path.poses.size()>1)
          //     {
          //       for(int h=1; h<final_added_path.poses.size();h++)//前后相互联系 add 和edge
          //       {
          //         dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(final_added_path.poses[h].pose.position,
          //                                                                                     final_added_path.poses[h-1].pose.position);
          //         AddNodeAndEdge(final_added_path.poses[h].pose.position, nodes_.size(), current_keypose_id_, true, nodes_.size()-1, dist_temp);
          //       }
          //     }
          //     dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(final_added_path.poses[final_added_path.poses.size()-1].pose.position,
          //                                                                                   current_keypose_position_);
          //     AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, nodes_.size()-1, dist_temp);
          //   }
          //   else{
          //     double dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[final_added_ind].position_,
          //                                                                                   current_keypose_position_);
          //     AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, final_added_ind,
          //             dist_temp);
          //   }
          //   std::cout<<"串联  "<<final_added_path.poses.size()<<std::endl;
          // }



        }
        else{
          // std::cout<<"放错机制失效一次--仅仅新keypose node 与last相连"<<std::endl;
          AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, last_keypose_ind,
                      last_keypose_dist);
        }
        newest_keypose_ind=nodes_.size()-1;
        // std::cout<<"周围5m内keypose："<<neigbour_keypose_count<<"   10-15数量："<<min_dist_vec_long.size()<<std::endl;
        if(neigbour_keypose_count>50)//定死了跟周围所有的5-15进行一次相连
        {
          count_time++;
          if(count_time>20)
          {
            count_time=0;
            std::cout<<"触发卡死机制，开始撒网自救！"<<std::endl;
            //   min_dist_vec_long.push_back(dist);
            // min_dist_ind_vec_long.push_back(i);
            int connected_num=0;
            for(int i=min_dist_ind_vec_long.size()-1;i>=0;i--)//遍历last的向量
            {
              
              if(connected_num>=15)
              {
                break;
              }
              start_position.x() = nodes_[min_dist_ind_vec_long[i]].position_.x;
              start_position.y() = nodes_[min_dist_ind_vec_long[i]].position_.y;
              start_position.z() = nodes_[min_dist_ind_vec_long[i]].position_.z;
              end_position.x() = current_keypose_position_.x;
              end_position.y() = current_keypose_position_.y;
              end_position.z() = current_keypose_position_.z;
              nav_msgs::msg::Path path_in_between = uav_viewpoint_manager->GetViewPointShortestPath_no_warn(
                            start_position, end_position);
              if(path_in_between.poses.size()==0)
              {
                // std::cout<<"没法连接"<<std::endl;
                continue;
              }
              else{//找到了路径，弄他
                //去头去尾
                path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
                // path_in_between.poses.erase(path_in_between.poses.begin()+path_in_between.poses.size()-1);
                // path_in_between.poses.erase(path_in_between.poses.begin());
                //第一个和lastkey相连
                double dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[0].pose.position,
                                                                                              nodes_[min_dist_ind_vec_long[i]].position_);
                AddNodeAndEdge(path_in_between.poses[0].pose.position, nodes_.size(), current_keypose_id_, true, min_dist_ind_vec_long[i], dist_temp);
                for(int h=1; h<path_in_between.poses.size();h++)//前后相互联系 add 和edge
                {
                  dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[h].pose.position,
                                                                                              path_in_between.poses[h-1].pose.position);
                  AddNodeAndEdge(path_in_between.poses[h].pose.position, nodes_.size(), current_keypose_id_, true, nodes_.size()-1, dist_temp);
                }
                dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(path_in_between.poses[path_in_between.poses.size()-1].pose.position,
                                                                                              current_keypose_position_);
                // AddNodeAndEdge(current_keypose_position_, nodes_.size(), current_keypose_id_, true, nodes_.size()-1, dist_temp);
                AddEdge(nodes_.size()-1,newest_keypose_ind,dist_temp);
              }
              connected_num++;
            }
          }
          
        }
        else{
          count_time=0;
        }
      }
      // If the last keypose is within range
      
      else if (last_keypose_dist < kAddEdgeToLastKeyposeDistThr && last_keypose_ind >= 0 &&
          last_keypose_ind < nodes_.size())
      {
        // Add edge to the last keypose node
        AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, last_keypose_ind,
                      last_keypose_dist);
      }
      else
      {
        // Add edge to the nearest node
        AddNodeAndEdge(current_keypose_position_, new_node_ind, current_keypose_id_, true, min_dist_ind, min_dist);
      }
      new_node_ind = newest_keypose_ind;
      
      // Check other nodes
      if (!in_range_node_indices.empty())
      {
        for (int idx = 0; idx < in_range_node_indices.size(); idx++)
        {
          int in_range_ind = in_range_node_indices[idx];
          if (in_range_ind >= 0 && in_range_ind < nodes_.size())
          {
              start_position.x() = nodes_[in_range_ind].position_.x;
              start_position.y() = nodes_[in_range_ind].position_.y;
              start_position.z() = nodes_[in_range_ind].position_.z;
              end_position.x() = nodes_[new_node_ind].position_.x;
              end_position.y() = nodes_[new_node_ind].position_.y;
              end_position.z() = nodes_[new_node_ind].position_.z;
              nav_msgs::msg::Path path_in_between = uav_viewpoint_manager->GetViewPointShortestPath_no_warn(
                            start_position, end_position);
              if(path_in_between.poses.size()==0)
              {
                // std::cout<<"没法连接"<<std::endl;
                continue;
              }
              else{//找到了路径，弄他
                //去头去尾
                path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
                double dist_temp = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[in_range_ind].position_,                                                                                              nodes_[new_node_ind].position_);
                AddEdge(in_range_ind,new_node_ind,dist_temp);

              }
          }
        }
      }
      // std::cout<<"keypose+普通点"<<std::endl;
      return new_node_ind;
    }
    else
    {
      // ROS_ERROR_STREAM("KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::AddKeyposeNode: Nearest keypose ind out of range: " << min_dist_ind);

      return new_node_ind;
    }
  }
}

bool KeyposeGraph::IsPositionReachable(const geometry_msgs::msg::Point& point, double dist_threshold)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < dist_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool KeyposeGraph::IsPositionReachable(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  GetClosestConnectedNodeIndAndDistance(point, closest_node_ind, closest_node_dist);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size() && closest_node_dist < kAddNonKeyposeNodeMinDist)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int KeyposeGraph::GetClosestNodeInd(const geometry_msgs::msg::Point& point)
{
  int node_ind = 0;
  double min_dist = DBL_MAX;
  GetClosestNodeIndAndDistance(point, node_ind, min_dist);
  return node_ind;
}

void KeyposeGraph::GetClosestNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
{
  node_ind = -1;
  dist = DBL_MAX;
  if (nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices, nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    // ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
    //                 << nodes_cloud_->points.size() << " nodes.");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::GetClosestNodeIndAndDistance: search for nearest neighbor failed with "
                    << nodes_cloud_->points.size() << " nodes.");  
    if (!nearest_neighbor_node_indices.empty())
    {
      // ROS_WARN_STREAM("Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
      RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "Nearest neighbor node Ind: " << nearest_neighbor_node_indices.front());
    }
    for (int i = 0; i < nodes_.size(); i++)
    {
      geometry_msgs::msg::Point node_position = nodes_[i].position_;
      double dist_to_query =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(point, node_position);
      if (dist_to_query < dist)
      {
        dist = dist_to_query;
        node_ind = i;
      }
    }
  }
}

void KeyposeGraph::GetClosestConnectedNodeIndAndDistance(const geometry_msgs::msg::Point& point, int& node_ind, double& dist)
{
  if (connected_nodes_cloud_->points.empty())
  {
    node_ind = -1;
    dist = DBL_MAX;
    return;
  }
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = point.z;
  std::vector<int> nearest_neighbor_node_indices(1);
  std::vector<float> nearest_neighbor_squared_dist(1);
  kdtree_connected_nodes_->nearestKSearch(search_point, 1, nearest_neighbor_node_indices,
                                          nearest_neighbor_squared_dist);
  if (!nearest_neighbor_node_indices.empty() && nearest_neighbor_node_indices.front() >= 0 &&
      nearest_neighbor_node_indices.front() < connected_nodes_cloud_->points.size())
  {
    node_ind = static_cast<int>(connected_nodes_cloud_->points[nearest_neighbor_node_indices.front()].intensity);
    dist = sqrt(nearest_neighbor_squared_dist.front());
  }
  else
  {
    // ROS_WARN_STREAM("KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
    //                 << connected_nodes_cloud_->points.size() << " connected nodes.");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::GetClosestNodeInd: search for nearest neighbor failed with "
                    << connected_nodes_cloud_->points.size() << " connected nodes.");           
    node_ind = -1;
    dist = 0;
  }
}

int KeyposeGraph::GetClosestKeyposeID(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].keypose_id_;
  }
  else
  {
    return -1;
  }
}

geometry_msgs::msg::Point KeyposeGraph::GetClosestNodePosition(const geometry_msgs::msg::Point& point)
{
  int closest_node_ind = GetClosestNodeInd(point);
  if (closest_node_ind >= 0 && closest_node_ind < nodes_.size())
  {
    return nodes_[closest_node_ind].position_;
  }
  else
  {
    geometry_msgs::msg::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    return point;
  }
}

bool KeyposeGraph::GetShortestPathWithMaxLength(const geometry_msgs::msg::Point& start_point,
                                                const geometry_msgs::msg::Point& target_point, double max_path_length,
                                                bool get_path, nav_msgs::msg::Path& path)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (allow_vertical_edge_)
    {
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist = DBL_MAX;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(graph_, dist_, node_positions, from_idx, to_idx,
                                                                get_path, path_indices, shortest_dist, max_path_length);
  if (found_path && get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return found_path;
}

double KeyposeGraph::GetShortestPath(const geometry_msgs::msg::Point& start_point, const geometry_msgs::msg::Point& target_point,
                                     bool get_path, nav_msgs::msg::Path& path, bool use_connected_nodes)
{
  if (nodes_.size() < 2)
  {
    if (get_path)
    {
      geometry_msgs::msg::PoseStamped start_pose;
      start_pose.pose.position = start_point;
      geometry_msgs::msg::PoseStamped target_pose;
      target_pose.pose.position = target_point;
      path.poses.push_back(start_pose);
      path.poses.push_back(target_pose);
    }
    return misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(start_point, target_point);
  }
  int from_idx = 0;
  int to_idx = 0;
  double min_dist_to_start = DBL_MAX;
  double min_dist_to_target = DBL_MAX;
  for (int i = 0; i < nodes_.size(); i++)
  {
    if (use_connected_nodes && !nodes_[i].is_connected_)
    {
      continue;
    }
    if (allow_vertical_edge_)
    {
      double dist_to_start =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
      double dist_to_target =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
      if (dist_to_start < min_dist_to_start)
      {
        min_dist_to_start = dist_to_start;
        from_idx = i;
      }
      if (dist_to_target < min_dist_to_target)
      {
        min_dist_to_target = dist_to_target;
        to_idx = i;
      }
    }
    else
    {
      double z_diff_to_start = std::abs(nodes_[i].position_.z - start_point.z);
      double z_diff_to_target = std::abs(nodes_[i].position_.z - target_point.z);
      // TODO: parameterize this
      if (z_diff_to_start < 1.5)
      {
        double xy_dist_to_start =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, start_point);
        if (xy_dist_to_start < min_dist_to_start)
        {
          min_dist_to_start = xy_dist_to_start;
          from_idx = i;
        }
      }
      if (z_diff_to_target < 1.5)
      {
        double xy_dist_to_target =
            misc_utils_ns::PointXYDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(nodes_[i].position_, target_point);
        if (xy_dist_to_target < min_dist_to_target)
        {
          min_dist_to_target = xy_dist_to_target;
          to_idx = i;
        }
      }
    }
  }

  std::vector<geometry_msgs::msg::Point> node_positions;
  for (int i = 0; i < nodes_.size(); i++)
  {
    node_positions.push_back(nodes_[i].position_);
  }
  std::vector<int> path_indices;
  double shortest_dist =
      misc_utils_ns::AStarSearch(graph_, dist_, node_positions, from_idx, to_idx, get_path, path_indices);
  if (get_path)
  {
    path.poses.clear();
    for (const auto& ind : path_indices)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = nodes_[ind].position_;
      pose.pose.orientation.w = nodes_[ind].keypose_id_;
      pose.pose.orientation.x = ind;
      path.poses.push_back(pose);
    }
  }

  return shortest_dist;
}

geometry_msgs::msg::Point KeyposeGraph::GetFirstKeyposePosition()
{
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

geometry_msgs::msg::Point KeyposeGraph::GetKeyposePosition(int keypose_id)
{
  geometry_msgs::msg::Point point;
  point.x = 0;
  point.y = 0;
  point.z = 0;
  for (const auto& node : nodes_)
  {
    if (node.keypose_id_ == keypose_id)
    {
      point = node.position_;
      break;
    }
  }
  return point;
}

void KeyposeGraph::GetKeyposePositions(std::vector<Eigen::Vector3d>& positions)
{
  positions.clear();
  for (const auto& node : nodes_)
  {
    if (node.IsKeypose())
    {
      Eigen::Vector3d position(node.position_.x, node.position_.y, node.position_.z);
      positions.push_back(position);
    }
  }
}

geometry_msgs::msg::Point KeyposeGraph::GetNodePosition(int node_ind)
{
  geometry_msgs::msg::Point node_position;
  node_position.x = 0;
  node_position.y = 0;
  node_position.z = 0;
  if (node_ind >= 0 && node_ind < nodes_.size())
  {
    node_position = nodes_[node_ind].position_;
  }
  else
  {
    // ROS_WARN_STREAM("KeyposeGraph::GetNodePosition: node_ind " << node_ind << " out of bound [0, " << nodes_.size() - 1
    //                                                            << "]");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "KeyposeGraph::GetNodePosition: node_ind " << node_ind << " out of bound [0, " << nodes_.size() - 1
                                                               << "]");                                                          
  }
  return node_position;
}

}  // namespace keypose_graph_ns
