/**
 * @file viewpoint_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 * 
 * @modified_by Huazhang Zhu (22210720332@m.fudan.edu.cn)
 * @date 2023-08-17
 */
#include "uav_viewpoint_manager/viewpoint_manager.h"

namespace uav_viewpoint_manager_ns
{
bool ViewPointManagerParameter::ReadParameters(std::shared_ptr<rclcpp::Node>& nh)
{
  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);
  visited_rad = 3;//机器人在移动过程中对半径3里面的viewpoint都设置成visitd true
  dimension_ = 3;
  big_box_size_.x()=5;
  big_box_size_.y()=5;
  big_box_size_.z()=3;
  middle_box_size_.x()=8;
  middle_box_size_.y()=8;
  middle_box_size_.z()=3;
  middle_box_resolution.x()=1.2;
  middle_box_resolution.y()=1.2;
  middle_box_resolution.z()=1.2;
  
  
  
  visited_rad = misc_utils_ns::getParam<double>(nh, "visitd_radius", 3);
  visited_rad_z = misc_utils_ns::getParam<double>(nh, "visitd_radius_z", 1.2);
  visited_rad_copy = visited_rad;
  // kNumber.x() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_x", 80);
  // kNumber.y() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_y", 80);
  // kNumber.z() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_z", 40);
  
  for(int i=0;i<3;i++)
  {
    kRolloverStepsize(i) = 1;
  }
  first_frame_grace = misc_utils_ns::getParam<bool>(nh, "viewpoint_manager/first_frame_grace", false);
  disable_outsider_viewpoints = misc_utils_ns::getParam<bool>(nh, "viewpoint_manager/disable_outsider_viewpoints", false);
  smaller_visited_radius = misc_utils_ns::getParam<bool>(nh, "viewpoint_manager/smaller_visited_radius", true);
  show_time = misc_utils_ns::getParam<bool>(nh, "show_time", false);

  std::cout<<"first_frame_grace: "<<first_frame_grace<<std::endl;
  std::cout<<"disable_outsider_viewpoints: "<<disable_outsider_viewpoints<<std::endl;
  std::cout<<"smaller_visited_radius: "<<smaller_visited_radius<<std::endl;
  std::cout<<"show_time: "<<show_time<<std::endl;
  //TODO 这个resolution的意思是viewpoint点之间的间隔，跟上面那个middle_box_resolution是一个意义
  kResolution.x() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 0.5);
  kResolution.y() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_y", 0.5);
  kResolution.z() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_z", 0.5);
  middle_box_resolution.x() = kResolution.x();
  middle_box_resolution.y() = kResolution.y();
  middle_box_resolution.z() = kResolution.z();
  std::cout<<"viewpoint_manager/resolution_x: "<<middle_box_resolution.x()<<std::endl;
  std::cout<<"viewpoint_manager/resolution_y: "<<middle_box_resolution.y()<<std::endl;
  std::cout<<"viewpoint_manager/resolution_z: "<<middle_box_resolution.z()<<std::endl;
  // std::cout<<<<<<std::endl;
  big_box_size_.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/big_box_size_x", 5);
  big_box_size_.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/big_box_size_y", 5);
  big_box_size_.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/big_box_size_z", 3);
  middle_box_size_.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/min_box_size_x", 8);
  middle_box_size_.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/min_box_size_y", 8);
  middle_box_size_.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/min_box_size_z", 3);
  incollision_barrier_level = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/incollision_barrier_level", 0);
  std::cout<<"incollision_barrier_level: "<<incollision_barrier_level<<std::endl;
  std::cout<<"viewpoint_manager/big_box_size_x: "<<big_box_size_.x()<<std::endl;
  std::cout<<"viewpoint_manager/big_box_size_y: "<<big_box_size_.y()<<std::endl;
  std::cout<<"viewpoint_manager/big_box_size_z: "<<big_box_size_.z()<<std::endl;
  std::cout<<"viewpoint_manager/min_box_size_x: "<<middle_box_size_.x()<<std::endl;
  std::cout<<"viewpoint_manager/min_box_size_y: "<<middle_box_size_.y()<<std::endl;
  std::cout<<"viewpoint_manager/min_box_size_z: "<<middle_box_size_.z()<<std::endl;
  big_box_resolution.x()=middle_box_size_.x() * kResolution.x();
  big_box_resolution.y()=middle_box_size_.y() * kResolution.y();
  big_box_resolution.z()=middle_box_size_.z() * kResolution.z();//大盒子的resolution = 小格子的resolution * size
  middle_box_viewpoint_num = middle_box_size_.x()*middle_box_size_.y()*middle_box_size_.z();
  viewpoint_num = big_box_size_.x()*big_box_size_.y()*big_box_size_.z()*middle_box_size_.x()*middle_box_size_.y()*middle_box_size_.z();
  kNumber.x() = big_box_size_.x()*middle_box_size_.x();
  kNumber.y() = big_box_size_.y()*middle_box_size_.y();
  kNumber.z() = big_box_size_.z()*middle_box_size_.z();;
  kViewPointNumber = kNumber.x() * kNumber.y() * kNumber.z();
  length_size.x() = big_box_size_.x() * middle_box_size_.x() * middle_box_resolution.x();
  length_size.y() = big_box_size_.y() * middle_box_size_.y() * middle_box_resolution.y();
  length_size.z() = big_box_size_.z() * middle_box_size_.z() * middle_box_resolution.z();
  if(middle_box_size_.x() * middle_box_resolution.x() > middle_box_size_.z() * middle_box_resolution.z())
  {
    middle_box_bigger_side = middle_box_size_.x() * middle_box_resolution.x();
  }
  else{
    middle_box_bigger_side = middle_box_size_.z() * middle_box_resolution.z();
  }
  
  big_box_bigger_side = big_box_size_.x() * middle_box_bigger_side;
  middle_box_side_.x() = middle_box_size_.x() * middle_box_resolution.x();
  middle_box_side_.y() = middle_box_size_.y() * middle_box_resolution.y();
  middle_box_side_.z() = middle_box_size_.z() * middle_box_resolution.z();


  kViewPointCollisionMargin = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMargin", 0.5);
  kViewPointCollisionMarginZPlus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZPlus", 0.5);
  kViewPointCollisionMarginZMinus = misc_utils_ns::getParam<double>(nh, "kViewPointCollisionMarginZMinus", 0.5);
  kCollisionGridZScale = misc_utils_ns::getParam<double>(nh, "kCollisionGridZScale", 2.0);
  kCollisionGridResolution.x() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionX", 0.5);
  kCollisionGridResolution.y() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionY", 0.5);
  kCollisionGridResolution.z() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionZ", 0.5);
  kLineOfSightStopAtNearestObstacle = misc_utils_ns::getParam<bool>(nh, "kLineOfSightStopAtNearestObstacle", true);
  kCheckDynamicObstacleCollision = misc_utils_ns::getParam<bool>(nh, "kCheckDynamicObstacleCollision", true);
  kCollisionFrameCountMax = misc_utils_ns::getParam<int>(nh, "kCollisionFrameCountMax", 3);
  kViewPointHeightFromTerrain = misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrain", 0.75);
  kViewPointHeightFromTerrainChangeThreshold =
      misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrainChangeThreshold", 0.6);
  
  kCollisionPointThr = misc_utils_ns::getParam<int>(nh, "kCollisionPointThr", 3);

  kCollisionGridSize = Eigen::Vector3i::Ones();
  for (int i = 0; i < dimension_; i++)
  {
    kCollisionGridSize(i) =
        ceil((kNumber(i) * kResolution(i) + kViewPointCollisionMargin * 2) / kCollisionGridResolution(i));
  }

  kCoverageOcclusionThr = misc_utils_ns::getParam<double>(nh, "kCoverageOcclusionThr", 1.0);
  kCoverageDilationRadius = misc_utils_ns::getParam<double>(nh, "kCoverageDilationRadius", 1.0);
  kCoveragePointCloudResolution = misc_utils_ns::getParam<double>(nh, "kSurfaceCloudDwzLeafSize", 1.0);
  kSensorRange = misc_utils_ns::getParam<double>(nh, "kSensorRange", 10.0);
  kNeighborRange = misc_utils_ns::getParam<double>(nh, "kNeighborRange", 3.0);

  kVerticalFOVRatio = tan(M_PI / 15);
  kDiffZMax = kSensorRange * kVerticalFOVRatio;
  kInFovXYDistThreshold = 3 * (kCoveragePointCloudResolution / 2) / tan(M_PI / 15);
  kInFovZDiffThreshold = 3 * kCoveragePointCloudResolution;

  return true;
}

// ViewPointManager::ViewPointManager(std::shared_ptr<rclcpp::Node>& nh) : initialized_(false)
// {
//   vp_.ReadParameters(nh);
//   kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
//   kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
//   viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
//   viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

//   grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.kNumber);
//   origin_ = Eigen::Vector3d::Zero();

//   viewpoints_.resize(vp_.kViewPointNumber);
//   for (int x = 0; x < vp_.kNumber.x(); x++)
//   {
//     for (int y = 0; y < vp_.kNumber.y(); y++)
//     {
//       for (int z = 0; z < vp_.kNumber.z(); z++)
//       {
//         Eigen::Vector3i sub(x, y, z);
//         int ind = grid_->Sub2Ind(sub);
//         viewpoints_[ind] = viewpoint_ns::ViewPoint();
//       }
//     }
//   }
  
//   graph_index_map_.resize(vp_.kViewPointNumber);
//   for (auto& ind : graph_index_map_)
//   {
//     ind = -1;
//   }

//   ComputeConnectedNeighborIndices();
//   ComputeInRangeNeighborIndices();
//   GetCollisionCorrespondence();

//   local_planning_horizon_size_ = Eigen::Vector3d::Zero();
//   for (int i = 0; i < vp_.dimension_; i++)
//   {
//     local_planning_horizon_size_(i) = vp_.kNumber(i) * vp_.kResolution(i);
//   }
// }
ViewPointManager::ViewPointManager(std::shared_ptr<rclcpp::Node>& nh) : initialized_(false), Print_visited_points_bool(false), update_Robot_Hier_ind_(false),relocation_(false)
{
  //总的来说，如果cells用指针的话先要往向量里push_back指针，然后实例化对象让指针再指向对象
  //①大方块的实例化、出现了大方块cells的空缺，此时用指针填满cells的空缺(至此工作在构造函数完成)，然后要实例化中方块赋值让cells的每个指针分别指向对应的中方块
  //②中方块实例化的部分要从这个脚本做，  时候出现了cells的空缺， 要实例化Viwpoint的类型让最小的cells指向它
  print_visited_points =
      nh->create_subscription<std_msgs::msg::String>("/print_visited_points", 5, std::bind(&ViewPointManager::Print_visited_points, this, std::placeholders::_1));  
  vp_.ReadParameters(nh);
  check_collision_margin = misc_utils_ns::getParam<int>(nh, "check_collision_margin", 5);
  intervel_radius = misc_utils_ns::getParam<double>(nh, "intervel_radius", 1.9);
  para_visited_radius = misc_utils_ns::getParam<double>(nh, "para_visited_radius", 2.4);
  para_visited_radius_z = misc_utils_ns::getParam<double>(nh, "para_visited_radius_z", 2.4);
  position_add_high_for_guv = misc_utils_ns::getParam<double>(nh, "position_add_high_for_guv", 0.0);
  least_visible_unknown_num = misc_utils_ns::getParam<int>(nh, "least_visible_unknown_num", 29);
  least_visible_free_num = misc_utils_ns::getParam<int>(nh, "least_visible_free_num", 29);
  Set_robot_neigbour_connected = misc_utils_ns::getParam<bool>(nh, "Set_robot_neigbour_connected", false);
  
  in_collision_viewpoint_get_connected_viewpoint_dis = misc_utils_ns::getParam<double>(nh, "in_collision_viewpoint_get_connected_viewpoint_dis", 1.1);
  search_radius_thr = misc_utils_ns::getParam<double>(nh, "search_radius_thr", 0.9);
  std::cout<<"intervel_radius:"<<intervel_radius<<std::endl;
  std::cout<<"para_visited_radius:"<<para_visited_radius<<std::endl;
  std::cout<<"para_visited_radius_z:"<<para_visited_radius_z<<std::endl;
  std::cout<<"position_add_high_for_guv:"<<position_add_high_for_guv<<std::endl;
  std::cout<<"least_visible_unknown_num:"<<least_visible_unknown_num<<std::endl;
  std::cout<<"least_visible_free_num:"<<least_visible_free_num<<std::endl;
  std::cout<<"Set_robot_neigbour_connected:"<<Set_robot_neigbour_connected<<std::endl;
  std::cout<<"check_collision_margin:"<<check_collision_margin<<std::endl;
  std::cout<<"in_collision_viewpoint_get_connected_viewpoint_dis:"<<in_collision_viewpoint_get_connected_viewpoint_dis<<std::endl;
  std::cout<<"search_radius_thr:"<<search_radius_thr<<std::endl;
  
  viewpoint_all_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("all_uav_viewpoint",10);//可视化
  viewpoint_free_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("free_uav_viewpoint",10);//可视化
  viewpoint_collision_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("viewpoint_collision_pub",10);//可视化
  viewpoint_real_collision_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("viewpoint_real_collision_pub",10);//可视化
  viewpoint_connected_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("connected_uav_viewpoint",10);//可视化
  viewpoint_connected_inexploringarea_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("viewpoint_connected_inexploringarea_",10);//可视化
  
  viewpoint_visited_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("visited_uav_viewpoint",10);//可视化
  viewpoints_in_lineofsight_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("viewpoints_in_lineofsight_uav",10);//可视化
  visited_positions_pub = nh->create_publisher<sensor_msgs::msg::PointCloud2>("visited_positions",10);//可视化
  robot_point = nh->create_publisher<geometry_msgs::msg::PointStamped>("viewpoit_robot_point",10);//可视化
  sub_sub = nh->create_subscription<geometry_msgs::msg::PoseStamped>("receive_sub",10,std::bind(&ViewPointManager::sub_deal, this, std::placeholders::_1));
  sub_robot_pos = nh->create_subscription<geometry_msgs::msg::PoseStamped>("receive_robot_pos",5,std::bind(&ViewPointManager::sub_robot_pos_deal, this, std::placeholders::_1));
  Eigen::Vector3d big_box_origin_(0, 0, 0);
  std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>> mid_cell_temp;
  big_box = std::make_unique<grid_ns::Grid<std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>>>>(
      vp_.big_box_size_, mid_cell_temp, big_box_origin_, vp_.big_box_resolution, 3);
  grid_  = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.big_box_size_);
  std::shared_ptr<viewpoint_ns::ViewPoint> little_cell_temp;//跟push_back配合，把指针复制后压进cells里
  for (int i = 0; i < big_box->GetCellNumber(); i++)
  {
    //EXPL2 中方体的实例化
    big_box->GetCell(i) = std::shared_ptr<grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>>
                      (new grid_ns::Grid<std::shared_ptr<viewpoint_ns::ViewPoint>>(vp_.middle_box_size_,little_cell_temp,big_box->Sub2Pos_ld_point((big_box->Ind2Sub(i))),vp_.middle_box_resolution,3));
    //EXPL2 小方体的实例化
    for(int h=0;h<big_box->GetCell(i)->GetCellNumber();h++)
    {
      big_box->GetCell(i)->GetCell(h) = std::shared_ptr<viewpoint_ns::ViewPoint>(new viewpoint_ns::ViewPoint);
    }
  }
  kdtree_occupy_unknow_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  
  if(vp_.incollision_barrier_level>0)
  {
    idx_addon_check_collision.push_back(Eigen::Vector3i(-1,0,0));
    idx_addon_check_collision.push_back(Eigen::Vector3i(1,0,0));
    idx_addon_check_collision.push_back(Eigen::Vector3i(0,-1,0));
    idx_addon_check_collision.push_back(Eigen::Vector3i(0,1,0));
    idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,-1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(1,1,1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(-1,1,1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(1,-1,1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(1,1,-1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(-1,-1,-1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(1,-1,-1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(-1,1,-1));
    idx_addon_check_collision.push_back(Eigen::Vector3i(-1,-1,1));

    if(vp_.incollision_barrier_level>1)
    {
      idx_addon_check_collision.push_back(Eigen::Vector3i(-2,0,0));
      idx_addon_check_collision.push_back(Eigen::Vector3i(2,0,0));
      idx_addon_check_collision.push_back(Eigen::Vector3i(0,-2,0));
      idx_addon_check_collision.push_back(Eigen::Vector3i(0,2,0));
      idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,-2));
      idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,2));
      if(vp_.incollision_barrier_level>2)
      {
        idx_addon_check_collision.push_back(Eigen::Vector3i(-3,0,0));
        idx_addon_check_collision.push_back(Eigen::Vector3i(3,0,0));
        idx_addon_check_collision.push_back(Eigen::Vector3i(0,-3,0));
        idx_addon_check_collision.push_back(Eigen::Vector3i(0,3,0));
        idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,-3));
        idx_addon_check_collision.push_back(Eigen::Vector3i(0,0,3));
      }
    }
    
  }


  Compute_Uav_Viewpoints_NeighborIndices();
  // kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  // kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  // viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.kNumber);
  // origin_ = Eigen::Vector3d::Zero();

  // viewpoints_.resize(vp_.kViewPointNumber);
  // for (int x = 0; x < vp_.kNumber.x(); x++)
  // {
  //   for (int y = 0; y < vp_.kNumber.y(); y++)
  //   {
  //     for (int z = 0; z < vp_.kNumber.z(); z++)
  //     {
  //       Eigen::Vector3i sub(x, y, z);
  //       int ind = grid_->Sub2Ind(sub);
  //       viewpoints_[ind] = viewpoint_ns::ViewPoint();
  //     }
  //   }
  // }

  graph_index_map_.resize(vp_.viewpoint_num);
  for (auto& ind : graph_index_map_)
  {
    ind = -1;
  }

  // ComputeConnectedNeighborIndices();
  // ComputeInRangeNeighborIndices();
  // GetCollisionCorrespondence();

  // local_planning_horizon_size_ = Eigen::Vector3d::Zero();
  // for (int i = 0; i < vp_.dimension_; i++)
  // {
  //   local_planning_horizon_size_(i) = vp_.kNumber(i) * vp_.kResolution(i);
  // }
  //   Eigen::Vector3d A(1,1,1);
  //   UpdateRobotPosition(A);
  // // Eigen::Vector3d B(1,1,1);
  // int tem1,tem2;
  // Eigen::Vector3i B = Hierarchy_Pos_2_Hierarchy_sub(A);
  // Hierarchy_sub_2_midbox_ind_and_litbox_ind(B,tem1,tem2);
  // int array_ind = grid_->GetArrayInd(tem1);
  // geometry_msgs::msg::Point pt = big_box->GetCell(array_ind)->GetCell(tem2)->GetPosition();
  // Eigen::Vector3d C = Hierarchy_sub_2Hierarchy_Pos_(B);
  //   std::cout<<"("<<origin_.x()<<","<<origin_.y()<<","<<origin_.z()<<"("<<A.x()<<","<<A.y()<<","<<A.z()<<")"<<" Hie_sub:("<<B.x()<<","<<B.y()<<","<<B.z()<<") "<<
  //   "mid_sub:("<<big_box->Ind2Sub(tem1).x()<<","<<big_box->Ind2Sub(tem1).y()<<","<<big_box->Ind2Sub(tem1).z()<<
  //   ") litt_sub:("<<big_box->GetCell(0)->Ind2Sub(tem2).x()<<","<<big_box->GetCell(0)->Ind2Sub(tem2).y()<<","<<big_box->GetCell(0)->Ind2Sub(tem2).z()<<
  //   ")  position"<<"("<<pt.x<<","<<pt.y<<","<<pt.z<<")  "<<
  //   "("<<C.x()<<","<<C.y()<<","<<C.z()<<")"<<std::endl;

  // Eigen::Vector3i AA(3,1,1);
  // Eigen::Vector3i BB(6,3,1);
  // pt = big_box->GetCell(grid_->GetArrayInd(big_box->Sub2Ind(AA)))->GetCell(big_box->GetCell(0)->Sub2Ind(BB))->GetPosition();
  // std::cout<<"111"<<std::endl;
  // midbox_ind_and_litbox_ind_2_Hierarchy_sub_(B,big_box->Sub2Ind(AA),big_box->GetCell(0)->Sub2Ind(BB));
  // C = Hierarchy_sub_2Hierarchy_Pos_(B);
  // std::cout<<")  position"<<"("<<pt.x<<","<<pt.y<<","<<pt.z<<")  "<<"hiePos  ("<<C.x()<<","<<C.y()<<","<<C.z()<<")"<<std::endl;
}
void ViewPointManager::Compute_Uav_Viewpoints_NeighborIndices()
{
  uav_viewpoints_neighbor_indices_.resize(vp_.viewpoint_num);
  uav_viewpoint_neighbor_dist_.resize(vp_.viewpoint_num);
  // connected_neighbor_dist_.resize(vp_.kViewPointNumber);

  std::vector<Eigen::Vector3i> idx_addon;
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (x == 0 && y == 0 && z == 0)
          continue;
        idx_addon.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }
  //M? 验证是否成功
  for (int x = 0; x < vp_.big_box_size_.x()*vp_.middle_box_size_.x(); x++)
  {
    for (int y = 0; y < vp_.big_box_size_.y()*vp_.middle_box_size_.y(); y++)
    {
      for (int z = 0; z < vp_.big_box_size_.z()*vp_.middle_box_size_.z(); z++)
      {
        Eigen::Vector3i Hierarchy_sub(x, y, z);
        int Hierarchy_ind = Hierarchy_sub_2_Hierarchy_ind(Hierarchy_sub);
        for (int i = 0; i < idx_addon.size(); i++)
        {
          Eigen::Vector3i neighbor_Hierarchy_sub = Hierarchy_sub + idx_addon[i];
          if (Hierarchy_InRange(neighbor_Hierarchy_sub))
          {
            uav_viewpoints_neighbor_indices_[Hierarchy_ind].push_back(Hierarchy_sub_2_Hierarchy_ind(neighbor_Hierarchy_sub));
            
            double dist = sqrt(vp_.middle_box_resolution.x() * vp_.middle_box_resolution.x() * std::abs(idx_addon[i].x()) +
                               vp_.middle_box_resolution.y() * vp_.middle_box_resolution.y() * std::abs(idx_addon[i].y()) +
                               vp_.middle_box_resolution.z() * vp_.middle_box_resolution.z() * std::abs(idx_addon[i].z()));
            uav_viewpoint_neighbor_dist_[Hierarchy_ind].push_back(dist);
          }
        }
      }
    }
  }
//   std::cout<<"idx_addon.size()"<<idx_addon.size()<<std::endl;
// std::cout<<"uav_viewpoints_neighbor_indices_"<<uav_viewpoints_neighbor_indices_.size()<<std::endl;
// std::cout<<"uav_viewpoints_neighbor_indices_[0]"<<uav_viewpoints_neighbor_indices_[0].size()<<std::endl;
// for(int i=0;i<uav_viewpoints_neighbor_indices_[0].size();i++)
// {
//   Eigen::Vector3i A = Hierarchy_ind_2_Hierarchy_sub(uav_viewpoints_neighbor_indices_[0][i]);
//   std::cout<<"("<<A.x()<<","<<A.y()<<","<<A.z()<<")"<<std::endl;
// }//
// Eigen::Vector3i A(0,0,0);
// Eigen::Vector3i B(1,1,1);
// std::cout<<"A ind:"<<Hierarchy_sub_2_Hierarchy_ind(A)<<std::endl;
// std::cout<<"B ind:"<<Hierarchy_sub_2_Hierarchy_ind(B)<<std::endl;
// Eigen::Vector3i C = Hierarchy_ind_2_Hierarchy_sub(Hierarchy_sub_2_Hierarchy_ind(A));
// Eigen::Vector3i D = Hierarchy_ind_2_Hierarchy_sub(Hierarchy_sub_2_Hierarchy_ind(B));
// std::cout<<"("<<C.x()<<","<<C.y()<<","<<C.y()<<")"<<std::endl;
// std::cout<<"("<<D.x()<<","<<D.y()<<","<<D.y()<<")"<<std::endl;
}


// void ViewPointManager::ComputeConnectedNeighborIndices()
// {
//   connected_neighbor_indices_.resize(vp_.kViewPointNumber);
//   connected_neighbor_dist_.resize(vp_.kViewPointNumber);

//   std::vector<Eigen::Vector3i> idx_addon;
//   for (int x = -1; x <= 1; x++)
//   {
//     for (int y = -1; y <= 1; y++)
//     {
//       for (int z = -1; z <= 1; z++)
//       {
//         if (x == 0 && y == 0 && z == 0)
//           continue;
//         idx_addon.push_back(Eigen::Vector3i(x, y, z));
//       }
//     }
//   }

//   for (int x = 0; x < vp_.kNumber.x(); x++)
//   {
//     for (int y = 0; y < vp_.kNumber.y(); y++)
//     {
//       for (int z = 0; z < vp_.kNumber.z(); z++)
//       {
//         Eigen::Vector3i sub(x, y, z);
//         int ind = grid_->Sub2Ind(sub);
//         for (int i = 0; i < idx_addon.size(); i++)
//         {
//           Eigen::Vector3i neighbor_sub = sub + idx_addon[i];
//           if (grid_->InRange(neighbor_sub))
//           {
//             connected_neighbor_indices_[ind].push_back(grid_->Sub2Ind(neighbor_sub));
//             double dist = sqrt(vp_.kResolution.x() * vp_.kResolution.x() * std::abs(idx_addon[i].x()) +
//                                vp_.kResolution.y() * vp_.kResolution.y() * std::abs(idx_addon[i].y()) +
//                                vp_.kResolution.z() * vp_.kResolution.z() * std::abs(idx_addon[i].z()));
//             connected_neighbor_dist_[ind].push_back(dist);
//           }
//         }
//       }
//     }
//   }
// }

void ViewPointManager::ComputeInRangeNeighborIndices()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    Eigen::Vector3i sub = grid_->Ind2Sub(i);
    pcl::PointXYZ point;
    point.x = sub.x() * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
    point.y = sub.y() * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
    point.z = sub.z() * vp_.kResolution.z() + vp_.kResolution.z() / 2.0;
    cloud->points.push_back(point);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree =
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree->setInputCloud(cloud);
  in_range_neighbor_indices_.resize(vp_.kViewPointNumber);
  std::vector<int> in_range_indices;
  std::vector<float> in_range_sqdist;
  for (int i = 0; i < in_range_neighbor_indices_.size(); i++)
  {
    pcl::PointXYZ point = cloud->points[i];
    kdtree->radiusSearch(point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
    for (const auto& ind : in_range_indices)
    {
      in_range_neighbor_indices_[i].push_back(ind);
    }
  }
}

void ViewPointManager::GetCollisionCorrespondence()
{
  misc_utils_ns::Timer timer("get collision grid correspondence");
  timer.Start();

  collision_grid_origin_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    collision_grid_origin_(i) -= vp_.kViewPointCollisionMargin;
  }
  std::vector<int> viewpoint_index_correspondence;
  collision_grid_ = std::make_unique<grid_ns::Grid<std::vector<int>>>(
      vp_.kCollisionGridSize, viewpoint_index_correspondence, collision_grid_origin_, vp_.kCollisionGridResolution, 2);
  collision_point_count_.resize(collision_grid_->GetCellNumber(), 0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  // Get viewpoint cloud
  // int count = 0;
  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
        pcl::PointXYZI point;
        point.x = (x + 0.5) * vp_.kResolution.x();
        point.y = (y + 0.5) * vp_.kResolution.y();
        point.z = (z + 0.5) * vp_.kResolution.z();
        point.z *= vp_.kCollisionGridZScale;
        point.intensity = ind;
        viewpoint_cloud->points.push_back(point);
      }
    }
  }
  // std::cout << "computing collision grid viewpoint cloud size: " << viewpoint_cloud->points.size() << std::endl;
  kdtree->setInputCloud(viewpoint_cloud);
  std::vector<int> nearby_viewpoint_indices;
  std::vector<float> nearby_viewpoint_sqdist;
  int count = 0;
  for (int x = 0; x < vp_.kCollisionGridSize.x(); x++)
  {
    for (int y = 0; y < vp_.kCollisionGridSize.y(); y++)
    {
      for (int z = 0; z < vp_.kCollisionGridSize.z(); z++)
      {
        Eigen::Vector3d query_point_position = collision_grid_->Sub2Pos(x, y, z);
        pcl::PointXYZI query_point;
        query_point.x = query_point_position.x();
        query_point.y = query_point_position.y();
        query_point.z = query_point_position.z();
        query_point.z *= vp_.kCollisionGridZScale;
        kdtree->radiusSearch(query_point, vp_.kViewPointCollisionMargin, nearby_viewpoint_indices,
                             nearby_viewpoint_sqdist);
        int grid_ind = collision_grid_->Sub2Ind(x, y, z);
        for (int i = 0; i < nearby_viewpoint_indices.size(); i++)
        {
          int ind = nearby_viewpoint_indices[i];
          int viewpoint_ind = (int)(viewpoint_cloud->points[ind].intensity);
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
          collision_grid_->GetCell(grid_ind).push_back(viewpoint_ind);
        }
      }
    }
  }

  timer.Stop(false);
}

bool ViewPointManager::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  robot_position_ = robot_position;
  if (!initialized_)
  {
    initialized_ = true;
    UpdateOrigin();
    geometry_msgs::msg::Point ini_position;
    for(int mid_box_ind=0; mid_box_ind<big_box->GetCellNumber(); mid_box_ind++)//对于每个中方格
    {
      for(int little_box_ind=0; little_box_ind<big_box->GetCell(mid_box_ind)->GetCellNumber(); little_box_ind++)
      {
        
        ini_position.x = big_box->GetCell(mid_box_ind)->Ind2Pos(little_box_ind).x();
        ini_position.y = big_box->GetCell(mid_box_ind)->Ind2Pos(little_box_ind).y();
        ini_position.z = big_box->GetCell(mid_box_ind)->Ind2Pos(little_box_ind).z();
        big_box->GetCell(mid_box_ind)->GetCell(little_box_ind)->SetPosition(ini_position);
        big_box->GetCell(mid_box_ind)->GetCell(little_box_ind)->Reset();
      }
    }
    // for (int x = 0; x < vp_.kNumber.x(); x++)
    // {
    //   for (int y = 0; y < vp_.kNumber.y(); y++)
    //   {
    //     for (int z = 0; z < vp_.kNumber.z(); z++)
    //     {
    //       int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
    //       geometry_msgs::msg::Point position;
    //       position.x = origin_.x() + x * vp_.kResolution.x() + vp_.kResolution.x() / 2.0;
    //       position.y = origin_.y() + y * vp_.kResolution.y() + vp_.kResolution.y() / 2.0;
    //       position.z = robot_position.z();
    //       SetViewPointPosition(ind, position, true);
    //       ResetViewPoint(ind, true);
    //     }
    //   }
    // }
  }
  Eigen::Vector3i robot_grid_sub;
  Eigen::Vector3d diff = robot_position_ - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (vp_.kRolloverStepsize(i) * vp_.big_box_resolution(i))) : -1;
    // std::cout<<diff(i) / (vp_.kRolloverStepsize(i) * vp_.big_box_resolution(i))<<std::endl;
  }

  // std::cout<<(vp_.kRolloverStepsize(0) * vp_.big_box_resolution(0))<<std::endl;
  // std::cout<<(vp_.kRolloverStepsize(2) * vp_.big_box_resolution(2))<<std::endl;

  Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    sub_diff(i) = int(vp_.big_box_size_(i) / vp_.kRolloverStepsize(i)) / 2 - robot_grid_sub(i);
  }//   5/1/2=2.5   

  if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
  {
    traspass = false;
    return false;
  }
  traspass = true;
  Eigen::Vector3i rollover_step;
  rollover_step.x() = std::abs(sub_diff.x()) > 0 ?
                          vp_.kRolloverStepsize.x() * ((sub_diff.x() > 0) ? 1 : -1) * std::abs(sub_diff.x()) :
                          0;
  rollover_step.y() = std::abs(sub_diff.y()) > 0 ?
                          vp_.kRolloverStepsize.y() * ((sub_diff.y() > 0) ? 1 : -1) * std::abs(sub_diff.y()) :
                          0;
  rollover_step.z() = std::abs(sub_diff.z()) > 0 ?
                          vp_.kRolloverStepsize.z() * ((sub_diff.z() > 0) ? 1 : -1) * std::abs(sub_diff.z()) :
                          0;

  // std::cout << "rolling x: " << rollover_step.x() << " y: " << rollover_step.y() << " z: " << rollover_step.z()
  //           << std::endl;
  grid_->Roll(rollover_step);

  misc_utils_ns::Timer reset_timer("reset viewpoint");
  reset_timer.Start();

  //   origin_ = origin_ - rollover_step.cast<double>() * vp_.kResolution;
  origin_last_for_viewpoint_update_fail = origin_;
  origin_.x() -= rollover_step.x() * vp_.big_box_resolution.x();
  origin_.y() -= rollover_step.y() * vp_.big_box_resolution.y();
  origin_.z() -= rollover_step.z() * vp_.big_box_resolution.z();
  big_box->SetOrigin(origin_);
  grid_->GetUpdatedIndices(updated_viewpoint_indices_);
  // std::cout<<"viewpoint更新："<<updated_viewpoint_indices_.size()<<std::endl;
  for (const auto& ind : updated_viewpoint_indices_)
  {
    //关于ind是什么意思我在tare_arm里面注释
    MY_ASSERT(grid_->InRange(ind));
    Eigen::Vector3i sub = grid_->Ind2Sub(ind);//ind是方块相对于原点的值，所以用来获取origin坐标点
    Eigen::Vector3d new_origin;     
    new_origin.x() = origin_.x() + sub.x() * vp_.big_box_resolution.x() ;
    new_origin.y() = origin_.y() + sub.y() * vp_.big_box_resolution.y() ;
    new_origin.z() = origin_.z() + sub.z() * vp_.big_box_resolution.z() ;
    int array_ind = grid_->GetArrayInd(ind);
    big_box->GetCell(array_ind)->SetOrigin(new_origin);//array_ind表示在cells中需要被改变的值，所以ind要先换算成array_ind来访问被覆盖元素
    
    geometry_msgs::msg::Point new_position;
    // double origin_x,origin_y,origin_z;
    // origin_x = big_box->GetCell(array_ind)->GetOrigin().x();
    // origin_y = big_box->GetCell(array_ind)->GetOrigin().y();
    // origin_z = big_box->GetCell(array_ind)->GetOrigin().z();
    for(int i=0;i<big_box->GetCell(array_ind)->GetCellNumber();i++)//viewpoint坐标的设置 和 reset()
    {
      new_position.x = big_box->GetCell(array_ind)->Ind2Pos(i).x();
      new_position.y = big_box->GetCell(array_ind)->Ind2Pos(i).y();
      new_position.z = big_box->GetCell(array_ind)->Ind2Pos(i).z();
      big_box->GetCell(array_ind)->GetCell(i)->SetPosition(new_position);
      big_box->GetCell(array_ind)->GetCell(i)->Reset();
    }
    // SetViewPointPosition(ind, new_position);
    // ResetViewPoint(ind);
  }
  reset_timer.Stop(false);
  return true;
}

void ViewPointManager::UpdateOrigin()
{
  for (int i = 0; i < vp_.dimension_; i++)
  {
    origin_(i) = robot_position_(i) - (vp_.big_box_resolution(i) * vp_.big_box_size_(i)) / 2.0;
  }//初始化manager的origin
  origin_last_for_viewpoint_update_fail = origin_;
  big_box->SetOrigin(origin_);//初始化大方块的origin
  for(int ind = 0;ind<big_box->GetCellNumber(); ind++)
  {
    big_box->GetCell(ind)->SetOrigin( big_box->Sub2Pos_ld_point((big_box->Ind2Sub(ind))));//初始化中方块的origin_
  }
}

int ViewPointManager::GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind) const
{
  MY_ASSERT(grid_->InRange(viewpoint_ind));
  return (use_array_ind ? viewpoint_ind : grid_->GetArrayInd(viewpoint_ind));
}

int ViewPointManager::GetViewPointInd(int viewpoint_array_ind) const
{
  return grid_->GetInd(viewpoint_array_ind);
}

Eigen::Vector3i ViewPointManager::GetViewPointSub(Eigen::Vector3d position)
{
  Eigen::Vector3d diff = position - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / vp_.kResolution(i)) : -1;
  }
  return sub;
}

int ViewPointManager::GetViewPointInd(Eigen::Vector3d position)
{
  Eigen::Vector3i sub = GetViewPointSub(position);
  if (grid_->InRange(sub))
  {
    return grid_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

void ViewPointManager::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->clear();
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    if (IsViewPointCandidate(i, true))
    {
      geometry_msgs::msg::Point position = GetViewPointPosition(i, true);
      pcl::PointXYZI vis_point;
      vis_point.x = position.x;
      vis_point.y = position.y;
      vis_point.z = position.z;
      if (ViewPointVisited(i, true))
      {
        vis_point.intensity = -1.0;
      }
      else
      {
        vis_point.intensity = GetViewPointCoveredPointNum(i, true);
        vis_point.intensity += i * 1.0 / 10000.0;
      }
      // if (viewpoints_[i].InCurrentFrameLineOfSight())
      // {
      //   vis_point.intensity = 100;
      // }
      // else
      // {
      //   vis_point.intensity = -1;
      // }
      vis_cloud->points.push_back(vis_point);
    }
  }
}

void ViewPointManager::CheckViewPointCollisionWithCollisionGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    if (ViewPointInCollision(i, true))
    {
      AddViewPointCollisionFrameCount(i, true);
    }
  }
  std::fill(collision_point_count_.begin(), collision_point_count_.end(), 0);
  collision_grid_origin_ = origin_ - Eigen::Vector3d::Ones() * vp_.kViewPointCollisionMargin;
  collision_grid_->SetOrigin(collision_grid_origin_);
  for (const auto& point : collision_cloud->points)
  {
    Eigen::Vector3i collision_grid_sub = collision_grid_->Pos2Sub(point.x, point.y, point.z);
    if (collision_grid_->InRange(collision_grid_sub))
    {
      int collision_grid_ind = collision_grid_->Sub2Ind(collision_grid_sub);
      collision_point_count_[collision_grid_ind]++;
      if (collision_point_count_[collision_grid_ind] >= vp_.kCollisionPointThr)
      {
        std::vector<int> collision_viewpoint_indices = collision_grid_->GetCellValue(collision_grid_ind);
        for (int i = 0; i < collision_viewpoint_indices.size(); i++)
        {
          int viewpoint_ind = collision_viewpoint_indices[i];
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNumber);
          double z_diff = point.z - GetViewPointHeight(viewpoint_ind);
          if ((z_diff >= 0 && z_diff <= vp_.kViewPointCollisionMarginZPlus) ||
              (z_diff < 0 && z_diff >= -vp_.kViewPointCollisionMarginZMinus))
          {
            SetViewPointCollision(viewpoint_ind, true);
            ResetViewPointCollisionFrameCount(viewpoint_ind);
          }
        }
      }
    }
  }
}

bool ViewPointManager::InCollision(const Eigen::Vector3d& position)
{
  Eigen::Vector3i Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(position);
  int Hier_ind = Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
  if (InRange_Hier_sub(Hier_sub) )
  {
    if (Get_littlebox_from_Hierarchy_ind(Hier_ind)->Get_in_collision_())
    {
      return true;
    }
  }
  return false;
}

bool ViewPointManager::InCurrentFrameLineOfSight(const Eigen::Vector3d& position,const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_)
{
  // Eigen::Vector3i Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(position);
  // int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
  // bool in_line_of_sight = false;
  
  // if (InRange_Hier_sub(Hier_sub))
  // {
  //   // if (ViewPointInCurrentFrameLineOfSight(viewpoint_Hier_ind))
  //   if (Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Connected())
  //   {
  //     return true;
  //   }
  // }
  // // std::cout<<"false"<<std::endl;
  // return false;

  Eigen::Vector3i path_point_sub_in_occupancy,robot_sub_in_occupancy;
  robot_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(robot_position_);
  path_point_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(position);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  int array_ind_for_occupancy;
  if( !rolling_occupancy_grid_->Get_occupancy_array_()->InRange(robot_sub_in_occupancy)||!rolling_occupancy_grid_->Get_occupancy_array_()->InRange(path_point_sub_in_occupancy))
  {
    return false;
  }

  rolling_occupancy_grid_->RayTraceHelper_for_viewpoint_connected(robot_sub_in_occupancy, path_point_sub_in_occupancy, ray_cast_cells);
  for(int h=0;h<ray_cast_cells.size();h++)//对于每个cells确定它是否是占据的
  {
    if(!rolling_occupancy_grid_->Get_rolling_grid_()->InRange(ray_cast_cells[h]))
    {
      return false;
      break;
    }
    array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(ray_cast_cells[h]);
    if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCell(array_ind_for_occupancy) != rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
    {
      return false;
    }
  }
  return true;
}

void ViewPointManager::CheckViewPointBoundaryCollision()
{
  // Check for the polygon boundary and nogo zones
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    geometry_msgs::msg::Point viewpoint_position = GetViewPointPosition(i, true);
    if ((!viewpoint_boundary_.points.empty() &&
         !misc_utils_ns::PointInPolygon(viewpoint_position, viewpoint_boundary_)))
    {
      SetViewPointCollision(i, true, true);
      continue;
    }
    for (int j = 0; j < nogo_boundary_.size(); j++)
    {
      if (!nogo_boundary_[j].points.empty() && misc_utils_ns::PointInPolygon(viewpoint_position, nogo_boundary_[j]))
      {
        SetViewPointCollision(i, true, true);

        break;
      }
    }
  }
}

void ViewPointManager::CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
  CheckViewPointBoundaryCollision();
}

void ViewPointManager::CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                          double collision_threshold)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& point : terrain_cloud->points)
  {
    if (point.intensity > collision_threshold)
    {
      collision_cloud->points.push_back(point);
    }
  }
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
}

void ViewPointManager::CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub)
{
  if (end_sub == start_sub)
    return;
  int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(end_sub);
  geometry_msgs::msg::Point viewpoint_position = Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->GetPosition();
  std::vector<Eigen::Vector3i> ray_cast_cells;
  misc_utils_ns::RayCast(start_sub, end_sub, max_sub, min_sub, ray_cast_cells);
  if (ray_cast_cells.size() > 1)
  {
    if (vp_.kLineOfSightStopAtNearestObstacle)
    {
      bool occlude = false;
      for (int i = 1; i < ray_cast_cells.size(); i++)
      {
        int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(ray_cast_cells[i]);
        if (Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_collision_() && GetViewPointCollisionFrameCount(viewpoint_Hier_ind) == 0)
        {
          
          occlude = true;
          break;
        }
        if (!occlude)
        {
          SetViewPointInLineOfSight(viewpoint_Hier_ind, true);
          if (vp_.kCheckDynamicObstacleCollision &&
              GetViewPointCollisionFrameCount(viewpoint_Hier_ind) > vp_.kCollisionFrameCountMax)

          {
            // SetViewPointCollision(viewpoint_ind, false);
          }
        }
      }
    }
    else
    {
      bool hit_obstacle = false;
      bool in_line_of_sight = false;
      for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
      {
        int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(ray_cast_cells[i]);
        if (Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_collision_() && GetViewPointCollisionFrameCount(viewpoint_Hier_ind) == 0)
        {
          
          hit_obstacle = true;
        }
        if (hit_obstacle && !Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_collision_())
        {
          in_line_of_sight = true;
        }
        if (hit_obstacle && Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_collision_() &&
            GetViewPointCollisionFrameCount(viewpoint_Hier_ind) > vp_.kCollisionFrameCountMax)

        {
          in_line_of_sight = true;
          if (vp_.kCheckDynamicObstacleCollision)
          {
            // SetViewPointCollision(viewpoint_ind, false);
          }
        }
        if (in_line_of_sight)
        {
          SetViewPointInLineOfSight(viewpoint_Hier_ind, true);
        }
      }
      if (!hit_obstacle)
      {
        for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
        {
          int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(ray_cast_cells[i]);
          SetViewPointInLineOfSight(viewpoint_Hier_ind, true);
        }
      }
    }

    // Set in current frame line of sight
    bool occlude = false;
    for (int i = 1; i < ray_cast_cells.size(); i++)
    {
      int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(ray_cast_cells[i]);
      if (Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Get_in_collision_() && GetViewPointCollisionFrameCount(viewpoint_Hier_ind) == 0)
      {
        occlude = true;
        break;
      }
      if (!occlude)
      {
        SetViewPointInCurrentFrameLineOfSight(viewpoint_Hier_ind, true);
      }
    }
  }
  for(int i=0;i<vp_.viewpoint_num;i++)
  {
    if(Get_littlebox_from_Hierarchy_ind(i)->InCollision())
    Get_littlebox_from_Hierarchy_ind(i)->AddCollisionFrame();
  }
  
}

void ViewPointManager::CheckViewPointLineOfSight()
{
  if (!initialized_)
    return;

  for (int i = 0; i < vp_.viewpoint_num; i++)
  {
    SetViewPointInCurrentFrameLineOfSight(i, false, true);
  }

  Eigen::Vector3i robot_Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(robot_position_);
  MY_ASSERT(InRange_Hier_sub(robot_Hier_sub));
  int robot_viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(robot_Hier_sub);
  SetViewPointInLineOfSight(robot_viewpoint_Hier_ind, true);
  SetViewPointInCurrentFrameLineOfSight(robot_viewpoint_Hier_ind, true);

  std::vector<bool> checked(vp_.kViewPointNumber, false);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  Eigen::Vector3i max_sub(vp_.kNumber.x() - 1, vp_.kNumber.y() - 1, vp_.kNumber.z() - 1);
  Eigen::Vector3i min_sub(0, 0, 0);

  int x_indices[2] = { 0, vp_.kNumber.x() - 1 };
  int y_indices[2] = { 0, vp_.kNumber.y() - 1 };
  int z_indices[2] = { 0, vp_.kNumber.z() - 1 };

  for (int xi = 0; xi < 2; xi++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int x = x_indices[xi];
        Eigen::Vector3i end_sub(x, y, z);
        int Hier_ind = Hierarchy_sub_2_Hierarchy_ind(end_sub);
        if (!checked[Hier_ind])
        {
          CheckViewPointLineOfSightHelper(robot_Hier_sub, end_sub, max_sub, min_sub);
          checked[Hier_ind] = true;
        }
      }
    }
  }

  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int yi = 0; yi < 2; yi++)
    {
      for (int z = 0; z < vp_.kNumber.z(); z++)
      {
        int y = y_indices[yi];
        Eigen::Vector3i end_sub(x, y, z);
        int Hier_ind = Hierarchy_sub_2_Hierarchy_ind(end_sub);
        if (!checked[Hier_ind])
        {
          CheckViewPointLineOfSightHelper(robot_Hier_sub, end_sub, max_sub, min_sub);
          checked[Hier_ind] = true;
        }
      }
    }
  }

  for (int x = 0; x < vp_.kNumber.x(); x++)
  {
    for (int y = 0; y < vp_.kNumber.y(); y++)
    {
      for (int zi = 0; zi < 2; zi++)
      {
        int z = z_indices[zi];
        Eigen::Vector3i end_sub(x, y, z);
        int Hier_ind = Hierarchy_sub_2_Hierarchy_ind(end_sub);
        if (!checked[Hier_ind])
        {
          CheckViewPointLineOfSightHelper(robot_Hier_sub, end_sub, max_sub, min_sub);
          checked[Hier_ind] = true;
        }
      }
    }
  }
}

void ViewPointManager::CheckViewPointInFOV()
{
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    geometry_msgs::msg::Point viewpoint_position = GetViewPointPosition(i, true);
    if (!InRobotFOV(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z)))
    {
      SetViewPointInLineOfSight(i, false, true);
    }
  }
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
}

bool ViewPointManager::InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  double z_diff = std::abs(diff.z());
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewPointManager::InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double z_diff = std::abs(diff.z());
  if (z_diff > vp_.kDiffZMax)
  {
    return false;
  }
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  if (xy_diff > vp_.kSensorRange)
  {
    return false;
  }
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewPointManager::InRobotFOV(const Eigen::Vector3d& position)
{
  return InFOV(position, robot_position_);
}

void ViewPointManager::UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions,const std::unique_ptr<grid_world_ns::GridWorld>& grid_world)
{
  if (!initialized_)
    return;
  visualize_visited_positions_(positions);
  double visited_rad_;
  ind_of_visited_viewpoints.clear();
  for (const auto& position : positions)
  {
    if (!InLocalPlanningHorizon(position))
    {
      continue;
    }
    Get_littlebox_from_Hierarchy_ind(Hierarchy_sub_2_Hierarchy_ind(Hierarchy_Pos_2_Hierarchy_sub(position)))->SetVisited(true);
    ind_of_visited_viewpoints.push_back(Hierarchy_sub_2_Hierarchy_ind(Hierarchy_Pos_2_Hierarchy_sub(position)));
    int Hier_ind;
    Eigen::Vector3i Hier_sub;
    Eigen::Vector3d position_neighbour;
    position_neighbour.z()=position.z();


    Eigen::Vector3i sub =
      grid_world->get_subspaces_()->Pos2Sub(position);
    if (grid_world->get_subspaces_()->InRange(sub))
    {
      int cell_ind = grid_world->get_subspaces_()->Sub2Ind(sub);
      if(vp_.smaller_visited_radius && grid_world->get_subspaces_()->GetCell(cell_ind).GetViewPointIndices().size() < Get_middle_box_viewpoint_num()/2)
      {
        visited_rad_ = 2 * Retrun_visited_rad_copy() * grid_world->get_subspaces_()->GetCell(cell_ind).GetViewPointIndices().size()/Get_middle_box_viewpoint_num();
        // std::cout<<"访问半径减小"<<std::endl;
        if(visited_rad_<vp_.middle_box_resolution.x())
        {
          visited_rad_ = vp_.middle_box_resolution.x();
        }
      }
      else{
        visited_rad_ = Retrun_visited_rad_copy();
      }
    }

    for(double position_x=position.x()-visited_rad_; position_x<=position.x()+visited_rad_; )
    {
      position_neighbour.x()=position_x;
      for(double position_y=position.y()-visited_rad_; position_y<=position.y()+visited_rad_; )
      {
        position_neighbour.y()=position_y;
        for(double position_z=position.z()-vp_.visited_rad_z; position_z<=position.z()+vp_.visited_rad_z; )
        { 
          position_neighbour.z() = position_z;
          Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(position_neighbour);
          Hier_ind = Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
          if(InRange_Hier_sub(Hier_sub)&&Get_littlebox_from_Hierarchy_ind(Hier_ind)->Connected())
          {
            Get_littlebox_from_Hierarchy_ind(Hier_ind)->SetVisited(true);
            ind_of_visited_viewpoints.push_back(Hier_ind);
          }
          position_z=position_z+vp_.middle_box_resolution.z();
        }
        position_y=position_y+vp_.middle_box_resolution.y();
      }
      position_x=position_x+vp_.middle_box_resolution.x();
    }
  }
  visualize_visited_viewpoints_();
  
  //这里把visited的点可视化出来
}

void ViewPointManager::UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for (int i = 0; i < ind_of_connected_viewpoints.size(); i++)
  {
    Eigen::Vector3d Position_vec = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(ind_of_connected_viewpoints[i]));
    geometry_msgs::msg::Point viewpoint_position;
    viewpoint_position.x = Position_vec.x();
    viewpoint_position.y = Position_vec.y();
    viewpoint_position.z = Position_vec.z();
    int cell_ind = grid_world->GetCellInd(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z);

    if (grid_world->IndInBound((cell_ind)))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS)
      {
        Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->SetVisited(true);
      }
    }
  }
}

void ViewPointManager::SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                     double terrain_height_threshold)
{
  // Set the height of the viewpoint nearby the robot to be the height of the robot, in case there is no terrain cloud
  // within the blind spot.
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  int robot_ind = grid_->Sub2Ind(robot_sub);
  MY_ASSERT(grid_->InRange(robot_sub));
  geometry_msgs::msg::Point robot_viewpoint_position = GetViewPointPosition(robot_ind);

  if (!ViewPointHasTerrainHeight(robot_ind) ||
      std::abs(robot_viewpoint_position.z - robot_position_.z()) > vp_.kViewPointHeightFromTerrainChangeThreshold)
  {
    robot_viewpoint_position.z = robot_position_.z();
    SetViewPointPosition(robot_ind, robot_viewpoint_position);
    for (int i = 0; i < in_range_neighbor_indices_[robot_ind].size(); i++)
    {
      int neighbor_ind = in_range_neighbor_indices_[robot_ind][i];
      MY_ASSERT(grid_->InRange(neighbor_ind));
      if (!ViewPointHasTerrainHeight(neighbor_ind) ||
          std::abs(GetViewPointHeight(neighbor_ind) - robot_position_.z()) > 0.6)
      {
        SetViewPointHeight(neighbor_ind, robot_position_.z());
      }
    }
  }

  // Set the height of other viewpoints
  for (const auto& terrain_point : terrain_cloud->points)
  {
    if (terrain_point.intensity > terrain_height_threshold)
    {
      continue;
    }
    Eigen::Vector3i viewpoint_sub = GetViewPointSub(Eigen::Vector3d(terrain_point.x, terrain_point.y, terrain_point.z));
    if (grid_->InRange(viewpoint_sub))
    {
      int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
      double target_height = terrain_point.z + vp_.kViewPointHeightFromTerrain;
      // If the viewpoint has not been set height with terrain points, or if there is a terrain point with a lower
      // height
      if (!ViewPointHasTerrainHeight(viewpoint_ind) || target_height < GetViewPointHeight(viewpoint_ind))
      {
        if (std::abs(target_height - GetViewPointHeight(viewpoint_ind)) >
            vp_.kViewPointHeightFromTerrainChangeThreshold)
        {
          ResetViewPoint(viewpoint_ind);
        }
        SetViewPointHeight(viewpoint_ind, target_height);
        SetViewPointHasTerrainHeight(viewpoint_ind, true);
      }
    }
  }

  // For viewpoints that are not set heights with terrain directly, use neighbors' heights
  for (int i = 0; i < vp_.kViewPointNumber; i++)
  {
    if (!ViewPointHasTerrainHeight(i))
    {
      for (const auto& neighbor_ind : in_range_neighbor_indices_[i])
      {
        MY_ASSERT(grid_->InRange(neighbor_ind));
        if (ViewPointHasTerrainHeight(neighbor_ind))
        {
          double neighbor_height = GetViewPointHeight(neighbor_ind);
          if (std::abs(neighbor_height - GetViewPointHeight(i)) > vp_.kViewPointHeightFromTerrainChangeThreshold)
          {
            geometry_msgs::msg::Point viewpoint_position = GetViewPointPosition(i);
            viewpoint_position.z = neighbor_height;
            ResetViewPoint(i);
            SetViewPointPosition(i, viewpoint_position);
          }
          else
          {
            SetViewPointHeight(i, neighbor_height);
          }
        }
      }
    }
  }
}

// Reset viewpoint
void ViewPointManager::ResetViewPoint(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].Reset();
}

void ViewPointManager::ResetViewPointCoverage()
{
  for(int ind=0;ind<vp_.viewpoint_num;ind++)//更新每个点的visitd情况
  {
    Get_littlebox_from_Hierarchy_ind(ind)->ResetCoverage();
  }
}

// Collision
bool ViewPointManager::ViewPointInCollision(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCollision();
}
void ViewPointManager::SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInCollision(in_collision);
}
// Line of Sight
bool ViewPointManager::ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InLineOfSight();
}
void ViewPointManager::SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind)
{
  Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->SetInCurrentFrameLineOfSight(in_line_of_sight);
}
// Connectivity
bool ViewPointManager::ViewPointConnected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Connected();
}
void ViewPointManager::SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetConnected(connected);
}
// Visited
bool ViewPointManager::ViewPointVisited(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Visited();
}
void ViewPointManager::SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetVisited(visited);
}
// Selected
bool ViewPointManager::ViewPointSelected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Selected();
}
void ViewPointManager::SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind)
{
  // int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  // viewpoints_[array_ind].SetSelected(selected);
  Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->SetSelected(selected);
}
// Candidacy
bool ViewPointManager::IsViewPointCandidate(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].IsCandidate();
}
void ViewPointManager::SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetCandidate(candidate);
}
// Terrain Height
bool ViewPointManager::ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].HasTerrainHeight();
}
void ViewPointManager::SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHasTerrainHeight(has_terrain_height);
}
// In exploring cell
bool ViewPointManager::ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InExploringCell();
}
void ViewPointManager::SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind)
{
  Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->SetInExploringCell(in_exploring_cell);
  // viewpoints_[array_ind].SetInExploringCell(in_exploring_cell);
}
// Height
double ViewPointManager::GetViewPointHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetHeight();
}
void ViewPointManager::SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHeight(height);
}
// In current frame line of sight
bool ViewPointManager::ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  
  return Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->InCurrentFrameLineOfSight();
}
void ViewPointManager::SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                                             bool use_array_ind)
{
  Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->SetInCurrentFrameLineOfSight(in_current_frame_line_of_sight);
}
// Position
geometry_msgs::msg::Point ViewPointManager::GetViewPointPosition(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetPosition();
}
void ViewPointManager::SetViewPointPosition(int viewpoint_ind, geometry_msgs::msg::Point position, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetPosition(position);
}
// Cell Ind
int ViewPointManager::GetViewPointCellInd(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCellInd();
}
void ViewPointManager::SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind)
{
  // int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->SetCellInd(cell_ind);
  // viewpoints_[array_ind].SetCellInd(cell_ind);
}
// Collision frame count
int ViewPointManager::GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  // int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return Get_littlebox_from_Hierarchy_ind(viewpoint_ind)->GetCollisionFrameCount();
}
void ViewPointManager::AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCollisionFrame();
}
void ViewPointManager::ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCollisionFrameCount();
}
// Covered point list
void ViewPointManager::ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCoveredPointList();
  viewpoints_[array_ind].ResetCoveredFrontierPointList();
}
void ViewPointManager::AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredPoint(point_ind);
}
void ViewPointManager::AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredFrontierPoint(point_ind);
}
const std::vector<int>& ViewPointManager::GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointList();
}
const std::vector<int>& ViewPointManager::GetViewPointCoveredFrontierPointList(int viewpoint_ind,
                                                                               bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointList();
}

int ViewPointManager::GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointNum();
}

int ViewPointManager::GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointNum();
}

int ViewPointManager::GetViewPointCoveredPointNum(const std::vector<bool>& point_list, int viewpoint_Hier_ind,
                                                  bool use_array_ind)
{
  int covered_point_num = 0;
  for (const auto& point_ind : Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->GetCoveredPointList())
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    if (!point_list[point_ind])
    {
      covered_point_num++;
    }
  }
  return covered_point_num;
}
int ViewPointManager::GetViewPointCoveredFrontierPointNum(const std::vector<bool>& frontier_point_list,
                                                          int viewpoint_index, bool use_array_ind)
{
  int covered_frontier_point_num = 0;
  for (const auto& point_ind : Get_littlebox_from_Hierarchy_ind(viewpoint_index)->GetCoveredFrontierPointList())
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    if (!frontier_point_list[point_ind])
    {
      covered_frontier_point_num++;
    }
  }
  return covered_frontier_point_num;
}

void ViewPointManager::UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index,
                                                   bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
void ViewPointManager::UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                                           bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

void ViewPointManager::UpdateViewPointGraph(bool add_frontier_viewpoint)
{

  // Construct a graph of all the viewpoints
  GetCandidateViewPointGraph(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,add_frontier_viewpoint);

  // return candidate_indices_.size();
}

nav_msgs::msg::Path ViewPointManager::GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind, bool use_frontier_viewpoint)
{
  //
  bool start_frontier_viewpoint_flag=false;
  bool end_frontier_viewpoint_flag=false;
  if(use_frontier_viewpoint)
  {
    if(Is_frontier_viewpoint_Hier_ind(start_viewpoint_ind))
    {
      start_frontier_viewpoint_flag = true;
    }
    if(Is_frontier_viewpoint_Hier_ind(target_viewpoint_ind))
    {
      end_frontier_viewpoint_flag = true;
    }
  }

  ////
  nav_msgs::msg::Path path;
  if (!InRange_Hier_ind(start_viewpoint_ind) && !start_frontier_viewpoint_flag)
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start viewpoint ind: " << start_viewpoint_ind
    //                                                                                    << " not in range");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath start viewpoint ind: " << start_viewpoint_ind
                                                                                       << " not in range");                                                                              
    return path;
  }
  if (!InRange_Hier_ind(target_viewpoint_ind) && !end_frontier_viewpoint_flag)
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target viewpoint ind: " << target_viewpoint_ind
    //                                                                                     << " not in range");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath target viewpoint ind: " << target_viewpoint_ind
                                                                                        << " not in range");                                                                             
    return path;
  }

  int start_graph_ind ;   
  int target_graph_ind ;

///
if(start_frontier_viewpoint_flag)
{
  start_graph_ind = graph_index_map_for_frontier_viewpoint_[Hier_ind_2_frontier_viewpoint_ind(start_viewpoint_ind)]; 
}
else{
  start_graph_ind = graph_index_map_[start_viewpoint_ind]; 
}
if(end_frontier_viewpoint_flag)
{
  target_graph_ind = graph_index_map_for_frontier_viewpoint_[Hier_ind_2_frontier_viewpoint_ind(target_viewpoint_ind)]; 
}
else{
  target_graph_ind = graph_index_map_[target_viewpoint_ind];
}

///
  std::vector<int> path_graph_indices;
  bool is_frontier_viewpoint_ind_=false;
  double path_length =
      misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,
                                 start_graph_ind, target_graph_ind, true, path_graph_indices);
  if (path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      is_frontier_viewpoint_ind_ = false;
      int graph_idx = path_graph_indices[i];
      if(use_frontier_viewpoint)
      {
        if(graph_idx>=ind_of_connected_viewpoints.size())
        {
          is_frontier_viewpoint_ind_ = true;
        }
      }
      if(is_frontier_viewpoint_ind_)
      {
        pose.pose.position = frontier_viewpoints_cantidate[graph_idx-ind_of_connected_viewpoints.size()]->GetPosition();
      }
      else{
        int ind = ind_of_connected_viewpoints[graph_idx];
        pose.pose.position = Get_littlebox_from_Hierarchy_ind(ind)->GetPosition();
      }
      path.poses.push_back(pose);
    }
  }
  return path;


}

nav_msgs::msg::Path ViewPointManager::GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                                          const Eigen::Vector3d& target_position)
{
  nav_msgs::msg::Path path;
  if (!InLocalPlanningHorizon(start_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
    //                                                                              << " not in local planning horizon");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "PlanningEnv::UpdateRegisteredCloud(): registered cloud empty");
    return path;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
    //                                                                               << " not in local planning horizon");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return path;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);

  return GetViewPointShortestPath(start_viewpoint_ind, target_viewpoint_ind);
}
nav_msgs::msg::Path ViewPointManager::GetViewPointShortestPath_no_warn(const Eigen::Vector3d& start_position,
                                                          const Eigen::Vector3d& target_position)
{
  nav_msgs::msg::Path path;
  if (!InLocalPlanningHorizon(start_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
    //                                                                              << " not in local planning horizon");
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "PlanningEnv::UpdateRegisteredCloud(): registered cloud empty");
    return path;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
    //                                                                               << " not in local planning horizon");
    // RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  // << " not in local planning horizon");
    return path;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);
  
  return GetViewPointShortestPath(start_viewpoint_ind, target_viewpoint_ind);
}

bool ViewPointManager::GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                                             const Eigen::Vector3d& target_position,
                                                             double max_path_length, nav_msgs::msg::Path& path)
{
  if (!InLocalPlanningHorizon(start_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
    //                                                                              << " not in local planning horizon");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                 << " not in local planning horizon");
    return false;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    // ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
    //                                                                               << " not in local planning horizon");
    RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return false;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);
  int start_graph_ind = graph_index_map_[start_viewpoint_ind];
  int target_graph_ind = graph_index_map_[target_viewpoint_ind];

  std::vector<int> path_graph_indices;
  double shortest_path_dist = 0;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(
      candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_, start_graph_ind,
      target_graph_ind, true, path_graph_indices, shortest_path_dist, max_path_length);

  if (found_path && path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      int graph_idx = path_graph_indices[i];
      int ind = ind_of_connected_viewpoints[graph_idx];
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position = Get_littlebox_from_Hierarchy_ind(ind)->GetPosition();
      path.poses.push_back(pose);
    }
  }
  return found_path;
}

void ViewPointManager::UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for(int i=0;i<ind_of_connected_viewpoints.size();i++)//每个元素
  {
    int cell_ind = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetCellInd();
    if (grid_world->IndInBound(cell_ind))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::UNSEEN || cell_status == grid_world_ns::CellStatus::EXPLORING)
      {
        SetViewPointInExploringCell(ind_of_connected_viewpoints[i], true);
      }
      else
      {
        SetViewPointInExploringCell(ind_of_connected_viewpoints[i], false);
      }
    }
    else
    {
      // ROS_WARN_STREAM("ViewPointManager::UpdateCandidateViewPointCellStatus: cell ind " << cell_ind << " out of bound");
      RCLCPP_WARN_STREAM(rclcpp::get_logger("my_logger"), "ViewPointManager::UpdateCandidateViewPointCellStatus: cell ind " << cell_ind << " out of bound");
    }
  }
}

void ViewPointManager::GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph,
                                                  std::vector<std::vector<double>>& dist,
                                                  std::vector<geometry_msgs::msg::Point>& positions,
                                                  bool add_frontier_viewpoint)
{


if(!add_frontier_viewpoint)
{
  graph.clear();
  dist.clear();
  positions.clear();
  if (ind_of_connected_viewpoints.empty())
  {
    return;
  }
  graph.resize(ind_of_connected_viewpoints.size());
  dist.resize(graph.size());

  for (int i = 0; i < ind_of_connected_viewpoints.size(); i++)
  {
    int ind = ind_of_connected_viewpoints[i];
    graph_index_map_[ind] = i;
  }
  // Build the graph
  for (int i = 0; i < ind_of_connected_viewpoints.size(); i++)
  {
    int cur_ind = ind_of_connected_viewpoints[i];
    positions.push_back(Get_littlebox_from_Hierarchy_ind(cur_ind)->GetPosition());
    for (int j = 0; j < uav_viewpoints_neighbor_indices_[cur_ind].size(); j++)
    {
      int neighbor_ind = uav_viewpoints_neighbor_indices_[cur_ind][j];
      double neighbor_dist = uav_viewpoint_neighbor_dist_[cur_ind][j];
      if (Get_littlebox_from_Hierarchy_ind(neighbor_ind)->Connected())
      {
        graph[i].push_back(graph_index_map_[neighbor_ind]);
        dist[i].push_back(neighbor_dist);
      }
    }
  }
  // std::cout<<"-------------------------"<<std::endl;
  // for(int i=0; i<graph.size();i++)
  // {
  //   for(int h=0; h<graph[i].size(); h++)
  //   {
  //     std::cout<<graph[i][h]<<", ";
  //   }
  //   std::cout<<std::endl;
  // }
}
else
{


  //graph_index_map_输入普通viewpoint 的 Hier ind 可以获得该普通viewpoint在graph的关系索引值
  //graph[graph_index_map_[hier_ind]]
  graph.resize(graph.size()+frontier_viewpoints_cantidate.size());
  dist.resize(graph.size());
  Hier_ind_of_cantidate_frontier_viewpoints.clear();
  Hier_ind_of_cantidate_frontier_viewpoints.resize(frontier_viewpoints_cantidate.size());
  graph_index_map_for_frontier_viewpoint_.resize(frontier_viewpoints_cantidate.size());
  int tem_i = ind_of_connected_viewpoints.size();
  for (auto& ind : graph_index_map_for_frontier_viewpoint_)//frontier_viewpoint第X个point  对应  map向量第X个元素的值是在graph_的索引 
  {
    ind = tem_i;
    tem_i++;
  }

  tem_i = vp_.viewpoint_num;
  for (auto& ind : Hier_ind_of_cantidate_frontier_viewpoints)
  {
    ind = tem_i;
    tem_i++;
  }
  
  double temp_dist;
  
    for(int i = 0;i < frontier_viewpoints_cantidate.size(); i++)
    {
      positions.push_back(frontier_viewpoints_cantidate[i]->GetPosition());
      temp_dist =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(frontier_viewpoints_cantidate[i]->GetPosition(), Get_littlebox_from_Hierarchy_ind(frontier_viewpoints_cantidate[i]->conntedcomviewpointHierind())->GetPosition());
      //neighbor_ind =frontier_viewpoints_cantidate[i]->conntedcomviewpointHierind()
      //neighbor_dist=temp_dist
      graph[graph_index_map_for_frontier_viewpoint_[i]].push_back(graph_index_map_[frontier_viewpoints_cantidate[i]->conntedcomviewpointHierind()]);
      dist[graph_index_map_for_frontier_viewpoint_[i]].push_back(temp_dist);
      graph[graph_index_map_[frontier_viewpoints_cantidate[i]->conntedcomviewpointHierind()]].push_back(graph_index_map_for_frontier_viewpoint_[i]);
      dist[graph_index_map_[frontier_viewpoints_cantidate[i]->conntedcomviewpointHierind()]].push_back(temp_dist);
    }
  
  // std::cout<<"普通点数量："<<ind_of_connected_viewpoints.size()<<"  frontierviewpoint："<<frontier_viewpoints_cantidate.size()<<std::endl;
  // for(int i=0; i<graph.size();i++)
  // {
  //   for(int h=0; h<graph[i].size(); h++)
  //   {
  //     std::cout<<graph[i][h]<<", ";
  //   }
  //   std::cout<<std::endl;
  // }
}
}

int ViewPointManager::GetNearestCandidateViewPointInd(const Eigen::Vector3d& position)
{
  Eigen::Vector3i Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(position);
  int viewpoint_Hier_ind = Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
  if (InRange_Hier_sub(Hier_sub))
  {
    if (Get_littlebox_from_Hierarchy_ind(viewpoint_Hier_ind)->Connected())
    {
      return viewpoint_Hier_ind;
    }
  }
  if (!ind_of_connected_viewpoints.empty())
  {
    // Find the closest viewpoint that is a candidate viewpoint
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    geometry_msgs::msg::Point query_position;
    query_position.x = position.x();
    query_position.y = position.y();
    query_position.z = position.z();
    for (const auto& cur_viewpoint_ind : ind_of_connected_viewpoints)
    {
      geometry_msgs::msg::Point cur_position = Get_littlebox_from_Hierarchy_ind(cur_viewpoint_ind)->GetPosition();
      double dist =
          misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(cur_position, query_position);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_dist_ind = cur_viewpoint_ind;
      }
    }
    // std::cout<<"min_dist_ind"<<min_dist_ind<<std::endl;
    return min_dist_ind;
  }
  else
  {
    std::cout << "Candidate viewpoint empty, can't find nearest candidate viewpoints to the position" << std::endl;
    return -1;
  }
}

void ViewPointManager::GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& point : viewpoint_in_collision_cloud_->points)
  {
    cloud->points.push_back(point);
  }
}

bool ViewPointManager::InLocalPlanningHorizon(const Eigen::Vector3d& position)
{
  Eigen::Vector3i Hier_sub = Hierarchy_Pos_2_Hierarchy_sub(position);
  int Hier_ind = Hierarchy_sub_2_Hierarchy_ind(Hier_sub);
  if (InRange_Hier_sub(Hier_sub))
  {
    if (Get_littlebox_from_Hierarchy_ind(Hier_ind)->Connected())
    {
      return true;
    }
  }
  return false;
}

Eigen::Vector3i ViewPointManager::Hierarchy_ind_2_Hierarchy_sub(int Hierarchy_ind)
{
  Eigen::Vector3i Hierarchy_sub(0, 0, 0);
  // Eigen::Vector3i sub;
  // std::cout<<"Hierarchy_ind："<<Hierarchy_ind<<std::endl;
  Hierarchy_sub.z() = Hierarchy_ind / (vp_.big_box_size_.y()*vp_.middle_box_size_.y()* vp_.big_box_size_.x()*vp_.middle_box_size_.x());
  Hierarchy_ind -= (Hierarchy_sub.z() * vp_.big_box_size_.y()*vp_.middle_box_size_.y()* vp_.big_box_size_.x()*vp_.middle_box_size_.x());
  Hierarchy_sub.y() = Hierarchy_ind / (vp_.big_box_size_.x()*vp_.middle_box_size_.x());
  Hierarchy_sub.x() = Hierarchy_ind % (vp_.big_box_size_.x()*vp_.middle_box_size_.x());
  // std::cout<<"Hierarchy_ind"<<Hierarchy_ind<<std::endl;
  return Hierarchy_sub;
  

}
int ViewPointManager::Hierarchy_sub_2_Hierarchy_ind(const Eigen::Vector3i& Hierarchy_sub)
{
  // int Hierarchy_ind;
  // int x = Hierarchy_sub.x()/
  
  return Hierarchy_sub.x() + (Hierarchy_sub.y() * vp_.big_box_size_.x()*vp_.middle_box_size_.x()) + (Hierarchy_sub.z() * vp_.big_box_size_.y()*vp_.middle_box_size_.y()* vp_.big_box_size_.x()*vp_.middle_box_size_.x());
  
}

Eigen::Vector3i ViewPointManager::Hierarchy_Pos_2_Hierarchy_sub(const Eigen::Vector3d& pos)
{
  Eigen::Vector3i Hierarchy_sub(0, 0, 0);
  for (int i = 0; i < vp_.dimension_; i++)
  {
    Hierarchy_sub(i) = pos(i) - big_box->GetOrigin()(i) > 0 ? static_cast<int>((pos(i) - origin_(i)) / vp_.middle_box_resolution(i)) : -1;
  }
  return Hierarchy_sub;
}

Eigen::Vector3d ViewPointManager::Hierarchy_sub_2Hierarchy_Pos_(const Eigen::Vector3i& Hierarchy_sub)
{
  Eigen::Vector3d pos(0, 0, 0);
  for (int i = 0; i < vp_.dimension_; i++)
  {
    pos(i) = origin_(i) + Hierarchy_sub(i) * vp_.middle_box_resolution(i) + vp_.middle_box_resolution(i) / 2;
  }
  return pos;
}
Eigen::Vector3d ViewPointManager::Hierarchy_ind_2Hierarchy_Pos_(int Hierarchy_ind)
{
  return Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(Hierarchy_ind));
}

bool ViewPointManager::Hierarchy_InRange(const Eigen::Vector3i& sub) 
{
  bool in_range = true;
  for (int i = 0; i < vp_.dimension_; i++)
  {
    in_range &= sub(i) >= 0 && sub(i) < (vp_.big_box_size_(i)*vp_.middle_box_size_(i));
  }
  return in_range;
}

void ViewPointManager::Hierarchy_sub_2_midbox_ind_and_litbox_ind(const Eigen::Vector3i& Hierarchy_sub,int& mid_box_ind,int& lit_box_ind) 
{
  Eigen::Vector3i mid_sub(0,0,0);
  Eigen::Vector3i little_sub(0,0,0);
  mid_sub.x() = Hierarchy_sub.x()/vp_.middle_box_size_.x();
  little_sub.x() = Hierarchy_sub.x()-mid_sub.x()*vp_.middle_box_size_.x();
  // std::cout<<"little_sub.x():"<<little_sub.x()<<"  Hierarchy_sub.x()"<<Hierarchy_sub.x()<<" mid_sub.x()*vp_.middle_box_size_.x()"<<mid_sub.x()*vp_.middle_box_size_.x()<<std::endl;
  mid_sub.y() = Hierarchy_sub.y()/vp_.middle_box_size_.y();
  little_sub.y() = Hierarchy_sub.y()-mid_sub.y()*vp_.middle_box_size_.y();
  mid_sub.z() = Hierarchy_sub.z()/vp_.middle_box_size_.z();
  little_sub.z() = Hierarchy_sub.z()-mid_sub.z()*vp_.middle_box_size_.z();
  mid_box_ind = big_box->Sub2Ind(mid_sub);
  lit_box_ind = big_box->GetCell(0)->Sub2Ind(little_sub);//所有cell的sub2ind都一样，所以随便选了个0
}
void ViewPointManager::midbox_ind_and_litbox_ind_2_Hierarchy_sub_( Eigen::Vector3i& Hierarchy_sub,const int& mid_box_ind,const int& lit_box_ind) 
{
  Eigen::Vector3i mid_sub(0,0,0);
  Eigen::Vector3i little_sub(0,0,0);
  mid_sub = big_box->Ind2Sub(mid_box_ind);
  little_sub = big_box->GetCell(0)->Ind2Sub(lit_box_ind);//所有cell的sub2ind都一样，所以随便选了个0
  Hierarchy_sub.x() =  mid_sub.x()*vp_.middle_box_size_.x() + little_sub.x();
  Hierarchy_sub.y() =  mid_sub.y()*vp_.middle_box_size_.y() + little_sub.y();
  Hierarchy_sub.z() =  mid_sub.z()*vp_.middle_box_size_.z() + little_sub.z();
}


std::shared_ptr<viewpoint_ns::ViewPoint>& ViewPointManager::Get_littlebox_from_Hierarchy_ind(int Hierarchy_ind)
{
  
  //TODO 这个先转Hierarchy_sub 再 little box 和mid box
  int ind_of_middle_box,ind_of_little_box,array_ind;
  // Hierarchy_sub_2_midbox_ind_and_litbox_ind(Hierarchy_ind_2_Hierarchy_sub(Hierarchy_ind),ind_of_middle_box,ind_of_little_box);
  Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,Hierarchy_ind);
  array_ind = grid_->GetArrayInd(ind_of_middle_box);
  return big_box->GetCell(array_ind)->GetCell(ind_of_little_box);
}

void ViewPointManager::Two_index_to_Hierarchy_ind(const int& ind_of_middle_box, const int& ind_of_little_box, int &Hierarchy_ind)
{
  Eigen::Vector3i Hierarchy_sub(0,0,0);
  midbox_ind_and_litbox_ind_2_Hierarchy_sub_(Hierarchy_sub,ind_of_middle_box,ind_of_little_box);
  Hierarchy_ind = Hierarchy_sub_2_Hierarchy_ind(Hierarchy_sub);
}
void ViewPointManager::Hierarchy_ind_to_into_Two(int &ind_of_middle_box, int &ind_of_little_box,const int& Hierarchy_ind)
{
  Hierarchy_sub_2_midbox_ind_and_litbox_ind(Hierarchy_ind_2_Hierarchy_sub(Hierarchy_ind),ind_of_middle_box,ind_of_little_box);
}
bool ViewPointManager::check_viewpoint_in_real_collision(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_, viewpoint_ns::ViewPoint& viewpoint)
{
  int vp_ind;
  Eigen::Vector3d vp_xyz;
  geometry_msgs::msg::Point point = viewpoint.GetPosition();
  vp_xyz.x() = point.x;
  vp_xyz.y() = point.y;
  vp_xyz.z() = point.z;
  vp_ind = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Ind(vp_xyz);
  if (rolling_occupancy_grid_->Get_occupancy_array_()->InRange(vp_ind))
  {
    Eigen::Vector3i vp_sub = rolling_occupancy_grid_->Get_occupancy_array_()->Ind2Sub(vp_ind);
    Eigen::Vector3i  niegbour_sub;
    int array_ind_for_occupancy;
    for(int idx_addon_ind=0;idx_addon_ind<idx_addon_check_collision.size();idx_addon_ind++)
    {
      niegbour_sub = vp_sub+idx_addon_check_collision[idx_addon_ind];//包括自己在内的niegbour_sub
      // niegbour_sub = vp_sub;//不管邻居，只管自己
      if(!rolling_occupancy_grid_->Get_occupancy_array_()->InRange(niegbour_sub))
      {
        continue;
      }
      array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(niegbour_sub);
      

      if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCellValue(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::OCCUPIED)
      {
        viewpoint.SetIn_real_Collision(true);
        return true;
      }
    }
    return false;
  }
  else{
    return false;
  }
}
void ViewPointManager::Abstract_free_viewpoints(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_)
{
  // for()//对于每个viewpoints点
  ind_of_free_viewpoints.clear();
  geometry_msgs::msg::Point pt_xyz;
  Eigen::Vector3d vp_xyz;
  int vp_ind;
  int array_ind_for_occupancy,array_ind_for_middle_box;
  int Hierarchy_ind;
  std::vector<Eigen::Vector3i> idx_addon;
  // idx_addon.resize(6);
  idx_addon.push_back(Eigen::Vector3i(0,0,0));
  if(vp_.incollision_barrier_level>0)
  {
    idx_addon.push_back(Eigen::Vector3i(-1,0,0));
    idx_addon.push_back(Eigen::Vector3i(1,0,0));
    idx_addon.push_back(Eigen::Vector3i(0,-1,0));
    idx_addon.push_back(Eigen::Vector3i(0,1,0));
    idx_addon.push_back(Eigen::Vector3i(0,0,-1));
    idx_addon.push_back(Eigen::Vector3i(0,0,1));
    idx_addon.push_back(Eigen::Vector3i(1,1,1));
    idx_addon.push_back(Eigen::Vector3i(-1,1,1));
    idx_addon.push_back(Eigen::Vector3i(1,-1,1));
    idx_addon.push_back(Eigen::Vector3i(1,1,-1));
    idx_addon.push_back(Eigen::Vector3i(-1,-1,-1));
    idx_addon.push_back(Eigen::Vector3i(1,-1,-1));
    idx_addon.push_back(Eigen::Vector3i(-1,1,-1));
    idx_addon.push_back(Eigen::Vector3i(-1,-1,1));

    if(vp_.incollision_barrier_level>1)
    {
      idx_addon.push_back(Eigen::Vector3i(-2,0,0));
      idx_addon.push_back(Eigen::Vector3i(2,0,0));
      idx_addon.push_back(Eigen::Vector3i(0,-2,0));
      idx_addon.push_back(Eigen::Vector3i(0,2,0));
      idx_addon.push_back(Eigen::Vector3i(0,0,-2));
      idx_addon.push_back(Eigen::Vector3i(0,0,2));
      if(vp_.incollision_barrier_level>2)
      {
        idx_addon.push_back(Eigen::Vector3i(-3,0,0));
        idx_addon.push_back(Eigen::Vector3i(3,0,0));
        idx_addon.push_back(Eigen::Vector3i(0,-3,0));
        idx_addon.push_back(Eigen::Vector3i(0,3,0));
        idx_addon.push_back(Eigen::Vector3i(0,0,-3));
        idx_addon.push_back(Eigen::Vector3i(0,0,3));
      }
    }
    
  }
  
  
  bool occupy_flag;
  Eigen::Vector3i vp_sub,niegbour_sub;
  for(int mid_box_ind=0; mid_box_ind<big_box->GetCellNumber();mid_box_ind++)
  {
    for(int little_box_ind=0; little_box_ind<big_box->GetCell(mid_box_ind)->GetCellNumber();little_box_ind++)
    {
      array_ind_for_middle_box = grid_->GetArrayInd(mid_box_ind);
      pt_xyz = big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->GetPosition();
      vp_xyz.x() = pt_xyz.x;
      vp_xyz.y() = pt_xyz.y;
      vp_xyz.z() = pt_xyz.z;
      vp_ind = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Ind(vp_xyz);
      if (rolling_occupancy_grid_->Get_occupancy_array_()->InRange(vp_ind))
      {
        occupy_flag=true;
        vp_sub = rolling_occupancy_grid_->Get_occupancy_array_()->Ind2Sub(vp_ind);

        // if(rolling_occupancy_grid_->Get_occupancy_array_()->InRange(vp_sub))
        // {
        //   if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCellValue(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::OCCUPIED)
        //   {
        //     big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->SetIn_real_Collision(true);
        //   }
        // }

        for(int idx_addon_ind=0;idx_addon_ind<idx_addon.size();idx_addon_ind++)
        {
          niegbour_sub = vp_sub+idx_addon[idx_addon_ind];//包括自己在内的niegbour_sub
          // niegbour_sub = vp_sub;//不管邻居，只管自己
          if(!rolling_occupancy_grid_->Get_occupancy_array_()->InRange(niegbour_sub))
          {
            continue;
          }
          array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(niegbour_sub);
          if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCellValue(array_ind_for_occupancy) != rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
          {
            occupy_flag=false;//并且要所在的free的occupancy邻居都是free点，太靠近不行
            if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCellValue(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::OCCUPIED)
            {
              big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->SetIn_real_Collision(true);
              break;
            }
            
          }
        }
        // array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(vp_ind);
        if(occupy_flag)
        // if (rolling_occupancy_grid_->Get_occupancy_array_()->GetCellValue(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
        {
          
            big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->SetInCollision(false);
            big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->Setin_free(true);
            Two_index_to_Hierarchy_ind(mid_box_ind,little_box_ind,Hierarchy_ind);
            ind_of_free_viewpoints.push_back(Hierarchy_ind);
        }
        else
        {
          big_box->GetCell(array_ind_for_middle_box)->GetCell(little_box_ind)->SetInCollision(true);
        }

      }
    }
  }
}




void ViewPointManager::visualize_free_viewpoints_()//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for(int i=0;i<ind_of_free_viewpoints.size();i++)//每个元素
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_free_viewpoints[i])->GetPosition();
    pt.x = pt_xyz.x;
    pt.y = pt_xyz.y;
    pt.z = pt_xyz.z;
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_free_pub->publish(viewpoints_msg);
  // std::cout<<"free:"<<ind_of_free_viewpoints.size()<<std::endl;
}

void ViewPointManager::visualize_in_collision_viewpoints_()//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud2;
  sensor_msgs::msg::PointCloud2 viewpoints_msg2;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for(int i=0;i<vp_.viewpoint_num;i++)//每个元素
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    if(Get_littlebox_from_Hierarchy_ind(i)->Get_in_real_collision_())
    {
      pt_xyz = Get_littlebox_from_Hierarchy_ind(i)->GetPosition();
      pt.x = pt_xyz.x;
      pt.y = pt_xyz.y;
      pt.z = pt_xyz.z;
      viewpoints_cloud2.points.push_back(pt);
    }
    if(!Get_littlebox_from_Hierarchy_ind(i)->Get_in_collision_())//只要障碍的
    {
      continue;
    } 
    pt_xyz = Get_littlebox_from_Hierarchy_ind(i)->GetPosition();
    pt.x = pt_xyz.x;
    pt.y = pt_xyz.y;
    pt.z = pt_xyz.z;
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_collision_pub->publish(viewpoints_msg);

  pcl::toROSMsg(viewpoints_cloud2,viewpoints_msg2);
  viewpoints_msg2.header.frame_id = "map";
  viewpoint_real_collision_pub->publish(viewpoints_msg2);
  // std::cout<<"colli:"<<viewpoints_cloud.points.size()<<std::endl;
}

void ViewPointManager::visualize_connected_viewpoints_()//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for(int i=0;i<ind_of_connected_viewpoints.size();i++)//每个元素
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetPosition();
    pt.x = pt_xyz.x;
    pt.y = pt_xyz.y;
    pt.z = pt_xyz.z;
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_connected_pub->publish(viewpoints_msg);
  
  
}
void ViewPointManager::visualize_connected_viewpoints_in_exploringarea()//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for(int i=0;i<ind_of_connected_viewpoints_in_noncovered_area.size();i++)//每个元素
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetPosition();
    pt.x = pt_xyz.x;
    pt.y = pt_xyz.y;
    pt.z = pt_xyz.z;
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_connected_inexploringarea_pub->publish(viewpoints_msg);
}




void ViewPointManager::visualize_visited_viewpoints_()//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  pcl::PointCloud<pcl::PointXYZ> viewpoints_in_lineofsight_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  sensor_msgs::msg::PointCloud2 viewpoints_in_lineofsight_msg;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for(int i=0;i<ind_of_visited_viewpoints.size();i++)//每个元素
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    if(Get_littlebox_from_Hierarchy_ind(ind_of_visited_viewpoints[i])->InCurrentFrameLineOfSight())
    {
      pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_visited_viewpoints[i])->GetPosition();
      pt.x = pt_xyz.x;
      pt.y = pt_xyz.y;
      pt.z = pt_xyz.z;
      viewpoints_in_lineofsight_cloud.points.push_back(pt);
    }
    if(!Get_littlebox_from_Hierarchy_ind(ind_of_visited_viewpoints[i])->Visited())
    {
      continue;
    }
    pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_visited_viewpoints[i])->GetPosition();
    pt.x = pt_xyz.x;
    pt.y = pt_xyz.y;
    pt.z = pt_xyz.z;
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_visited_pub->publish(viewpoints_msg);

  pcl::toROSMsg(viewpoints_in_lineofsight_cloud,viewpoints_in_lineofsight_msg);
  viewpoints_in_lineofsight_msg.header.frame_id = "map";
  viewpoints_in_lineofsight_pub->publish(viewpoints_in_lineofsight_msg);
  
  if(Print_visited_points_bool)
  {
    Print_visited_points_bool=false;
    std::cout<<"visited个数为："<<viewpoints_cloud.points.size()<<"   robot_position:("<<robot_position_.x()<<","<<robot_position_.y()<<","<<robot_position_.z()<<")"<<std::endl;
    for(auto point: viewpoints_cloud.points)
    {
      std::cout<<"("<<point.x<<","<<point.y<<","<<point.z<<");  ";
    }
    std::cout<<std::endl;
  }
  
}
void ViewPointManager::visualize_visited_positions_(const std::vector<Eigen::Vector3d>& positions)//可视化
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointXYZ pt;
  // geometry_msgs::msg::Point pt_xyz;
  // int ind_of_middle_box,ind_of_little_box;
  for (const auto& position : positions)
  {
    // Hierarchy_ind_to_into_Two(ind_of_middle_box,ind_of_little_box,ind_of_free_viewpoints[i]);
    // pt_xyz = big_box->GetCell(ind_of_middle_box)->GetCell(ind_of_little_box)->GetPosition();
    
    // pt_xyz = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->GetPosition();
    pt.x = position.x();
    pt.y = position.y();
    pt.z = position.z();
    viewpoints_cloud.points.push_back(pt);
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  visited_positions_pub->publish(viewpoints_msg);
  
}

void ViewPointManager::visualize_all_viewpoints_()
{
  pcl::PointCloud<pcl::PointXYZ> viewpoints_cloud;
  sensor_msgs::msg::PointCloud2 viewpoints_msg;
  pcl::PointXYZ pt;
  geometry_msgs::msg::Point pt_xyz;
  for(int mid_box_ind=0; mid_box_ind<big_box->GetCellNumber();mid_box_ind++)
  {
    for(int little_box_ind=0; little_box_ind<big_box->GetCell(mid_box_ind)->GetCellNumber();little_box_ind++)
    {
      pt_xyz = big_box->GetCell(mid_box_ind)->GetCell(little_box_ind)->GetPosition();
      pt.x = pt_xyz.x;
      pt.y = pt_xyz.y;
      pt.z = pt_xyz.z;
      viewpoints_cloud.points.push_back(pt);
    }
  }
  pcl::toROSMsg(viewpoints_cloud,viewpoints_msg);
  viewpoints_msg.header.frame_id = "map";
  viewpoint_all_pub->publish(viewpoints_msg);
  // for(int little_box_ind=0; little_box_ind<uav_viewpoint_manager_->Get_big_box()->GetCell(mid_box_array_ind)->GetCellNumber(); little_box_ind++)
  //   {
  //     little_box_sub = uav_viewpoint_manager_->Get_big_box()->GetCell(mid_box_array_ind)->Ind2Sub(little_box_ind);
  //     litlle_box_sub_x = little_box_sub.x();
  //     litlle_box_sub_y = little_box_sub.y();  
  //     litlle_box_sub_z = little_box_sub.z();
  //     little_box_store_point = uav_viewpoint_manager_->Get_big_box()->GetCell(mid_box_array_ind)->GetCell(little_box_ind)->GetPosition();
  //     litlle_box_x = little_box_store_point.x;
  //     litlle_box_y = little_box_store_point.y;
  //     litlle_box_z = little_box_store_point.z;
  //     std::cout<<"("<<litlle_box_sub_x<<","<<litlle_box_sub_y<<","<<litlle_box_sub_z<<") :  "<<"("<<litlle_box_x<<","<<litlle_box_y<<","<<litlle_box_z<<")"<<std::endl;
  //   }
}
void ViewPointManager::sub_robot_pos_deal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//可视化
{
  Eigen::Vector3d robot_pose(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
  if(UpdateRobotPosition(robot_pose))//如果发生了位移
  {
    // std::cout<<"位移!!"<<std::endl;
    // uav_viewpoint_manager_->Get_grid_()->GetArrayInd()
    
  }
}
void ViewPointManager::sub_deal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)//可视化
{
  double origin_x,origin_y,origin_z,sub_x,sub_y,sub_z;
  sub_x=msg->pose.position.x;
  sub_y=msg->pose.position.y;
  sub_z=msg->pose.position.z;
  // for(int i=0;i<uav_viewpoint_manager_->Get_vp_().big_box_size_.x();i++)
  // {
  //   for(int h=0;h<uav_viewpoint_manager_->Get_vp_().big_box_size_.y();h++)
  //   {
  //     for(int j=0;j<uav_viewpoint_manager_->Get_vp_().big_box_size_.z();j++)
  //     {
  //       int ind = uav_viewpoint_manager_->Get_big_box()->Sub2Ind(i,h,j);//后面需要用rolling_grid滚动起来，实验array_ind
  //       //M? 需要设置放错机制防止访问超越向量长度  那个in bound
  //       int array_ind = uav_viewpoint_manager_->Get_grid_()->GetArrayInd(ind);
  //       origin_x = uav_viewpoint_manager_->Get_big_box()->GetCell(array_ind)->GetOrigin().x();
  //       origin_y = uav_viewpoint_manager_->Get_big_box()->GetCell(array_ind)->GetOrigin().y();
  //       origin_z = uav_viewpoint_manager_->Get_big_box()->GetCell(array_ind)->GetOrigin().z();
  //       std::cout<<"("<<i<<","<<h<<","<<j<<") :  "<<"("<<origin_x<<","<<origin_y<<","<<origin_z<<")"<<std::endl;
  //     }
  //   }
  // }
  //这里能将可视化效果视为成功的是：让以大方块为坐标原点的sub或者ind值，去换算成array_ind索引对应元素的成员变量origin_的时候。让origin_=（sub）*分辨率+大方块原点坐标
    int mid_box_ind = big_box->Sub2Ind(sub_x,sub_y,sub_z);//后面需要用rolling_grid滚动起来，实验array_ind
    //M? 需要设置放错机制防止访问超越向量长度  那个in bound
    
    int mid_box_array_ind = grid_->GetArrayInd(mid_box_ind);

    double litlle_box_x,litlle_box_y,litlle_box_z,litlle_box_sub_x,litlle_box_sub_y,litlle_box_sub_z;
    Eigen::Vector3i little_box_sub;
    geometry_msgs::msg::Point little_box_store_point;
    std::cout<<"("<<big_box->GetCell(mid_box_array_ind)->GetOrigin().x()<<","<<big_box->GetCell(mid_box_array_ind)->GetOrigin().y()<<","<<big_box->GetCell(mid_box_array_ind)->GetOrigin().z()<<") "<<std::endl;
    for(int little_box_ind=0; little_box_ind<big_box->GetCell(mid_box_array_ind)->GetCellNumber(); little_box_ind++)
    {
      little_box_sub = big_box->GetCell(mid_box_array_ind)->Ind2Sub(little_box_ind);
      litlle_box_sub_x = little_box_sub.x();
      litlle_box_sub_y = little_box_sub.y();  
      litlle_box_sub_z = little_box_sub.z();
      little_box_store_point = big_box->GetCell(mid_box_array_ind)->GetCell(little_box_ind)->GetPosition();
      litlle_box_x = little_box_store_point.x;
      litlle_box_y = little_box_store_point.y;
      litlle_box_z = little_box_store_point.z;
      std::cout<<"("<<litlle_box_sub_x<<","<<litlle_box_sub_y<<","<<litlle_box_sub_z<<") :  "<<"("<<litlle_box_x<<","<<litlle_box_y<<","<<litlle_box_z<<")"<<std::endl;
    }
    
    
}
void ViewPointManager::Abstract_connected_viewpoints(const std::unique_ptr<rolling_occupancy_grid_ns::RollingOccupancyGrid> & rolling_occupancy_grid_)
{
  std::vector<bool> checked(vp_.viewpoint_num, false);
  // robot_position_
  geometry_msgs::msg::PointStamped msg;
  msg.header.frame_id = "map";
  msg.point.x = robot_position_.x();
  msg.point.y = robot_position_.y();
  msg.point.z = robot_position_.z();
  robot_point->publish(msg);//可视化
  Eigen::Vector3i robot_Hierarchy_sub= Hierarchy_Pos_2_Hierarchy_sub(robot_position_);
  int robot_Hierarchy_ind = Hierarchy_sub_2_Hierarchy_ind(robot_Hierarchy_sub);
  checked[robot_Hierarchy_ind] = true;
  std::list<int> queue;
  queue.push_back(robot_Hierarchy_ind);
  Robot_Hier_ind_ = robot_Hierarchy_ind;
  update_Robot_Hier_ind_ = true;
  Eigen::Vector3d cur_pos;
  Eigen::Vector3d neighbor_pos,robot_pos;
  Eigen::Vector3i cur_sub_in_occupancy,neighbor_sub_in_occupancy,robot_sub_in_occupancy;
  int cur_Hierarchy_ind,neighbor_Hierarchy_ind;
  std::vector<Eigen::Vector3i> ray_cast_cells;
  int array_ind_for_occupancy;
  bool connected_flag;
  ind_of_connected_viewpoints.clear();
  ind_of_connected_viewpoints.push_back(robot_Hierarchy_ind);
  // std::cout<<"111111"<<std::endl;
  robot_pos = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(robot_Hierarchy_ind));
  robot_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(robot_pos);
  // std::vector<int> temp_push_back_neighbour_H_ind;
  connected_viewpoint_marker_.points.clear();//可视化
  bool test_temp;//可视化
  if(!rolling_occupancy_grid_->Get_occupancy_array_()->InRange(robot_sub_in_occupancy) )//假设它不在在occupancy内，直接下一个
  {//放错机制，一般不会出问题，robot所在点一般不可能是非range区域
    std::cout<<"robot_Hierarchy_ind不在occupancy范围内，无法选取connected viewpoint"<<std::endl;
  }
  else{
    test_temp=false;
    array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(robot_sub_in_occupancy);
    if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCell(array_ind_for_occupancy)!= rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
    {//这个是防错机制，防止机器人的坐标处于被障碍viewpoint管辖区域，
      //获取28个邻居点，
      std::vector<int> rotbot_neighbour_Hierarchy_ind = uav_viewpoints_neighbor_indices_[robot_Hierarchy_ind];
      Eigen::Vector3i robot_sub_tmep = Hierarchy_ind_2_Hierarchy_sub(robot_Hierarchy_ind);
      Eigen::Vector3i robot_sub_tmep_up = robot_sub_tmep;
      robot_sub_tmep_up.z() += 2; 
      Eigen::Vector3i robot_sub_tmep_down = robot_sub_tmep;
      robot_sub_tmep_down.z() -= 2; 
      Eigen::Vector3i robot_sub_tmep_left = robot_sub_tmep;
      robot_sub_tmep_left.x() -= 2; 
      Eigen::Vector3i robot_sub_tmep_right = robot_sub_tmep;
      robot_sub_tmep_right.x() += 2; 
      Eigen::Vector3i robot_sub_tmep_front = robot_sub_tmep;
      robot_sub_tmep_front.y() += 2; 
      Eigen::Vector3i robot_sub_tmep_back = robot_sub_tmep;
      robot_sub_tmep_back.y() -= 2; 
      std::vector<Eigen::Vector3i> robot_sub_tmep_nei_sub;
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_up);
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_down);
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_right);
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_left);
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_front);
      robot_sub_tmep_nei_sub.push_back(robot_sub_tmep_back);
      for(auto sub:robot_sub_tmep_nei_sub)
      {
        if(InRange_Hier_sub(sub))
        {
          for(auto neibour_ind:uav_viewpoints_neighbor_indices_[Hierarchy_sub_2_Hierarchy_ind(sub)]) 
          {
            rotbot_neighbour_Hierarchy_ind.push_back(neibour_ind);
          }
        }
      }

      // for(auto neibourind_Hier:uav_viewpoints_neighbor_indices_[robot_Hierarchy_ind])
      // {
      //   for(auto nei_neibour_hier:uav_viewpoints_neighbor_indices_[neibourind_Hier])
      //   {
      //     for(auto nei_nei_neibour_hier:uav_viewpoints_neighbor_indices_[nei_neibour_hier])
      //       rotbot_neighbour_Hierarchy_ind.push_back(nei_nei_neibour_hier);
      //   }
      // }
      std::vector<double> rotbot_neighbour_dis;

      Eigen::Vector3d neighbour_pos_tem;
      rotbot_neighbour_dis.resize(rotbot_neighbour_Hierarchy_ind.size());
      int tmep_swap;
      double tmep_double_swap;
      for(int i=0;i<rotbot_neighbour_Hierarchy_ind.size();i++)//赋值距离
      {
        neighbour_pos_tem = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(rotbot_neighbour_Hierarchy_ind[i]));
        //点到点的距离平方
        rotbot_neighbour_dis[i] = (neighbour_pos_tem.x()-robot_position_.x())*(neighbour_pos_tem.x()-robot_position_.x()) + (neighbour_pos_tem.y()-robot_position_.y())*(neighbour_pos_tem.y()-robot_position_.y())+(neighbour_pos_tem.z()-robot_position_.z())*(neighbour_pos_tem.z()-robot_position_.z());
      }
      for(int i=0;i<rotbot_neighbour_Hierarchy_ind.size()-1;i++)//冒泡排序，数值大的往下沉
      {
        for(int h=0;h<rotbot_neighbour_Hierarchy_ind.size()-i-1;h++)
        {
          if(rotbot_neighbour_dis[h]>rotbot_neighbour_dis[h+1])
          {
            //交换顺序往下沉
            tmep_double_swap = rotbot_neighbour_dis[h];
            rotbot_neighbour_dis[h] = rotbot_neighbour_dis[h+1];
            rotbot_neighbour_dis[h+1] = tmep_double_swap;

            tmep_swap = rotbot_neighbour_Hierarchy_ind[h];
            rotbot_neighbour_Hierarchy_ind[h] = rotbot_neighbour_Hierarchy_ind[h+1];
            rotbot_neighbour_Hierarchy_ind[h+1] = tmep_swap;
          }
        }
      }
      for(int i=0;i<rotbot_neighbour_Hierarchy_ind.size();i++)//
      {
        robot_pos = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(rotbot_neighbour_Hierarchy_ind[i]));
        robot_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(robot_pos);
        array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(robot_sub_in_occupancy);
        if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCell(array_ind_for_occupancy) == rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
        {
          robot_Hierarchy_ind = rotbot_neighbour_Hierarchy_ind[i];
          queue.pop_front();
          queue.push_back(robot_Hierarchy_ind);
          Robot_Hier_ind_ = robot_Hierarchy_ind;
          std::cout<<"所在点在障碍viewpoint管辖范围，并选择最近的空闲邻居"<<std::endl;
          test_temp=true;
          break;
        }
      }
    }
    /////
    if(Set_robot_neigbour_connected)
    {
      Eigen::Vector3i robot_posistion_in_sub = Hierarchy_ind_2_Hierarchy_sub(robot_Hierarchy_ind);
      std::vector<Eigen::Vector3i> idx_addon_temp;
      for (int x = -1; x <= 1; ++x) {
          for (int y = -1; y <= 1; ++y) {
              for (int z = -1; z <= 1; ++z) {
                  idx_addon_temp.push_back(Eigen::Vector3i(x, y, z));
              }
          }
      }
      for(auto add_sub : idx_addon_temp)
      {
        int tem_hier_ind = Hierarchy_sub_2_Hierarchy_ind(robot_posistion_in_sub+add_sub);
        if(Get_littlebox_from_Hierarchy_ind(tem_hier_ind)->In_free())
        {
          ind_of_connected_viewpoints.push_back(tem_hier_ind);
          queue.push_back(tem_hier_ind);
        }
      }
    }
    
    ////
    while (!queue.empty())
    {
      // temp_push_back_neighbour_H_ind.clear();
      cur_Hierarchy_ind = queue.front();
      queue.pop_front();
      cur_pos = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(cur_Hierarchy_ind));
      cur_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(cur_pos);
      for (int i = 0; i < uav_viewpoints_neighbor_indices_[cur_Hierarchy_ind].size(); i++)//3*3邻居
      {
        connected_flag = true;
        
        neighbor_Hierarchy_ind = uav_viewpoints_neighbor_indices_[cur_Hierarchy_ind][i];
        if(Get_littlebox_from_Hierarchy_ind(neighbor_Hierarchy_ind)->Get_in_collision_() || checked[neighbor_Hierarchy_ind])//判断viewpoint非空闲点，和已经加入的点 直接continue
        {
          continue;
        }
        //先获取cur_ind和neighbor_ind对应的pos值
        neighbor_pos = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(neighbor_Hierarchy_ind));
        //把pos值转成occupancy的sub值
        neighbor_sub_in_occupancy = rolling_occupancy_grid_->Get_occupancy_array_()->Pos2Sub(neighbor_pos);
        
        // std::cout<<"--------------"<<std::endl;
        if( !rolling_occupancy_grid_->Get_occupancy_array_()->InRange(neighbor_sub_in_occupancy))
        {//邻居不在其中，那么直接下一轮
          continue;
        }
        rolling_occupancy_grid_->RayTraceHelper_for_viewpoint_connected(neighbor_sub_in_occupancy, cur_sub_in_occupancy, ray_cast_cells);//获取occupancy从一点到另一点 路径上经过的cells的 sub值；从后往前，因为判断不过的地方一般在后面
        for(int h=0;h<ray_cast_cells.size();h++)//对于每个cells确定它是否是占据的
        {
          if(!rolling_occupancy_grid_->Get_rolling_grid_()->InRange(ray_cast_cells[h]))
          {
            // std::cout<<"这就是bug"<<std::endl;
            connected_flag=false;
            break;
          }
          array_ind_for_occupancy = rolling_occupancy_grid_->Get_rolling_grid_()->GetArrayInd(ray_cast_cells[h]);
          if(rolling_occupancy_grid_->Get_occupancy_array_()->GetCell(array_ind_for_occupancy) != rolling_occupancy_grid_ns::RollingOccupancyGrid::CellState::FREE)
          {//如果不是free的，那么就不能视作连接
            connected_flag=false;
            break;
          }
        }
          if(connected_flag)//这里是结算操作
          {
            if(vp_.disable_outsider_viewpoints)
            {
              int ind_of_middle_box, ind_of_little_box;
              Hierarchy_ind_to_into_Two(ind_of_middle_box, ind_of_little_box, neighbor_Hierarchy_ind);
              Eigen::Vector3i mbox_sub = grid_->Ind2Sub(ind_of_middle_box);
              if (!(mbox_sub.x()>0 && mbox_sub.x()<(vp_.big_box_size_.x()-1) && mbox_sub.y()>0 && mbox_sub.y()<(vp_.big_box_size_.y()-1)))
              {
                // std::cout<<"在外围"<<std::endl;
                continue;
              }
            }
            ind_of_connected_viewpoints.push_back(neighbor_Hierarchy_ind);
            queue.push_back(neighbor_Hierarchy_ind);
            checked[neighbor_Hierarchy_ind] = true;

            geometry_msgs::msg::Point temp_point;//可视化
            temp_point.x=cur_pos.x();
            temp_point.y=cur_pos.y();
            temp_point.z=cur_pos.z();
            connected_viewpoint_marker_.points.push_back(temp_point);//可视化
            Eigen::Vector3d A = Hierarchy_sub_2Hierarchy_Pos_( Hierarchy_ind_2_Hierarchy_sub(neighbor_Hierarchy_ind));
            temp_point.x=A.x();
            temp_point.y=A.y();
            temp_point.z=A.z();
            connected_viewpoint_marker_.points.push_back(temp_point);//可视化
          }
        }
      //   if(connected_flag)//这里是结算操作
      //   {
      //     temp_push_back_neighbour_H_ind.push_back(neighbor_Hierarchy_ind);
      //   }
      // }
      // if(temp_push_back_neighbour_H_ind.size()>=2)//防止窜层粘连，
      // {
      //   for(int i=0;i<temp_push_back_neighbour_H_ind.size();i++)
      //   {
      //     ind_of_connected_viewpoints.push_back(temp_push_back_neighbour_H_ind[i]);
      //     queue.push_back(temp_push_back_neighbour_H_ind[i]);
      //     checked[temp_push_back_neighbour_H_ind[i]] = true;

      //     geometry_msgs::msg::Point temp_point;//可视化
      //     temp_point.x=cur_pos.x();
      //     temp_point.y=cur_pos.y();
      //     temp_point.z=cur_pos.z();
      //     connected_viewpoint_marker_.points.push_back(temp_point);//可视化
      //     Eigen::Vector3d A = Hierarchy_sub_2Hierarchy_Pos_( Hierarchy_ind_2_Hierarchy_sub(temp_push_back_neighbour_H_ind[i]));
      //     temp_point.x=A.x();
      //     temp_point.y=A.y();
      //     temp_point.z=A.z();
      //     connected_viewpoint_marker_.points.push_back(temp_point);//可视化
      //   }
        
      // }


    }//while
    if(ind_of_connected_viewpoints.size()<3 && last_ind_of_connected_viewpoints.size()>3)
    {
      
      if(traspass)
      {
        std::cout<<"Viewpont更新失败，并且越界了"<<std::endl;
        last_ind_of_connected_viewpoints.clear();
        for(auto pos:last_viewpoint_pos)
        {
         auto sub_ = Hierarchy_Pos_2_Hierarchy_sub(pos);
         if(InRange_Hier_sub(sub_))
          last_ind_of_connected_viewpoints.push_back(Hierarchy_sub_2_Hierarchy_ind(sub_));
        }
      }
      ind_of_connected_viewpoints = last_ind_of_connected_viewpoints;
      // std::cout<<"ind_of_free_viewpoints数量为："<<ind_of_free_viewpoints.size()<<std::endl;
    }
    else
    {
      last_ind_of_connected_viewpoints = ind_of_connected_viewpoints;
      last_viewpoint_pos.clear();
      for(auto Hier_ind:ind_of_connected_viewpoints)
      {
        last_viewpoint_pos.push_back(Hierarchy_ind_2Hierarchy_Pos_(Hier_ind));
      }
    }
    if(ind_of_connected_viewpoints.size()<3)
    {
      // std::cout<<"警告，viewpoint数量为："<<ind_of_connected_viewpoints.size()<<std::endl;
      // std::cout<<"origin_ ("<<origin_.x()<<","<<origin_.y()<<","<<origin_.y()<<")"<<std::endl;
      if(vp_.first_frame_grace)
      {
        vp_.first_frame_grace = false;
        ind_of_connected_viewpoints = ind_of_free_viewpoints;
        last_ind_of_connected_viewpoints = ind_of_connected_viewpoints;
      }
      
      // std::cout<<"ind_of_free_viewpoints数量为："<<ind_of_free_viewpoints.size()<<std::endl;
    }

    if(test_temp)
    {
      // std::cout<<"ind_of_connected_viewpoints.size: "<<ind_of_connected_viewpoints.size()<<std::endl;;
    }
    for(int ind=0;ind<vp_.viewpoint_num;ind++)
    {
      Get_littlebox_from_Hierarchy_ind(ind)->SetConnected(false);

    }
    // std::shared_ptr<viewpoint_ns::ViewPoint> viewpoint_ptr;
    // Eigen::Vector3d position_vec;
    // geometry_msgs::msg::Point position;
    for(int i=0;i<ind_of_connected_viewpoints.size();i++)//连接点的都设置为true
    {
      Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i])->SetConnected(true);
      // viewpoint_ptr = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[i]);
      // viewpoint_ptr->SetConnected(true);
      // position_vec = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(ind_of_connected_viewpoints[i]));
      // position.x = position_vec.x();
      // position.y = position_vec.y();
      // position.z = position_vec.z();
      // viewpoint_ptr->SetPosition(position);
    }
    
    // viewpoint_ptr = Get_littlebox_from_Hierarchy_ind(ind_of_connected_viewpoints[0]);
    // position_vec = Hierarchy_sub_2Hierarchy_Pos_(Hierarchy_ind_2_Hierarchy_sub(ind_of_connected_viewpoints[0]));

    // std::cout<<"("<<position_vec.x()<<","<<position_vec.y()<<","<<position_vec.z()<<");  ("<<viewpoint_ptr->GetPosition().x<<","<<viewpoint_ptr->GetPosition().y<<","<<viewpoint_ptr->GetPosition().z<<"); "<<std::endl;

  }
}
bool ViewPointManager::Get_nearest_connected_viewpoint(const Eigen::Vector3d& pos, int& Get_ind)
{
  double min_dist = DBL_MAX;
  int min_ind;
  double temp_dist;
  bool success=false;
  geometry_msgs::msg::Point point_1;
  point_1.x = pos.x();
  point_1.y = pos.y();
  point_1.z = pos.z();

  for(auto Hier_ind : ind_of_connected_viewpoints)
  {
    temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(
              point_1, Get_littlebox_from_Hierarchy_ind(Hier_ind)->GetPosition());


    if(temp_dist < min_dist)
    {
      min_dist = temp_dist;
      min_ind = Hier_ind;
      success = true;
    }
  }
  Get_ind = min_ind;
  return success;
}
void ViewPointManager::Get_connected_Hier_ind_vec_in_exploring_area(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world_)
{
  geometry_msgs::msg::Point viewpoint_position;
  ind_of_connected_viewpoints_in_noncovered_area.clear();
  //工作
  for (auto Hier_ind : ind_of_connected_viewpoints)//对于每个viewpoints
  {
    viewpoint_position = Get_littlebox_from_Hierarchy_ind(Hier_ind)->GetPosition();
    Eigen::Vector3i sub =
          grid_world_->get_subspaces_()->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));//候选点转到subspaces_尺度上的Sub
    if (grid_world_->get_subspaces_()->InRange(sub))
    {
      int cell_ind = grid_world_->get_subspaces_()->Sub2Ind(sub);
      if(grid_world_->get_subspaces_()->GetCell(cell_ind).GetStatus() != grid_world_ns::CellStatus::COVERED)
      {
        ind_of_connected_viewpoints_in_noncovered_area.push_back(Hier_ind);
        Get_littlebox_from_Hierarchy_ind(Hier_ind)->Setin_covered_box(false);
      }
      else{
        //M? 可能回报 gearray的错 但是我觉得应该没问题
        Get_littlebox_from_Hierarchy_ind(Hier_ind)->Setin_covered_box(true);
      }
    }
  }
}
// template <class PlannerCloudPointType>
// void ViewPointManager::Get_frontier_viewpoint(const std::unique_ptr<planning_env_ns::PlanningEnv> & planning_env_, const std::vector<Eigen::Vector3d> & visited_positions_)
// {
//   
//   double intervel_radius = 2;
//   double para_visited_radius = 0.3;
//   int least_visible_unknown_num = 5;
//   geometry_msgs::msg::Point temp_point1,temp_point2;
//   double temp_dist;
//   bool addmision_flag;
//   std::shared_ptr<viewpoint_ns::ViewPoint> temp_ptr;
//   for(auto new_point:  planning_env_->frontier_center_point->cloud_->points)
//   {
//     addmision_flag=true;
//     //加入
//     if(frontier_viewpoints.size()==0)//没有的时候直接进
//     {
//       frontier_viewpoints.push_back(temp_ptr);
//       frontier_viewpoints[0] = std::shared_ptr<viewpoint_ns::ViewPoint> (new viewpoint_ns::ViewPoint(new_point.x,new_point.y,new_point.z));
//       frontier_viewpoints[0]->Reset();
//       continue;
//     }
//     //判断，距离够大的留下来
//     for(auto viewpoint: frontier_viewpoints)
//     {
//       temp_point1.x = new_point.x;
//       temp_point1.y = new_point.y;
//       temp_point1.z = new_point.z;
//       temp_point2 = viewpoint->GetPosition();
//       temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(temp_point1, temp_point2);
//       if(temp_dist < intervel_radius)
//       {
//         addmision_flag=false;
//         break;
//       }
//     }
    
//     if(addmision_flag)
//     {
//       frontier_viewpoints.push_back(temp_ptr);
//       frontier_viewpoints[frontier_viewpoints.size()-1] = std::shared_ptr<viewpoint_ns::ViewPoint> (new viewpoint_ns::ViewPoint(new_point.x,new_point.y,new_point.z));
//       frontier_viewpoints[frontier_viewpoints.size()-1]->Reset();
//     }
//     //加入
//   }

//   //更新下面判断点使命是否完成的依据   ，收集所有半径内的unknow点，让下面判断是否还能够覆盖到这些点（只收集了有限范围内的点）
//   pcl::PointXYZI tem_point;
//   std::vector<int> in_range_indices;
//   std::vector<float> in_range_sqdist;
//   planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points.clear();
//   kdtree_occupy_unknow_->setInputCloud(planning_env_->Occupancy_unknow_cloud->cloud_);
//   for(auto viewpoint: frontier_viewpoints)
//   {
//     temp_point1 = viewpoint->GetPosition();
//     tem_point.x = temp_point1.x;
//     tem_point.y = temp_point1.y;
//     tem_point.z = temp_point1.z;
//     kdtree_occupy_unknow_->radiusSearch(tem_point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
//     for(auto ind : in_range_indices)
//     {
//       planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points.push_back(planning_env_->Occupancy_unknow_cloud->cloud_->points[ind]);
//     }
//   }
//   //更新下面判断点使命是否完成的依据
//   planning_env_->Occupancy_unknow_cloud->Publish();//可视化
//   planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->Publish();//可视化

//   //踢出去  没被踢出去的都连接地铁//工作
//   bool erase_flag = false;
//   Eigen::Vector3d temp_forntier_point_position;
//   int visible_unknow_point_count=0;
//   for(int i=0; i<frontier_viewpoints.size(); i++)
//   {
//     //（real collision）
//     if(check_viewpoint_in_real_collision(planning_env_->Get_rolling_occupancy_grid_(),*frontier_viewpoints[i]))
//     {
//       frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
//       i--;
//       continue;
//     }
//     //（real collision）
//     //机器人访问到了也踢出去
//     erase_flag = false;
//     for ( auto position : visited_positions_)
//     {
//       temp_point1 = frontier_viewpoints[i]->GetPosition();
//       temp_point2.x = position.x(); 
//       temp_point2.y = position.y(); 
//       temp_point2.z = position.z(); 
//       temp_dist = misc_utils_ns::PointXYZDist<geometry_msgs::msg::Point, geometry_msgs::msg::Point>(temp_point1, temp_point2);
//       if(temp_dist < para_visited_radius)
//       {
//         erase_flag = true;
//         break;
//       }
//     }
//     if(erase_flag)
//     {
//       frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
//       i--;
//       continue;
//     }
//     //机器人访问到了也踢出去
//     temp_forntier_point_position.x() = frontier_viewpoints[i]->GetPosition().x;
//     temp_forntier_point_position.y() = frontier_viewpoints[i]->GetPosition().y;
//     temp_forntier_point_position.z() = frontier_viewpoints[i]->GetPosition().z;
//     //TODO半径之内不再能够照射到unknow点也踢出去
//     // 只针对在 范围内 的这些点踢出去，不然超出occupancy的范围，这样的剔除是无意义的
//     visible_unknow_point_count = 0;
//     if(InRange_Hier_sub(Hierarchy_Pos_2_Hierarchy_sub(temp_forntier_point_position)))
//     {
//       //更新壁垒
//       UpdateViewPointCoverage_for_frontier_viewpoint<PlannerCloudPointType>(planning_env_->GetDiffCloud(),*frontier_viewpoints[i]);
//       UpdateRolledOverViewPointCoverage_for_frontier_viewpoint<PlannerCloudPointType>(planning_env_->GetStackedCloud(),*frontier_viewpoints[i]);
//       //计算能看见的unknow数量
//       for(auto point : planning_env_->Occupancy_unknow_cloud_for_frontier_viewpoint->cloud_->points)
//       {
//         if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
//                                     temp_forntier_point_position,
//                                     vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
//                                     vp_.kInFovZDiffThreshold))//0.2125 12  3.5294 1.5
//         {
//           if(frontier_viewpoints[i]->CheckVisibility<pcl::PointXYZI>(point, vp_.kCoverageOcclusionThr))
//           {
//             visible_unknow_point_count++;
//           }
//         }
        
//       }
//       if(visible_unknow_point_count <= least_visible_unknown_num)
//       {
//         frontier_viewpoints.erase(frontier_viewpoints.begin()+i);
//         i--;
//         continue;
//       }
//       std::cout<<"visible_unknow_point_count"<<visible_unknow_point_count<<std::endl;
//     }
//     //半径之内不再能够照射到unknow点也踢出去
    



//     //TODO连接地铁 
//     // if(!)//没连接过的地铁去连接
//     // {

//     // }
//     //连接地铁
//   }
//   //踢出去（real collision）
  

void ViewPointManager::Push_boundary_index(int Hier)
{
  boundary_index.push_back(Hier);
}
void ViewPointManager::Clear_boundary_index()
{
  boundary_index.clear();
}
// }
}  // namespace uav_viewpoint_manager_ns






int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node_handle = std::make_shared<rclcpp::Node>("test_node"); 
  // std::shared_ptr<uav_viewpoint_manager_ns::ViewPointManager> uav_viewpoint_manager_;
  // uav_viewpoint_manager_ = std::make_unique<uav_viewpoint_manager_ns::ViewPointManager>(node_handle);
  rclcpp::Rate rate(20);
  std::cout<<"-----------"<<std::endl;
  std::vector<std::vector<Eigen::Vector3i>> idx_addon;
  idx_addon.resize(3);
  
  

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
  
  
  // Eigen::Vector3i BB(0,0,0);
  // Eigen::Vector3i CC(1,1,1);
  // Eigen::Vector3i DD;
  // for(int i=0;i<idx_addon.size();i++)
  // {
  //   for(int h=0;h<idx_addon[i].size();h++)
  //   {
  //     DD=BB+idx_addon[i][h];
  //     std::cout<<"("<<DD.x()<<","<<DD.y()<<","<<DD.z()<<")  ";
  //   }
      
  // }
  // std::cout<<std::endl;
  // for(int i=0;i<idx_addon.size();i++)
  // {
  //   for(int h=0;h<idx_addon[i].size();h++)
  //   {
  //     DD=CC+idx_addon[i][h];
  //     std::cout<<"("<<DD.x()<<","<<DD.y()<<","<<DD.z()<<")  ";
  //   }
      
  // }
  // std::cout<<std::endl;
  // std::cout<<"11111";

  std::vector<int> intvect(3, 10);
  
  intvect.insert(intvect.end(), 2, 11);
  intvect.push_back(1);
  intvect.push_back(1);
  intvect.push_back(4);
  intvect.push_back(3);
  intvect.push_back(9);
  intvect.push_back(4);
  intvect.push_back(3);
  intvect.push_back(5);
  
  for(int h=0;h<intvect.size();h++)
    {
      std::cout<<intvect[h]<<" ";
    }
    std::cout<<"-----------"<<std::endl;
  std::cout<<std::min(std::min(3, 2), 4);
  std::cout<<"-----------"<<std::endl;
  while (true)
  {
    /* code */
    rate.sleep();
    // uav_viewpoint_manager_->visualize_all_viewpoints_();
    rclcpp::spin_some(node_handle);
  }
  
  
  return 0;
}