import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.actions import TimerAction


def generate_launch_description():
    
#TODO 记得检查所有参数间的配合   之前就是局部路径规划的 和  grid_word 不合吃了亏 
#TODO 地铁 还有个可能出bug的点就是 原本keypose的选值就只限定机器人当前帧位置， 但是为了解决某些bug 现在如果当前位置被所管辖的viewpoint设定为不连接 则选取最近的连接viewpoint作为keypose新坐标
#TODO 关于visible_unknow_point_count 这里还可以看一下是否又可以改进的地方
    # M? 下面这加了这段 不知道garage  Eigen::Vector3i robot_posistion_in_sub = Hierarchy_ind_2_Hierarchy_sub(robot_Hierarchy_ind);
#M? 可能出现的问题  connected的整体错位：   这是由于机器人位姿一直是靠墙导致 在  局部框滚动 的时候一直沿用的上一次的ind_of_connected_viewpoint 出现一帧基本上就完蛋了（会影响地铁构建）  所以每个小中方格适当选大一点
    uav_planner_node = Node(
        package='uav_planner',
        executable='main_node',
        output='screen',
        parameters=[
                    {"kAutoStart": True},
                    {"kRushHome": False},
                    {"kExtendWayPoint":True},
                    {"kUseLineOfSightLookAheadPoint":True},
                    {"kNoExplorationReturnHome": True},
                    {"kExtendWayPointDistanceBig":8.0},
                    {"kExtendWayPointDistanceSmall":3.5},
                    {"kKeyposeCloudDwzFilterLeafSize":0.2},
                    {"kRushHomeDist":10.0},
                    {"kAtHomeDistThreshold":0.5},
                    {"kLookAheadDistance":8.0},
                    {"kUseMomentum":True},
                    {"kDirectionChangeCounterThr":6},
                    {"kDirectionNoChangeCounterThr":5},
                    #PlanningEnv
                    {"kUseFrontier":True},
                    {"kFrontierClusterTolerance":1.0},
                    {"kFrontierClusterMinSize":30},
                    {"kUseCoverageBoundaryOnFrontier":False},
                    {"kUseCoverageBoundaryOnObjectSurface":False},
                    {"valid_frontier_point_num_thred":500},#100以内的簇才有资格让frontier viewpoint来操作
                    # #Rolling occupancy grid
                    {"rolling_occupancy_grid/resolution_x":0.2},
                    {"rolling_occupancy_grid/resolution_y":0.2},
                    {"rolling_occupancy_grid/resolution_z":0.2},
                    
                    {"kSurfaceCloudDwzLeafSize":0.3},
                    {"kCollisionCloudDwzLeafSize":0.2},
                    {"kKeyposeCloudStackNum":5},
                    {"kPointCloudRowNum":50},
                    {"kPointCloudColNum":50},
                    {"kPointCloudLevelNum":30},
                    {"kMaxCellPointNum":100000},

                    {"kPointCloudCellSize":18.0},#这个是一个格子occupancy的长  下面是宽
                    {"kPointCloudCellHeight":1.8},

                    {"kPointCloudManagerNeighborCellNum":5},
                    {"kCoverCloudZSqueezeRatio":2.0},
                    # KeyposeGraph
                    {"keypose_graph/kAddNodeMinDist":1.0},
                    {"keypose_graph/kAddNonKeyposeNodeMinDist":0.5},
                    {"keypose_graph/kAddEdgeConnectDistThr":3.0},
                    {"keypose_graph/kAddEdgeToLastKeyposeDistThr":3.0},
                    {"keypose_graph/kAddEdgeVerticalThreshold":1.0},
                    {"keypose_graph/kAddEdgeCollisionCheckResolution":0.4},
                    {"keypose_graph/kAddEdgeCollisionCheckRadius":0.4},
                    {"keypose_graph/kAddEdgeCollisionCheckPointNumThr":1},
                    {"keypose_graph/ckeck_loger_than_length":4},  #去除过 错误加入的kepose 跨越未知区域路径  的时候使用  ①选
                    {"keypose_graph/eliminate_edge_non_free_thr":10},#出现10个nonfree以上的我们认为是错误跨越的路径  给它去除掉②除掉            
                    # ViewPointManager
                    # {"viewpoint_manager/number_x":40},
                    # {"viewpoint_manager/number_y":40},
                    # {"viewpoint_manager/number_z":1},
                    {"viewpoint_manager/resolution_x":0.6},#要保证occupancy包括他们
                    {"viewpoint_manager/resolution_y":0.6},
                    {"viewpoint_manager/resolution_z":0.6},
                    {"viewpoint_manager/big_box_size_x":5},
                    {"viewpoint_manager/big_box_size_y":5},
                    {"viewpoint_manager/big_box_size_z":3},
                    {"viewpoint_manager/min_box_size_x":10},
                    {"viewpoint_manager/min_box_size_y":10},
                    {"viewpoint_manager/min_box_size_z":3},
                    {"viewpoint_manager/incollision_barrier_level":0},#0是自己裸奔， 1是四周往外一层occupy  2是两层 3是三层
                    {"viewpoint_manager/first_frame_grace":True},
                    {"check_collision_margin":10},#这里是把1/5的边搞掉不判断
                    {"intervel_radius":4.0}, #frontier点之间不小于
                    {"para_visited_radius":1.5},#标记访问了半径
                    {"para_visited_radius_z":2.5},
                    {"position_add_high_for_guv":0.0},
                    {"least_visible_unknown_num":150},#覆盖到未标绿点小于这个数 则使命结束
                    {"least_visible_free_num":100},
                    {"Set_robot_neigbour_connected":True},
                    {"in_collision_viewpoint_get_connected_viewpoint_dis":0.1},
                    {"search_radius_thr":2.0},#如果frontier viewpoint点卡在墙里了，用周围 1 内的空闲点设置新的position



                    {"kGreedyViewPointSampleRange":3},
                    {"kLocalPathOptimizationItrMax":10},
                    {"kViewPointCollisionMargin":0.5},
                    {"kViewPointCollisionMarginZPlus":0.5},
                    {"kViewPointCollisionMarginZMinus":0.5},
                    {"kCollisionGridZScale":1.0},
                    {"kCollisionGridResolutionX":0.2},
                    {"kCollisionGridResolutionY":0.2},
                    {"kCollisionGridResolutionZ":0.0},
                    {"kCollisionPointThr":1},
                    {"kLineOfSightStopAtNearestObstacle":True},
                    {"kViewPointHeightFromTerrain":0.75},
                    {"kViewPointHeightFromTerrainChangeThreshold":0.6},
                    {"kCheckDynamicObstacleCollision":False},
                    {"kCollisionFrameCountMax":3},
                    {"kSensorRange":7.5},
                    {"kNeighborRange":3.0},
                    {"kCoverageOcclusionThr":0.3},
                    {"kCoverageDilationRadius":1.0},#这项废弃
                    # Grid World
                    {"kGridWorldXNum":121},
                    {"kGridWorldYNum":121},
                    {"kGridWorldZNum":121},
                    {"kGridWorldCellHeight":3.0},#这项废弃
                    {"kGridWorldNearbyGridNum":5},
                    {"kMinAddPointNumSmall":40},
                    {"kMinAddPointNumBig":60},
                    {"kMinAddFrontierPointNum":30},
                    {"kCellExploringToCoveredThr":1},
                    {"kCellCoveredToExploringThr":10},
                    {"kCellExploringToAlmostCoveredThr":10},
                    {"kCellAlmostCoveredToExploringThr":20},
                    {"kCellUnknownToExploringThr":1},
                    {"Estime_any_Frontier_viewpoint_asexploring":False},
                    # Visualization
                    {"kExploringSubspaceMarkerColorGradientAlpha":True},
                    {"kExploringSubspaceMarkerColorMaxAlpha":0.8},
                    {"kExploringSubspaceMarkerColorR":0.0},
                    {"kExploringSubspaceMarkerColorG":1.0},
                    {"kExploringSubspaceMarkerColorB":0.0},
                    {"kExploringSubspaceMarkerColorA":1.0},
                    {"kLocalPlanningHorizonMarkerColorR":0.0},
                    {"kLocalPlanningHorizonMarkerColorG":1.0},
                    {"kLocalPlanningHorizonMarkerColorB":0.0},
                    {"kLocalPlanningHorizonMarkerColorA":1.0},
                    {"kLocalPlanningHorizonMarkerWidth":0.3},
                    {"kLocalPlanningHorizonHeight":3.0},
                    
                    {"kextendz":False},
                    {"pub_mode_":1}, #0是原版 1是发一次环境再更新一次 其余数值为路径规划一直工作，不等待环境更新
                    {"waypointpub_triger_time":50.0},
                    {"excute_triger_time":100.0},
                    {"low_speed_run":False  },
                    {"use_frontier_viewpoint":True},
                    {"least_explore_time_s":60}, #最少探索时长，防止开头就停
                    {'visitd_radius': 2.0},#访问半径
                    {"visitd_radius_z":2.0},
                    {'show_time': False },
                    {"keypose_graph/add_edge_Thr": 200},
                    {"invalidate_viewpoint_and_point_in_covered_boxes":True},
                    {"viewpoint_manager/disable_outsider_viewpoints":False},
                    {"viewpoint_manager/smaller_visited_radius":False},
                    {"use_time_interval":False},
                    {"way_point_in_free":False},#tare的车导航比较靠谱，可以false  无人机都不大行 得用true
                    {"smaller_viewpoint_wait_time_relocation":0},
                    ],
        namespace="uav0",
        remappings=[
        # 重映射 position_cmd 话题
            # ('uav_3d_lidar_msg', '/drone_0_pcl_render_node/cloud'),
            # ('uav_state_estimation', '/drone_0_visual_slam/odom'),
            # ('uav_3d_lidar_msg', '/registered_scan'),
            # ('uav_state_estimation', '/state_estimation_at_scan'),
            ('/uav_3d_lidar_msg', '/registered_scan'),
            ('/uav_state_estimation', '/state_estimation_at_scan'),
            ('/terrain_map_ext', "/terrain_map_ext"),
            ('/terrain_map', "/terrain_map"),
            ('/start_signal', "/start_signal"),
            ('/way_point', '/way_point'),#pose类型 用于给ros1的那个环境发消息

            #注意！！！ 冠名命名空间后，“/”不能缺，不能随便的去掉“/”
            ],
            
            
    )
    rviz_para_dir = os.path.join(get_package_share_directory('uav_planner'), 'rviz', 'uav_planner_tunnel_guv.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_para_dir]
    )

    ld = LaunchDescription()
    ld.add_action(uav_planner_node)
    timer_duration = 1.0  # Set the delay duration in seconds##延时启动节点
    ld.add_action(TimerAction(actions=[rviz_node], period=timer_duration))

    return ld
