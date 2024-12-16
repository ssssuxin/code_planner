import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
from launch.actions import TimerAction


def generate_launch_description():
    

    uav_planner_node = Node(
        package='uav_planner',
        executable='main_node',
        output='screen',
        parameters=[             
                    
                    
                    

                    {"kAutoStart": True},
                    {"kRushHome": True},
                    {"kExtendWayPoint":True},
                    {"kUseLineOfSightLookAheadPoint":True},
                    {"kNoExplorationReturnHome": True},
                    {"kExtendWayPointDistanceBig":8.0},
                    {"kExtendWayPointDistanceSmall":4.5},
                    {"kKeyposeCloudDwzFilterLeafSize":0.2},
                    {"kRushHomeDist":10.0},
                    {"kAtHomeDistThreshold":0.5},
                    {"kLookAheadDistance":8.0},
                    {"kUseMomentum":False},
                    {"kDirectionChangeCounterThr":6},
                    {"kDirectionNoChangeCounterThr":5},
                    # #PlanningEnv
                    {"kUseFrontier":True},
                    {"kFrontierClusterTolerance":1.0},
                    {"kFrontierClusterMinSize":30},
                    {"kUseCoverageBoundaryOnFrontier":False},
                    {"kUseCoverageBoundaryOnObjectSurface":False},
                    {"valid_frontier_point_num_thred":1000},#100以内的簇才有资格让frontier viewpoint来操作
                    # #Rolling occupancy grid
                    {"rolling_occupancy_grid/resolution_x":0.3},
                    {"rolling_occupancy_grid/resolution_y":0.3},
                    {"rolling_occupancy_grid/resolution_z":0.3},
                    
                    {"kSurfaceCloudDwzLeafSize":0.5},
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
                    # # KeyposeGraph
                    {"keypose_graph/kAddNodeMinDist":1.0},
                    {"keypose_graph/kAddNonKeyposeNodeMinDist":0.5},
                    {"keypose_graph/kAddEdgeConnectDistThr":3.0},
                    {"keypose_graph/kAddEdgeToLastKeyposeDistThr":3.0},
                    {"keypose_graph/kAddEdgeVerticalThreshold":1.0},
                    {"keypose_graph/kAddEdgeCollisionCheckResolution":0.4},
                    {"keypose_graph/kAddEdgeCollisionCheckRadius":0.4},
                    {"keypose_graph/kAddEdgeCollisionCheckPointNumThr":1},
                    {"keypose_graph/ckeck_loger_than_length":0},  #去除过 错误加入的kepose 跨越未知区域路径  的时候使用  ①选
                    {"keypose_graph/eliminate_edge_non_free_thr":1},#出现10个nonfree以上的我们认为是错误跨越的路径  给它去除掉②除掉  
                    # # ViewPointManager
                    # # {"viewpoint_manager/number_x":40},
                    # # {"viewpoint_manager/number_y":40},
                    # # {"viewpoint_manager/number_z":1},
                    {"viewpoint_manager/resolution_x":1.2},
                    {"viewpoint_manager/resolution_y":1.2},
                    {"viewpoint_manager/resolution_z":0.6},
                    {"viewpoint_manager/big_box_size_x":5},
                    {"viewpoint_manager/big_box_size_y":5},
                    {"viewpoint_manager/big_box_size_z":5},
                    {"viewpoint_manager/min_box_size_x":6},
                    {"viewpoint_manager/min_box_size_y":6},
                    {"viewpoint_manager/min_box_size_z":2},
                    {"viewpoint_manager/incollision_barrier_level":2},#0是自己裸奔， 1是四周往外一层occupy  2是两层 3是三层
                    {"viewpoint_manager/first_frame_grace":True},
                    {"check_collision_margin":10},#这里是把1/5的边搞掉不判断
                    {"intervel_radius":10.0}, #frontier点之间不小于
                    {"para_visited_radius":5.5},#标记访问了半径
                    {"para_visited_radius_z":5.5},
                    {"position_add_high_for_guv":0.0},
                    {"least_visible_unknown_num":100},#覆盖到未标绿点小于这个数 则使命结束
                    {"least_visible_free_num":100},
                    {"Set_robot_neigbour_connected":False},#设置机器人周围3*3的free为connected
                    {"in_collision_viewpoint_get_connected_viewpoint_dis":3.2},
                    {"search_radius_thr":3.0},#如果frontier viewpoint点卡在墙里了，用周围 1 内的空闲点设置新的position



                    {"kGreedyViewPointSampleRange":3},
                    {"kLocalPathOptimizationItrMax":10},
                    {"kViewPointCollisionMargin":0.7},
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
                    {"kSensorRange":12.0},
                    {"kNeighborRange":3.0},
                    {"kCoverageOcclusionThr":0.3},
                    {"kCoverageDilationRadius":1.0},
                    # # Grid World
                    {"kGridWorldXNum":121},
                    {"kGridWorldYNum":121},
                    {"kGridWorldZNum":121},
                    {"kGridWorldCellHeight":3.0},#这项废弃
                    {"kGridWorldNearbyGridNum":5},
                    {"kMinAddPointNumSmall":20},#候选点包含 墙面点 的最低数量（local 和 grid world）
                    {"kMinAddPointNumBig":40},  #gridworld中为了返exploring 候选点包含 墙面点 的数量
                    {"kMinAddFrontierPointNum":15},#候选点包含 frontier 的最低数量（local 和 grid world）
                    {"kCellExploringToCoveredThr":1}, #候选点小于这个数量 设定covered
                    {"kCellCoveredToExploringThr":5}, #候选个数 大于这个数量 才能反exploring
                    {"kCellExploringToAlmostCoveredThr":10},#废弃
                    {"kCellAlmostCoveredToExploringThr":20},#废弃
                    {"kCellUnknownToExploringThr":1},#废弃
                    {"Estime_any_Frontier_viewpoint_asexploring":True},#只要是fontier在格子里就定义为exploring
                    # # Visualization
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
                    {"waypointpub_triger_time":10.0},
                    {"excute_triger_time":10.0},
                    {"low_speed_run":False},
                    {"use_frontier_viewpoint":False},
                    {"least_explore_time_s":60}, #最少探索时长，防止开头就停
                    {'visitd_radius': 3.5},#访问半径
                    {"visitd_radius_z":1.2},#这个一般取z的resolution
                    {'show_time': True },
                    {"keypose_graph/add_edge_Thr": 200},
                    {"invalidate_viewpoint_and_point_in_covered_boxes":True},
                    {"viewpoint_manager/disable_outsider_viewpoints":False},
                    {"viewpoint_manager/smaller_visited_radius":True},
                    {"use_time_interval":True},
                    {"way_point_in_free":True},
                    {"smaller_viewpoint_wait_time_relocation":1},
                    ],
        namespace="uav0",
        remappings=[
        # 重映射 position_cmd 话题
            # ('uav_3d_lidar_msg', '/drone_0_pcl_render_node/cloud'),
            # ('uav_state_estimation', '/drone_0_visual_slam/odom'),
            # ('uav_3d_lidar_msg', '/registered_scan'),
            # ('uav_state_estimation', '/state_estimation_at_scan'),
            ('/uav_3d_lidar_msg', '/pcl_scan_at_map'),
            ('/uav_state_estimation', '/drone_0_visual_slam/odom'),
            ('/terrain_map_ext', "/terrain_map_ext"),
            ('/terrain_map', "/terrain_map"),
            ('/start_signal', "/start_signal"),
            ('/way_point', '/move_base_simple/goal_point'),#point类型，用于给ros2的ego发消息

            #注意！！！ 冠名命名空间后，“/”不能缺，不能随便的去掉“/”
            ],
            
            
    )
    rviz_para_dir = os.path.join(get_package_share_directory('uav_planner'), 'rviz', 'uav_planner.rviz')
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
