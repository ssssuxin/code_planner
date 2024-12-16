cmake_minimum_required(VERSION 3.5)
project(uav_planner)
# EXPL project(tare_planner)生成了这个变量${PROJECT_NAME}
# add_compile_options(-std=c++11)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# EXPL ${PROJECT_SOURCE_DIR}表示当前所在目录

#####-----------------------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
set(MY_DEPENDENCIES
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  PCL
  Eigen3
  visualization_msgs
)
# EXPL 这里find_package和上面的不一样，这里会独立生成 ${PCL_LIBRARIES_DIRS}  或者  ${PCL_INCLUDE_DIRS}

# EXPL include_directories()这是个独立的功能，指库文件所在的路径，编译的时候去这些路径找库文件
include_directories(
  include
  # ${ament_INCLUDE_DIRS}
  # ${PCL_INCLUDE_DIRS}
  # "${PROJECT_SOURCE_DIR}/include"
  or-tools/include
  # ${EIGEN3_INCLUDE_DIR}
  # /opt/nea/topaz/include
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
  or-tools/lib
)
# EXPL 这里重要的是or-tools/lib，其他两个不重要   是为了target_link_libraries(tsp_solver libortools.so ) 能够连得上

add_library(lidar_model SHARED
src/lidar_model/lidar_model.cpp
)
ament_target_dependencies(lidar_model
  ${MY_DEPENDENCIES}   
)
target_link_libraries(lidar_model ${PCL_LIBRARIES})
install(
  TARGETS lidar_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(graph SHARED
  src/graph/graph.cpp
)
ament_target_dependencies(graph
  ${MY_DEPENDENCIES}
)
install(
  TARGETS graph
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(misc_utils SHARED
  src/utils/misc_utils.cpp
)
ament_target_dependencies(misc_utils
  ${MY_DEPENDENCIES}  
)
target_link_libraries(misc_utils ${PCL_LIBRARIES})
install(
  TARGETS misc_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(pointcloud_utils SHARED
  src/utils/pointcloud_utils.cpp
)
ament_target_dependencies(pointcloud_utils
  ${MY_DEPENDENCIES}   
)
target_link_libraries(pointcloud_utils ${PCL_LIBRARIES})
install(
  TARGETS pointcloud_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(tsp_solver SHARED
  src/tsp_solver/tsp_solver.cpp
)
ament_target_dependencies(tsp_solver
  ${MY_DEPENDENCIES} 
)
target_link_libraries(tsp_solver libortools.so )
target_link_libraries(tsp_solver libglog.so )
install(
  TARGETS tsp_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(viewpoint SHARED
src/viewpoint/viewpoint.cpp
)
ament_target_dependencies(viewpoint
  ${MY_DEPENDENCIES}    
)
target_link_libraries(viewpoint lidar_model)
install(
  TARGETS viewpoint
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(rolling_grid SHARED
src/rolling_grid/rolling_grid.cpp
)
ament_target_dependencies(rolling_grid
  ${MY_DEPENDENCIES}    
)
target_link_libraries(rolling_grid misc_utils pointcloud_utils)
install(
  TARGETS rolling_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(viewpoint_manager SHARED
src/viewpoint_manager/viewpoint_manager.cpp
)
ament_target_dependencies(viewpoint_manager
  ${MY_DEPENDENCIES}   
)
target_link_libraries(viewpoint_manager rolling_grid viewpoint )
install(
  TARGETS viewpoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(uav_viewpoint_manager SHARED
src/uav_viewpoint_manager/viewpoint_manager.cpp
)
ament_target_dependencies(uav_viewpoint_manager
  ${MY_DEPENDENCIES}   
)
target_link_libraries(uav_viewpoint_manager rolling_grid viewpoint )
install(
  TARGETS uav_viewpoint_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(local_coverage_planner SHARED
src/local_coverage_planner/local_coverage_planner.cpp
)
ament_target_dependencies(local_coverage_planner
  ${MY_DEPENDENCIES}    
)
target_link_libraries(local_coverage_planner viewpoint_manager uav_viewpoint_manager)
install(
  TARGETS local_coverage_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(grid_world SHARED
src/grid_world/grid_world.cpp
)
ament_target_dependencies(grid_world
  ${MY_DEPENDENCIES}    
)
target_link_libraries(grid_world viewpoint_manager uav_viewpoint_manager tsp_solver  exploration_path)
install(
  TARGETS grid_world
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(pointcloud_manager SHARED
src/pointcloud_manager/pointcloud_manager.cpp
)
ament_target_dependencies(pointcloud_manager
  ${MY_DEPENDENCIES}    
)
install(
  TARGETS pointcloud_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(keypose_graph SHARED
src/keypose_graph/keypose_graph.cpp
)
ament_target_dependencies(keypose_graph
  ${MY_DEPENDENCIES}      
)
target_link_libraries(keypose_graph viewpoint_manager uav_viewpoint_manager ${PCL_LIBRARIES})
install(
  TARGETS keypose_graph
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_library(uav_keypose_graph SHARED
src/uav_keypose_graph/keypose_graph.cpp
)
ament_target_dependencies(uav_keypose_graph
  ${MY_DEPENDENCIES}      
)
target_link_libraries(uav_keypose_graph viewpoint_manager uav_viewpoint_manager ${PCL_LIBRARIES})
install(
  TARGETS uav_keypose_graph
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(rolling_occupancy_grid SHARED
src/rolling_occupancy_grid/rolling_occupancy_grid.cpp
)
ament_target_dependencies(rolling_occupancy_grid
  ${MY_DEPENDENCIES}   
)

install(
  TARGETS rolling_occupancy_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(planning_env SHARED
src/planning_env/planning_env.cpp
)
ament_target_dependencies(planning_env
  ${MY_DEPENDENCIES}    
)
target_link_libraries(planning_env rolling_occupancy_grid ${PCL_LIBRARIES})
install(
  TARGETS planning_env
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(exploration_path SHARED
src/exploration_path/exploration_path.cpp
)
ament_target_dependencies(exploration_path
  ${MY_DEPENDENCIES}    
)
target_link_libraries(exploration_path ${PCL_LIBRARIES})
install(
  TARGETS exploration_path
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(tare_visualizer SHARED
src/tare_visualizer/tare_visualizer.cpp
)
ament_target_dependencies(tare_visualizer
  ${MY_DEPENDENCIES}    
)
target_link_libraries(tare_visualizer ${PCL_LIBRARIES})
install(
  TARGETS tare_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
add_library(sensor_coverage_planner_ground SHARED
src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp
)
ament_target_dependencies(sensor_coverage_planner_ground
  ${MY_DEPENDENCIES}   
)
target_link_libraries(sensor_coverage_planner_ground graph planning_env keypose_graph uav_keypose_graph viewpoint_manager uav_viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer )
install(
  TARGETS sensor_coverage_planner_ground
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# add_executable(test_node  src/uav_viewpoint_manager/viewpoint_manager.cpp)
# ament_target_dependencies(test_node
#   ${MY_DEPENDENCIES}   
# )
# target_link_libraries(test_node rolling_grid viewpoint )
# install(TARGETS
# test_node
#   DESTINATION lib/${PROJECT_NAME}
# )
add_executable(test_node  src/uav_viewpoint_manager/viewpoint_manager.cpp)
ament_target_dependencies(test_node
  ${MY_DEPENDENCIES}   
)
target_link_libraries(test_node rolling_grid viewpoint grid_world keypose_graph uav_keypose_graph planning_env pointcloud_manager ${PCL_LIBRARIES})
install(TARGETS
test_node
  DESTINATION lib/${PROJECT_NAME}
)
# install(
#   TARGETS test_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )




add_executable(main_node src/main_node/main_node.cpp)
ament_target_dependencies(main_node
${MY_DEPENDENCIES} 
)
target_link_libraries(main_node ${PCL_LIBRARIES}  sensor_coverage_planner_ground )

#############
## Install ##
#############


# EXPL 这上面两个没什么用
#############
## Testing ##
#############
install(TARGETS
main_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/#把存yaml文件的文件夹config移到share
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
##1.自身工程的.h库
ament_package()