cmake_minimum_required(VERSION 3.5)
project(azj_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    dynamic_reconfigure
    roslib
    quadrotor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

# rog_map 只作为库依赖使用，不修改其源码。
# 它已编译为 devel/lib/librog_map.a 并导出 rog_mapConfig.cmake。
find_package(rog_map REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES azj_core
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs
                   visualization_msgs pcl_ros pcl_conversions tf2 tf2_ros
                   dynamic_reconfigure roslib quadrotor_msgs
    DEPENDS Eigen3 PCL yaml-cpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${rog_map_INCLUDE_DIRS}
)

# rog_map 依赖的 libdw 装在用户目录下（无 sudo 安装）
link_directories(/home/spadea/.local/deps/usr/lib/x86_64-linux-gnu)
include_directories(SYSTEM /home/spadea/.local/deps/usr/include)

# ---------------- 核心算法库（不依赖 ROS，可离线单测） ----------------
add_library(azj_core
    src/path_search.cpp
    src/trajectory.cpp
    src/curve_optimizer.cpp
)
target_include_directories(azj_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(azj_core PUBLIC ${Eigen3_LIBRARIES})


# ---------------- ROS 节点 ----------------
add_executable(azj_planner_node
    src/azj_planner_node.cpp
)

# rog_map 的头文件里有 #error 检查，必须显式声明 ROS 版本与索引原点约定。
# ROOT_DIR 是 rog_map config.hpp 里 replaceCmakeRootDir() 用的；本包不加载 pcd，
# 但符号必须存在才能编译。指向本包目录即可。
target_compile_definitions(azj_planner_node PRIVATE
    USE_ROS1
    ORIGIN_AT_CORNER
    "ROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\""
)

target_link_libraries(azj_planner_node
    azj_core
    ${catkin_LIBRARIES}
    ${rog_map_LIBRARIES}
    ${PCL_LIBRARIES}
    yaml-cpp
    dw
)
add_dependencies(azj_planner_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# ---------------- 离线验证（仅构建，不安装） ----------------
add_executable(offline_clearance_test
    test/offline_clearance_test.cpp
)
target_link_libraries(offline_clearance_test azj_core)

add_executable(trajectory_test
    test/trajectory_test.cpp
)
target_link_libraries(trajectory_test azj_core)

add_executable(curve_optimizer_test
    test/curve_optimizer_test.cpp
)
target_link_libraries(curve_optimizer_test azj_core)

add_executable(plan_mode_test
    test/plan_mode_test.cpp
)
target_link_libraries(plan_mode_test azj_core)

# ---------------- 安装 ----------------
install(TARGETS azj_planner_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
