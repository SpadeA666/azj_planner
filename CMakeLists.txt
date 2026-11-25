cmake_minimum_required(VERSION 3.5)
project(azj_planner)

# ═══════════════════════════════════════════════════════════════════════
# ★★★ 必须显式设置 Release（2026-11-17 性能根因修复）
#
# catkin 默认**不设置** CMAKE_BUILD_TYPE → CMake 退回空字符串 → 编译器
# 用 -O0（无任何优化）。而这个包重度依赖 Eigen 表达式模板和模板内联，
# -O0 下性能会差 **10~50 倍**。
#
# 实测（同一份代码、同一场景）：
#   CXX_FLAGS = "-fPIC -Wall ... -std=c++11"（无 -O）→ curve = 400ms
#   CXX_FLAGS = "-O3 ..."                          → curve = 30~50ms
#
# 对照：SUPER 的 CMakeLists.txt:4-5 明确写了
#   set(CMAKE_BUILD_TYPE "Release")
#   set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
# 所以它（以及被它带成 Release 的 rog_map）一直是 -O3，
# 而我们一直是 -O0 —— 这是"规划慢"的最大单一原因，与算法无关。
#
# 只在本项目未指定时兜底，不覆盖外部显式设置（如 Debug 排查）。
# ═══════════════════════════════════════════════════════════════════════
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(NOT CMAKE_CXX_FLAGS_RELEASE)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()
# 确保 Release 分支真的带 -O3（有些环境把它覆盖成 -O2 或空）
if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NOT CMAKE_CXX_FLAGS_RELEASE MATCHES "-O[23]")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

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
    src/minco.cpp
    src/minco_optimizer.cpp
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

add_executable(minco_test
    test/minco_test.cpp
)
target_link_libraries(minco_test azj_core)

add_executable(minco_optimizer_test
    test/minco_optimizer_test.cpp
)
target_link_libraries(minco_optimizer_test azj_core)

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
