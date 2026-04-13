cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-O3)

## 使用C++14标准（Eigen3、PCL等需要至少C++14）
add_compile_options(-std=c++14)

add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)
  

find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

## 添加编译警告选项，帮助发现潜在错误
add_compile_options(
  -Wall              # 启用所有常见警告
  -Wextra            # 额外警告
  -Wuninitialized    # 警告未初始化变量（需要 -O1 或更高优化级别）
  -Wmaybe-uninitialized  # 警告可能未初始化的变量
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_conversions
  tf
)
add_message_files(
  FILES
  CloudWithPose.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

## 查找系统依赖：PCL、Eigen3、yaml-cpp
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLOG REQUIRED libglog)
find_package(Ceres QUIET)
if (Ceres_FOUND)
  set(CERES_LIBRARIES Ceres::ceres)
else()
  find_path(CERES_INCLUDE_DIRS ceres/ceres.h
    PATHS /usr/local/include /usr/include
  )
  find_library(CERES_LIBRARY NAMES ceres
    PATHS /usr/local/lib /usr/lib /usr/lib/x86_64-linux-gnu
  )
  find_library(SUITESPARSE_CONFIG_LIBRARY NAMES suitesparseconfig
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(CHOLMOD_LIBRARY NAMES cholmod
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(SPQR_LIBRARY NAMES spqr
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(CXSPARSE_LIBRARY NAMES cxsparse
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(AMD_LIBRARY NAMES amd
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(CAMD_LIBRARY NAMES camd
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(COLAMD_LIBRARY NAMES colamd
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(CCOLAMD_LIBRARY NAMES ccolamd
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  find_library(METIS_LIBRARY NAMES metis
    PATHS /lib /usr/lib /usr/lib/x86_64-linux-gnu /lib/x86_64-linux-gnu
  )
  if (NOT CERES_INCLUDE_DIRS OR NOT CERES_LIBRARY OR NOT SUITESPARSE_CONFIG_LIBRARY OR
      NOT CHOLMOD_LIBRARY OR NOT SPQR_LIBRARY OR NOT CXSPARSE_LIBRARY OR
      NOT AMD_LIBRARY OR NOT CAMD_LIBRARY OR NOT COLAMD_LIBRARY OR NOT CCOLAMD_LIBRARY)
    message(FATAL_ERROR "Ceres Solver not found. Install libceres-dev or provide a CeresConfig.cmake.")
  endif()
  set(CERES_LIBRARIES
    ${CERES_LIBRARY}
    ${SPQR_LIBRARY}
    ${CHOLMOD_LIBRARY}
    ${CXSPARSE_LIBRARY}
    ${AMD_LIBRARY}
    ${CAMD_LIBRARY}
    ${COLAMD_LIBRARY}
    ${CCOLAMD_LIBRARY}
    ${SUITESPARSE_CONFIG_LIBRARY}
  )
  if (METIS_LIBRARY)
    list(APPEND CERES_LIBRARIES ${METIS_LIBRARY})
  endif()
  list(APPEND CERES_LIBRARIES ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES})
endif()


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions tf
  DEPENDS PCL EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

## 定义 PROJ_DIR 宏，使 globaldefine.h 中的 PROJ_DIR 指向包的实际路径
add_definitions(-DPROJ_DIR="${PROJECT_SOURCE_DIR}")

## ============================================
## 1) 核心算法库 ieskf_slam_lib（与ROS无关）
## ============================================
## 包含 FrontEnd、IESKF 等核心模块的实现
add_library(ieskf_slam_lib
  src/ieskf_slam/modules/frontend/frontend.cpp
  src/ieskf_slam/modules/ieskf/ieskf.cpp
  src/ieskf_slam/modules/map/rect_map_manager.cpp
  src/ieskf_slam/modules/map/map_manager_factory.cpp
  src/ieskf_slam/modules/map/ikdtree_map_manager.cpp
  src/ieskf_slam/modules/frontbackPropagate/frontback_propagate.cpp
  src/ieskf_slam/modules/backend/keyframe_store.cpp
  src/ieskf_slam/modules/backend/scan_context_loop_detector.cpp
  src/ieskf_slam/modules/backend/icp_loop_registrar.cpp
  src/ieskf_slam/modules/backend/ceres_pose_graph_optimizer.cpp
  src/ieskf_slam/modules/backend/backend_coordinator.cpp
  include/ieskf_slam/common/scan_context/Scancontext.cpp
  include/ieskf_slam/third_party/ikd_tree/ikd_Tree.cpp
)
add_dependencies(ieskf_slam_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ieskf_slam_lib
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${CERES_LIBRARIES}
  Threads::Threads
)

## ============================================
## 2) ROS 封装库 ieskf_wrapper_lib
## ============================================
## 包含 IESKFFrontEndWrapper 等 ROS 封装层的实现
add_library(ieskf_wrapper_lib
  src/wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.cpp
  src/wrapper/ros_noetic/ieskf_backend_noetic_wrapper.cpp
)
add_dependencies(ieskf_wrapper_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ieskf_wrapper_lib
  ieskf_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

## ============================================
## 3) 可执行节点 ieskf_frontend_node
## ============================================
## launch 文件中 type="ieskf_frontend_node"，所以可执行文件名必须匹配
add_executable(ieskf_frontend_node
  src/app/ieskf_noetic_running_node.cpp
)
add_dependencies(ieskf_frontend_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ieskf_frontend_node
  ieskf_wrapper_lib
  ieskf_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(ieskf_backend_node
  src/app/ieskf_backend_node.cpp
)
add_dependencies(ieskf_backend_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ieskf_backend_node
  ieskf_wrapper_lib
  ieskf_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
)

add_executable(ieskf_slam_node
  src/app/ieskf_slam_node.cpp
)
add_dependencies(ieskf_slam_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ieskf_slam_node
  ieskf_wrapper_lib
  ieskf_slam_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ieskf_slam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
