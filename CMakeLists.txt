cmake_minimum_required(VERSION 3.16)
project(glim_localization VERSION 0.1.0 LANGUAGES CXX)

add_compile_options(-std=c++17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(glim REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(spdlog REQUIRED)

option(BUILD_WITH_CUDA "Build glim_localization GPU registration support" OFF)
if(BUILD_WITH_CUDA)
  if(DEFINED GTSAM_POINTS_USE_CUDA AND GTSAM_POINTS_USE_CUDA)
    add_definitions(-DGLIM_LOCALIZATION_USE_CUDA)
    list(APPEND glim_localization_GPU_SRCS
      src/glim_localization/registration/gpu_vgicp_map_registration.cpp
    )
  else()
    message(WARNING "BUILD_WITH_CUDA is ON, but gtsam_points CUDA support was not detected. GPU VGICP will not be built.")
  endif()
endif()

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(visualization_msgs REQUIRED)
  elseif($ENV{ROS_VERSION} EQUAL 1)
    find_package(catkin REQUIRED)
  endif()
endif()

add_library(glim_localization SHARED
  src/glim_localization/core/localization_options.cpp
  src/glim_localization/initialization/config_initial_pose_provider.cpp
  src/glim_localization/initialization/runtime_initial_pose.cpp
  src/glim_localization/map/localization_map.cpp
  src/glim_localization/map/local_target_map.cpp
  src/glim_localization/map/submap_index.cpp
  src/glim_localization/map_loader/map_format_checker.cpp
  src/glim_localization/map_loader/glim_map_loader.cpp
  src/glim_localization/output/trajectory_writer.cpp
  src/glim_localization/registration/cpu_gicp_map_registration.cpp
  src/glim_localization/relocalization/geometric_verifier.cpp
  src/glim_localization/relocalization/runtime_relocalization_request.cpp
  src/glim_localization/relocalization/scan_context_relocalizer.cpp
  ${glim_localization_GPU_SRCS}
)
target_include_directories(glim_localization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(glim_localization
  glim::glim
  gtsam_points::gtsam_points
  spdlog::spdlog
)

add_library(odometry_estimation_localization_cpu SHARED
  modules/odometry/localization_cpu/odometry_estimation_localization_cpu.cpp
  modules/odometry/localization_cpu/odometry_estimation_localization_cpu_create.cpp
)
target_include_directories(odometry_estimation_localization_cpu PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/odometry/localization_cpu
)
target_link_libraries(odometry_estimation_localization_cpu
  glim_localization
  glim::glim
)

set(glim_localization_LIBRARIES
  glim_localization
  odometry_estimation_localization_cpu
)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    add_library(localization_publisher SHARED
      modules/extension/localization_publisher/localization_publisher.cpp
    )
    target_link_libraries(localization_publisher
      glim_localization
      glim::glim
    )
    ament_target_dependencies(localization_publisher
      rclcpp
      std_msgs
      std_srvs
      nav_msgs
      sensor_msgs
      geometry_msgs
      tf2_ros
      visualization_msgs
    )
    list(APPEND glim_localization_LIBRARIES localization_publisher)
  endif()
endif()

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    set(CMAKE_INSTALL_INCLUDEDIR "include")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "bin/glim_localization")
    set(CMAKE_INSTALL_DATADIR "share/glim_localization")
    set(CMAKE_CONFIG_INSTALL_DIR "share/glim_localization")
  elseif($ENV{ROS_VERSION} EQUAL 2)
    set(CMAKE_INSTALL_INCLUDEDIR "include")
    set(CMAKE_INSTALL_LIBDIR "lib")
    set(CMAKE_INSTALL_BINDIR "lib/${PROJECT_NAME}")
    set(CMAKE_INSTALL_DATADIR "share/glim_localization")
    set(CMAKE_CONFIG_INSTALL_DIR "share/glim_localization")
  endif()
else()
  include(GNUInstallDirs)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config DESTINATION ${CMAKE_INSTALL_DATADIR})
install(DIRECTORY rviz DESTINATION ${CMAKE_INSTALL_DATADIR})
install(PROGRAMS
  tools/run_offline_localization.sh
  tools/plot_trajectory.py
  tools/monitor/run_with_time.sh
  tools/monitor/monitor_ps.sh
  tools/monitor/monitor_pidstat.sh
  tools/monitor/monitor_iostat.sh
  tools/monitor/monitor_gpu.sh
  tools/monitor/summarize_usage.py
  tools/monitor/usage_report_lib.py
  tools/monitor/plot_usage.py
  tools/monitor/generate_usage_report.py
  DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS ${glim_localization_LIBRARIES}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_executable(glim_localization_map_info
  tools/glim_localization_map_info.cpp
)
target_link_libraries(glim_localization_map_info
  glim_localization
)

add_executable(benchmark_localization
  tools/benchmark_localization.cpp
)
target_link_libraries(benchmark_localization
  glim_localization
)

install(TARGETS glim_localization_map_info benchmark_localization
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CTest)
if(BUILD_TESTING)
  add_executable(test_localization_options
    tests/test_localization_options.cpp
  )
  target_link_libraries(test_localization_options
    glim_localization
  )
  add_test(NAME test_localization_options COMMAND test_localization_options)

  add_executable(test_map_format_checker
    tests/test_map_format_checker.cpp
  )
  target_link_libraries(test_map_format_checker
    glim_localization
  )
  add_test(NAME test_map_format_checker COMMAND test_map_format_checker)

  add_executable(test_glim_map_loader
    tests/test_glim_map_loader.cpp
  )
  target_link_libraries(test_glim_map_loader
    glim_localization
  )
  add_test(NAME test_glim_map_loader COMMAND test_glim_map_loader)

  add_executable(test_localization_map
    tests/test_localization_map.cpp
  )
  target_link_libraries(test_localization_map
    glim_localization
  )
  add_test(NAME test_localization_map COMMAND test_localization_map)

  add_executable(test_submap_index
    tests/test_submap_index.cpp
  )
  target_link_libraries(test_submap_index
    glim_localization
  )
  add_test(NAME test_submap_index COMMAND test_submap_index)

  add_executable(test_cpu_map_registration
    tests/test_cpu_map_registration.cpp
  )
  target_link_libraries(test_cpu_map_registration
    glim_localization
  )
  add_test(NAME test_cpu_map_registration COMMAND test_cpu_map_registration)

  add_executable(test_scan_context_relocalizer
    tests/test_scan_context_relocalizer.cpp
  )
  target_link_libraries(test_scan_context_relocalizer
    glim_localization
  )
  add_test(NAME test_scan_context_relocalizer COMMAND test_scan_context_relocalizer)

  add_executable(test_plugin_module_loading
    tests/test_plugin_module_loading.cpp
  )
  target_link_libraries(test_plugin_module_loading
    ${CMAKE_DL_LIBS}
  )
  add_test(
    NAME test_plugin_module_loading_odometry
    COMMAND test_plugin_module_loading $<TARGET_FILE:odometry_estimation_localization_cpu> create_odometry_estimation_module
  )
  if(DEFINED ENV{ROS_VERSION})
    if($ENV{ROS_VERSION} EQUAL 2)
      add_test(
        NAME test_plugin_module_loading_extension
        COMMAND test_plugin_module_loading $<TARGET_FILE:localization_publisher> create_extension_module
      )
    endif()
  endif()
endif()

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    ament_target_dependencies(glim_localization glim)
    ament_target_dependencies(odometry_estimation_localization_cpu glim)
    ament_export_include_directories(include)
    ament_export_dependencies(glim gtsam_points spdlog rclcpp std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs tf2_ros visualization_msgs)
    ament_export_libraries(${glim_localization_LIBRARIES})
    ament_package()
  elseif($ENV{ROS_VERSION} EQUAL 1)
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${glim_localization_LIBRARIES}
    )
  endif()
endif()
