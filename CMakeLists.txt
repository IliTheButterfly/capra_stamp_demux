cmake_minimum_required(VERSION 3.8)
project(capra_stamp_demux)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PROJECT_DEPEND
  rclcpp
  capra_control_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PROJECT_DEPEND})
  find_package(${dep} REQUIRED)
endforeach()

add_library(stamp_demux_node src/stamp_demux_node.cpp)
target_include_directories(stamp_demux_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stamp_demux_node
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(stamp_demux_node PUBLIC cxx_std_20)

add_executable(stamp_demux src/stamp_demux_main.cpp)
target_link_libraries(stamp_demux stamp_demux_node)
ament_target_dependencies(stamp_demux
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(stamp_demux PRIVATE cxx_std_20)

install(TARGETS
  stamp_demux_node
  stamp_demux
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_unstamper test/test_unstamper.cpp)
  if(TARGET test_unstamper)
    target_include_directories(test_unstamper PRIVATE include)
    ament_target_dependencies(test_unstamper
      ${THIS_PROJECT_DEPEND}
    )
  endif()
endif()

ament_package()
